#include "alde3030.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#include <cstdio>
#ifdef USE_ESP32
#include "esphome/components/uart/uart_component_esp_idf.h"
#endif

namespace esphome {
namespace alde3030 {

// ── LIN PID with parity bits ──────────────────────────────────────────────
uint8_t Alde3030Component::lin_pid_(uint8_t id) {
  uint8_t p0 = ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 1;
  uint8_t p1 = (~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5))) & 1;
  return (id & 0x3F) | (p0 << 6) | (p1 << 7);
}

// ── LIN checksum ──────────────────────────────────────────────────────────
uint8_t Alde3030Component::lin_checksum_(uint8_t pid, const uint8_t *data, size_t len, bool include_pid) {
  uint16_t sum = include_pid ? pid : 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
    if (sum > 0xFF) sum -= 0xFF;
  }
  return (uint8_t) (~sum & 0xFF);
}

// ── LIN break (baud-rate swap) ────────────────────────────────────────────
void Alde3030Component::send_lin_break_() {
  this->flush();
#ifdef USE_ESP32
  uart_port_t uart_num = static_cast<uart_port_t>(
      static_cast<uart::IDFUARTComponent *>(this->parent_)->get_hw_serial_number());
  uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
  uart_set_baudrate(uart_num, 4800);
  this->write_byte(0x00);
  uart_wait_tx_done(uart_num, pdMS_TO_TICKS(20));
  uart_set_baudrate(uart_num, 19200);
#else
  this->write_byte(0x00);
  delayMicroseconds(1000);
#endif
}

void Alde3030Component::send_lin_header_(uint8_t lin_id) {
  send_lin_break_();
  this->write_byte(0x55);
  this->write_byte(lin_pid_(lin_id));
}

uint8_t Alde3030Component::encode_setpoint_(float temp_c) {
  if (temp_c < 5.0f) temp_c = 5.0f;
  if (temp_c > 30.0f) temp_c = 30.0f;
  return (uint8_t) ((temp_c - 5.0f) / 0.5f + 0.25f);
}

float Alde3030Component::decode_temp_(uint8_t raw) {
  if (raw <= 0xFA) return (raw * 0.5f) - 42.0f;
  return NAN;
}

// ── Diagnostic "hello" frame ──────────────────────────────────────────────
void Alde3030Component::send_diag_frame_() {
  uint8_t pid = lin_pid_(LIN_ID_DIAG);
  uint8_t chk = lin_checksum_(pid, DIAG_PAYLOAD, 8, true);
  send_lin_header_(LIN_ID_DIAG);
  for (int i = 0; i < 8; i++) this->write_byte(DIAG_PAYLOAD[i]);
  this->write_byte(chk);
  this->flush();
  last_diag_ms_ = millis();
  ESP_LOGD(TAG, "TX diag/hello frame");
}

// ── Prepare a change: start from what the panel reports right now ─────────
bool Alde3030Component::begin_change_() {
  if (!first_info_received_) {
    ESP_LOGW(TAG, "Ignoring command: no status read from panel yet");
    return false;
  }
  if (!pending_) {
    desired_zone1_      = zone1_setpoint_;
    desired_gas_        = gas_active_;
    desired_elec_kw_    = electric_kw_;
    desired_panel_      = panel_on_;
    desired_water_mode_ = water_mode_;
    pending_since_ms_   = millis();
  }
  pending_ = true;
  return true;
}

// ── Control frame (only called when a change is pending) ──────────────────
void Alde3030Component::send_control_frame_() {
  uint8_t data[8];
  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x00;

  data[3] = encode_setpoint_(desired_zone1_);
  if (desired_gas_) data[3] |= (1 << 6);
  if (gas_valve_open_) data[3] |= (1 << 7);

  data[4] = (raw_b4_ & 0x3F) | ((desired_elec_kw_ & 0x03) << 6);

  data[5] = raw_b5_ & 0b11100110;  // keep read-only flags, clear panel + water bits
  if (desired_panel_) data[5] |= (1 << 0);
  data[5] |= ((desired_water_mode_ & 0x03) << 3);

  data[6] = 0xFF;
  data[7] = 0xFF;

  uint8_t pid = lin_pid_(LIN_ID_CONTROL);
  uint8_t chk = lin_checksum_(pid, data, 8, !use_classic_checksum_);

  send_lin_header_(LIN_ID_CONTROL);
  for (int i = 0; i < 8; i++) this->write_byte(data[i]);
  this->write_byte(chk);
  this->flush();

  ESP_LOGI(TAG, "TX Control: %02X %02X %02X %02X %02X %02X %02X %02X [chk=%02X]",
           data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], chk);
}

void Alde3030Component::request_info_frame_() {
  while (this->available()) this->read();
  send_lin_header_(LIN_ID_INFO);
  rx_pos_ = 0;
  rx_active_ = true;
  rx_start_ms_ = millis();
}

void Alde3030Component::parse_info_frame_(const uint8_t *d) {
  memcpy(last_info_, d, 8);
  ESP_LOGV(TAG, "RX Info: %02X %02X %02X %02X %02X %02X %02X %02X",
           d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

  zone1_actual_   = decode_temp_(d[0]);
  outdoor_actual_ = decode_temp_(d[2]);

  uint8_t sp1_raw = d[3] & 0x3F;
  if (sp1_raw <= 0x32) zone1_setpoint_ = (sp1_raw * 0.5f) + 5.0f;
  gas_active_     = (d[3] >> 6) & 1;
  gas_valve_open_ = (d[3] >> 7) & 1;

  raw_b4_      = d[4];
  electric_kw_ = (d[4] >> 6) & 0x03;

  raw_b5_        = d[5];
  panel_on_      = (d[5] >> 0) & 1;
  panel_busy_    = (d[5] >> 1) & 1;
  error_present_ = (d[5] >> 2) & 1;
  uint8_t wm     = (d[5] >> 3) & 0x03;
  water_mode_    = (wm == 3) ? 1 : wm;
  ac_available_  = (d[5] >> 5) & 1;
  pump_running_  = (d[5] >> 7) & 1;

  if (!first_info_received_) {
    ESP_LOGI(TAG, "First status read from panel - reading works");
  }
  first_info_received_ = true;
  last_info_ms_ = millis();

  ESP_LOGD(TAG, "Room=%.1f set=%.1f Out=%.1f Gas=%s Elec=%ukW Panel=%s Water=%u Pump=%s Err=%s [%02X %02X %02X]",
           zone1_actual_, zone1_setpoint_, outdoor_actual_, gas_active_ ? "on" : "off",
           electric_kw_, panel_on_ ? "on" : "off", water_mode_, pump_running_ ? "yes" : "no",
           error_present_ ? "YES" : "no", d[3], d[4], d[5]);

  for (auto &cb : on_state_callbacks_) cb();
}

void Alde3030Component::setup() {
  ESP_LOGCONFIG(TAG, "Alde CI-bus (3020, safe mode) starting...");
  delay(100);
  send_diag_frame_();
  delay(50);
}

void Alde3030Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Alde CI-bus (3020, safe mode):");
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms, control only on change", SEND_INTERVAL_MS);
}

void Alde3030Component::loop() {
  // ── Receive info frame ──────────────────────────────────────────────
  if (rx_active_) {
    const uint8_t pid = lin_pid_(LIN_ID_INFO);
    while (this->available() && rx_pos_ < RX_BUF_SIZE) rx_buf_[rx_pos_++] = this->read();

    bool found = false;
    bool classic = false;
    uint8_t offset = 0;
    if (rx_pos_ >= 9) {
      for (uint8_t off = 0; off <= rx_pos_ - 9; off++) {
        const uint8_t *c = &rx_buf_[off];
        bool enh = (c[8] == lin_checksum_(pid, c, 8, true));
        bool cla = (c[8] == lin_checksum_(pid, c, 8, false));
        if (enh || cla) {
          found = true;
          classic = cla && !enh;
          offset = off;
          break;
        }
      }
    }

    if (found) {
      if (classic != use_classic_checksum_) {
        use_classic_checksum_ = classic;
        ESP_LOGI(TAG, "Detected %s checksum", classic ? "classic" : "enhanced");
      }
      parse_info_frame_(&rx_buf_[offset]);
      rx_active_ = false;
      rx_pos_ = 0;
      while (this->available()) this->read();
    } else if (millis() - rx_start_ms_ > RX_TIMEOUT_MS) {
      timeouts_++;
      if (timeouts_ <= 5 || timeouts_ % 20 == 0) {
        char hex[RX_BUF_SIZE * 3 + 1];
        uint16_t p = 0;
        for (uint8_t i = 0; i < rx_pos_ && p + 4 < sizeof(hex); i++)
          p += snprintf(&hex[p], sizeof(hex) - p, "%02X ", rx_buf_[i]);
        hex[p] = '\0';
        ESP_LOGW(TAG, "No valid info frame (%u bytes) [%s] (timeouts: %u)", rx_pos_, hex, timeouts_);
      }
      rx_active_ = false;
      rx_pos_ = 0;
      while (this->available()) this->read();
    }
  }

  if (rx_active_) return;

  uint32_t now = millis();
  if (now - last_send_ms_ < SEND_INTERVAL_MS) return;
  last_send_ms_ = now;

  // Re-send hello if the panel has been silent for a while
  if ((!first_info_received_ || now - last_info_ms_ > 10000) && now - last_diag_ms_ > 10000) {
    send_diag_frame_();
    delay(10);
  }

  // Send a control frame only when something was changed from HA
  if (pending_ && first_info_received_) {
    send_control_frame_();
    pending_ = false;
    delay(10);
  } else if (pending_ && now - pending_since_ms_ > 5000) {
    pending_ = false;
  }

  request_info_frame_();
}

// ── Setters ───────────────────────────────────────────────────────────────
void Alde3030Component::set_zone1_target(float t) {
  if (begin_change_()) desired_zone1_ = t;
}
void Alde3030Component::set_gas_enabled(bool on) {
  if (begin_change_()) desired_gas_ = on;
}
void Alde3030Component::set_electric_power(uint8_t kw) {
  if (begin_change_()) desired_elec_kw_ = kw;
}
void Alde3030Component::set_panel_on(bool on) {
  if (begin_change_()) desired_panel_ = on;
}
void Alde3030Component::set_water_mode(uint8_t m) {
  if (begin_change_()) desired_water_mode_ = m;
}

}  // namespace alde3030
}  // namespace esphome
