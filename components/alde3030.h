#pragma once

// Alde CI-bus (LIN 19200, ESP = master) – SAFE version for Alde Compact 3020.
// Based on holdestmade/esphome-alde-3020 (alde3030) and daviddootson/alde_3030_plus.
// Safety changes:
//  * Control frames are ONLY sent when something is changed from Home Assistant.
//  * A control frame is always built from the last state read from the panel
//    (read-modify-write), so nothing else is touched.
//  * Nothing is sent until a valid info frame has been received.

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <functional>

namespace esphome {
namespace alde3030 {

static const char *const TAG = "alde3030";

static const uint8_t LIN_ID_CONTROL = 0x1A;
static const uint8_t LIN_ID_INFO    = 0x1B;
static const uint8_t LIN_ID_DIAG    = 0x3C;

// Read-by-identifier "hello": NAD 0x10, supplier 0x41DE (Alde), function 0x0001 (Compact 3020)
static const uint8_t DIAG_PAYLOAD[8] = {0x10, 0x06, 0xB2, 0x00, 0xDE, 0x41, 0x01, 0x00};

class Alde3030Component : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ── Setters (called by sub-components) ───────────────────────────────
  void set_zone1_target(float temp_c);
  void set_gas_enabled(bool on);
  void set_electric_power(uint8_t kw);   // 0=Off 1=1kW 2=2kW 3=3kW
  void set_panel_on(bool on);
  void set_water_mode(uint8_t mode);     // 0=Off 1=On 2=Boost

  // ── Getters (latest parsed info frame) ───────────────────────────────
  float   get_zone1_actual()   const { return zone1_actual_; }
  float   get_outdoor_actual() const { return outdoor_actual_; }
  float   get_zone1_setpoint() const { return zone1_setpoint_; }
  bool    get_gas_active()     const { return gas_active_; }
  bool    get_gas_valve_open() const { return gas_valve_open_; }
  uint8_t get_electric_kw()    const { return electric_kw_; }
  bool    get_panel_on()       const { return panel_on_; }
  bool    get_panel_busy()     const { return panel_busy_; }
  bool    get_error_present()  const { return error_present_; }
  uint8_t get_water_mode()     const { return water_mode_; }
  bool    get_ac_available()   const { return ac_available_; }
  bool    get_pump_running()   const { return pump_running_; }
  uint8_t get_raw_byte(uint8_t i) const { return i < 8 ? last_info_[i] : 0; }

  void add_on_state_callback(std::function<void()> cb) {
    on_state_callbacks_.push_back(std::move(cb));
  }

 protected:
  void    send_lin_break_();
  void    send_lin_header_(uint8_t lin_id);
  uint8_t lin_pid_(uint8_t id);
  uint8_t lin_checksum_(uint8_t pid, const uint8_t *data, size_t len, bool include_pid);
  void    send_diag_frame_();
  void    send_control_frame_();
  void    request_info_frame_();
  void    parse_info_frame_(const uint8_t *data);
  bool    begin_change_();   // prepares desired_* from last report

  float   decode_temp_(uint8_t raw);
  uint8_t encode_setpoint_(float temp_c);

  // ── Desired state (only used when a change is pending) ───────────────
  float    desired_zone1_{20.0f};
  bool     desired_gas_{false};
  uint8_t  desired_elec_kw_{0};
  bool     desired_panel_{true};
  uint8_t  desired_water_mode_{0};
  bool     pending_{false};
  uint8_t  attempts_{0};
  static const uint8_t MAX_ATTEMPTS = 12;
  bool     report_matches_desired_();
  uint32_t pending_since_ms_{0};

  // ── Last info frame ──────────────────────────────────────────────────
  uint8_t  last_info_[8]{};
  uint8_t  raw_b4_{0x00};
  uint8_t  raw_b5_{0x00};
  bool     first_info_received_{false};
  uint32_t last_info_ms_{0};
  uint32_t last_diag_ms_{0};

  // ── Reported state ───────────────────────────────────────────────────
  float    zone1_actual_{NAN};
  float    outdoor_actual_{NAN};
  float    zone1_setpoint_{NAN};
  bool     gas_active_{false};
  bool     gas_valve_open_{false};
  uint8_t  electric_kw_{0};
  bool     panel_on_{false};
  bool     panel_busy_{false};
  bool     error_present_{false};
  uint8_t  water_mode_{0};
  bool     ac_available_{false};
  bool     pump_running_{false};
  bool     use_classic_checksum_{false};

  // ── Timing ───────────────────────────────────────────────────────────
  uint32_t last_send_ms_{0};
  static const uint32_t SEND_INTERVAL_MS = 500;

  // ── RX buffer ────────────────────────────────────────────────────────
  static const uint8_t RX_BUF_SIZE = 32;
  uint8_t  rx_buf_[RX_BUF_SIZE]{};
  uint8_t  rx_pos_{0};
  bool     rx_active_{false};
  uint32_t rx_start_ms_{0};
  static const uint32_t RX_TIMEOUT_MS = 150;
  uint32_t timeouts_{0};

  std::vector<std::function<void()>> on_state_callbacks_;
};

}  // namespace alde3030
}  // namespace esphome
