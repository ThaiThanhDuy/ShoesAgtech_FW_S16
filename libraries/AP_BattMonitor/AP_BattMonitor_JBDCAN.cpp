#include "AP_BattMonitor_JBDCAN.h"
#include <GCS_MAVLink/GCS.h>
#include <cstdio>
#include <cstring> // memset, memcpy

extern const AP_HAL::HAL &hal;

// Constructor: store pointer to state and params
AP_BattMonitor_JBDCAN::AP_BattMonitor_JBDCAN(
    AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state,
    AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params),
      _can_interface_index(0) // Default to CAN1 (index 0)
{}

// Initialize CAN interface (default CAN2 = hal.can[1])
void AP_BattMonitor_JBDCAN::init() {
  _iface = hal.can[1];
  if (_iface == nullptr) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "JBDCAN: CAN interface not available");
    return;
  }

  // Register CAN frame callback (CANManager already initialized bitrate)
  if (!_iface->register_frame_callback(
          FUNCTOR_BIND_MEMBER(&AP_BattMonitor_JBDCAN::handle_frame_callback,
                              void, uint8_t, const AP_HAL::CANFrame &,
                              AP_HAL::CANIface::CanIOFlags),
          _callback_id)) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                  "JBDCAN: Failed to register frame callback");
    return;
  }

  _last_update_us = AP_HAL::micros64();
  GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JBDCAN: Interface ready");
}

// Periodic reading of battery status
void AP_BattMonitor_JBDCAN::read() {
  if (_iface == nullptr)
    return;

  uint64_t now = AP_HAL::micros();
  if (now - _last_update_us < 200000)
    return;

  uint16_t id = 0x100;
  AP_HAL::CANFrame rtr_frame;
  rtr_frame.id = id | AP_HAL::CANFrame::FlagRTR;
  rtr_frame.dlc = 0;
  rtr_frame.setCanFD(false);

  int16_t ret = _iface->send(rtr_frame, now + 100000, 0);
  if (ret > 0) {
    _last_update_us = now;
  } else if (ret < 0) {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "JBDCAN: Failed to send RTR frame");
  }
}

// CAN frame callback handler
void AP_BattMonitor_JBDCAN::handle_frame_callback(
    uint8_t iface_num, const AP_HAL::CANFrame &frame,
    AP_HAL::CANIface::CanIOFlags flags) {
  // Process only frames with matching ID (response from JBD BMS)
  if ((frame.id & 0xFFFF) == 0x100) {
    handle_frame(frame);
  }
}

// Process received CAN frame from JBD BMS
// void AP_BattMonitor_JBDCAN::handle_frame(const AP_HAL::CANFrame &frame) {
//   const uint8_t *data = frame.data;

//   uint16_t raw_voltage = (data[0] << 8) | data[1];
//   float voltage = raw_voltage * 0.01f;

//   int16_t raw_current = (data[2] << 8) | data[3];
//   float current = raw_current * 0.01f;

//   uint16_t raw_capacity = (data[4] << 8) | data[5];
//   float remaining_mah = raw_capacity * 10.0f;

//   // Calculate consumed capacity if pack capacity is available
//   if (_params._pack_capacity > 0) {
//     _state.consumed_mah = _params._pack_capacity - remaining_mah;
//   }

//   _state.voltage = voltage;
//   _state.current_amps = current;
//   _state.healthy = true;
//   _last_update_us = AP_HAL::micros();

//   if (voltage > 0) {
//     _state.consumed_wh = _state.consumed_mah * voltage * 0.001f;
//   }
// }
// Duy điều chỉnh V 63V -> 58.8V cho pack
void AP_BattMonitor_JBDCAN::handle_frame(const AP_HAL::CANFrame &frame) {
  const uint8_t *data = frame.data;

  uint16_t raw_voltage = (data[0] << 8) | data[1];
  float voltage = raw_voltage * 0.01f;

  int16_t raw_current = (data[2] << 8) | data[3];
  float current = raw_current * 0.01f;

  // --- LOGIC FIX CHO PIN 14S ---
  const float V_MAX = 58.8f;
  const float V_MIN = 44.8f;

  // Tính tỷ lệ % dựa trên áp thực tế (0.0 đến 1.0)
  float pct = (voltage - V_MIN) / (V_MAX - V_MIN);
  if (pct > 1.0f)
    pct = 1.0f;
  if (pct < 0.0f)
    pct = 0.0f;

  // Ép Consumed mAh theo thực tế pin 14S
  if (_params._pack_capacity > 0) {
    float remaining_mah = pct * _params._pack_capacity;
    _state.consumed_mah = _params._pack_capacity - remaining_mah;
  }
  // -----------------------------

  _state.voltage = voltage;
  _state.current_amps = current;
  _state.healthy = true;
  _last_update_us = AP_HAL::micros();

  if (voltage > 0) {
    _state.consumed_wh = _state.consumed_mah * voltage * 0.001f;
  }
}
// Calculate remaining capacity percentage
bool AP_BattMonitor_JBDCAN::capacity_remaining_pct(uint8_t &percentage) const {
  if (_params._pack_capacity > 0) {
    percentage = 100 * (_params._pack_capacity - _state.consumed_mah) /
                 _params._pack_capacity;
  }
  return true;
}
