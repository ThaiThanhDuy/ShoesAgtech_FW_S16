#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_HAL/AP_HAL.h>

// Timeout for waiting response from JBD BMS (5 seconds)
#define AP_BATTMONITOR_JBDCAN_TIMEOUT_MICROS 5000000

class AP_BattMonitor_JBDCAN : public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_JBDCAN(AP_BattMonitor& mon,
                          AP_BattMonitor::BattMonitor_State& mon_state,
                          AP_BattMonitor_Params& params);

    // Initialize backend
    void init() override;
    // Read battery voltage and current. Should be called at 10Hz
    void read() override;
    // Return remaining capacity in percentage. Returns true if percentage is valid
    bool capacity_remaining_pct(uint8_t &percentage) const override;
    // Always reports current measurement capability
    bool has_current() const override { return true; }
    // Always reports consumed energy capability
    bool has_consumed_energy() const override { return true; }
private:
    // Pointer to the active CAN interface
    AP_HAL::CANIface* _iface = nullptr;
    uint8_t _callback_id;
    uint8_t _can_interface_index;
    // Timestamp of last update (us)
    uint64_t _last_update_us;
    // Timestamp of last successfully received & parsed frame (us) — used to
    // detect a stale/disconnected BMS in read(), independent of the RTR
    // send-throttle timer above
    uint64_t _last_frame_us = 0;
    // Frame callback handler
    void handle_frame_callback(uint8_t iface_num,
                               const AP_HAL::CANFrame &frame,
                               AP_HAL::CANIface::CanIOFlags flags);

    // Parse CAN frame data from JBD
    void handle_frame(const AP_HAL::CANFrame &frame);
    // Constants for JBD CAN message IDs and timing
    static const uint16_t JBD_REQUEST_ID  = 0x100;
    static const uint16_t JBD_RESPONSE_ID = 0x100;
    static const uint32_t UPDATE_INTERVAL_US = 200000;  // 200ms
    static const uint32_t TIMEOUT_US        = 100000;   // 100ms
};
