#include "AP_ShoesAgtech.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

volatile uint32_t AP_ShoesAgtech::_pulse_count = 0;

const AP_Param::GroupInfo AP_ShoesAgtech::var_info[] = {
    // ================================================================
    // MODULE 0 — Shared across modules
    // ================================================================

    // @Param: ENABLE
    // @DisplayName: Enable ShoesAgtech library
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_ShoesAgtech, _enable_flag, 1),

    // @Param: ROBOT_ID
    // @DisplayName: Robot identification number
    // @Description: Free-form identifier for this robot/unit. Not used by
    //   the library itself; for external tracking/logging only.
    // @Range: 0 999
    // @User: Standard
    AP_GROUPINFO("ROBOT_ID", 63, AP_ShoesAgtech, _robot_id, 0),

    // @Param: POND_IDX
    // @DisplayName: Active pond index (1-100)
    // @Description: Manually selects the active pond, shared by Module 2 (pH)
    //   and Module 3 (dosing). GPS only validates position; set 0 to disable
    //   pH.
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("POND_IDX", 59, AP_ShoesAgtech, _pond_select, 1),

    // @Param: SIM
    // @DisplayName: Simulation mode
    // @Description: When 1, replaces real sensor data with sinusoidal test
    //   values. Affects all modules (flow, pH, dosing).
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SIM", 32, AP_ShoesAgtech, _simulation, 0),

    // ================================================================
    // MODULE 1 — Flow sensor YF-S402B + spray control
    // ================================================================

    // @Param: CAL_FAC
    // @DisplayName: Flow sensor calibration factor (pulses/L)
    // @Description: Pulses per litre for YF-S402B. Operating range 0.3-6 L/min.
    // @User: Standard
    AP_GROUPINFO("CAL_FAC", 2, AP_ShoesAgtech, _cal_factor, 3874.5f),

    // @Param: EMA_AL
    // @DisplayName: Flow EMA smoothing alpha (0.01-1.0)
    // @Description: EMA alpha applied to raw flow rate. Lower = smoother.
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("EMA_AL", 3, AP_ShoesAgtech, _ema_alpha, 0.1f),

    // @Param: FLOW_LOG
    // @DisplayName: Flow console log enable
    // @Description: Prints spray mode, flow target/actual/avg and pump PWM at
    //   SA_LOG_FL_MS interval.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FLOW_LOG", 4, AP_ShoesAgtech, _flow_log_enable, 0),

    // @Param: RC_CHAN
    // @DisplayName: RC channel for spray mode switch (1-indexed)
    // @Description: PWM<1300=mode0, 1300-1700=mode1, >1700=mode2.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_CHAN", 5, AP_ShoesAgtech, _rc_chan, 6),

    // @Param: RC_PUMP
    // @DisplayName: RC channel for manual pump passthrough in mode 0
    // (1-indexed)
    // @Description: In mode 0, this channel is read and written directly to
    //   SA_PUMP_CHAN (software passthrough). Requires SERVOx_FUNCTION=0(None).
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_PUMP", 6, AP_ShoesAgtech, _rc_pump, 9),

    // @Param: PUMP_CHAN
    // @DisplayName: Pump servo output channel (1-indexed)
    // @Description: Requires SERVOx_FUNCTION=0(None). Mode 0: RC passthrough.
    //   Modes 1/2: PID-controlled.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("PUMP_CHAN", 7, AP_ShoesAgtech, _pump_chan, 8),

    // @Param: FLOW_SP
    // @DisplayName: Flow setpoint (L/min) for mode 1 FLOW_MODE=0
    // @Range: 0 200
    // @User: Standard
    AP_GROUPINFO("FLOW_SP", 8, AP_ShoesAgtech, _flow_setpoint, 5.0f),

    // @Param: PID_P
    // @DisplayName: Flow PID P gain (us per L/min error)
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("PID_P", 9, AP_ShoesAgtech, _pid_p, 80.0f),

    // @Param: PID_I
    // @DisplayName: Flow PID I gain (us per L/min/s)
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("PID_I", 10, AP_ShoesAgtech, _pid_i, 20.0f),

    // @Param: PID_LPF
    // @DisplayName: Flow PID output LPF alpha (0.01=smooth, 1.0=raw)
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("PID_LPF", 11, AP_ShoesAgtech, _pid_lpf, 0.3f),

    // @Param: LOG_FL_MS
    // @DisplayName: Flow console log interval (ms)
    // @Description: Interval between flow console prints when SA_FLOW_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("LOG_FL_MS", 22, AP_ShoesAgtech, _flow_log_ms, 1000),

    // @Param: FLOW_PIN
    // @DisplayName: Flow sensor GPIO pin number
    // @Description: GPIO pin connected to YF-S402B signal wire. Default 55
    //   (Pixhawk/CubeOrange AUX). Change to match hardware.
    // @Range: 1 200
    // @User: Standard
    AP_GROUPINFO("FLOW_PIN", 33, AP_ShoesAgtech, _flow_pin, 55),

    // @Param: TANK_VOL
    // @DisplayName: Chemical tank volume (L)
    // @Description: Used in FLOW_MODE=1 to compute flow target and in mode 2
    //   to estimate remaining spray distance. Set 0 to disable both.
    // @Units: L
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("TANK_VOL", 34, AP_ShoesAgtech, _tank_vol, 0.0f),

    // @Param: FLOW_MODE
    // @DisplayName: Flow target source in mode 1
    // @Description: 0=use SA_FLOW_SP directly. 1=compute from tank/mission:
    //   flow=(TANK_VOL*speed*60)/mission_dist, falls back to SA_FLOW_SP if
    //   unset.
    // @Values: 0:DirectSetpoint,1:TankMissionFormula
    // @User: Standard
    AP_GROUPINFO("FLOW_MODE", 35, AP_ShoesAgtech, _flow_mode, 0),

    // @Param: MIX_STD
    // @DisplayName: Spray ratio at mid RC position (standard nozzle)
    // @Description: Biocide fraction of total flow at mid RC. In FLOW_MODE=1
    //   used directly; in FLOW_MODE=0 the setpoint is SA_FLOW_SP.
    // @Range: 0.01 1.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("MIX_STD", 37, AP_ShoesAgtech, _mix_std, 0.35f),

    // @Param: MIX_CNT
    // @DisplayName: Spray ratio at high RC position (anti-clog nozzle)
    // @Description: Biocide fraction at high RC. In FLOW_MODE=0, flow target
    //   is scaled by MIX_CNT/MIX_STD to keep boom output consistent.
    // @Range: 0.01 1.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("MIX_CNT", 38, AP_ShoesAgtech, _mix_cnt, 0.50f),

    // @Param: FLOW_VEL
    // @DisplayName: Override ground speed for FLOW_MODE=1 (m/s)
    // @Description: 0=use real GPS/AHRS speed. >0=force this speed for flow
    //   target calibration while stationary. Ignored when SA_SIM=1.
    // @Range: 0 10
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("FLOW_VEL", 39, AP_ShoesAgtech, _flow_vel, 0.0f),

    // ================================================================
    // MODULE 2 — pH sensor + alkalinity + pond management
    // ================================================================

    // @Param: PH_EN
    // @DisplayName: Enable pH sensor (Nengshi ASPS3801D-0.5M)
    // @Description: Enables Modbus RTU pH sensor via RS485-TTL adapter.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_EN", 14, AP_ShoesAgtech, _ph_en, 0),

    // @Param: PH_PORT
    // @DisplayName: UART port for pH sensor (SERIALx number)
    // @Description: Set SERIALx_BAUD=9 (9600) and SERIALx_PROTOCOL=0 (None)
    //   on the matching port.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("PH_PORT", 15, AP_ShoesAgtech, _ph_port, 2),

    // @Param: PH_TOFF
    // @DisplayName: Temperature offset (°C)
    // @Description: Added to raw sensor temperature after /10 decode.
    // @Range: -10 10
    // @User: Standard
    AP_GROUPINFO("PH_TOFF", 16, AP_ShoesAgtech, _ph_toff, -3.5f),

    // @Param: PH_OFF
    // @DisplayName: pH calibration offset
    // @Description: Added to decoded pH value. Determine using a buffer
    // solution.
    // @Range: -2.0 2.0
    // @User: Standard
    AP_GROUPINFO("PH_OFF", 17, AP_ShoesAgtech, _ph_off, 0.0f),

    // @Param: PH_KH
    // @DisplayName: Base alkalinity from test kit (dKH)
    // @Description: Reference alkalinity measured with a test kit. Used as the
    //   base value for the delta-pH alkalinity estimate. Update periodically.
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("PH_KH", 18, AP_ShoesAgtech, _ph_kh, 4.0f),

    // @Param: PH_LOG
    // @DisplayName: pH console log enable (independent of SA_FLOW_LOG)
    // @Description: Prints pH, temperature, mV and alkalinity status at
    //   SA_PH_LOG_MS interval. Independent of SA_FLOW_LOG.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_LOG", 20, AP_ShoesAgtech, _ph_log_enable, 0),

    // @Param: PH_TZ
    // @DisplayName: Local timezone offset (hours, UTC+N)
    // @Description: Local time = UTC + PH_TZ. Used to classify readings into
    //   morning [PH_MS..PH_ME] or afternoon [PH_AS..PH_AE] slots.
    // @Range: -12 14
    // @User: Standard
    AP_GROUPINFO("PH_TZ", 21, AP_ShoesAgtech, _ph_tz, 7),

    // @Param: PH_LOG_MS
    // @DisplayName: pH console log interval (ms)
    // @Description: Interval between pH console prints when SA_PH_LOG=1.
    //   Keep >= 2000 to match the Modbus poll rate.
    // @Range: 500 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("PH_LOG_MS", 23, AP_ShoesAgtech, _ph_log_ms, 2000),

    // @Param: PH_TIMEOUT
    // @DisplayName: pH disconnect timeout (s)
    // @Description: Seconds without a valid frame before a warning is sent and
    //   pH fields in SA_DATA are zeroed.
    // @Range: 1 300
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("PH_TIMEOUT", 24, AP_ShoesAgtech, _ph_timeout, 2),

    // @Param: PH_MS
    // @DisplayName: Morning slot start time (fractional hours)
    // @Description: pH readings in [PH_MS, PH_ME] update the morning slot.
    //   Use decimal: 5.5 = 05:30.
    // @Range: 0 23.99
    // @User: Standard
    AP_GROUPINFO("PH_MS", 55, AP_ShoesAgtech, _ph_ms, 5.0f),

    // @Param: PH_ME
    // @DisplayName: Morning slot end time (fractional hours, inclusive)
    // @Description: Readings at exactly this time are still captured (<=).
    //   Set 24.0 to extend to end of day.
    // @Range: 0 24
    // @User: Standard
    AP_GROUPINFO("PH_ME", 56, AP_ShoesAgtech, _ph_me, 11.0f),

    // @Param: PH_AS
    // @DisplayName: Afternoon slot start time (fractional hours)
    // @Description: pH readings in [PH_AS, PH_AE] update the afternoon slot.
    //   Use decimal: 13.5 = 13:30.
    // @Range: 0 23.99
    // @User: Standard
    AP_GROUPINFO("PH_AS", 57, AP_ShoesAgtech, _ph_as, 12.0f),

    // @Param: PH_AE
    // @DisplayName: Afternoon slot end time (fractional hours, inclusive)
    // @Description: Readings at exactly this time are still captured (<=).
    //   Set 24.0 to extend to end of day.
    // @Range: 0 24
    // @User: Standard
    AP_GROUPINFO("PH_AE", 58, AP_ShoesAgtech, _ph_ae, 16.0f),

    // @Param: PH_POND_D
    // @DisplayName: Same-pond GPS distance threshold (m)
    // @Description: Maximum distance between the robot and the pond centroid
    //   to still consider it the same pond. Increase for large ponds.
    // @Range: 10 5000
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("PH_POND_D", 60, AP_ShoesAgtech, _ph_pond_dist, 300.0f),

    // @Param: PH_CAP_S
    // @DisplayName: pH sample interval within a slot (s)
    // @Description: Minimum time between samples in a slot. Samples are
    //   last-write-wins until SA_PH_CAP_SAM samples are reached.
    // @Range: 1 3600
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("PH_CAP_S", 61, AP_ShoesAgtech, _ph_cap_s, 20),

    // @Param: PH_CAP_SAM
    // @DisplayName: pH samples required to complete a slot
    // @Description: Number of samples before the slot is reported as complete.
    //   The slot value used is the last sample written (last-write-wins).
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("PH_CAP_SAM", 62, AP_ShoesAgtech, _ph_cap_sam, 20),

    // slot 12 reused (previously retired APP_RATE, unused since before this
    // library's current history — no field-flashed unit ever stored a value
    // there).
    // @Param: PH_CAP_M
    // @DisplayName: pH capture radius (m)
    // @Description: Capture radius in metres, used by the companion app
    //   only. Firmware initialises this value but does not read it anywhere.
    // @Range: 1 100
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("PH_CAP_M", 12, AP_ShoesAgtech, _ph_cap_m, 1),

    // ================================================================
    // MODULE 3 — Dosing motor: continuous-rotation 360° servo
    // ================================================================

    // @Param: DOS_CHAN
    // @DisplayName: Dosing motor servo output channel (1-indexed)
    // @Description: Requires SERVOx_FUNCTION=0(None), MIN=800, TRIM=1500,
    //   MAX=2200 before the motor is allowed to run.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_CHAN", 25, AP_ShoesAgtech, _dos_chan, 10),

    // @Param: DOS_RC
    // @DisplayName: RC channel to toggle dosing motor on/off (1-indexed)
    // @Description: PWM>1500 runs motor; PWM<=1500 stops (outputs 1500).
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_RC", 26, AP_ShoesAgtech, _dos_rc, 8),

    // slot 27 (DOS_RATE) retired — DOS_MODE=0 now uses DOS_Fx/DOS_Dx (per
    // food type) instead of a single global rate. Do not reuse this index.

    // @Param: DOS_SP
    // @DisplayName: Dosing setpoint (grams per run)
    // @Description: Target food weight per run for the active pond (POND_IDX).
    //   Each pond keeps its own value: switching ponds loads that pond's
    //   stored setpoint here; editing this saves back to the active pond.
    // @Units: g
    // @User: Standard
    AP_GROUPINFO("DOS_SP", 28, AP_ShoesAgtech, _dos_sp, 0.0f),

    // @Param: DOS_REV
    // @DisplayName: Dosing motor rotation direction
    // @Description: 0=forward: PWM 800-1500 (800=fastest). 1=reverse: PWM
    //   1500-2200. Motor stops at 1500 in both directions.
    // @Values: 0:Forward (800-1500),1:Reverse (1500-2200)
    // @User: Standard
    AP_GROUPINFO("DOS_REV", 29, AP_ShoesAgtech, _dos_rev, 0),

    // @Param: DOS_LOG
    // @DisplayName: Dosing motor console log enable
    // @Description: Prints motor state, setpoint and PWM at SA_DOS_LOG_MS
    //   interval when enabled.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DOS_LOG", 30, AP_ShoesAgtech, _dos_log_enable, 0),

    // @Param: DOS_LOG_MS
    // @DisplayName: Dosing motor console log interval (ms)
    // @Description: Interval between dosing console prints when SA_DOS_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("DOS_LOG_MS", 31, AP_ShoesAgtech, _dos_log_ms, 1000),

    // @Param: DOS_MODE
    // @DisplayName: Dosing motor speed mode
    // @Description: 0=fixed speed from DOS_SP/DOS_Fx/DOS_Dx. 1=spreads DOS_SP
    //   evenly over the mission route by ground speed. Stops if no mission
    //   or speed<0.05m/s.
    // @Values: 0:Fixed,1:MissionProportional
    // @User: Standard
    AP_GROUPINFO("DOS_MODE", 36, AP_ShoesAgtech, _dos_mode, 0),

    // @Param: DOS_FOOD
    // @DisplayName: Active food type selector (1-7) for the active pond
    // @Description: Selects SA_DOS_Fx (volume rate) and SA_DOS_Dx (bulk
    //   density) used to compute the PWM offset. Each pond keeps its own
    //   value: switching ponds (POND_IDX) loads that pond's stored food
    //   type here; editing this saves back to the active pond.
    // @Range: 1 7
    // @User: Standard
    AP_GROUPINFO("DOS_FOOD", 40, AP_ShoesAgtech, _dos_food, 1),

    // @Param: DOS_F1
    // @DisplayName: Auger volume rate for food type 1 (mL/50us)
    // @Description: Used with SA_DOS_D1 to compute PWM offset in both
    //   DOS_MODE=0 and DOS_MODE=1. Calibrate per food type.
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F1", 41, AP_ShoesAgtech, _dos_fr[0], 100.0f),

    // @Param: DOS_F2
    // @DisplayName: Auger volume rate for food type 2 (mL/50us)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F2", 42, AP_ShoesAgtech, _dos_fr[1], 100.0f),

    // @Param: DOS_F3
    // @DisplayName: Auger volume rate for food type 3 (mL/50us)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F3", 43, AP_ShoesAgtech, _dos_fr[2], 100.0f),

    // @Param: DOS_F4
    // @DisplayName: Auger volume rate for food type 4 (mL/50us)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F4", 44, AP_ShoesAgtech, _dos_fr[3], 100.0f),

    // @Param: DOS_F5
    // @DisplayName: Auger volume rate for food type 5 (mL/50us)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F5", 45, AP_ShoesAgtech, _dos_fr[4], 100.0f),

    // @Param: DOS_F6
    // @DisplayName: Auger volume rate for food type 6 (mL/50us)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F6", 46, AP_ShoesAgtech, _dos_fr[5], 100.0f),

    // @Param: DOS_F7
    // @DisplayName: Auger volume rate for food type 7 (mL/50us)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F7", 47, AP_ShoesAgtech, _dos_fr[6], 100.0f),

    // Bulk density per food type (slots 48-54).
    // offset(us) = SP(g) * 50 / (vol_rate(mL/50us) x density(g/mL)).
    // Default 1.0 g/mL is backward-compatible with the old g/50us formula.

    // @Param: DOS_D1
    // @DisplayName: Bulk density food type 1 (g/mL)
    // @Description: Bulk density of food type 1. Measure: weigh 1L of food,
    //   divide by 1000 to get g/mL.
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D1", 48, AP_ShoesAgtech, _dos_dr[0], 1.0f),

    // @Param: DOS_D2
    // @DisplayName: Bulk density food type 2 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D2", 49, AP_ShoesAgtech, _dos_dr[1], 1.0f),

    // @Param: DOS_D3
    // @DisplayName: Bulk density food type 3 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D3", 50, AP_ShoesAgtech, _dos_dr[2], 1.0f),

    // @Param: DOS_D4
    // @DisplayName: Bulk density food type 4 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D4", 51, AP_ShoesAgtech, _dos_dr[3], 1.0f),

    // @Param: DOS_D5
    // @DisplayName: Bulk density food type 5 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D5", 52, AP_ShoesAgtech, _dos_dr[4], 1.0f),

    // @Param: DOS_D6
    // @DisplayName: Bulk density food type 6 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D6", 53, AP_ShoesAgtech, _dos_dr[5], 1.0f),

    // @Param: DOS_D7
    // @DisplayName: Bulk density food type 7 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D7", 54, AP_ShoesAgtech, _dos_dr[6], 1.0f),

    AP_GROUPEND};

// =============================================================
// HÀM KHỞI TẠO (CONSTRUCTOR)
// =============================================================
AP_ShoesAgtech::AP_ShoesAgtech()
    : _last_timestamp_ms(0), _last_pulse_snapshot(0), _last_log_ms(0),
      _flow_rate_filtered(0.0f), _flow_rate_avg(0.0f), _is_initialized(false),
      _buffer_index(0), _buffer_sum(0.0f), _samples_count(0), _spray_mode(0),
      _pump_pwm(0), _flow_target(0.0f), _pid_integral(0.0f),
      _pid_output_lpf(0.0f), _pid_last_ms(0), _last_pump_chan(-1),
      _last_pump_func_val(-1), _pump_config_ok(false), _last_warn_ms(0),
      _mission_dist_m(0.0f), _mission_ncmds(0), _tank_warn_ms(0),
      _arm_dist_warned(false), _was_armed(false), _tank_empty_detected(false),
      _tank_empty_ms(0), _sim_speed(0.0f), _ph_uart(nullptr), _ph_update_ms(0),
      _ph_req_sent_ms(0), _ph_req_pending(false), _ph_last_good_ms(0),
      _ph_nodata_warn_ms(0), _ph_last_log_ms(0), _ph_value(0.0f),
      _ph_value_ma(0.0f), _ph_mv(0), _ph_temp(25.0f), _ph_buf_idx(0),
      _ph_buf_count(0), _ph_buf_sum(0.0f), _pond_count(0), _ph_morn_val(0.0f),
      _ph_morn_lat(0), _ph_morn_lng(0), _ph_aft_val(0.0f), _delta_ph(0.0f),
      _alk_dkh(0.0f), _alk_mgl(0.0f), _alk_slot_status(4), _alk_pond_idx(0),
      _active_pond_idx(0), _slot_warn_ms(0), _ponds_save_ms(0),
      _pond_first_detect_done(false), _ponds_dirty(false), _ponds_loaded(false),
      _dos_pwm(1500), _dos_config_ok(false), _dos_warn_ms(0),
      _dos_was_ok(false), _dos_was_on(false), _dos_last_log_ms(0),
      _dos_sync_pond(0xFF), _dos_sp_sync_val(0.0f), _dos_food_sync_val(0) {
  memset(_sample_buffer, 0, sizeof(_sample_buffer));
  memset(_ph_buf, 0, sizeof(_ph_buf));
  memset(_ponds, 0, sizeof(_ponds));
  AP_Param::setup_object_defaults(this, var_info);
}

// =============================================================
// KHỞI ĐỘNG (INIT)
// =============================================================
void AP_ShoesAgtech::init(void) {
  if (!is_enabled()) {
    return;
  }

  uint8_t flow_pin = (uint8_t)constrain_int16(_flow_pin.get(), 1, 200);
  hal.gpio->pinMode(flow_pin, HAL_GPIO_INPUT);

  if (!hal.gpio->attach_interrupt(flow_pin, irq_handler,
                                  AP_HAL::GPIO::INTERRUPT_RISING)) {
    gcs().send_text(MAV_SEVERITY_CRITICAL, "ShoesAgtech: IRQ attach failed");
  } else {
    gcs().send_text(MAV_SEVERITY_INFO, "ShoesAgtech: Flow sensor ready");
  }

  _flow_rate_avg = 0.0f;
  _buffer_index = 0;
  _buffer_sum = 0.0f;
  _samples_count = 0;
  _pid_integral = 0.0f;
  _pid_output_lpf = 0.0f;
  _pid_last_ms = 0;
  memset(_sample_buffer, 0, sizeof(_sample_buffer));

  _ph_init();
  // Đăng ký tiến trình IO thread để đọc/ghi file (load/save dữ liệu ao).
  // AP::FS() phải gọi từ IO thread, không được gọi từ scheduler task.
  hal.scheduler->register_io_process(
      FUNCTOR_BIND_MEMBER(&AP_ShoesAgtech::_io_update, void));
}

void AP_ShoesAgtech::irq_handler(void) { _pulse_count++; }

// =============================================================
// VÒNG LẶP CHÍNH — gọi ở tần số 10Hz từ scheduler
// Cảm biến lưu lượng: YF-S402B, dãy hoạt động 0.3–6 L/min, GPIO pin 55
// =============================================================
void AP_ShoesAgtech::update(void) {
  if (!is_enabled()) {
    _flow_rate_filtered = 0.0f;
    _flow_rate_avg = 0.0f;
    return;
  }

  _check_pump_config();

  if (_simulation.get() > 0) {
    _run_simulation();
  } else {
    _ph_update();
  }

  _update_dosing_motor();

  uint32_t now = AP_HAL::millis();
  uint32_t delta_t_ms = now - _last_timestamp_ms;

  if (!_is_initialized) {
    _last_timestamp_ms = now;
    _last_pulse_snapshot = _pulse_count;
    _pid_last_ms = now;
    _is_initialized = true;
    return;
  }

  // ---- 1. TÍNH LƯU LƯỢNG (mỗi 100ms) ----
  if (delta_t_ms >= 100) {
    _last_timestamp_ms = now;

    if (_simulation.get() > 0) {
      _buffer_sum -= _sample_buffer[_buffer_index];
      _sample_buffer[_buffer_index] = _flow_rate_filtered;
      _buffer_sum += _flow_rate_filtered;
      _buffer_index = (_buffer_index + 1) % WINDOW_SIZE;
      if (_samples_count < WINDOW_SIZE) {
        _samples_count++;
      }
      if (_samples_count > 0) {
        _flow_rate_avg = _buffer_sum / _samples_count;
      }
    } else {
      uint32_t snap = _pulse_count;
      uint32_t pulses = (snap >= _last_pulse_snapshot)
                            ? (snap - _last_pulse_snapshot)
                            : (UINT32_MAX - _last_pulse_snapshot) + snap + 1;
      _last_pulse_snapshot = snap;

      float dt = delta_t_ms * 0.001f;
      float cal = (_cal_factor.get() > 0.0f) ? _cal_factor.get() : 3874.5f;
      float raw = (dt > 0.0f) ? ((float)pulses / cal) * (60.0f / dt) : 0.0f;

      float alpha = constrain_float(_ema_alpha.get(), 0.01f, 1.0f);
      _flow_rate_filtered = _flow_rate_filtered * (1.0f - alpha) + raw * alpha;

      _buffer_sum -= _sample_buffer[_buffer_index];
      _sample_buffer[_buffer_index] = _flow_rate_filtered;
      _buffer_sum += _flow_rate_filtered;
      _buffer_index = (_buffer_index + 1) % WINDOW_SIZE;
      if (_samples_count < WINDOW_SIZE) {
        _samples_count++;
      }
      if (_samples_count > 0) {
        _flow_rate_avg = _buffer_sum / _samples_count;
      }
    }
  }

  // ---- 2. ĐIỀU KHIỂN PHUN ----
  float dt_pid = (now - _pid_last_ms) * 0.001f;
  if (dt_pid <= 0.0f || dt_pid > 1.0f) {
    dt_pid = 0.1f;
  }
  _pid_last_ms = now;

  _update_spray_mode();

  bool now_armed = hal.util->get_soft_armed();

  // Khi vừa ARM: in trạng thái FM1 một lần, bất kể SA_FLOW_LOG
  if (now_armed && !_was_armed) {
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f &&
        (_spray_mode == 1 || _spray_mode == 2)) {
      float r = (_spray_mode == 2) ? _mix_cnt.get() : _mix_std.get();
      _print_fm1_arm_status(r);
    }
  }

  // Khi disarm: reset cảnh báo + cache mission + bộ phát hiện hết thùng
  if (!now_armed) {
    _arm_dist_warned = false;
    _mission_ncmds = 0;
    _mission_dist_m = 0.0f;
    _tank_empty_detected = false;
    _tank_empty_ms = 0;
  }
  _was_armed = now_armed;

  switch (_spray_mode) {

  case 0: {
    // ---- MODE 0: TRUYỀN THẲNG PHẦN MỀM ----
    uint8_t rc_pump_idx = (uint8_t)constrain_int16(_rc_pump.get() - 1, 0, 15);
    uint16_t rc_pwm = RC_Channels::get_radio_in(rc_pump_idx);
    if (rc_pwm < 800 || rc_pwm > 2200) {
      rc_pwm = 1500;
    }
    _flow_target = 0.0f;
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    _pump_pwm = rc_pwm;
    _write_pump_pwm(_pump_pwm);
    break;
  }

  case 1: {
    // ---- MODE 1: FLOW PID (nấc giữa — MIX_STD / béc mặc định) ----
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch1 =
          SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
      if (ch1 != nullptr) {
        _write_pump_pwm(ch1->get_output_min());
      }
      break;
    }
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f) {
      _flow_target = _compute_visin_target(_mix_std.get());
      if (_flow_target < 0.01f) {
        SRV_Channel *ch1 =
            SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
        if (ch1 != nullptr) {
          _write_pump_pwm(ch1->get_output_min());
        }
        break;
      }
    } else {
      _flow_target = _flow_setpoint.get();
    }
    _pump_pwm = _run_flow_pid(_flow_target, dt_pid);
    _write_pump_pwm(_pump_pwm);
    break;
  }

  case 2: {
    // ---- MODE 2: FLOW PID (nấc cao — MIX_CNT / béc chống nghẹt) ----
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch2 =
          SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
      if (ch2 != nullptr) {
        _write_pump_pwm(ch2->get_output_min());
      }
      break;
    }
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f) {
      _flow_target = _compute_visin_target(_mix_cnt.get());
      if (_flow_target < 0.01f) {
        SRV_Channel *ch2 =
            SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
        if (ch2 != nullptr) {
          _write_pump_pwm(ch2->get_output_min());
        }
        break;
      }
    } else {
      float ratio =
          (_mix_std.get() > 0.01f) ? (_mix_cnt.get() / _mix_std.get()) : 1.0f;
      _flow_target =
          constrain_float(_flow_setpoint.get() * ratio, 0.0f, 200.0f);
      if (_tank_vol.get() > 0.0f && _flow_target > 0.01f) {
        float speed_ms2 = _get_spray_speed();
        if (speed_ms2 > 0.01f && now - _tank_warn_ms >= 30000U) {
          _tank_warn_ms = now;
          float dist_m = (_tank_vol.get() / _flow_target) * speed_ms2 * 60.0f;
          gcs().send_text(
              MAV_SEVERITY_INFO, "SA: Tank đủ ~%.0fm (%.1fL @%.1fL/min)",
              (double)dist_m, (double)_tank_vol.get(), (double)_flow_target);
        }
      }
    }
    _pump_pwm = _run_flow_pid(_flow_target, dt_pid);
    _write_pump_pwm(_pump_pwm);
    break;
  }

  default:
    break;
  }

  // ---- 3. PHÁT HIỆN THÙNG HẾT VI SINH ----
  if ((_spray_mode == 1 || _spray_mode == 2) && hal.util->get_soft_armed()) {
    if (!_tank_empty_detected) {
      if (_flow_rate_filtered > 1.7f) {
        if (_tank_empty_ms == 0) {
          _tank_empty_ms = now;
        } else if (now - _tank_empty_ms >= 3000U) {
          _tank_empty_detected = true;
          gcs().send_text(
              MAV_SEVERITY_CRITICAL,
              "SA: THÙNG HẾT VI SINH - flow %.1fL/ph > 1.7 trong 3s",
              (double)_flow_rate_filtered);
        }
      } else {
        _tank_empty_ms = 0;
      }
    }
  }

  // ---- 5. IN LOG LƯU LƯỢNG RA CONSOLE (SA_LOG_EN) ----
  if (_flow_log_enable.get() > 0 &&
      now - _last_log_ms >= (uint32_t)_flow_log_ms.get()) {
    _last_log_ms = now;
    const char *flow_pfx = (_simulation.get() > 0) ? "[SIM][FLOW]" : "[FLOW]";
    gcs().send_text(MAV_SEVERITY_INFO,
                    "%s M%u Tgt:%.1f Act:%.1f Avg:%.1f PWM:%u", flow_pfx,
                    (unsigned)_spray_mode, (double)_flow_target,
                    (double)_flow_rate_filtered, (double)_flow_rate_avg,
                    (unsigned)_pump_pwm);

    // ---- DEBUG: chẩn đoán phần cứng cảm biến flow ----
    // In số pin GPIO đang dùng, mức tín hiệu tức thời (0/1), và số xung
    // cộng dồn (_pulse_count) — không đổi giữa các lần in nghĩa là ISR
    // chưa từng bắt được xung nào (sai wiring/mode chân, không phải lỗi
    // tính toán lưu lượng).
    {
      uint8_t dbg_pin = (uint8_t)constrain_int16(_flow_pin.get(), 1, 200);
      uint8_t dbg_level = hal.gpio->read(dbg_pin);
      gcs().send_text(MAV_SEVERITY_INFO, "%s DBG Pin:%u Lvl:%u Pulses:%u",
                      flow_pfx, (unsigned)dbg_pin, (unsigned)dbg_level,
                      (unsigned)_pulse_count);
    }

    if ((_spray_mode == 1 || _spray_mode == 2) && _flow_mode.get() == 1 &&
        _tank_vol.get() > 0.0f) {
      float r = (_spray_mode == 2) ? _mix_cnt.get() : _mix_std.get();
      float mdist = _get_mission_dist();
      float spd = _get_spray_speed();
      float vi_per_run = _tank_vol.get() * r;
      float q1_now = (mdist > 1.0f && spd > 0.1f)
                         ? (_tank_vol.get() * r * spd * 60.0f / mdist)
                         : 0.0f;
      float dmax =
          (spd > 0.1f) ? (_tank_vol.get() * r * spd * 60.0f / 0.3f) : 0.0f;
      gcs().send_text(MAV_SEVERITY_INFO,
                      "%s FM1 r:%.2f q1:%.2fL/ph miss:%.0fm dmax:%.0fm "
                      "spd:%.2fm/s vi/run:%.1fL",
                      flow_pfx, (double)r, (double)q1_now, (double)mdist,
                      (double)dmax, (double)spd, (double)vi_per_run);
    }
  }
}

// =============================================================
// MODULE 1 — CẢM BIẾN LƯU LƯỢNG + ĐIỀU KHIỂN PHUN
// =============================================================

// =============================================================
// KIỂM TRA CẤU HÌNH BƠM
// Chạy trong mỗi update(). In MIN/TRIM/MAX của servo ở lần boot
// đầu hoặc khi PUMP_CHAN đổi. Cảnh báo mỗi 5s nếu function != 0.
// =============================================================
void AP_ShoesAgtech::_check_pump_config(void) {
  int8_t chan = _pump_chan.get();
  int32_t func_val = (int32_t)SRV_Channels::channel_function(
      (uint8_t)constrain_int16(chan - 1, 0, 15));

  bool changed = (chan != _last_pump_chan) || (func_val != _last_pump_func_val);

  if (!changed && _pump_config_ok) {
    return;
  }

  if (changed) {
    _last_pump_chan = chan;
    _last_pump_func_val = func_val;
  }

  if (func_val != (int32_t)SRV_Channel::k_none) {
    uint32_t now = AP_HAL::millis();
    if (changed || now - _last_warn_ms >= 5000) {
      _last_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA: SERVO%d_FUNCTION=%d must be 0(None)!", (int)chan,
                      (int)func_val);
    }
    _pump_config_ok = false;
  } else {
    SRV_Channel *ch = SRV_Channels::srv_channel((uint8_t)(chan - 1));
    if (ch != nullptr) {
      gcs().send_text(MAV_SEVERITY_INFO, "SA: SERVO%d OK Min:%u Trim:%u Max:%u",
                      (int)chan, (unsigned)ch->get_output_min(),
                      (unsigned)ch->get_trim(), (unsigned)ch->get_output_max());
    }
    _pump_config_ok = true;
  }
}

// RC → CHẾ ĐỘ PHUN
//   Nấc 1 (PWM < 1300) : mode 0 — passthrough
//   Nấc 2 (1300-1700)  : mode 1 — FLOW PID, tỉ lệ SA_MIX_STD
//   Nấc 3 (PWM > 1700) : mode 2 — FLOW PID, tỉ lệ SA_MIX_CNT
void AP_ShoesAgtech::_update_spray_mode(void) {
  uint8_t idx = (uint8_t)constrain_int16(_rc_chan.get() - 1, 0, 15);
  uint16_t pwm_in = RC_Channels::get_radio_in(idx);

  if (pwm_in == 0) {
    return;
  }

  if (pwm_in < 1300) {
    _spray_mode = 0;
  } else if (pwm_in < 1700) {
    _spray_mode = 1;
  } else {
    _spray_mode = 2;
  }
}

// =============================================================
// BỘ ĐIỀU KHIỂN PI + LỌC LPF ĐẦU RA
// PWM gốc = servo TRIM (tham số SERVOx_TRIM)
// Dải giá trị = servo MIN/MAX (tham số SERVOx_MIN/MAX)
// =============================================================
uint16_t AP_ShoesAgtech::_run_flow_pid(float target_lmin, float dt) {
  SRV_Channel *ch = SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
  uint16_t pwm_min = (ch != nullptr) ? ch->get_output_min() : 1000;
  uint16_t pwm_max = (ch != nullptr) ? ch->get_output_max() : 2000;
  uint16_t pwm_trim = (ch != nullptr) ? ch->get_trim() : 1500;

  float error = target_lmin - _flow_rate_filtered;
  float i_gain = _pid_i.get();

  _pid_integral += error * dt;
  if (i_gain > 0.0f) {
    float ilimit = (pwm_max - pwm_min) * 0.5f / i_gain;
    _pid_integral = constrain_float(_pid_integral, -ilimit, ilimit);
  }

  float pid_raw = _pid_p.get() * error + i_gain * _pid_integral;

  float alpha = constrain_float(_pid_lpf.get(), 0.01f, 1.0f);
  _pid_output_lpf = _pid_output_lpf * (1.0f - alpha) + pid_raw * alpha;

  return (uint16_t)constrain_float((float)pwm_trim + _pid_output_lpf,
                                   (float)pwm_min, (float)pwm_max);
}

// =============================================================
// GHI KÊNH BƠM
// Gọi set_output_pwm_chan() để bật have_pwm_mask, nhờ đó
// calc_pwm() sẽ KHÔNG ghi đè giá trị này ở các vòng lặp sau.
// Yêu cầu SERVOx_FUNCTION = 0 (None) trên kênh bơm.
// =============================================================
void AP_ShoesAgtech::_write_pump_pwm(uint16_t pwm) {
  uint8_t chan_idx = (uint8_t)constrain_int16(_pump_chan.get() - 1, 0, 15);
  SRV_Channels::set_output_pwm_chan(chan_idx, pwm);
}

// =============================================================
// TỐC ĐỘ PHUN — ưu tiên: SA_SIM > SA_FLOW_VEL > AHRS
// SA_SIM=1       : dùng _sim_speed (sóng sin, để test màn hình)
// SA_FLOW_VEL>0  : dùng giá trị cố định (hiệu chỉnh FLOW_MODE=1 khi xe đứng
// yên) Mặc định (=0)  : dùng vận tốc thật từ AP::ahrs().groundspeed()
// =============================================================
float AP_ShoesAgtech::_get_spray_speed(void) {
  if (_simulation.get() > 0) {
    return _sim_speed;
  }
  float vel = _flow_vel.get();
  if (vel > 0.0f) {
    return vel;
  }
  return AP::ahrs().groundspeed();
}

// =============================================================
// MỤC TIÊU VI SINH — FLOW_MODE=1 (nấc giữa/cao)
//
// Công thức: q1 = TANK_VOL * r * speed * 60 / mission_dist
//   → vi sinh được phân phối đều trên toàn tuyến.
//   → vi_per_run = TANK_VOL * r (hằng số, không đổi theo speed/dist)
// Kiểm tra trước khi chạy PID:
//   1. Mission: dist <= 1m → cảnh báo + dừng
//   2. Speed:   < 0.1 m/s → reset PID, dừng
//   3. Range:   q1 < 0.3 hoặc q1 > 2.0 → cảnh báo + dừng
// =============================================================
float AP_ShoesAgtech::_compute_visin_target(float r) {
  uint32_t now = AP_HAL::millis();
  r = constrain_float(r, 0.01f, 1.0f);

  float speed_ms = _get_spray_speed();
  float dist = _get_mission_dist();

  if (dist <= 1.0f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA FM1: chưa có mission - bơm dừng");
    }
    return 0.0f;
  }

  if (speed_ms < 0.1f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    return 0.0f;
  }

  float q1 = _tank_vol.get() * r * speed_ms * 60.0f / dist;

  if (q1 < 0.3f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(
          MAV_SEVERITY_WARNING,
          "SA FM1: q1=%.2fL/ph < 0.3 - rút ngắn mission hoặc tăng speed",
          (double)q1);
    }
    return 0.0f;
  }
  if (q1 > 2.0f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(
          MAV_SEVERITY_WARNING,
          "SA FM1: q1=%.2fL/ph > 2.0 - kéo dài mission hoặc giảm speed",
          (double)q1);
    }
    return 0.0f;
  }

  return constrain_float(q1, 0.0f, 200.0f);
}

// =============================================================
// _print_fm1_arm_status — in trạng thái FLOW_MODE=1 khi ARM (bất kể FLOW_LOG).
// Gọi một lần mỗi phiên ARM khi spray_mode = 1 hoặc 2.
// =============================================================
void AP_ShoesAgtech::_print_fm1_arm_status(float r) {
  r = constrain_float(r, 0.01f, 1.0f);
  float dist = _get_mission_dist();

  if (dist <= 1.0f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: chưa có mission - bơm sẽ dừng");
    return;
  }

  float vi_per_run = _tank_vol.get() * r;

  float speed = _get_spray_speed();
  if (speed <= 0.1f) {
    gcs().send_text(MAV_SEVERITY_INFO,
                    "SA FM1 SẴN SÀNG: r=%.2f miss=%.0fm vi/run=%.1fL | vận "
                    "tốc=0 bơm chờ xe chạy",
                    (double)r, (double)dist, (double)vi_per_run);
    return;
  }

  float q1 = _tank_vol.get() * r * speed * 60.0f / dist;
  float dist_max = _tank_vol.get() * r * speed * 60.0f / 0.3f;

  if (q1 < 0.3f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: q1=%.2fL/ph < 0.3 @%.1fm/s dist=%.0fm - rút ngắn "
                    "mission (dmax=%.0fm)",
                    (double)q1, (double)speed, (double)dist, (double)dist_max);
    return;
  }
  if (q1 > 2.0f) {
    float dist_min = _tank_vol.get() * r * speed * 60.0f / 2.0f;
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: q1=%.2fL/ph > 2.0 @%.1fm/s dist=%.0fm - kéo dài "
                    "mission (dmin=%.0fm)",
                    (double)q1, (double)speed, (double)dist, (double)dist_min);
    return;
  }

  uint32_t eta_s = (uint32_t)(dist / speed);
  uint32_t eta_min = eta_s / 60U;
  uint32_t eta_sec = eta_s % 60U;

  gcs().send_text(MAV_SEVERITY_INFO,
                  "SA FM1 OK: r=%.2f q1=%.2fL/ph miss=%.0fm dmax=%.0fm "
                  "~%um%02us vi/run=%.1fL",
                  (double)r, (double)q1, (double)dist, (double)dist_max,
                  (unsigned)eta_min, (unsigned)eta_sec, (double)vi_per_run);
}

// =============================================================
// KHOẢNG CÁCH MISSION — SA_FLOW_MODE = 1 (mode 1) + theo dõi mode 2
// Duyệt các waypoint AP_Mission và cộng dồn khoảng cách từng chặng.
// Kết quả được cache đến khi num_commands() thay đổi (mission bị
// sửa hoặc upload lại). Trả về 0 nếu chưa có mission.
// =============================================================
float AP_ShoesAgtech::_get_mission_dist(void) {
  AP_Mission *mission = AP::mission();
  if (mission == nullptr) {
    return 0.0f;
  }

  uint16_t n = mission->num_commands();
  if (n < 2) {
    return 0.0f;
  }

  if (n == _mission_ncmds && _mission_dist_m > 0.0f) {
    return _mission_dist_m;
  }

  float total = 0.0f;
  Location prev_loc;
  bool have_prev = false;

  for (uint16_t i = 0; i < n; i++) {
    AP_Mission::Mission_Command cmd;
    if (!mission->read_cmd_from_storage(i, cmd)) {
      continue;
    }
    if (cmd.id != MAV_CMD_NAV_WAYPOINT && cmd.id != MAV_CMD_NAV_LOITER_UNLIM &&
        cmd.id != MAV_CMD_NAV_LOITER_TURNS &&
        cmd.id != MAV_CMD_NAV_LOITER_TIME &&
        cmd.id != MAV_CMD_NAV_SPLINE_WAYPOINT) {
      continue;
    }
    Location loc = cmd.content.location;
    if (loc.lat == 0 && loc.lng == 0) {
      continue;
    }
    if (have_prev) {
      total += prev_loc.get_distance(loc);
    }
    prev_loc = loc;
    have_prev = true;
  }

  _mission_dist_m = total;
  _mission_ncmds = n;
  return _mission_dist_m;
}

// =============================================================
// MODULE 2 — CẢM BIẾN pH + ĐỘ KIỀM + QUẢN LÝ AO
// =============================================================

// Giao thức: Modbus RTU, 9600 8N1, Function Code 04
// Phần cứng: Cảm biến RS485 A/B → module RS485-TTL → cổng TELEM RX/TX
// Yêu cầu:  [01][04][00 00][00 09][30 0C] (9 thanh ghi từ 0x0000)
// Phản hồi: [01][04][12][Reg0..Reg8 × 2B][CRC × 2B] = 23 byte
//
// Bảng thanh ghi (chỉ số 0 trong buffer phản hồi, dữ liệu tại buf[3]+):
//   0x0000 (buf[3..4])   pH × 100          không dấu
//   0x0002 (buf[7..8])   điện thế mV       có dấu, 16-bit
//   0x0008 (buf[19..20]) nhiệt độ × 10     có dấu, 16-bit

// CRC16/Modbus: đa thức 0xA001, khởi tạo 0xFFFF, kết quả LSB đứng trước trong
// frame
static uint16_t _ph_crc16(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001U) : (crc >> 1);
    }
  }
  return crc;
}

void AP_ShoesAgtech::_ph_init(void) {
  if (!ph_is_enabled()) {
    return;
  }

  uint8_t port_num = (uint8_t)constrain_int16(_ph_port.get(), 0, 4);
  _ph_uart = hal.serial(port_num);

  if (_ph_uart == nullptr) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: pH sensor SERIAL%d not found",
                    (int)port_num);
    return;
  }

  _ph_uart->begin(9600);
  memset(_ph_buf, 0, sizeof(_ph_buf));
  _ph_buf_idx = 0;
  _ph_buf_count = 0;
  _ph_buf_sum = 0.0f;
  _ph_req_pending = false;
  _ph_update_ms = 0;
  _ph_last_good_ms = 0;
  _ph_nodata_warn_ms = 0;
  _ph_last_log_ms = 0;

  gcs().send_text(MAV_SEVERITY_INFO,
                  "SA: pH sensor on SERIAL%d (Modbus RTU 9600)", (int)port_num);
}

void AP_ShoesAgtech::_ph_update(void) {
  if (_ph_uart == nullptr) {
    return;
  }

  uint32_t now = AP_HAL::millis();

  // ---- Gửi yêu cầu mới mỗi 2 giây ----
  if (!_ph_req_pending) {
    if (now - _ph_update_ms < 2000) {
      return;
    }
    _ph_update_ms = now;

    uint16_t stale = _ph_uart->available();
    while (stale > 0) {
      _ph_uart->read();
      stale--;
    }

    // Modbus FC04: đọc 9 thanh ghi từ 0x0000, CRC = 0x0C30 (LSB trước: 30
    // 0C)
    static const uint8_t req[8] = {0x01, 0x04, 0x00, 0x00,
                                   0x00, 0x09, 0x30, 0x0C};
    _ph_uart->write(req, sizeof(req));
    _ph_req_pending = true;
    _ph_req_sent_ms = now;
    return;
  }

  // ---- Chờ tối thiểu 150ms để cảm biến phản hồi ----
  if (now - _ph_req_sent_ms < 150) {
    return;
  }

  uint16_t avail = _ph_uart->available();

  // Timeout: nếu không đủ frame trong 500ms thì bỏ qua, cảnh báo nếu mất cảm
  // biến
  if (avail < 23) {
    if (now - _ph_req_sent_ms > 500) {
      _ph_req_pending = false;
      const uint32_t timeout_ms = (uint32_t)MAX(_ph_timeout.get(), 1) * 1000U;
      bool no_data =
          (_ph_last_good_ms == 0) || (now - _ph_last_good_ms > timeout_ms);
      if (no_data && now - _ph_nodata_warn_ms >= 10000) {
        _ph_nodata_warn_ms = now;
        if (_ph_last_good_ms == 0) {
          gcs().send_text(MAV_SEVERITY_WARNING,
                          "SA: pH sensor chưa có dữ liệu - kiểm tra dây RS485");
        } else {
          gcs().send_text(
              MAV_SEVERITY_WARNING,
              "SA: pH sensor mất kết nối (%.0fs) - kiểm tra dây RS485",
              (double)((now - _ph_last_good_ms) / 1000U));
        }
      }
    }
    return;
  }

  _ph_req_pending = false;

  // ---- Đọc đúng 23 byte ----
  uint8_t buf[23];
  for (uint8_t i = 0; i < 23; i++) {
    int16_t b = _ph_uart->read();
    buf[i] = (b >= 0) ? (uint8_t)b : 0;
  }

  // ---- Kiểm tra header Modbus và số byte ----
  if (buf[0] != 0x01 || buf[1] != 0x04 || buf[2] != 18) {
    return;
  }

  // ---- Kiểm tra CRC (tính trên byte 0..20, so với byte 21-22)
  // ----
  uint16_t crc_calc = _ph_crc16(buf, 21);
  uint16_t crc_recv = (uint16_t)buf[21] | ((uint16_t)buf[22] << 8);
  if (crc_calc != crc_recv) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: pH CRC fail (noise on RS485?)");
    return;
  }

  // ---- Giải mã thanh ghi ----
  uint16_t raw_ph = ((uint16_t)buf[3] << 8) | buf[4];
  int16_t raw_mv = (int16_t)(((uint16_t)buf[7] << 8) | buf[8]);
  int16_t raw_temp = (int16_t)(((uint16_t)buf[19] << 8) | buf[20]);

  float ph_cal = constrain_float(raw_ph / 100.0f + _ph_off.get(), 0.0f, 14.0f);
  _ph_mv = raw_mv;
  _ph_temp = raw_temp / 10.0f + _ph_toff.get();
  _ph_value = ph_cal;
  _ph_last_good_ms = now;

  // ---- Trung bình trượt (PH_WINDOW = 10 mẫu) ----
  _ph_buf_sum -= _ph_buf[_ph_buf_idx];
  _ph_buf[_ph_buf_idx] = ph_cal;
  _ph_buf_sum += ph_cal;
  _ph_buf_idx = (_ph_buf_idx + 1) % PH_WINDOW;
  if (_ph_buf_count < PH_WINDOW) {
    _ph_buf_count++;
  }
  _ph_value_ma = _ph_buf_sum / _ph_buf_count;

  // ---- Theo dõi slot hàng ngày + tính kiềm từ ΔpH ----
  _ph_update_daily_slots(ph_cal);

  // ---- IN LOG pH RA CONSOLE (SA_PH_LOG) — độc lập với SA_FLOW_LOG ----
  if (_ph_log_enable.get() > 0 &&
      now - _ph_last_log_ms >= (uint32_t)_ph_log_ms.get()) {
    _ph_last_log_ms = now;
    const char *slot_tag;
    switch (_alk_slot_status) {
    case 0:
      slot_tag = "FULL";
      break;
    case 1:
      slot_tag = "MORN";
      break;
    case 2:
      slot_tag = "AFT";
      break;
    case 3:
      slot_tag = "PREV";
      break;
    default:
      slot_tag = "NODATA";
      break;
    }
    const char *ph_pfx = (_simulation.get() > 0) ? "[SIM][WM]" : "[WM]";
    gcs().send_text(MAV_SEVERITY_INFO,
                    "%s pH:%.2f MA:%.2f Tmp:%.1fC mV:%d [%s]", ph_pfx,
                    (double)_ph_value, (double)_ph_value_ma, (double)_ph_temp,
                    (int)_ph_mv, slot_tag);
    if (_alk_slot_status == 0) {
      gcs().send_text(MAV_SEVERITY_INFO, "%s Alk:%.2fdKH %.1fmg/L dPH:%+.2f",
                      ph_pfx, (double)_alk_dkh, (double)_alk_mgl,
                      (double)_delta_ph);
    }
    {
      float ms = constrain_float(_ph_ms.get(), 0.0f, 23.99f);
      float me = constrain_float(_ph_me.get(), 0.0f, 24.0f);
      float as_ = constrain_float(_ph_as.get(), 0.0f, 23.99f);
      float ae = constrain_float(_ph_ae.get(), 0.0f, 24.0f);
      int ms_h = (int)ms, ms_m = (int)((ms - (int)ms) * 60.0f + 0.5f);
      int me_h = (int)me, me_m = (int)((me - (int)me) * 60.0f + 0.5f);
      int as_h = (int)as_, as_m = (int)((as_ - (int)as_) * 60.0f + 0.5f);
      int ae_h = (int)ae, ae_m = (int)((ae - (int)ae) * 60.0f + 0.5f);
      gcs().send_text(MAV_SEVERITY_INFO,
                      "%s Sang:%d:%02d-%d:%02d Chieu:%d:%02d-%d:%02d", ph_pfx,
                      ms_h, ms_m, me_h, me_m, as_h, as_m, ae_h, ae_m);
    }
  }
}

// =============================================================
// THEO DÕI SLOT ΔpH HÀNG NGÀY
//
// Chọn ao theo SA_POND_IDX (nhập thủ công, 1-100).
// GPS chỉ dùng để validate vị trí (in 1 lần khi đổi ao). Tích lũy mẫu pH
// last-write-wins trong slot sáng/chiều, tính kiềm khi ao đủ cả hai slot.
// Dữ liệu ao không reset hàng ngày — chỉ xóa slot hôm nay khi quay lại
// ao đó vào ngày mới. Hỗ trợ tối đa 100 ao, lưu trữ qua reboot vào SD card.
// =============================================================
void AP_ShoesAgtech::_ph_update_daily_slots(float ph_cal) {
  uint32_t now = AP_HAL::millis();

  // ---- Yêu cầu có giờ GPS ----
  uint64_t utc_usec = 0;
  if (!AP::rtc().get_utc_usec(utc_usec)) {
    if (_ph_log_enable.get() > 0 && now - _slot_warn_ms >= 60000) {
      _slot_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO, "[WM] Chưa GPS - kiềm đợi GPS/giờ");
    }
    return;
  }

  // ---- Yêu cầu GPS có fix 3D ----
  int32_t cur_lat_i = 0, cur_lng_i = 0;
  const AP_GPS &gps_inst = AP::gps();
  if (gps_inst.status(0) < AP_GPS::GPS_OK_FIX_3D) {
    if (_ph_log_enable.get() > 0 && now - _slot_warn_ms >= 60000) {
      _slot_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO, "[WM] Chưa GPS - kiềm đợi GPS/giờ");
    }
    return;
  }
  {
    const Location &loc = gps_inst.location(0);
    cur_lat_i = loc.lat;
    cur_lng_i = loc.lng;
  }

  int8_t tz = (int8_t)constrain_int16(_ph_tz.get(), -12, 14);
  uint32_t utc_sec = (uint32_t)(utc_usec / 1000000ULL);
  uint32_t local_sec = utc_sec + (uint32_t)((int32_t)tz * 3600);
  uint32_t today = local_sec / 86400U;
  float local_h = (float)(local_sec % 86400U) / 3600.0f;
  float cur_lat_f = cur_lat_i * 1.0e-7f;
  float cur_lng_f = cur_lng_i * 1.0e-7f;

  // ---- Khung giờ của slot ----
  float ms = constrain_float(_ph_ms.get(), 0.0f, 23.99f);
  float me = constrain_float(_ph_me.get(), 0.0f, 24.0f);
  float as_ = constrain_float(_ph_as.get(), 0.0f, 23.99f);
  float ae = constrain_float(_ph_ae.get(), 0.0f, 24.0f);
  const bool in_morn = (local_h >= ms && local_h <= me);
  const bool in_aft = (local_h >= as_ && local_h <= ae);

  // ---- Chọn ao theo SA_POND_IDX (nhập thủ công, 1-based) ----
  const int8_t sel =
      (int8_t)constrain_int16(_pond_select.get(), 1, (int16_t)MAX_PONDS);
  const uint8_t pond_idx = (uint8_t)(sel - 1);

  const bool pond_is_new = !_ponds[pond_idx].valid;
  if (pond_is_new) {
    memset(&_ponds[pond_idx], 0, sizeof(PondEntry));
    _ponds[pond_idx].center_lat = cur_lat_f;
    _ponds[pond_idx].center_lng = cur_lng_f;
    _ponds[pond_idx].gps_count = 1;
    _ponds[pond_idx].valid = true;
    _ponds[pond_idx].last_day = today;
    _ponds[pond_idx].dos_sp = _dos_sp.get();
    _ponds[pond_idx].dos_food = _clamp_food((int8_t)_dos_food.get());
    if (pond_idx >= _pond_count)
      _pond_count = pond_idx + 1;
    _ponds_dirty = true;
  }

  PondEntry &pond = _ponds[pond_idx];
  const unsigned disp_idx = (unsigned)pond_idx + 1;

  // ---- Thông báo khi ao thay đổi (kể cả lần đầu boot) — in 1 lần ----
  if (pond_idx != _active_pond_idx || !_pond_first_detect_done) {
    _pond_first_detect_done = true;
    gcs().send_text(MAV_SEVERITY_INFO, "[SA] Chuyen sang ao #%u%s", disp_idx,
                    pond_is_new ? " (ao moi)" : "");
    if (pond.gps_count > 5) {
      const float DEG2M = 111320.0f;
      const float coslat = cosf(cur_lat_f * DEG_TO_RAD);
      float dlat_m = (cur_lat_f - pond.center_lat) * DEG2M;
      float dlng_m = (cur_lng_f - pond.center_lng) * DEG2M * coslat;
      float dist_m = sqrtf(dlat_m * dlat_m + dlng_m * dlng_m);
      float pond_thr = constrain_float(_ph_pond_dist.get(), 10.0f, 5000.0f);
      if (dist_m <= pond_thr) {
        gcs().send_text(MAV_SEVERITY_INFO, "[SA] Ao#%u GPS OK (%.0fm)",
                        disp_idx, (double)dist_m);
      } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Ao#%u GPS lech %.0fm - can check lai vi tri ao",
                        disp_idx, (double)dist_m);
      }
    }
  }
  _active_pond_idx = pond_idx;

  // ---- Cập nhật centroid GPS (rolling average, cap 1000) ----
  if (pond.gps_count < 1000)
    pond.gps_count++;
  float w = 1.0f / (float)pond.gps_count;
  pond.center_lat = pond.center_lat * (1.0f - w) + cur_lat_f * w;
  pond.center_lng = pond.center_lng * (1.0f - w) + cur_lng_f * w;

  // ---- Ngày mới cho ao này: xóa slot hôm nay, giữ alk từ ngày trước ----
  if (pond.last_day != today) {
    pond.ph_morn = 0.0f;
    pond.ph_aft = 0.0f;
    pond.morn_count = 0;
    pond.aft_count = 0;
    pond.status = 0;
    pond.delta_ph = 0.0f;
    pond.alk_pending = false;
    pond.gcs_alk_pending = false;
    pond.alk_computed = false;
    pond.morn_reported = false;
    pond.aft_reported = false;
    pond.morn_last_ms = 0;
    pond.aft_last_ms = 0;
    pond.morn_lat = 0;
    pond.morn_lng = 0;
    pond.last_day = today;
  }

  // ---- Lấy mẫu pH — chỉ khi ARM, rate-limited SA_PH_CAP_S ----
  const bool is_armed = hal.util->get_soft_armed();
  if ((in_morn || in_aft) && is_armed) {
    uint32_t cap_ms =
        (uint32_t)constrain_int16(_ph_cap_s.get(), 1, 3600) * 1000U;
    uint8_t cap_min = (uint8_t)constrain_int16(_ph_cap_sam.get(), 1, 100);

    if (in_morn && now - pond.morn_last_ms >= cap_ms) {
      pond.ph_morn = ph_cal;
      pond.morn_lat = cur_lat_i;
      pond.morn_lng = cur_lng_i;
      pond.morn_last_ms = now;
      pond.morn_count++;
      pond.status |= 1;
      _ponds_dirty = true;
      if (pond.morn_count == cap_min && !pond.morn_reported) {
        pond.morn_reported = true;
        gcs().send_text(MAV_SEVERITY_INFO, "[SA] Ao#%u pH sang: %.2f (%u mau)",
                        disp_idx, (double)pond.ph_morn, (unsigned)cap_min);
      }
    } else if (in_aft && now - pond.aft_last_ms >= cap_ms) {
      pond.ph_aft = ph_cal;
      pond.aft_last_ms = now;
      pond.aft_count++;
      pond.status |= 2;
      _ponds_dirty = true;
      if (pond.aft_count == cap_min && !pond.aft_reported) {
        pond.aft_reported = true;
        gcs().send_text(MAV_SEVERITY_INFO, "[SA] Ao#%u pH chieu: %.2f (%u mau)",
                        disp_idx, (double)pond.ph_aft, (unsigned)cap_min);
      }
    }
  }

  // ---- Tính kiềm khi ao đủ cả hai slot (chỉ tính một lần mỗi ngày) ----
  if (pond.status == 3 && !pond.alk_computed) {
    pond.delta_ph = pond.ph_aft - pond.ph_morn;
    float kh_scaled =
        _ph_kh.get() *
        (1.0f + constrain_float(pond.delta_ph * 0.375f, -0.5f, 1.0f));
    pond.alk_dkh = _ph_calc_alkalinity(pond.ph_morn, kh_scaled, _ph_temp);
    pond.alk_mgl = pond.alk_dkh * 17.85f;
    pond.alk_computed = true;
    pond.alk_pending = true;     // kích hoạt ghi PHAK vào SD (Log.cpp)
    pond.gcs_alk_pending = true; // kích hoạt gửi SA_PHK qua MAVLink (GCS)
    _ponds_dirty = true;
    _delta_ph = pond.delta_ph;
    _alk_dkh = pond.alk_dkh;
    _alk_mgl = pond.alk_mgl;
    gcs().send_text(MAV_SEVERITY_INFO, "[SA] Ao#%u S:%.2f C:%.2f dPH:%.2f",
                    disp_idx, (double)pond.ph_morn, (double)pond.ph_aft,
                    (double)pond.delta_ph);
    gcs().send_text(MAV_SEVERITY_INFO, "[SA] Ao#%u kiem:%.1fdKH/%.0fmgL",
                    disp_idx, (double)pond.alk_dkh, (double)pond.alk_mgl);
  }

  // ---- Cập nhật trạng thái hiển thị từ ao đang active ----
  if (pond.status == 3) {
    _alk_slot_status = 0;
  } else if (pond.status == 1) {
    _alk_slot_status = 1;
  } else if (pond.status == 2) {
    _alk_slot_status = 2;
  } else if (pond.alk_dkh > 0.0f) {
    _alk_dkh = pond.alk_dkh;
    _alk_mgl = pond.alk_mgl;
    _alk_slot_status = 3;
  } else {
    _alk_slot_status = 4;
  }
}

// =============================================================
// Thuật toán ước tính độ kiềm
//
// Ước tính thay đổi tương đối của độ kiềm (KH) dựa trên độ lệch
// giữa pH đo được so với mốc tham chiếu (pH ≈ 8.0) và nhiệt độ.
//
// Cơ sở vật lý:
//   - Trong nước ao, cân bằng CO2/HCO3-/CO3²- gắn kết pH và KH.
//   - pH > 8.3: HCO3- chiếm ưu thế, ít CO2 tự do → KH hiệu dụng cao hơn.
//   - pH < 7.6: nhiều CO2 tự do tiêu hao KH → KH hiệu dụng thấp hơn.
//   - Nhiệt độ cao hơn → ít CO2 hòa tan hơn → pH tăng nhẹ.
//
// Đây là mô hình ước lượng, KHÔNG thay thế cho đo KH trực tiếp bằng
// test kit. Dùng để theo dõi xu hướng; cập nhật SA_PH_KH định kỳ.
// =============================================================
float AP_ShoesAgtech::_ph_calc_alkalinity(float ph, float base_kh_dkh,
                                          float temp_c) {
  ph = constrain_float(ph, 0.0f, 14.0f);
  temp_c = constrain_float(temp_c, -10.0f, 50.0f);

  float tf = constrain_float(1.0f - (temp_c - 28.0f) * 0.008f, 0.85f, 1.10f);

  float pf;
  if (ph >= 8.3f) {
    pf = 1.0f + (ph - 8.3f) * 0.32f;
  } else if (ph <= 7.6f) {
    pf = 1.0f - (7.6f - ph) * 0.45f;
  } else {
    pf = 1.0f + (ph - 8.0f) * 0.15f;
  }

  return base_kh_dkh * constrain_float(pf * tf, 0.55f, 1.75f);
}

// =============================================================
// consume_alk_log_pending — lấy ra MỘT ao đang chờ ghi PHAK mỗi lần gọi.
// Gán các field mirror đầu ra (_ph_morn_val, _alk_dkh, ...) để Log.cpp
// đọc qua getter và ghi bản ghi PHAK. Trả về false khi không còn ao
// nào đang chờ.
// =============================================================
bool AP_ShoesAgtech::consume_alk_log_pending(void) {
  for (uint8_t i = 0; i < _pond_count; i++) {
    PondEntry &p = _ponds[i];
    if (!p.valid || !p.alk_pending)
      continue;
    p.alk_pending = false;
    _alk_pond_idx = i;
    _ph_morn_val = p.ph_morn;
    _ph_aft_val = p.ph_aft;
    _ph_morn_lat = p.morn_lat;
    _ph_morn_lng = p.morn_lng;
    _delta_ph = p.delta_ph;
    _alk_dkh = p.alk_dkh;
    _alk_mgl = p.alk_mgl;
    return true;
  }
  return false;
}

// =============================================================
// consume_gcs_alk_pending — cùng mẫu với consume_alk_log_pending,
// dùng cờ riêng cho đường MAVLink (gcs_alk_pending).
// KHÔNG ảnh hưởng đến consume_alk_log_pending.
// =============================================================
bool AP_ShoesAgtech::consume_gcs_alk_pending(void) {
  for (uint8_t i = 0; i < _pond_count; i++) {
    PondEntry &p = _ponds[i];
    if (!p.valid || !p.gcs_alk_pending)
      continue;
    p.gcs_alk_pending = false;
    _alk_pond_idx = i;
    _ph_morn_val = p.ph_morn;
    _ph_aft_val = p.ph_aft;
    _ph_morn_lat = p.morn_lat;
    _ph_morn_lng = p.morn_lng;
    _delta_ph = p.delta_ph;
    _alk_dkh = p.alk_dkh;
    _alk_mgl = p.alk_mgl;
    return true;
  }
  return false;
}

// =============================================================
// CẬP NHẬT IO THREAD — chạy trong IO thread của ArduPilot (an toàn cho
// AP::FS()) Xử lý load ao ở lần gọi đầu và save khi có thay đổi (rate-limited).
// Được đăng ký qua hal.scheduler->register_io_process() trong init().
// =============================================================
void AP_ShoesAgtech::_io_update(void) {
  if (!_ponds_loaded) {
    _ponds_loaded = true;
    _pond_load();
  }
  if (_ponds_dirty) {
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _ponds_save_ms >= 5000U) {
      _pond_save();
      _ponds_save_ms = now_ms;
      _ponds_dirty = false;
    }
  }
}

// =============================================================
// LƯU TRỮ TRẠNG THÁI AO — /APM/SA_PONDS.bin
// Lưu toàn bộ _ponds[] vào SD card mỗi khi có thay đổi (rate-limited 5s).
// Đọc lại khi boot để khôi phục dữ liệu pH, kiềm, dos_sp của từng ao.
// Format: magic(4) + version(1) + count(1) + PondEntry[MAX_PONDS]
// =============================================================
#define SA_PONDS_FILE "/APM/SA_PONDS.bin"
#define SA_PONDS_MAGIC 0x504F4E44UL // 'POND'
#define SA_PONDS_VER 3 // v3: thêm PondEntry::dos_food — file v2 cũ sẽ bị bỏ qua

struct PondStateHdr {
  uint32_t magic;
  uint8_t version;
  uint8_t count;
};

void AP_ShoesAgtech::_pond_save(void) {
  int fd = AP::FS().open(SA_PONDS_FILE, O_WRONLY | O_CREAT | O_TRUNC);
  if (fd < 0)
    return;

  // Chỉ ghi đúng _pond_count slot đã từng dùng tới (không phải cả MAX_PONDS)
  // để giảm I/O thẻ SD. _pond_load() vẫn đọc an toàn: phần còn lại của
  // _ponds[] giữ nguyên trạng thái zero-init (valid=false) nếu file ngắn hơn.
  PondStateHdr hdr{SA_PONDS_MAGIC, SA_PONDS_VER, _pond_count};
  AP::FS().write(fd, &hdr, sizeof(hdr));
  AP::FS().write(fd, _ponds, sizeof(PondEntry) * _pond_count);
  AP::FS().close(fd);
}

void AP_ShoesAgtech::_pond_load(void) {
  int fd = AP::FS().open(SA_PONDS_FILE, O_RDONLY);
  if (fd < 0)
    return;

  PondStateHdr hdr{};
  if (AP::FS().read(fd, &hdr, sizeof(hdr)) != (ssize_t)sizeof(hdr) ||
      hdr.magic != SA_PONDS_MAGIC || hdr.version != SA_PONDS_VER) {
    AP::FS().close(fd);
    return;
  }

  AP::FS().read(fd, _ponds, sizeof(PondEntry) * MAX_PONDS);
  AP::FS().close(fd);

  _pond_count = hdr.count;

  for (uint8_t i = 0; i < MAX_PONDS; i++) {
    if (!_ponds[i].valid)
      continue;
    _ponds[i].morn_last_ms = 0;
    _ponds[i].aft_last_ms = 0;
    _ponds[i].alk_pending = false;
    _ponds[i].gcs_alk_pending = false;
  }

  gcs().send_text(MAV_SEVERITY_INFO, "[SA] Load %u ao tu SD card",
                  (unsigned)_pond_count);
}

// =============================================================
// MODULE 3 — MOTOR CHO ĂN (DOSING)
// vít tải thức ăn tôm, servo xoay liên tục 360°
//
// RC SA_DOS_RC bật/tắt: PWM > 1500 -> bật, PWM <= 1500 -> tắt (xuất 1500).
//
// Quy đổi lượng thức ăn (SA_DOS_SP, gam) -> độ lệch PWM. SA_DOS_Fx
// (mL/50us) + SA_DOS_Dx (g/mL) chọn theo SA_DOS_FOOD, dùng chung cho
// cả 2 mode (calib riêng theo từng loại thức ăn):
//   DOS_MODE=0 (toc do co dinh):
//     offset = SA_DOS_SP * 50 / (SA_DOS_Fx(mL/50us) x SA_DOS_Dx(g/mL))
//   DOS_MODE=1 (phan bo deu theo mission):
//     dos_gpm = SA_DOS_SP * speed * 60 / mission_dist
//     offset  = dos_gpm * 50 / (SA_DOS_Fx(mL/50us) x SA_DOS_Dx(g/mL))
//
// Chiều quay theo SA_DOS_REV (servo 360°, 1500 = dừng):
//   0 = thuận: pwm = constrain(1500 - offset,  800, 1500)
//   1 = ngược: pwm = constrain(1500 + offset, 1500, 2200)
// Yêu cầu SERVOx_FUNCTION = 0 (None) trên kênh SA_DOS_CHAN.
// =============================================================

// =============================================================
// KIỂM TRA CẤU HÌNH KÊNH MOTOR CHO ĂN
// Servo SA_DOS_CHAN BẮT BUỘC phải thoả cả 4 điều kiện mới cho phép
// motor chạy: FUNCTION=0 (None), MIN=800, TRIM=1500, MAX=2200
// Sai điều kiện nào -> báo lỗi mỗi 5 giây.
// Khi vừa đạt đủ cả 4 -> báo "setup thành công" một lần.
// =============================================================
void AP_ShoesAgtech::_check_dosing_config(void) {
  uint8_t chan_idx = (uint8_t)constrain_int16(_dos_chan.get() - 1, 0, 15);
  SRV_Channel *ch = SRV_Channels::srv_channel(chan_idx);
  int32_t func_val = (int32_t)SRV_Channels::channel_function(chan_idx);
  int32_t chan = (int32_t)_dos_chan.get();

  bool have_chan = (ch != nullptr);
  bool func_ok = have_chan && (func_val == (int32_t)SRV_Channel::k_none);
  bool min_ok = have_chan && (ch->get_output_min() == 800);
  bool trim_ok = have_chan && (ch->get_trim() == 1500);
  bool max_ok = have_chan && (ch->get_output_max() == 2200);

  _dos_config_ok = func_ok && min_ok && trim_ok && max_ok;

  if (_dos_config_ok) {
    if (!_dos_was_ok) {
      gcs().send_text(MAV_SEVERITY_INFO,
                      "SA: SERVO%d setup thành công - dosing motor sẵn sàng",
                      (int)chan);
    }
    _dos_was_ok = true;
    return;
  }
  _dos_was_ok = false;

  uint32_t now = AP_HAL::millis();
  if (now - _dos_warn_ms < 5000) {
    return;
  }
  _dos_warn_ms = now;

  if (!have_chan) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d không tồn tại",
                    (int)chan);
    return;
  }
  if (!func_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA: SERVO%d FUNCTION=%d, cần đặt =0 (None)", (int)chan,
                    (int)func_val);
  }
  if (!min_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MIN=%u, cần đặt =800",
                    (int)chan, (unsigned)ch->get_output_min());
  }
  if (!trim_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d TRIM=%u, cần đặt =1500",
                    (int)chan, (unsigned)ch->get_trim());
  }
  if (!max_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MAX=%u, cần đặt =2200",
                    (int)chan, (unsigned)ch->get_output_max());
  }
}

// Kẹp giá trị loại thức ăn về dải hợp lệ 1-7 (khớp SA_DOS_FOOD @Range).
int8_t AP_ShoesAgtech::_clamp_food(int8_t food) {
  return (int8_t)constrain_int16(food, 1, 7);
}

// Chuyển offset PWM (us, luôn dương, xem công thức đầu Module 3) thành PWM
// xuất ra theo chiều quay SA_DOS_REV, constrain đúng nửa dải servo.
uint16_t AP_ShoesAgtech::_offset_to_dos_pwm(float offset) const {
  if (_dos_rev.get() == 0) {
    return (uint16_t)constrain_float(1500.0f - offset, 800.0f, 1500.0f);
  }
  return (uint16_t)constrain_float(1500.0f + offset, 1500.0f, 2200.0f);
}

// =============================================================
// ĐỒNG BỘ SETPOINT + LOẠI THỨC ĂN THEO AO
// SA_DOS_SP / SA_DOS_FOOD là tham số người dùng đọc/sửa, nhưng giá trị
// điều khiển motor lấy từ dos_sp/dos_food riêng của ao đang active
// (_ponds[]).
//   - Vừa đổi ao: nạp dos_sp + dos_food đã lưu của ao đó lên
//     SA_DOS_SP / SA_DOS_FOOD.
//   - SA_DOS_SP hoặc SA_DOS_FOOD bị người dùng sửa (khác lần đồng bộ
//     trước): ghi giá trị mới xuống ao đang active, đánh dấu lưu SD.
// Nếu ao đang active chưa được tạo (vd chưa có GPS) thì bỏ qua,
// _update_dosing_motor() sẽ dùng thẳng SA_DOS_SP/SA_DOS_FOOD làm giá
// trị chung.
// =============================================================
void AP_ShoesAgtech::_sync_dosing_setpoint(void) {
  PondEntry &pond = _ponds[_active_pond_idx];
  if (!pond.valid) {
    return;
  }

  if (_dos_sync_pond != _active_pond_idx) {
    _dos_sp.set(pond.dos_sp);
    _dos_sp_sync_val = pond.dos_sp;
    _dos_food.set(pond.dos_food);
    _dos_food_sync_val = pond.dos_food;
    _dos_sync_pond = _active_pond_idx;
    return;
  }

  float cur_sp = _dos_sp.get();
  if (fabsf(cur_sp - _dos_sp_sync_val) > 0.001f) {
    pond.dos_sp = cur_sp;
    _dos_sp_sync_val = cur_sp;
    _ponds_dirty = true;
  }

  int8_t cur_food = _clamp_food((int8_t)_dos_food.get());
  if (cur_food != _dos_food_sync_val) {
    pond.dos_food = cur_food;
    _dos_food_sync_val = cur_food;
    _ponds_dirty = true;
  }
}

void AP_ShoesAgtech::_update_dosing_motor(void) {
  _sync_dosing_setpoint();
  const PondEntry &active_pond = _ponds[_active_pond_idx];
  const float dos_sp_active =
      active_pond.valid ? active_pond.dos_sp : _dos_sp.get();
  const int8_t dos_food_active =
      active_pond.valid ? active_pond.dos_food : (int8_t)_dos_food.get();

  _check_dosing_config();
  if (!_dos_config_ok) {
    _dos_pwm = 1500;
    return;
  }

  uint32_t now = AP_HAL::millis();
  uint8_t rc_idx = (uint8_t)constrain_int16(_dos_rc.get() - 1, 0, 15);
  uint16_t rc_pwm = RC_Channels::get_radio_in(rc_idx);
  bool motor_on = (rc_pwm > 1500);

  if (motor_on != _dos_was_on) {
    _dos_was_on = motor_on;
    gcs().send_text(MAV_SEVERITY_INFO, "SA: Dosing motor %s",
                    motor_on ? "ON" : "OFF");
  }

  float dos_rate_gpm = 0.0f; // tốc độ cấp tức thời (g/phút) — dùng để in log

  if (motor_on) {
    float pwm_f = 1500.0f;

    // Tốc độ vít tải + mật độ đều lấy theo loại thức ăn đang active
    // (SA_DOS_Fx/SA_DOS_Dx), dùng chung cho cả 2 mode — calib
    // SA_DOS_Fx/SA_DOS_Dx riêng cho từng loại thức ăn.
    uint8_t food_idx = (uint8_t)_clamp_food(dos_food_active) - 1;
    float vol_rate = _dos_fr[food_idx].get();
    if (vol_rate < 0.1f) {
      vol_rate = 0.1f;
    }
    float density = _dos_dr[food_idx].get();
    if (density < 0.01f) {
      density = 0.01f;
    }
    float effective = vol_rate * density;

    if (_dos_mode.get() == 0) {
      // ---- DOS_MODE 0: tốc độ cố định — SA_DOS_SP CHÍNH LÀ tốc độ (g/phút) ----
      dos_rate_gpm = dos_sp_active;
      float offset = dos_rate_gpm * 50.0f / effective;
      pwm_f = (float)_offset_to_dos_pwm(offset);
    } else {
      // ---- DOS_MODE 1: phân bố đều theo mission — SA_DOS_SP là TỔNG gam,
      // dos_rate_gpm là tốc độ tức thời suy ra từ speed/mission_dist ----
      float mission_dist = _get_mission_dist();
      float speed_ms = _get_spray_speed(); // đồng bộ nguồn tốc độ với Module 1
                                            // (SIM > SA_FLOW_VEL > AHRS)
      if (mission_dist > 1.0f && speed_ms >= 0.05f) {
        dos_rate_gpm = (dos_sp_active * speed_ms * 60.0f) / mission_dist;
        float offset = dos_rate_gpm * 50.0f / effective;
        pwm_f = (float)_offset_to_dos_pwm(offset);
      } else {
        pwm_f = 1500.0f;
        dos_rate_gpm = 0.0f;
        if (now - _dos_warn_ms >= 5000U) {
          _dos_warn_ms = now;
          if (mission_dist <= 1.0f) {
            gcs().send_text(
                MAV_SEVERITY_WARNING,
                "SA DOS1: chưa có mission (dist=%.1fm) - motor dừng",
                (double)mission_dist);
          } else {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "SA DOS1: tốc độ quá thấp (%.2fm/s) - motor dừng",
                            (double)speed_ms);
          }
        }
      }
    }
    _dos_pwm = (uint16_t)pwm_f;
  } else {
    _dos_pwm = 1500;
  }

  uint8_t chan_idx = (uint8_t)constrain_int16(_dos_chan.get() - 1, 0, 15);
  SRV_Channels::set_output_pwm_chan(chan_idx, _dos_pwm);

  // ---- IN LOG MOTOR CHO ĂN RA CONSOLE (SA_DOS_LOG) ----
  // Format khác nhau theo mode: mode 0 thì SA_DOS_SP CHÍNH LÀ tốc độ (g/ph)
  // nên chỉ in 1 field "Rate"; mode 1 thì SA_DOS_SP là tổng gam cho cả
  // mission, nên in thêm "Rate" (tốc độ tức thời suy ra) bên cạnh "SP" (tổng).
  if (_dos_log_enable.get() > 0) {
    if (now - _dos_last_log_ms >= (uint32_t)_dos_log_ms.get()) {
      _dos_last_log_ms = now;
      uint8_t food_log = (uint8_t)_clamp_food(dos_food_active) - 1;
      if (_dos_mode.get() == 0) {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "[DOS] M0 F%d SERVO%d %s Rate:%.0fg/ph D:%.2fg/mL PWM:%u",
            (int)dos_food_active, (int)_dos_chan.get(),
            motor_on ? "ON" : "OFF", (double)dos_rate_gpm,
            (double)_dos_dr[food_log].get(), (unsigned)_dos_pwm);
      } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[DOS] M1 F%d SERVO%d %s SP:%.0fg Rate:%.2fg/ph "
                        "D:%.2fg/mL PWM:%u",
                        (int)dos_food_active, (int)_dos_chan.get(),
                        motor_on ? "ON" : "OFF", (double)dos_sp_active,
                        (double)dos_rate_gpm, (double)_dos_dr[food_log].get(),
                        (unsigned)_dos_pwm);
      }
    }
  }
}

// =============================================================
// MÔ PHỎNG — SA_SIM = 1
// Sinh dữ liệu cảm biến giả dạng sóng sin để kiểm tra mode 1/2 và
// hiển thị GCS mà không cần gắn phần cứng thật.
// Được gọi từ update() thay cho _ph_update() khi bật.
// Giá trị lưu lượng được ghi thẳng vào _flow_rate_filtered và
// _sim_speed; buffer trung bình trượt vẫn do update() (hàm gọi)
// cập nhật như bình thường.
// =============================================================
void AP_ShoesAgtech::_run_simulation(void) {
  uint32_t now = AP_HAL::millis();
  float t = now * 0.001f;

  // ---- Module 1: lưu lượng (2.5 ± 1.5 L/min, chu kỳ 20s) ----
  _flow_rate_filtered = 2.5f + 1.5f * sinf(2.0f * M_PI * t / 20.0f);

  // ---- Module 1: tốc độ mặt đất giả lập (1.0 ± 0.8 m/s, chu kỳ 30s) ----
  _sim_speed =
      constrain_float(1.0f + 0.8f * sinf(2.0f * M_PI * t / 30.0f), 0.1f, 2.0f);

  // ---- Module 2: giá trị cảm biến pH ----
  float ph_sim = 7.3f + 0.4f * sinf(2.0f * M_PI * t / 60.0f);
  float temp_sim = 28.0f + 2.0f * sinf(2.0f * M_PI * t / 120.0f);
  int16_t mv_sim = (int16_t)((7.0f - ph_sim) * 59.16f);

  _ph_value = ph_sim;
  _ph_value_ma = ph_sim;
  _ph_mv = mv_sim;
  _ph_temp = temp_sim;

  _ph_last_good_ms = now;

  // Chạy đủ pipeline slot + tính kiềm để log PHAK ra SD hoạt động trong SITL.
  _ph_update_daily_slots(ph_sim);
}
