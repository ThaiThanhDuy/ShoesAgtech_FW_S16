#include "Rover.h"

#include <AP_Gripper/AP_Gripper.h>

/*
  Rover parameter definitions
*/

const AP_Param::Info Rover::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the
    // eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 1),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what log types to enable in on-board logger. This
    // value is made up of the sum of each of the log types you want to be
    // saved. On boards supporting microSD cards or other large block-storage
    // devices it is usually best just to enable all basic log types by setting
    // this to 65535.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System
    // Performance,4:Throttle,5:Navigation Tuning,7:IMU,8:Mission
    // Commands,9:Battery
    // Monitor,10:Rangefinder,11:Compass,12:Camera,13:Steering,14:RC
    // Input-Output,19:Raw IMU,20:Video Stabilization,21:Optical Flow
    // @User: Advanced
    GSCALAR(log_bitmask, "LOG_BITMASK", DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode after
    // geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan, "RST_SWITCH_CH", 0),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial driving mode
    // @Description: This selects the mode to start in on boot. This is useful
    // for when you want to start in AUTO mode on boot without a receiver.
    // Usually used in combination with when AUTO_TRIGGER_PIN or AUTO_KICKSTART.
    // @CopyValuesFrom: MODE1
    // @User: Advanced
    GSCALAR(initial_mode, "INITIAL_MODE", (int8_t)Mode::Number::MANUAL),

    // SYSID_THISMAV was here

    // SYSID_MYGCS was here

    // TELEM_DELAY was here

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:Steering,1:Throttle,2:Pitch,3:Left Wheel,4:Right
    // Wheel,5:Sailboat Heel,6:Velocity North,7:Velocity East
    GSCALAR(gcs_pid_mask, "GCS_PID_MASK", 0),

    // @Param: AUTO_TRIGGER_PIN
    // @DisplayName: Auto mode trigger pin
    // @Description: pin number to use to enable the throttle in auto mode. If
    // set to -1 then don't use a trigger, otherwise this is a pin number which
    // if held low in auto mode will enable the motor to run. If the switch is
    // released while in AUTO then the motor will stop again. This can be used
    // in combination with INITIAL_MODE to give a 'press button to start' rover
    // with no receiver.
    // @Values: -1:Disabled,0:APM TriggerPin0,1:APM TriggerPin1,2:APM
    // TriggerPin2,3:APM TriggerPin3,4:APM TriggerPin4,5:APM TriggerPin5,6:APM
    // TriggerPin6,7:APM TriggerPin7,8:APM
    // TriggerPin8,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    GSCALAR(auto_trigger_pin, "AUTO_TRIGGER_PIN", -1),

    // @Param: AUTO_KICKSTART
    // @DisplayName: Auto mode trigger kickstart acceleration
    // @Description: X acceleration in meters/second/second to use to trigger
    // the motor start in auto mode. If set to zero then auto throttle starts
    // immediately when the mode switch happens, otherwise the rover waits for
    // the X acceleration to go above this value before it will start the motor
    // @Units: m/s/s
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(auto_kickstart, "AUTO_KICKSTART", 0.0f),

    // @Param: CRUISE_SPEED
    // @DisplayName: Target cruise speed in auto modes
    // @Description: The target speed in auto missions.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(speed_cruise, "CRUISE_SPEED", CRUISE_SPEED),

    // @Param: CRUISE_THROTTLE
    // @DisplayName: Base throttle percentage in auto
    // @Description: The base throttle percentage to use in auto mode. The
    // CRUISE_SPEED parameter controls the target speed, but the rover starts
    // with the CRUISE_THROTTLE setting as the initial estimate for how much
    // throttle is needed to achieve that speed. It then adjusts the throttle
    // based on how fast the rover is actually going.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_cruise, "CRUISE_THROTTLE", 50),

    // @Param: PILOT_STEER_TYPE
    // @DisplayName: Pilot input steering type
    // @Description: Pilot RC input interpretation
    // @Values: 0:Default,1:Two Paddles Input,2:Direction reversed when backing
    // up,3:Direction unchanged when backing up
    // @User: Standard
    GSCALAR(pilot_steer_type, "PILOT_STEER_TYPE", 0),

    // @Param: FS_ACTION
    // @DisplayName: Failsafe Action
    // @Description: What to do on a failsafe event
    // @Values: 0:Nothing,1:RTL,2:Hold,3:SmartRTL or RTL,4:SmartRTL or
    // Hold,5:Terminate,6:Loiter or Hold
    // @User: Standard
    GSCALAR(fs_action, "FS_ACTION", (int8_t)FailsafeAction::Hold),

    // @Param: FS_TIMEOUT
    // @DisplayName: Failsafe timeout
    // @Description: The time in seconds that a failsafe condition must persist
    // before the failsafe action is triggered
    // @Units: s
    // @Range: 1 100
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout, "FS_TIMEOUT", 1.5),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software
    // failsafe activated by a setting on the throttle input channel to a low
    // value. This can be used to detect the RC transmitter going out of range.
    // Failsafe will be triggered when the throttle channel goes below the
    // FS_THR_VALUE for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled,2:Enabled Continue with Mission in Auto
    // @User: Standard
    GSCALAR(fs_throttle_enabled, "FS_THR_ENABLE", FS_THR_ENABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on the throttle channel below which throttle
    // failsafe triggers.
    // @Range: 910 1100
    // @Increment: 1
    // @User: Standard
    GSCALAR(fs_throttle_value, "FS_THR_VALUE", 910),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. When
    // enabled the Rover will execute the FS_ACTION when it fails to receive
    // MAVLink heartbeat packets for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled,2:Enabled Continue with Mission in Auto
    // @User: Standard
    GSCALAR(fs_gcs_enabled, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check action
    // @Description: What to do on a crash event. When enabled the rover will go
    // to hold if a crash is detected.
    // @Values: 0:Disabled,1:Hold,2:HoldAndDisarm
    // @User: Standard
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK", FS_CRASH_DISABLE),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe
    // is invoked
    // @Values: 0:Disabled,1:Hold,2:ReportOnly
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION", FS_EKF_HOLD),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity
    // variance
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH", 0.8f),

    // @Param: MODE_CH
    // @DisplayName: Mode channel
    // @Description: RC Channel to use for driving mode control
    // @User: Advanced
    GSCALAR(mode_channel, "MODE_CH", MODE_CHANNEL),

    // @Param: MODE1
    // @DisplayName: Mode1
    // @Values:
    // 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,8:Dock,9:Circle,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    // @Description: Driving mode for switch position 1 (910 to 1230 and above
    // 2049)
    GSCALAR(mode1, "MODE1", (int8_t)Mode::Number::MANUAL),

    // @Param: MODE2
    // @DisplayName: Mode2
    // @Description: Driving mode for switch position 2 (1231 to 1360)
    // @CopyValuesFrom: MODE1
    // @User: Standard
    GSCALAR(mode2, "MODE2", (int8_t)Mode::Number::MANUAL),

    // @Param: MODE3
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode3
    // @Description: Driving mode for switch position 3 (1361 to 1490)
    GSCALAR(mode3, "MODE3", (int8_t)Mode::Number::MANUAL),

    // @Param: MODE4
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode4
    // @Description: Driving mode for switch position 4 (1491 to 1620)
    GSCALAR(mode4, "MODE4", (int8_t)Mode::Number::MANUAL),

    // @Param: MODE5
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode5
    // @Description: Driving mode for switch position 5 (1621 to 1749)
    GSCALAR(mode5, "MODE5", (int8_t)Mode::Number::MANUAL),

    // @Param: MODE6
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode6
    // @Description: Driving mode for switch position 6 (1750 to 2049)
    GSCALAR(mode6, "MODE6", (int8_t)Mode::Number::MANUAL),

    // =========================================================
    // === SHARED PITCH SAFETY THRESHOLDS
    // ===   Dung chung cho Manual
    // =========================================================

    // @Param: SAFE_PITCH_DN
    // @DisplayName: Safe Pitch Down Limit
    // @Description: Nguong goc chui mui an toan (nhap gia tri duong)
    // @Range: 0 45
    // @Units: deg
    // @User: Standard
    GSCALAR(safe_pitch_down, "SAFE_PITCH_DN", 10.0f),

    // @Param: SAFE_PITCH_UP
    // @DisplayName: Safe Pitch Up Limit
    // @Description: Nguong goc ngua mui an toan
    // @Range: 0 45
    // @Units: deg
    // @User: Standard
    GSCALAR(safe_pitch_up, "SAFE_PITCH_UP", 10.0f),

    // @Param: SAFE_PITCH_ACCEL
    // @DisplayName: Safe Pitch Angular Acceleration Limit
    // @Description: Gioi han gia toc goc Pitch sau khi loc nhieu LPF ~4Hz
    // @Range: 30.0 300.0
    // @Units: deg/s/s
    // @Increment: 5.0
    // @User: Standard
    GSCALAR(safe_pitch_accel, "SAFE_PITCH_ACCEL", 120.0f),

    // =========================================================
    // === MANUAL MODE — PITCH SAFETY
    // =========================================================

    // @Param: MAN_PITCH_EN
    // @DisplayName: Manual Pitch Safety Enable
    // @Description: Kich hoat giam toc khi Pitch vuot nguong trong Manual Mode
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(man_pitch_en, "MAN_PITCH_EN", 1),

    // @Param: MAN_PITCH_SCL
    // @DisplayName: Manual Pitch Throttle Scale
    // @Description: Ti le (%) ga con lai khi vi pham Pitch trong Manual Mode
    // @Range: 10 100
    // @Units: %
    // @User: Advanced
    GSCALAR(man_pitch_scale, "MAN_PITCH_SCL", 50),

    // @Param: MAN_PITCH_DLY
    // @DisplayName: Manual Pitch Recovery Delay
    // @Description: Thoi gian tre (ms) duy tri giam ga sau khi Pitch on dinh
    // tro lai trong Manual Mode
    // @Range: 0 5000
    // @Units: ms
    // @User: Advanced
    GSCALAR(man_pitch_delay, "MAN_PITCH_DLY", 2000),

    // =========================================================
    // === AUTO MODE — PITCH SAFETY
    // =========================================================

    // @Param: AUTO_PITCH_EN
    // @DisplayName: Auto Pitch Safety Enable
    // @Description: Kich hoat giam target_speed khi Pitch vuot nguong trong Auto Mode
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(auto_pitch_en, "AUTO_PITCH_EN", 1),

    // @Param: AUTO_PITCH_SCL
    // @DisplayName: Auto Pitch Speed Scale
    // @Description: Ti le (%) target_speed con lai khi vi pham Pitch trong Auto Mode
    // @Range: 10 100
    // @Units: %
    // @User: Advanced
    GSCALAR(auto_pitch_scale, "AUTO_PITCH_SCL", 50),

    // @Param: AUTO_PITCH_DLY
    // @DisplayName: Auto Pitch Recovery Delay
    // @Description: Thoi gian tre (ms) duy tri giam toc do sau khi Pitch on dinh
    // tro lai trong Auto Mode
    // @Range: 0 5000
    // @Units: ms
    // @User: Advanced
    GSCALAR(auto_pitch_delay, "AUTO_PITCH_DLY", 2000),

    // =========================================================
    // === AUTO MODE — PID AUTO-TUNE ANALYZER
    // =========================================================

    // @Param: AUTO_TUNE
    // @DisplayName: Auto Mode PID Tuning Analyzer
    // @Description: Khi bat (1): bat dau thu thap chi so on dinh (cross-track
    // error, dao dong, sai so toc do) tu luc Arm trong Auto Mode, ket thuc
    // khi Disarm hoac doi sang mode khac. Ket thuc 1 chu ky se in 1 dong
    // STATUSTEXT recommend gia tri ATC_STR_RAT_P/D va ATC_SPEED_P moi, KHONG
    // tu dong ghi de tham so hien tai.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    GSCALAR(auto_tune, "AUTO_TUNE", 0),

    // =========================================================
    // === SCHEDULED AUTO-RUN (AUTO_TIMER)
    // =========================================================

    // @Param: AUTO_TIMER
    // @DisplayName: Scheduled auto-run enable
    // @Description: Khi bat (1): theo doi gio dia phuong (can GPS/RTC), den
    //   dung 1 trong 3 moc AUTO_TIMER1/2/3 ma dang o mode MANUAL va CHUA ARM
    //   thi tu dong reset mission ve waypoint dau va chuyen sang mode AUTO
    //   (KHONG tu dong ARM). Neu da ARM luc do gio thi chi canh bao, khong
    //   doi mode. Kiem tra lai moi giay, moi moc chi kich hoat 1 lan/ngay.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(auto_timer, "AUTO_TIMER", 0),

    // @Param: AUTO_TIMER_TZ
    // @DisplayName: Scheduled auto-run timezone offset
    // @Description: Do lech gio dia phuong so voi UTC (gio), dung de tinh gio
    //   hien tai khop voi AUTO_TIMER1/2/3. Viet Nam = 7.
    // @Range: -12 14
    // @Units: h
    // @User: Standard
    GSCALAR(auto_timer_tz, "AUTO_TIMER_TZ", 7),

    // @Param: AUTO_TIMER1
    // @DisplayName: Scheduled auto-run time slot 1
    // @Description: Gio hen chay tu dong, nhap THANG gio.phut (vd 15.30 =
    //   15h30p) - KHONG phai phan so gio. 0 = tat slot nay. Phan phut (2 chu
    //   so sau dau cham) phai < 60, neu khong se bi coi la khong hop le va bi
    //   bo qua (co canh bao GCS). Muon chay dung 0h00 (nua dem) thi nhap 24.00.
    // @Range: 0 24
    // @User: Standard
    GSCALAR(auto_timer1, "AUTO_TIMER1", 0.0f),

    // @Param: AUTO_TIMER2
    // @DisplayName: Scheduled auto-run time slot 2
    // @Description: Xem mo ta AUTO_TIMER1.
    // @Range: 0 24
    // @User: Standard
    GSCALAR(auto_timer2, "AUTO_TIMER2", 0.0f),

    // @Param: AUTO_TIMER3
    // @DisplayName: Scheduled auto-run time slot 3
    // @Description: Xem mo ta AUTO_TIMER1.
    // @Range: 0 24
    // @User: Standard
    GSCALAR(auto_timer3, "AUTO_TIMER3", 0.0f),

    // =========================================================
    // === AUTO MODE — SPEED-BAND PID SCHEDULING
    // =========================================================

    // @Param: AUTO_SPD_EN
    // @DisplayName: Auto speed-band PID scheduling enable
    // @Description: Khi bat (1): so sanh toc do da dat (WP_SPEED/DO_CHANGE_SPEED)
    //   voi AUTO_SPD_MIN/MAX de chon 1 trong 3 bo PID toc do trong Auto Mode:
    //   duoi AUTO_SPD_MIN dung bo AUTO_SPDLO_*, tren AUTO_SPD_MAX dung bo
    //   AUTO_SPDHI_*, con lai (binh thuong) dung ATC_SPEED_* nhu hien tai.
    //   Doi bo PID co do tre AUTO_SPD_DLY de tranh nhay qua lai lien tuc.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(auto_spd_en, "AUTO_SPD_EN", 0),

    // @Param: AUTO_SPD_MIN
    // @DisplayName: Speed-band lower threshold
    // @Description: Nguong duoi (m/s). Toc do da dat thap hon muc nay ->
    //   dung bo PID AUTO_SPDLO_*. Neu AUTO_SPD_MAX <= AUTO_SPD_MIN thi coi
    //   nhu cau hinh khong hop le, giu nguyen ATC_SPEED_* (co canh bao GCS).
    // @Range: 0 50
    // @Units: m/s
    // @User: Standard
    GSCALAR(auto_spd_min, "AUTO_SPD_MIN", 1.0f),

    // @Param: AUTO_SPD_MAX
    // @DisplayName: Speed-band upper threshold
    // @Description: Nguong tren (m/s). Toc do da dat cao hon muc nay ->
    //   dung bo PID AUTO_SPDHI_*. Xem them mo ta AUTO_SPD_MIN.
    // @Range: 0 50
    // @Units: m/s
    // @User: Standard
    GSCALAR(auto_spd_max, "AUTO_SPD_MAX", 1.5f),

    // @Param: AUTO_SPD_DLY
    // @DisplayName: Speed-band switch delay
    // @Description: Toc do da dat phai lien tuc nam trong 1 dai moi (THAP/
    //   BINH THUONG/CAO) it nhat khoang thoi gian nay truoc khi thuc su doi
    //   bo PID - tranh doi qua lai lien tuc khi toc do dao dong sat nguong.
    // @Range: 0 10000
    // @Units: ms
    // @User: Advanced
    GSCALAR(auto_spd_dly, "AUTO_SPD_DLY", 1000),

    // @Param: AUTO_SPDLO_P
    // @DisplayName: Low-speed-band throttle P
    // @Description: He so P bo PID toc do dung khi toc do da dat < AUTO_SPD_MIN.
    // @User: Advanced
    GSCALAR(auto_spdlo_p, "AUTO_SPDLO_P", 0.20f),

    // @Param: AUTO_SPDLO_I
    // @DisplayName: Low-speed-band throttle I
    // @Description: He so I bo PID toc do dung khi toc do da dat < AUTO_SPD_MIN.
    // @User: Advanced
    GSCALAR(auto_spdlo_i, "AUTO_SPDLO_I", 0.20f),

    // @Param: AUTO_SPDLO_D
    // @DisplayName: Low-speed-band throttle D
    // @Description: He so D bo PID toc do dung khi toc do da dat < AUTO_SPD_MIN.
    // @User: Advanced
    GSCALAR(auto_spdlo_d, "AUTO_SPDLO_D", 0.0f),

    // @Param: AUTO_SPDLO_FF
    // @DisplayName: Low-speed-band throttle FF
    // @Description: He so FF bo PID toc do dung khi toc do da dat < AUTO_SPD_MIN.
    // @User: Advanced
    GSCALAR(auto_spdlo_ff, "AUTO_SPDLO_FF", 0.0f),

    // @Param: AUTO_SPDLO_IMAX
    // @DisplayName: Low-speed-band throttle IMAX
    // @Description: Gioi han tich phan bo PID toc do dung khi toc do da dat
    //   < AUTO_SPD_MIN.
    // @Range: 0 1
    // @User: Advanced
    GSCALAR(auto_spdlo_imax, "AUTO_SPDLO_IMAX", 1.0f),

    // @Param: AUTO_SPDHI_P
    // @DisplayName: High-speed-band throttle P
    // @Description: He so P bo PID toc do dung khi toc do da dat > AUTO_SPD_MAX.
    // @User: Advanced
    GSCALAR(auto_spdhi_p, "AUTO_SPDHI_P", 0.20f),

    // @Param: AUTO_SPDHI_I
    // @DisplayName: High-speed-band throttle I
    // @Description: He so I bo PID toc do dung khi toc do da dat > AUTO_SPD_MAX.
    // @User: Advanced
    GSCALAR(auto_spdhi_i, "AUTO_SPDHI_I", 0.20f),

    // @Param: AUTO_SPDHI_D
    // @DisplayName: High-speed-band throttle D
    // @Description: He so D bo PID toc do dung khi toc do da dat > AUTO_SPD_MAX.
    // @User: Advanced
    GSCALAR(auto_spdhi_d, "AUTO_SPDHI_D", 0.0f),

    // @Param: AUTO_SPDHI_FF
    // @DisplayName: High-speed-band throttle FF
    // @Description: He so FF bo PID toc do dung khi toc do da dat > AUTO_SPD_MAX.
    // @User: Advanced
    GSCALAR(auto_spdhi_ff, "AUTO_SPDHI_FF", 0.0f),

    // @Param: AUTO_SPDHI_IMAX
    // @DisplayName: High-speed-band throttle IMAX
    // @Description: Gioi han tich phan bo PID toc do dung khi toc do da dat
    //   > AUTO_SPD_MAX.
    // @Range: 0 1
    // @User: Advanced
    GSCALAR(auto_spdhi_imax, "AUTO_SPDHI_IMAX", 1.0f),

    // @Param: AUTO_STRLO_P
    // @DisplayName: Low-speed-band steering rate P
    // @Description: He so P bo PID lai (steering rate) dung khi toc do da dat
    //   < AUTO_SPD_MIN (dung chung dai voi AUTO_SPDLO_*).
    // @User: Advanced
    GSCALAR(auto_strlo_p, "AUTO_STRLO_P", 0.20f),

    // @Param: AUTO_STRLO_I
    // @DisplayName: Low-speed-band steering rate I
    // @Description: He so I bo PID lai dung khi toc do da dat < AUTO_SPD_MIN.
    // @User: Advanced
    GSCALAR(auto_strlo_i, "AUTO_STRLO_I", 0.20f),

    // @Param: AUTO_STRLO_D
    // @DisplayName: Low-speed-band steering rate D
    // @Description: He so D bo PID lai dung khi toc do da dat < AUTO_SPD_MIN.
    // @User: Advanced
    GSCALAR(auto_strlo_d, "AUTO_STRLO_D", 0.0f),

    // @Param: AUTO_STRLO_FF
    // @DisplayName: Low-speed-band steering rate FF
    // @Description: He so FF bo PID lai dung khi toc do da dat < AUTO_SPD_MIN.
    // @User: Advanced
    GSCALAR(auto_strlo_ff, "AUTO_STRLO_FF", 0.20f),

    // @Param: AUTO_STRLO_IMAX
    // @DisplayName: Low-speed-band steering rate IMAX
    // @Description: Gioi han tich phan bo PID lai dung khi toc do da dat
    //   < AUTO_SPD_MIN.
    // @Range: 0 1
    // @User: Advanced
    GSCALAR(auto_strlo_imax, "AUTO_STRLO_IMAX", 1.0f),

    // @Param: AUTO_STRHI_P
    // @DisplayName: High-speed-band steering rate P
    // @Description: He so P bo PID lai dung khi toc do da dat > AUTO_SPD_MAX
    //   (dung chung dai voi AUTO_SPDHI_*).
    // @User: Advanced
    GSCALAR(auto_strhi_p, "AUTO_STRHI_P", 0.20f),

    // @Param: AUTO_STRHI_I
    // @DisplayName: High-speed-band steering rate I
    // @Description: He so I bo PID lai dung khi toc do da dat > AUTO_SPD_MAX.
    // @User: Advanced
    GSCALAR(auto_strhi_i, "AUTO_STRHI_I", 0.20f),

    // @Param: AUTO_STRHI_D
    // @DisplayName: High-speed-band steering rate D
    // @Description: He so D bo PID lai dung khi toc do da dat > AUTO_SPD_MAX.
    // @User: Advanced
    GSCALAR(auto_strhi_d, "AUTO_STRHI_D", 0.0f),

    // @Param: AUTO_STRHI_FF
    // @DisplayName: High-speed-band steering rate FF
    // @Description: He so FF bo PID lai dung khi toc do da dat > AUTO_SPD_MAX.
    // @User: Advanced
    GSCALAR(auto_strhi_ff, "AUTO_STRHI_FF", 0.20f),

    // @Param: AUTO_STRHI_IMAX
    // @DisplayName: High-speed-band steering rate IMAX
    // @Description: Gioi han tich phan bo PID lai dung khi toc do da dat
    //   > AUTO_SPD_MAX.
    // @Range: 0 1
    // @User: Advanced
    GSCALAR(auto_strhi_imax, "AUTO_STRHI_IMAX", 1.0f),

    // variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass, "COMPASS_", Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

#if AP_RELAY_ENABLED
    // @Group: RELAY
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay, "RELAY", AP_Relay),
#endif

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_", RCMapper),

// SR0 through SR6 were here

// AP_SerialManager was here

#if AP_RANGEFINDER_ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder, "RNGFND", RangeFinder),
#endif

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins, "INS", AP_InertialSensor),

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs, "AHRS_", AP_AHRS),

#if AP_CAMERA_ENABLED
    // @Group: CAM
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera, "CAM", AP_Camera),
#endif

#if AC_PRECLAND_ENABLED
    // @Group: PLND_
    // @Path: ../libraries/AC_PrecLand/AC_PrecLand.cpp
    GOBJECT(precland, "PLND_", AC_PrecLand),
#endif

#if HAL_MOUNT_ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount, "MNT", AP_Mount),
#endif

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming, "ARMING_", AP_Arming),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery, "BATT", AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig, "BRD_", AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr, "CAN_", AP_CANManager),
#endif

    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

#if HAL_NAVEKF2_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECTN(mode_auto.mission, mission, "MIS_", AP_Mission),

#if AP_RSSI_ENABLED
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_", AP_RSSI),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_", AP_Notify),

#if HAL_BUTTON_ENABLED
    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    GOBJECT(button, "BTN_", AP_Button),
#endif

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "", ParametersG2),

#if OSD_ENABLED || OSD_PARAM_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

#if AP_OPTICALFLOW_ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    GOBJECT(optflow, "FLOW", AP_OpticalFlow),
#endif

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

#if HAL_GCS_ENABLED
    // @Group: MAV
    // @Path: ../libraries/GCS_MAVLink/GCS.cpp
    GOBJECT(_gcs, "MAV", GCS),
#endif

    AP_VAREND};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {
    // 1 was AP_Stats

    // 2 was SYSID_ENFORCE

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 3, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 4, ParametersG2, RC_Channels_Rover),

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    AP_SUBGROUPINFO(afs, "AFS_", 5, ParametersG2, AP_AdvancedFailsafe),
#endif

#if AP_BEACON_ENABLED
    // @Group: BCN
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    AP_SUBGROUPINFO(beacon, "BCN", 6, ParametersG2, AP_Beacon),
#endif

    // 7 was used by AP_VisualOdometry

    // @Group: MOT_
    // @Path: ../libraries/AR_Motors/AP_MotorsUGV.cpp
    AP_SUBGROUPINFO(motors, "MOT_", 8, ParametersG2, AP_MotorsUGV),

    // @Group: WENC
    // @Path: ../libraries/AP_WheelEncoder/AP_WheelEncoder.cpp
    AP_SUBGROUPINFO(wheel_encoder, "WENC", 9, ParametersG2, AP_WheelEncoder),

    // @Group: ATC
    // @Path: ../libraries/APM_Control/AR_AttitudeControl.cpp
    AP_SUBGROUPINFO(attitude_control, "ATC", 10, ParametersG2,
                    AR_AttitudeControl),

    // @Param: TURN_RADIUS
    // @DisplayName: Turn radius of vehicle
    // @Description: Turn radius of vehicle in meters while at low speeds. Lower
    // values produce tighter turns in steering mode
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TURN_RADIUS", 11, ParametersG2, turn_radius, 0.9),

    // @Param: ACRO_TURN_RATE
    // @DisplayName: Acro mode turn rate maximum
    // @Description: Acro mode turn rate maximum
    // @Units: deg/s
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_TURN_RATE", 12, ParametersG2, acro_turn_rate, 180.0f),

    // @Group: SRTL_
    // @Path: ../libraries/AP_SmartRTL/AP_SmartRTL.cpp
    AP_SUBGROUPINFO(smart_rtl, "SRTL_", 13, ParametersG2, AP_SmartRTL),

    // 14 was WP_SPEED and should not be re-used

    // @Param: RTL_SPEED
    // @DisplayName: Return-to-Launch speed default
    // @Description: Return-to-Launch speed default.  If zero use WP_SPEED or
    // CRUISE_SPEED.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RTL_SPEED", 15, ParametersG2, rtl_speed, 0.0f),

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Frame Class
    // @Values: 0:Undefined,1:Rover,2:Boat,3:BalanceBot
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 16, ParametersG2, frame_class, 1),

#if HAL_PROXIMITY_ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 18, ParametersG2, AP_Proximity),
#endif

#if AP_AVOIDANCE_ENABLED
    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
    AP_SUBGROUPINFO(avoid, "AVOID_", 19, ParametersG2, AC_Avoid),
#endif

    // 20 was PIVOT_TURN_RATE and should not be re-used

    // @Param: BAL_PITCH_MAX
    // @DisplayName: BalanceBot Maximum Pitch
    // @Description: Pitch angle in degrees at 100% throttle
    // @Units: deg
    // @Range: 0 15
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BAL_PITCH_MAX", 21, ParametersG2, bal_pitch_max, 10),

    // @Param: CRASH_ANGLE
    // @DisplayName: Crash Angle
    // @Description: Pitch/Roll angle limit in degrees for crash check. Zero
    // disables check
    // @Units: deg
    // @Range: 0 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRASH_ANGLE", 22, ParametersG2, crash_angle, 0),

#if AP_FOLLOW_ENABLED
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 23, ParametersG2, AP_Follow),
#endif

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type
    // @Description: Frame Type
    // @Values: 0:Undefined,1:Omni3,2:OmniX,3:OmniPlus,4:Omni3Mecanum
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_TYPE", 24, ParametersG2, frame_type, 0),

    // @Param: LOIT_TYPE
    // @DisplayName: Loiter type
    // @Description: Loiter behaviour when moving to the target point
    // @Values: 0:Forward or reverse to target point,1:Always face bow towards
    // target point,2:Always face stern towards target point
    // @User: Standard
    AP_GROUPINFO("LOIT_TYPE", 25, ParametersG2, loit_type, 0),

#if HAL_SPRAYER_ENABLED
    // @Group: SPRAY_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    AP_SUBGROUPINFO(sprayer, "SPRAY_", 26, ParametersG2, AC_Sprayer),
#endif

    // @Group: WRC
    // @Path: ../libraries/AP_WheelEncoder/AP_WheelRateControl.cpp
    AP_SUBGROUPINFO(wheel_rate_control, "WRC", 27, ParametersG2,
                    AP_WheelRateControl),

#if HAL_RALLY_ENABLED
    // @Group: RALLY_
    // @Path: AP_Rally.cpp,../libraries/AP_Rally/AP_Rally.cpp
    AP_SUBGROUPINFO(rally, "RALLY_", 28, ParametersG2, AP_Rally_Rover),
#endif

    // @Param: SIMPLE_TYPE
    // @DisplayName: Simple_Type
    // @Description: Simple mode types
    // @Values: 0:InitialHeading,1:CardinalDirections
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SIMPLE_TYPE", 29, ParametersG2, simple_type, 0),

    // @Param: LOIT_RADIUS
    // @DisplayName: Loiter radius
    // @Description: Vehicle will drift when within this distance of the target
    // position
    // @Units: m
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOIT_RADIUS", 30, ParametersG2, loit_radius, 2),

    // @Group: WNDVN_
    // @Path: ../libraries/AP_WindVane/AP_WindVane.cpp
    AP_SUBGROUPINFO(windvane, "WNDVN_", 31, ParametersG2, AP_WindVane),

    // 32 to 36 were old sailboat params

    // 37 was airspeed

    // @Param: MIS_DONE_BEHAVE
    // @DisplayName: Mission done behave
    // @Description: Behaviour after mission completes
    // @Values: 0:Hold in Auto Mode,1:Loiter in Auto Mode,2:Acro Mode,3:Manual
    // Mode
    // @User: Standard
    AP_GROUPINFO("MIS_DONE_BEHAVE", 38, ParametersG2, mis_done_behave, 0),

    // 39 was AP_Gripper

    // @Param: BAL_PITCH_TRIM
    // @DisplayName: Balance Bot pitch trim angle
    // @Description: Balance Bot pitch trim for balancing. This offsets the tilt
    // of the center of mass.
    // @Units: deg
    // @Range: -2 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BAL_PITCH_TRIM", 40, ParametersG2, bal_pitch_trim, 0),

    // 41 was Scripting

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // @Description: When enabled, this adds steering user stick input in auto
    // modes, allowing the user to have some degree of control without changing
    // modes.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("STICK_MIXING", 42, ParametersG2, stick_mixing, 0),

    // @Group: WP_
    // @Path: ../libraries/AR_WPNav/AR_WPNav.cpp
    AP_SUBGROUPINFO(wp_nav, "WP_", 43, ParametersG2, AR_WPNav_OA),

    // @Group: SAIL_
    // @Path: sailboat.cpp
    AP_SUBGROUPINFO(sailboat, "SAIL_", 44, ParametersG2, Sailboat),

#if AP_OAPATHPLANNER_ENABLED
    // @Group: OA_
    // @Path: ../libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    AP_SUBGROUPINFO(oa, "OA_", 45, ParametersG2, AP_OAPathPlanner),
#endif

    // @Param: SPEED_MAX
    // @DisplayName: Speed maximum
    // @Description: Maximum speed vehicle can obtain at full throttle. If 0, it
    // will be estimated based on CRUISE_SPEED and CRUISE_THROTTLE.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPEED_MAX", 46, ParametersG2, speed_max, 0.0f),

    // @Param: LOIT_SPEED_GAIN
    // @DisplayName: Loiter speed gain
    // @Description: Determines how aggressively LOITER tries to correct for
    // drift from loiter point. Higher is faster but default should be
    // acceptable.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("LOIT_SPEED_GAIN", 47, ParametersG2, loiter_speed_gain, 0.5f),

    // @Param: FS_OPTIONS
    // @DisplayName: Failsafe Options
    // @Description: Bitmask to enable failsafe options
    // @Bitmask: 0:Failsafe enabled in Hold mode
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 48, ParametersG2, fs_options, 0),

#if HAL_TORQEEDO_ENABLED
    // @Group: TRQ
    // @Path: ../libraries/AP_Torqeedo/AP_Torqeedo.cpp
    AP_SUBGROUPINFO(torqeedo, "TRQ", 49, ParametersG2, AP_Torqeedo),
#endif

    // @Group: PSC
    // @Path: ../libraries/APM_Control/AR_PosControl.cpp
    AP_SUBGROUPINFO(pos_control, "PSC", 51, ParametersG2, AR_PosControl),

    // @Param: GUID_OPTIONS
    // @DisplayName: Guided mode options
    // @Description: Options that can be applied to change guided mode behaviour
    // @Bitmask: 6:SCurves used for navigation
    // @User: Advanced
    AP_GROUPINFO("GUID_OPTIONS", 52, ParametersG2, guided_options, 0),

    // @Param: MANUAL_OPTIONS
    // @DisplayName: Manual mode options
    // @Description: Manual mode specific options
    // @Bitmask: 0:Enable steering speed scaling
    // @User: Advanced
    AP_GROUPINFO("MANUAL_OPTIONS", 53, ParametersG2, manual_options, 0),

#if MODE_DOCK_ENABLED
    // @Group: DOCK
    // @Path: mode_dock.cpp
    AP_SUBGROUPPTR(mode_dock_ptr, "DOCK", 54, ParametersG2, ModeDock),
#endif

    // @Param: MANUAL_STR_EXPO
    // @DisplayName: Manual Steering Expo
    // @Description: Manual steering expo to allow faster steering when stick at
    // edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very
    // High
    // @Range: -0.5 0.95
    // @User: Advanced
    AP_GROUPINFO("MANUAL_STR_EXPO", 55, ParametersG2, manual_steering_expo, 0),

    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FS_GCS_TIMEOUT", 56, ParametersG2, fs_gcs_timeout, 5),

    // @Group: CIRC
    // @Path: mode_circle.cpp
    AP_SUBGROUPINFO(mode_circle, "CIRC", 57, ParametersG2, ModeCircle),

    // [AP_ShoesAgtech] slot 58 — prefix SA_ — flow sensor + spray controller
    // @Group: SA_
    // @Path: ../libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp
    AP_SUBGROUPINFO(custom_nav, "SA_", 58, ParametersG2, AP_ShoesAgtech),
    // [/AP_ShoesAgtech]
    AP_GROUPEND};

// These auxiliary channel param descriptions are here so that users of beta
// Mission Planner (which uses the master branch as its source of descriptions)
// can get them.  These lines can be removed once Rover-3.6-beta testing begins
// or we improve the source of descriptions for GCSs.
//
// @Param: CH7_OPTION
// @DisplayName: Channel 7 option
// @Description: What to do use channel 7 for
// @Values:
// 0:Nothing,1:SaveWaypoint,2:LearnCruiseSpeed,3:ArmDisarm,4:Manual,5:Acro,6:Steering,7:Hold,8:Auto,9:RTL,10:SmartRTL,11:Guided,12:Loiter
// @User: Standard

// @Param: AUX_CH
// @DisplayName: Auxiliary switch channel
// @Description: RC Channel to use for auxiliary functions including saving
// waypoints
// @User: Advanced

// @Param: PIVOT_TURN_ANGLE
// @DisplayName: Pivot turn angle
// @Description: Navigation angle threshold in degrees to switch to pivot
// steering. This allows you to setup a skid steering rover to turn on the spot
// in auto mode when the angle it needs to turn it greater than this angle. An
// angle of zero means to disable pivot turning. Note that you will probably
// also want to set a low value for WP_RADIUS to get neat turns.
// @Units: deg
// @Range: 0 360
// @Increment: 1
// @User: Standard

// @Param: PIVOT_TURN_RATE
// @DisplayName: Pivot turn rate
// @Description: Desired pivot turn rate in deg/s.
// @Units: deg/s
// @Range: 0 360
// @Increment: 1
// @User: Standard

ParametersG2::ParametersG2(void)
    :
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
      afs(),
#endif
#if AP_BEACON_ENABLED
      beacon(),
#endif
      wheel_rate_control(wheel_encoder), motors(wheel_rate_control),
      attitude_control(), smart_rtl(),
#if HAL_PROXIMITY_ENABLED
      proximity(),
#endif
#if MODE_DOCK_ENABLED
      mode_dock_ptr(&rover.mode_dock),
#endif
#if AP_AVOIDANCE_ENABLED
      avoid(),
#endif
#if AP_FOLLOW_ENABLED
      follow(),
#endif
      windvane(), wp_nav(attitude_control, pos_control), sailboat(),
      pos_control(attitude_control) {
  AP_Param::setup_object_defaults(this, var_info);
}

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.

  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed

  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
const AP_Param::ConversionInfo conversion_table[] = {
    {Parameters::k_param_battery_monitoring, 0, AP_PARAM_INT8, "BATT_MONITOR"},
    {Parameters::k_param_battery_volt_pin, 0, AP_PARAM_INT8, "BATT_VOLT_PIN"},
    {Parameters::k_param_battery_curr_pin, 0, AP_PARAM_INT8, "BATT_CURR_PIN"},
    {Parameters::k_param_volt_div_ratio, 0, AP_PARAM_FLOAT, "BATT_VOLT_MULT"},
    {Parameters::k_param_curr_amp_per_volt, 0, AP_PARAM_FLOAT,
     "BATT_AMP_PERVOLT"},
    {Parameters::k_param_pack_capacity, 0, AP_PARAM_INT32, "BATT_CAPACITY"},
    {Parameters::k_param_serial0_baud, 0, AP_PARAM_INT16, "SERIAL0_BAUD"},
    {Parameters::k_param_serial1_baud, 0, AP_PARAM_INT16, "SERIAL1_BAUD"},
    {Parameters::k_param_serial2_baud, 0, AP_PARAM_INT16, "SERIAL2_BAUD"},
    {Parameters::k_param_throttle_min_old, 0, AP_PARAM_INT8, "MOT_THR_MIN"},
    {Parameters::k_param_throttle_max_old, 0, AP_PARAM_INT8, "MOT_THR_MAX"},
    {Parameters::k_param_compass_enabled_deprecated, 0, AP_PARAM_INT8,
     "COMPASS_ENABLE"},
    {Parameters::k_param_waypoint_radius_old, 0, AP_PARAM_FLOAT, "WP_RADIUS"},
    {Parameters::k_param_g2, 299, AP_PARAM_INT16, "WP_PIVOT_ANGLE"},
    {Parameters::k_param_g2, 363, AP_PARAM_INT16, "WP_PIVOT_RATE"},
    {Parameters::k_param_g2, 491, AP_PARAM_FLOAT, "WP_PIVOT_DELAY"},
    {Parameters::k_param_g2, 32, AP_PARAM_FLOAT, "SAIL_ANGLE_MIN"},
    {Parameters::k_param_g2, 33, AP_PARAM_FLOAT, "SAIL_ANGLE_MAX"},
    {Parameters::k_param_g2, 34, AP_PARAM_FLOAT, "SAIL_ANGLE_IDEAL"},
    {Parameters::k_param_g2, 35, AP_PARAM_FLOAT, "SAIL_HEEL_MAX"},
    {Parameters::k_param_g2, 36, AP_PARAM_FLOAT, "SAIL_NO_GO_ANGLE"},
    {Parameters::k_param_arming, 2, AP_PARAM_INT16, "ARMING_CHECK"},
    {Parameters::k_param_turn_max_g_old, 0, AP_PARAM_FLOAT, "ATC_TURN_MAX_G"},
    {Parameters::k_param_g2, 82, AP_PARAM_INT8, "PRX1_TYPE"},
    {Parameters::k_param_g2, 146, AP_PARAM_INT8, "PRX1_ORIENT"},
    {Parameters::k_param_g2, 210, AP_PARAM_INT16, "PRX1_YAW_CORR"},
    {Parameters::k_param_g2, 274, AP_PARAM_INT16, "PRX1_IGN_ANG1"},
    {Parameters::k_param_g2, 338, AP_PARAM_INT8, "PRX1_IGN_WID1"},
    {Parameters::k_param_g2, 402, AP_PARAM_INT16, "PRX1_IGN_ANG2"},
    {Parameters::k_param_g2, 466, AP_PARAM_INT8, "PRX1_IGN_WID2"},
    {Parameters::k_param_g2, 530, AP_PARAM_INT16, "PRX1_IGN_ANG3"},
    {Parameters::k_param_g2, 594, AP_PARAM_INT8, "PRX1_IGN_WID3"},
    {Parameters::k_param_g2, 658, AP_PARAM_INT16, "PRX1_IGN_ANG4"},
    {Parameters::k_param_g2, 722, AP_PARAM_INT8, "PRX1_IGN_WID4"},
    {Parameters::k_param_g2, 1234, AP_PARAM_FLOAT, "PRX1_MIN"},
    {Parameters::k_param_g2, 1298, AP_PARAM_FLOAT, "PRX1_MAX"},
    {Parameters::k_param_g2, 113, AP_PARAM_INT8, "TRQ1_TYPE"},
    {Parameters::k_param_g2, 177, AP_PARAM_INT8, "TRQ1_ONOFF_PIN"},
    {Parameters::k_param_g2, 241, AP_PARAM_INT8, "TRQ1_DE_PIN"},
    {Parameters::k_param_g2, 305, AP_PARAM_INT16, "TRQ1_OPTIONS"},
    {Parameters::k_param_g2, 369, AP_PARAM_INT8, "TRQ1_POWER"},
    {Parameters::k_param_g2, 433, AP_PARAM_FLOAT, "TRQ1_SLEW_TIME"},
    {Parameters::k_param_g2, 497, AP_PARAM_FLOAT, "TRQ1_DIR_DELAY"},
};

void Rover::load_parameters(void) {
  AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

  AP_Param::convert_old_parameters(&conversion_table[0],
                                   ARRAY_SIZE(conversion_table));

  AP_Param::set_frame_type_flags(AP_PARAM_FRAME_ROVER);

  SRV_Channels::set_default_function(CH_1, SRV_Channel::k_steering);
  SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttle);

  if (is_balancebot()) {
    g2.crash_angle.set_default(30);
  }

  SRV_Channels::upgrade_parameters();

  // convert CH7_OPTION to RC7_OPTION for Rover-3.4 to 3.5 upgrade
  const AP_Param::ConversionInfo ch7_option_info = {
      Parameters::k_param_ch7_option, 0, AP_PARAM_INT8, "RC7_OPTION"};
  AP_Int8 ch7_opt_old;
  if (AP_Param::find_old_parameter(&ch7_option_info, &ch7_opt_old)) {
    const uint8_t ch7_opt_map[] = {0,  7,  50, 41, 51, 52, 53,
                                   54, 16, 4,  42, 55, 56};
    const uint8_t ch7_opt_old_val = (uint8_t)ch7_opt_old.get();
    if (ch7_opt_old_val < ARRAY_SIZE(ch7_opt_map)) {
      AP_Param::set_default_by_name(ch7_option_info.new_name,
                                    ch7_opt_map[ch7_opt_old_val]);
    }
  }

  // set AR_WPNav's WP_SPEED to be old WP_SPEED (if set) or CRUISE_SPEED (if
  // set)
  const AP_Param::ConversionInfo wp_speed_old_info = {
      Parameters::k_param_g2, 14, AP_PARAM_FLOAT, "WP_SPEED"};
  const AP_Param::ConversionInfo cruise_speed_info = {
      Parameters::k_param_speed_cruise, 0, AP_PARAM_FLOAT, "WP_SPEED"};
  AP_Float wp_speed_old;
  if (AP_Param::find_old_parameter(&wp_speed_old_info, &wp_speed_old)) {
    // old WP_SPEED parameter value was set so copy to new WP_SPEED
    AP_Param::convert_old_parameter(&wp_speed_old_info, 1.0f);
  } else {
    // copy CRUISE_SPEED to new WP_SPEED
    AP_Param::convert_old_parameter(&cruise_speed_info, 1.0f);
  }

  // attitude control FF and FILT parameter changes for Rover-3.6
  const AP_Param::ConversionInfo ff_and_filt_conversion_info[] = {
      {Parameters::k_param_g2, 24650, AP_PARAM_FLOAT, "ATC_STR_RAT_FLTE"},
      {Parameters::k_param_g2, 28746, AP_PARAM_FLOAT, "ATC_STR_RAT_FF"},
      {Parameters::k_param_g2, 24714, AP_PARAM_FLOAT, "ATC_SPEED_FLTE"},
      {Parameters::k_param_g2, 28810, AP_PARAM_FLOAT, "ATC_SPEED_FF"},
      {Parameters::k_param_g2, 25226, AP_PARAM_FLOAT, "ATC_BAL_FLTE"},
      {Parameters::k_param_g2, 29322, AP_PARAM_FLOAT, "ATC_BAL_FF"},
      {Parameters::k_param_g2, 25354, AP_PARAM_FLOAT, "ATC_SAIL_FLTE"},
      {Parameters::k_param_g2, 29450, AP_PARAM_FLOAT, "ATC_SAIL_FF"},
  };
  AP_Param::convert_old_parameters(&ff_and_filt_conversion_info[0],
                                   ARRAY_SIZE(ff_and_filt_conversion_info));

  // configure safety switch to allow stopping the motors while armed
#if HAL_HAVE_SAFETY_SWITCH
  AP_Param::set_default_by_name(
      "BRD_SAFETYOPTION",
      AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF |
          AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON |
          AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED);
#endif

#if AP_AIRSPEED_ENABLED | AP_AIS_ENABLED | AP_FENCE_ENABLED
  // Find G2's Top Level Key
  AP_Param::ConversionInfo info;
  if (!AP_Param::find_top_level_key_by_pointer(&g2, info.old_key)) {
    return;
  }
#endif

  static const AP_Param::G2ObjectConversion g2_conversions[]{
#if AP_AIRSPEED_ENABLED
      // PARAMETER_CONVERSION - Added: JAN-2022
      {&airspeed, airspeed.var_info, 37},
#endif
#if AP_AIS_ENABLED
      // PARAMETER_CONVERSION - Added: MAR-2022
      {&ais, ais.var_info, 50},
#endif
#if AP_FENCE_ENABLED
      // PARAMETER_CONVERSION - Added: Mar-2022
      {&fence, fence.var_info, 17},
#endif
#if AP_STATS_ENABLED
      // PARAMETER_CONVERSION - Added: Jan-2024 for Rover-4.6
      {&stats, stats.var_info, 1},
#endif
#if AP_SCRIPTING_ENABLED
      // PARAMETER_CONVERSION - Added: Jan-2024 for Rover-4.6
      {&scripting, scripting.var_info, 41},
#endif
#if AP_GRIPPER_ENABLED
      // PARAMETER_CONVERSION - Added: Feb-2024 for Copter-4.6
      {&gripper, gripper.var_info, 39},
#endif
  };

  AP_Param::convert_g2_objects(&g2, g2_conversions, ARRAY_SIZE(g2_conversions));

  // PARAMETER_CONVERSION - Added: Feb-2024 for Rover-4.6
#if HAL_LOGGING_ENABLED
  AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, true);
#endif

  // PARAMETER_CONVERSION - Added: July-2025 for ArduPilot-4.7
#if AP_RPM_ENABLED
  AP_Param::convert_class(g.k_param_rpm_sensor_old, &rpm_sensor,
                          rpm_sensor.var_info, 0, true, true);
#endif

  static const AP_Param::TopLevelObjectConversion toplevel_conversions[]{
#if AP_SERIALMANAGER_ENABLED
      // PARAMETER_CONVERSION - Added: Feb-2024 for Rover-4.6
      {&serial_manager, serial_manager.var_info,
       Parameters::k_param_serial_manager_old},
#endif
  };

  AP_Param::convert_toplevel_objects(toplevel_conversions,
                                     ARRAY_SIZE(toplevel_conversions));

#if HAL_GCS_ENABLED
  // Move parameters into new MAV_ parameter namespace
  // PARAMETER_CONVERSION - Added: Mar-2025 for ArduPilot-4.7
  {
    static const AP_Param::ConversionInfo gcs_conversion_info[]{
        {Parameters::k_param_sysid_this_mav_old, 0, AP_PARAM_INT16,
         "MAV_SYSID"},
        {Parameters::k_param_sysid_my_gcs_old, 0, AP_PARAM_INT16,
         "MAV_GCS_SYSID"},
        {Parameters::k_param_g2, 2, AP_PARAM_INT8, "MAV_OPTIONS"},
        {Parameters::k_param_telem_delay_old, 0, AP_PARAM_INT8,
         "MAV_TELEM_DELAY"},
    };
    AP_Param::convert_old_parameters(&gcs_conversion_info[0],
                                     ARRAY_SIZE(gcs_conversion_info));
  }
#endif // HAL_GCS_ENABLED
}
