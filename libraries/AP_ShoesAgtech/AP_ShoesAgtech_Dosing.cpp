// =============================================================
// Tách ra từ AP_ShoesAgtech.cpp (2026-09-07) — chỉ di chuyển vị trí định
// nghĩa hàm sang file riêng, KHÔNG đổi bất kỳ logic nào. Toàn bộ hàm ở
// đây vẫn là method của class AP_ShoesAgtech (khai báo đầy đủ trong
// AP_ShoesAgtech.h), linker gộp chung như một file duy nhất.
// =============================================================
#include "AP_ShoesAgtech.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

// =============================================================
// BẢNG THAM SỐ MODULE 3 (Cho ăn) — var_info RIÊNG, subgroup lồng trong
// AP_ShoesAgtech (xem AP_SUBGROUPINFO trong AP_ShoesAgtech.cpp), tiền
// tố rỗng nên tên tham số không đổi so với trước (vd vẫn SA_DOS_MODE).
// =============================================================
const AP_Param::GroupInfo AP_ShoesAgtech_DosingParams::var_info[] = {
    // @Param: DOS_CHAN
    // @DisplayName: Dosing motor servo output channel (1-indexed)
    // @Description: Requires SERVOx_FUNCTION=0(None), MIN=800, TRIM=1500,
    //   MAX=2200 before the motor is allowed to run.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_CHAN", 1, AP_ShoesAgtech_DosingParams, dos_chan, 10),

    // @Param: DOS_RC
    // @DisplayName: RC channel to toggle dosing motor on/off (1-indexed)
    // @Description: PWM>1500 runs motor; PWM<=1500 stops (outputs 1500).
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_RC", 2, AP_ShoesAgtech_DosingParams, dos_rc, 8),

    // @Param: DOS_SP
    // @DisplayName: Dosing setpoint (meaning depends on SA_DOS_MODE)
    // @Description: Meaning changes with SA_DOS_MODE: MODE=0 -> raw PWM
    //   (us) written directly to the servo, no rate/formula involved.
    //   MODE=1 -> continuous feed rate (g/min). MODE=2 -> total grams for
    //   the whole mission route. Value is per active pond (POND_IDX): each
    //   pond keeps its own value, switching ponds loads that pond's stored
    //   setpoint here; editing this saves back to the active pond.
    // @User: Standard
    AP_GROUPINFO("DOS_SP", 3, AP_ShoesAgtech_DosingParams, dos_sp, 0.0f),

    // @Param: DOS_REV
    // @DisplayName: Dosing motor rotation direction
    // @Description: 0=forward: PWM 800-1500 (800=fastest). 1=reverse: PWM
    //   1500-2200. Motor stops at 1500 in both directions.
    // @Values: 0:Forward (800-1500),1:Reverse (1500-2200)
    // @User: Standard
    AP_GROUPINFO("DOS_REV", 4, AP_ShoesAgtech_DosingParams, dos_rev, 0),

    // @Param: DOS_LOG
    // @DisplayName: Dosing motor console log enable
    // @Description: Prints motor state, setpoint and PWM at SA_DOS_LOG_MS
    //   interval when enabled.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DOS_LOG", 5, AP_ShoesAgtech_DosingParams, dos_log_enable, 0),

    // @Param: DOS_LOG_MS
    // @DisplayName: Dosing motor console log interval (ms)
    // @Description: Interval between dosing console prints when SA_DOS_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("DOS_LOG_MS", 6, AP_ShoesAgtech_DosingParams, dos_log_ms, 1000),

    // @Param: DOS_MODE
    // @DisplayName: Dosing motor speed mode
    // @Description: 0=direct PWM: SA_DOS_SP (us) written straight to the
    //   servo, bypassing SA_DOS_V/Fx/Dx and SA_DOS_REV entirely -- for
    //   bench calibration (measuring RPM/output at a known PWM) without
    //   needing a separate servo-output test tool. 1=fixed speed from
    //   DOS_SP/DOS_Fx/DOS_Dx. 2=spreads DOS_SP evenly over the mission
    //   route by ground speed. Stops if no mission or speed too low.
    // @Values: 0:DirectPWM,1:Fixed,2:MissionProportional
    // @User: Standard
    AP_GROUPINFO("DOS_MODE", 7, AP_ShoesAgtech_DosingParams, dos_mode, 0),

    // @Param: DOS_FOOD
    // @DisplayName: Active food type selector (1-7) for the active pond
    // @Description: Selects SA_DOS_Fx (fill factor) and SA_DOS_Dx (bulk
    //   density) used with SA_DOS_V to compute the PWM offset. Each pond
    //   keeps its own value: switching ponds (POND_IDX) loads that pond's
    //   stored food type here; editing this saves back to the active pond.
    // @Range: 1 7
    // @User: Standard
    AP_GROUPINFO("DOS_FOOD", 8, AP_ShoesAgtech_DosingParams, dos_food, 1),

    // @Param: DOS_V
    // @DisplayName: Auger volumetric constant (shared, mL/50us at 100% fill)
    // @Description: Theoretical volumetric output of the auger screw per 50us
    //   PWM offset at full (100%) fill — pure geometry (screw diameter/pitch)
    //   plus the PWM-offset-to-speed mapping. Shared across ALL food types:
    //   it depends only on the physical screw, not on what is flowing
    //   through it. Change this ONLY when the physical auger is replaced
    //   with a different size. Effective volumetric rate used in the dosing
    //   formula = SA_DOS_V x SA_DOS_Fx (fill factor of the active food type).
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_V", 9, AP_ShoesAgtech_DosingParams, dos_v, 100.0f),

    // Fill-factor calibration. Effective volumetric rate used in the dosing
    // formula = SA_DOS_V x SA_DOS_Fx. Default 1.0 keeps a freshly-flashed/
    // uncalibrated unit numerically identical to the old single-number
    // SA_DOS_Fx default (100.0 = SA_DOS_V default x 1.0).

    // @Param: DOS_F1
    // @DisplayName: Auger fill factor for food type 1 (relative to SA_DOS_V)
    // @Description: Packing/fill-factor coefficient for food type 1, relative
    //   to the auger's physical volumetric constant SA_DOS_V (effective rate
    //   = SA_DOS_V x SA_DOS_F1). Typically 0.3-1.0: smaller/rounder grains
    //   pack tighter (closer to 1.0), larger/irregular grains leave more air
    //   gap (lower). Calibrate by running the motor, weighing actual output,
    //   and solving SA_DOS_F1 = measured_effective_rate / SA_DOS_V.
    // @Range: 0.05 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_F1", 10, AP_ShoesAgtech_DosingParams, dos_fr[0], 1.0f),

    // @Param: DOS_F2
    // @DisplayName: Auger fill factor for food type 2 (relative to SA_DOS_V)
    // @Range: 0.05 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_F2", 11, AP_ShoesAgtech_DosingParams, dos_fr[1], 1.0f),

    // @Param: DOS_F3
    // @DisplayName: Auger fill factor for food type 3 (relative to SA_DOS_V)
    // @Range: 0.05 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_F3", 12, AP_ShoesAgtech_DosingParams, dos_fr[2], 1.0f),

    // @Param: DOS_F4
    // @DisplayName: Auger fill factor for food type 4 (relative to SA_DOS_V)
    // @Range: 0.05 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_F4", 13, AP_ShoesAgtech_DosingParams, dos_fr[3], 1.0f),

    // @Param: DOS_F5
    // @DisplayName: Auger fill factor for food type 5 (relative to SA_DOS_V)
    // @Range: 0.05 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_F5", 14, AP_ShoesAgtech_DosingParams, dos_fr[4], 1.0f),

    // @Param: DOS_F6
    // @DisplayName: Auger fill factor for food type 6 (relative to SA_DOS_V)
    // @Range: 0.05 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_F6", 15, AP_ShoesAgtech_DosingParams, dos_fr[5], 1.0f),

    // @Param: DOS_F7
    // @DisplayName: Auger fill factor for food type 7 (relative to SA_DOS_V)
    // @Range: 0.05 2.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_F7", 16, AP_ShoesAgtech_DosingParams, dos_fr[6], 1.0f),

    // Bulk density per food type.
    // offset(us) = SP(g) * 50 / (SA_DOS_V(mL/50us) x fill_factor(-) x
    // density(g/mL)). Default 1.0 g/mL is backward-compatible with the old
    // g/50us formula.

    // @Param: DOS_D1
    // @DisplayName: Bulk density food type 1 (g/mL)
    // @Description: Bulk density of food type 1. Measure: weigh 1L of food,
    //   divide by 1000 to get g/mL.
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D1", 17, AP_ShoesAgtech_DosingParams, dos_dr[0], 1.0f),

    // @Param: DOS_D2
    // @DisplayName: Bulk density food type 2 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D2", 18, AP_ShoesAgtech_DosingParams, dos_dr[1], 1.0f),

    // @Param: DOS_D3
    // @DisplayName: Bulk density food type 3 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D3", 19, AP_ShoesAgtech_DosingParams, dos_dr[2], 1.0f),

    // @Param: DOS_D4
    // @DisplayName: Bulk density food type 4 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D4", 20, AP_ShoesAgtech_DosingParams, dos_dr[3], 1.0f),

    // @Param: DOS_D5
    // @DisplayName: Bulk density food type 5 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D5", 21, AP_ShoesAgtech_DosingParams, dos_dr[4], 1.0f),

    // @Param: DOS_D6
    // @DisplayName: Bulk density food type 6 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D6", 22, AP_ShoesAgtech_DosingParams, dos_dr[5], 1.0f),

    // @Param: DOS_D7
    // @DisplayName: Bulk density food type 7 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D7", 23, AP_ShoesAgtech_DosingParams, dos_dr[6], 1.0f),

    AP_GROUPEND};

// =============================================================
// MODULE 3 — MOTOR CHO ĂN (DOSING)
// vít tải thức ăn tôm, servo xoay liên tục 360°
//
// RC SA_DOS_RC bật/tắt: PWM > 1500 -> bật, PWM <= 1500 -> tắt (xuất 1500).
//
// Quy đổi lượng thức ăn (SA_DOS_SP, gam) -> độ lệch PWM. Lưu lượng thể
// tích hiệu dụng của vít tải = SA_DOS_V(mL/50us, hằng số hình học DÙNG
// CHUNG cho mọi loại thức ăn, chỉ đổi khi thay trục vít) x SA_DOS_Fx
// (hệ số điền đầy hạt, không thứ nguyên, ~0.05-2.0, RIÊNG theo từng loại
// thức ăn — hạt to/tròn/trơn khác nhau lấp khoảng trống trong vít khác
// nhau). SA_DOS_Dx (g/mL) chọn theo SA_DOS_FOOD, dùng chung cho cả 2 mode:
//   DOS_MODE=0 (toc do co dinh):
//     offset = SA_DOS_SP * 50 / (SA_DOS_V x SA_DOS_Fx x SA_DOS_Dx(g/mL))
//   DOS_MODE=1 (phan bo deu theo mission):
//     dos_gpm = SA_DOS_SP * speed * 60 / mission_dist
//     offset  = dos_gpm * 50 / (SA_DOS_V x SA_DOS_Fx x SA_DOS_Dx(g/mL))
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
  uint8_t chan_idx = (uint8_t)constrain_int16(_dos_params.dos_chan.get() - 1, 0, 15);
  SRV_Channel *ch = SRV_Channels::srv_channel(chan_idx);
  int32_t func_val = (int32_t)SRV_Channels::channel_function(chan_idx);
  int32_t chan = (int32_t)_dos_params.dos_chan.get();

  bool have_chan = (ch != nullptr);
  bool func_ok = have_chan && (func_val == (int32_t)SRV_Channel::k_none);
  bool min_ok = have_chan && (ch->get_output_min() == 800);
  bool trim_ok = have_chan && (ch->get_trim() == 1500);
  bool max_ok = have_chan && (ch->get_output_max() == 2200);

  _dos_config_ok = func_ok && min_ok && trim_ok && max_ok;

  if (_dos_config_ok) {
    if (!_dos_was_ok) {
      gcs().send_text(MAV_SEVERITY_INFO,
                      "SA: SERVO%d setup OK - dosing motor ready", (int)chan);
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
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d does not exist",
                    (int)chan);
    return;
  }
  if (!func_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA: SERVO%d FUNCTION=%d, must set =0 (None)", (int)chan,
                    (int)func_val);
  }
  if (!min_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MIN=%u, must set =800",
                    (int)chan, (unsigned)ch->get_output_min());
  }
  if (!trim_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d TRIM=%u, must set =1500",
                    (int)chan, (unsigned)ch->get_trim());
  }
  if (!max_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MAX=%u, must set =2200",
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
  if (_dos_params.dos_rev.get() == 0) {
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
    _dos_params.dos_sp.set(pond.dos_sp);
    _dos_sp_sync_val = pond.dos_sp;
    _dos_params.dos_food.set(pond.dos_food);
    _dos_food_sync_val = pond.dos_food;
    _dos_sync_pond = _active_pond_idx;
    return;
  }

  float cur_sp = _dos_params.dos_sp.get();
  if (fabsf(cur_sp - _dos_sp_sync_val) > 0.001f) {
    pond.dos_sp = cur_sp;
    _dos_sp_sync_val = cur_sp;
    _ponds_dirty = true;
  }

  int8_t cur_food = _clamp_food((int8_t)_dos_params.dos_food.get());
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
      active_pond.valid ? active_pond.dos_sp : _dos_params.dos_sp.get();
  const int8_t dos_food_active =
      active_pond.valid ? active_pond.dos_food : (int8_t)_dos_params.dos_food.get();

  _check_dosing_config();
  if (!_dos_config_ok) {
    _dos_pwm = 1500;
    return;
  }

  uint32_t now = AP_HAL::millis();
  uint8_t rc_idx = (uint8_t)constrain_int16(_dos_params.dos_rc.get() - 1, 0, 15);
  uint16_t rc_pwm = RC_Channels::get_radio_in(rc_idx);
  bool motor_on = (rc_pwm > 1500);

  // Chặn motor tự chạy lại nếu FC vừa reboot (vd mất điện chập chờn) trong
  // lúc switch SA_DOS_RC vẫn đang ở vị trí ON từ trước - _dos_rc_seen_off
  // reset về false mỗi lần boot, chỉ thành true khi thấy switch thực sự
  // ở OFF ít nhất 1 lần. Trước khi đó, ép motor_on=false dù rc_pwm>1500.
  if (!_dos_rc_seen_off) {
    if (rc_pwm > 0 && rc_pwm <= 1500) {
      _dos_rc_seen_off = true;
    } else {
      motor_on = false;
    }
  }

  if (motor_on != _dos_was_on) {
    _dos_was_on = motor_on;
    gcs().send_text(MAV_SEVERITY_INFO, "SA: Dosing motor %s",
                    motor_on ? "ON" : "OFF");
  }

  float dos_rate_gpm = 0.0f; // tốc độ cấp tức thời (g/phút) — dùng để in log

  if (motor_on) {
    float pwm_f = 1500.0f;

    if (_dos_params.dos_mode.get() == 0) {
      // ---- DOS_MODE 0: PWM trực tiếp — SA_DOS_SP LÀ xung PWM (µs), xuất
      // thẳng ra servo. Bỏ qua hoàn toàn công thức SA_DOS_V/Fx/Dx và
      // SA_DOS_REV (không phải offset, là giá trị tuyệt đối) — dùng để
      // hiệu chuẩn tại bàn (đo RPM/sản lượng ở 1 mức PWM biết trước) mà
      // không cần công cụ test servo riêng của GCS.
      // An toàn: SA_DOS_SP=0 (giá trị mặc định, chưa từng chỉnh) -> dừng
      // (1500), KHÔNG kẹp về pwm_min (có thể là tốc độ tối đa) — tránh
      // trường hợp mới bật RC dosing mà quên set SA_DOS_SP thì motor chạy
      // full tốc ngoài ý muốn.
      if (dos_sp_active <= 0.0f) {
        pwm_f = 1500.0f;
      } else {
        SRV_Channel *ch_dos =
            SRV_Channels::srv_channel((uint8_t)(_dos_params.dos_chan.get() - 1));
        uint16_t pwm_min = (ch_dos != nullptr) ? ch_dos->get_output_min() : 800;
        uint16_t pwm_max =
            (ch_dos != nullptr) ? ch_dos->get_output_max() : 2200;
        pwm_f = constrain_float(dos_sp_active, (float)pwm_min, (float)pwm_max);
      }
      dos_rate_gpm = 0.0f; // không có khái niệm tốc độ g/phút ở mode này
    } else {
      // Lưu lượng thể tích hiệu dụng của vít tải = SA_DOS_V (hằng số hình
      // học, DÙNG CHUNG mọi loại thức ăn — chỉ đổi khi thay trục vít khác)
      // x SA_DOS_Fx (hệ số điền đầy hạt, RIÊNG theo loại thức ăn đang
      // active — bù cho khoảng trống không khí giữa các hạt trong vít).
      // Mật độ SA_DOS_Dx cũng lấy theo loại thức ăn đang active, dùng
      // chung cho cả mode 1 và 2.
      uint8_t food_idx = (uint8_t)_clamp_food(dos_food_active) - 1;
      float fill_k = _dos_params.dos_fr[food_idx].get();
      if (fill_k < 0.01f) {
        fill_k = 0.01f;
      }
      // SA_DOS_Fx trước đây (trước khi tách ra SA_DOS_V x SA_DOS_Fx) là một
      // số ở thang mL/50us, thường cỡ ~100 — nếu máy đã hiệu chuẩn từ trước
      // và chưa đo lại theo công thức mới, SA_DOS_Fx sẽ vẫn còn giá trị lớn
      // kiểu này, bị hiểu nhầm thành hệ số điền đầy => cho ăn sai (thường là
      // quá ít). Cảnh báo rate-limit 5s để kỹ thuật viên biết cần hiệu
      // chuẩn lại SA_DOS_Fx sau khi cập nhật firmware.
      if (fill_k > 5.0f && now - _dos_warn_ms >= 5000U) {
        _dos_warn_ms = now;
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "SA: SA_DOS_F%d=%.1f looks uncalibrated for new V x "
                        "fill-factor formula (expected ~0.05-2.0)",
                        (int)(food_idx + 1), (double)fill_k);
      }
      float v_const = _dos_params.dos_v.get();
      if (v_const < 0.1f) {
        v_const = 0.1f;
      }
      float vol_rate = v_const * fill_k;
      float density = _dos_params.dos_dr[food_idx].get();
      if (density < 0.01f) {
        density = 0.01f;
      }
      float effective = vol_rate * density;

      if (_dos_params.dos_mode.get() == 1) {
        // ---- DOS_MODE 1 (đổi số từ mode 0 cũ, 2026-08-20): tốc độ cố
        // định — SA_DOS_SP CHÍNH LÀ tốc độ (g/phút) ----
        dos_rate_gpm = dos_sp_active;
        float offset = dos_rate_gpm * 50.0f / effective;
        pwm_f = (float)_offset_to_dos_pwm(offset);
      } else {
        // ---- DOS_MODE 2 (đổi số từ mode 1 cũ, 2026-08-20): phân bố đều
        // theo mission — SA_DOS_SP là TỔNG gam, dos_rate_gpm là tốc độ
        // tức thời suy ra từ speed/mission_dist ----
        float mission_dist = _get_mission_dist();
        float speed_ms = _get_spray_speed(); // đồng bộ nguồn tốc độ với
                                             // Module 1 (SIM > SA_FLOW_VEL
                                             // > AHRS)
        // Chỉ bắt đầu rải khi xe đã đạt ĐỦ tốc độ — không phải cứ nhích
        // bánh là rải. Ngưỡng = SA_SPD_START % tốc độ ĐẶT cho mission
        // (_target_speed = WP_SPEED, do Rover.cpp bơm vào — xem
        // set_target_speed()), tránh rải ngay lúc xe còn đang tăng tốc từ
        // lúc dừng/qua khúc cua (rải dồn vào đoạn xe đi chậm). Vẫn giữ
        // sàn tuyệt đối 0.05 m/s cho trường hợp chưa có _target_speed (vd
        // chưa từng vào Auto). SA_SPD_START=0 tắt hẳn kiểm tra %, quay
        // về hành vi cũ (chỉ cần vượt sàn 0.05 m/s là rải). Từ 2026-09-04,
        // `_speed_min_start()` DÙNG CHUNG với gate bật bơm FLOW_MODE=1 ở
        // Module 1 — đổi SA_SPD_START ảnh hưởng cả 2 module.
        float speed_min_start = _speed_min_start(); // dùng chung với Module 1
        if (mission_dist > 1.0f && speed_ms >= speed_min_start) {
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
                  "SA DOS2: no mission (dist=%.1fm) - motor stopped",
                  (double)mission_dist);
            } else {
              gcs().send_text(MAV_SEVERITY_WARNING,
                              "SA DOS2: speed too low (%.2fm/s < %.2fm/s min) "
                              "- motor stopped",
                              (double)speed_ms, (double)speed_min_start);
            }
          }
        }
      }
    }
    _dos_pwm = (uint16_t)pwm_f;
  } else {
    _dos_pwm = 1500;
  }

  uint8_t chan_idx = (uint8_t)constrain_int16(_dos_params.dos_chan.get() - 1, 0, 15);
  SRV_Channels::set_output_pwm_chan(chan_idx, _dos_pwm);

  // ---- IN LOG MOTOR CHO ĂN RA CONSOLE (SA_DOS_LOG) ----
  // Rút gọn (2026-09-04), giống hệt kiểu Module 1: đúng 1 dòng
  // "[DOS] FM<x> Q:<y>" — FM<x> = SA_DOS_MODE (0/1/2), Q = dos_rate_gpm
  // (lưu lượng g/phút; = 0 ở mode 0 vì không có khái niệm tốc độ, giống
  // cách Q=0 ở nấc manual bên Module 1). Bỏ SERVO/ON-OFF/PWM/mật độ khỏi
  // log định kỳ để đồng nhất với Module 1 — xem SA_DATA nếu cần chi tiết.
  if (_dos_params.dos_log_enable.get() > 0) {
    if (now - _dos_last_log_ms >= (uint32_t)_dos_params.dos_log_ms.get()) {
      _dos_last_log_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO, "[DOS] FM%d Q:%.2f",
                      (int)_dos_params.dos_mode.get(), (double)dos_rate_gpm);
    }
  }
}

