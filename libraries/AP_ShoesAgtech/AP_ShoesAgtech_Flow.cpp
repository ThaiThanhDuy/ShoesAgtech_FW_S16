// =============================================================
// MODULE 1 — CẢM BIẾN LƯU LƯỢNG + ĐIỀU KHIỂN PHUN (Bơm)
// Tách ra từ AP_ShoesAgtech.cpp (2026-09-07) — chỉ di chuyển vị trí định
// nghĩa hàm sang file riêng, KHÔNG đổi bất kỳ logic nào. Toàn bộ hàm ở
// đây vẫn là method của class AP_ShoesAgtech (khai báo đầy đủ trong
// AP_ShoesAgtech.h), linker gộp chung như một file duy nhất.
// =============================================================
#include "AP_ShoesAgtech.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

// =============================================================
// BẢNG THAM SỐ MODULE 1 (Bơm) — var_info RIÊNG, subgroup lồng trong
// AP_ShoesAgtech (xem AP_SUBGROUPINFO trong AP_ShoesAgtech.cpp), tiền
// tố rỗng nên tên tham số không đổi so với trước (vd vẫn SA_FLOW_MODE).
// =============================================================
const AP_Param::GroupInfo AP_ShoesAgtech_FlowParams::var_info[] = {
    // @Param: CAL_FAC
    // @DisplayName: Flow sensor calibration factor (pulses/L)
    // @Description: Pulses per litre for YF-S402B. Operating range 0.3-6 L/min.
    // @User: Standard
    AP_GROUPINFO("CAL_FAC", 1, AP_ShoesAgtech_FlowParams, cal_factor, 3874.5f),

    // @Param: EMA_AL
    // @DisplayName: Flow EMA smoothing alpha (0.01-1.0)
    // @Description: EMA alpha applied to raw flow rate. Lower = smoother.
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("EMA_AL", 2, AP_ShoesAgtech_FlowParams, ema_alpha, 0.1f),

    // @Param: FLOW_LOG
    // @DisplayName: Flow console log enable
    // @Description: Prints spray mode, flow target/actual/avg and pump PWM at
    //   SA_LOG_FL_MS interval.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FLOW_LOG", 3, AP_ShoesAgtech_FlowParams, flow_log_enable, 0),

    // @Param: RC_CHAN
    // @DisplayName: RC channel for spray mode switch (1-indexed)
    // @Description: PWM<1300=mode0, 1300-1700=mode1, >1700=mode2.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_CHAN", 4, AP_ShoesAgtech_FlowParams, rc_chan, 6),

    // @Param: RC_PUMP
    // @DisplayName: RC channel for manual pump passthrough in mode 0
    // (1-indexed)
    // @Description: In mode 0, this channel is read and written directly to
    //   SA_PUMP_CHAN (software passthrough). Requires SERVOx_FUNCTION=0(None).
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_PUMP", 5, AP_ShoesAgtech_FlowParams, rc_pump, 9),

    // @Param: PUMP_CHAN
    // @DisplayName: Pump servo output channel (1-indexed)
    // @Description: Requires SERVOx_FUNCTION=0(None). Mode 0: RC passthrough.
    //   Modes 1/2: PID-controlled.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("PUMP_CHAN", 6, AP_ShoesAgtech_FlowParams, pump_chan, 8),

    // @Param: FLOW_SP
    // @DisplayName: Flow setpoint (L/min) for mode 1 FLOW_MODE=0
    // @Range: 0 200
    // @User: Standard
    AP_GROUPINFO("FLOW_SP", 7, AP_ShoesAgtech_FlowParams, flow_setpoint, 5.0f),

    // @Param: PID_P
    // @DisplayName: Flow PID P gain (us per L/min error)
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("PID_P", 8, AP_ShoesAgtech_FlowParams, pid_p, 80.0f),

    // @Param: PID_I
    // @DisplayName: Flow PID I gain (us per L/min/s)
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("PID_I", 9, AP_ShoesAgtech_FlowParams, pid_i, 20.0f),

    // @Param: PID_LPF
    // @DisplayName: Flow PID output LPF alpha (0.01=smooth, 1.0=raw)
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("PID_LPF", 10, AP_ShoesAgtech_FlowParams, pid_lpf, 0.3f),

    // @Param: LOG_FL_MS
    // @DisplayName: Flow console log interval (ms)
    // @Description: Interval between flow console prints when SA_FLOW_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("LOG_FL_MS", 11, AP_ShoesAgtech_FlowParams, flow_log_ms, 1000),

    // @Param: FLOW_PIN
    // @DisplayName: Flow sensor GPIO pin number
    // @Description: GPIO pin connected to YF-S402B signal wire. Default 55
    //   (Pixhawk/CubeOrange AUX). Change to match hardware.
    // @Range: 1 200
    // @User: Standard
    AP_GROUPINFO("FLOW_PIN", 12, AP_ShoesAgtech_FlowParams, flow_pin, 55),

    // @Param: TANK_VOL
    // @DisplayName: Chemical tank volume (L)
    // @Description: Used in FLOW_MODE=1 to compute flow target and in mode 2
    //   to estimate remaining spray distance. Set 0 to disable both.
    // @Units: L
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("TANK_VOL", 13, AP_ShoesAgtech_FlowParams, tank_vol, 0.0f),

    // @Param: FLOW_MODE
    // @DisplayName: Flow target source in mode 1
    // @Description: 0=use SA_FLOW_SP directly. 1=compute from tank/mission:
    //   flow=(TANK_VOL*speed*60)/mission_dist, falls back to SA_FLOW_SP if
    //   unset.
    // @Values: 0:DirectSetpoint,1:TankMissionFormula
    // @User: Standard
    AP_GROUPINFO("FLOW_MODE", 14, AP_ShoesAgtech_FlowParams, flow_mode, 0),

    // @Param: MIX_STD
    // @DisplayName: Spray ratio at mid RC position (standard nozzle)
    // @Description: Biocide fraction of total flow at mid RC. In FLOW_MODE=1
    //   used directly; in FLOW_MODE=0 the setpoint is SA_FLOW_SP.
    // @Range: 0.01 1.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("MIX_STD", 15, AP_ShoesAgtech_FlowParams, mix_std, 0.35f),

    // @Param: MIX_CNT
    // @DisplayName: Spray ratio at high RC position (anti-clog nozzle)
    // @Description: Biocide fraction at high RC. In FLOW_MODE=0, flow target
    //   is scaled by MIX_CNT/MIX_STD to keep boom output consistent.
    // @Range: 0.01 1.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("MIX_CNT", 16, AP_ShoesAgtech_FlowParams, mix_cnt, 0.50f),

    // @Param: FLOW_VEL
    // @DisplayName: Override ground speed for FLOW_MODE=1 (m/s)
    // @Description: 0=use real GPS/AHRS speed. >0=force this speed for flow
    //   target calibration while stationary. Ignored when SA_SIM=1.
    // @Range: 0 10
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("FLOW_VEL", 17, AP_ShoesAgtech_FlowParams, flow_vel, 0.0f),

    AP_GROUPEND};

// =============================================================
// XỬ LÝ LƯU LƯỢNG + ĐIỀU KHIỂN PHUN — gọi từ update() mỗi chu kỳ 10Hz
// (tách từ update() thành hàm riêng 2026-09-07, logic giữ nguyên 100%)
// Cảm biến lưu lượng: YF-S402B, dãy hoạt động 0.3–6 L/min, GPIO pin 55
// =============================================================
void AP_ShoesAgtech::_update_flow(void) {
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
      float cal = (_flow_params.cal_factor.get() > 0.0f) ? _flow_params.cal_factor.get() : 3874.5f;
      float raw = (dt > 0.0f) ? ((float)pulses / cal) * (60.0f / dt) : 0.0f;

      float alpha = constrain_float(_flow_params.ema_alpha.get(), 0.01f, 1.0f);
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

  // Khi disarm: reset cảnh báo + cache mission + bộ phát hiện hết thùng +
  // toàn bộ trạng thái đọc lưu lượng (tránh "treo" giá trị cũ từ phiên
  // trước sang phiên chạy mới — vd dòng chảy dư do trọng lực trong lúc
  // disarm vẫn có thể tạo xung, nếu không reset _last_pulse_snapshot thì
  // lần arm kế tiếp sẽ cộng dồn hết số xung tích luỹ trong lúc disarm
  // thành 1 cú lưu lượng ảo tăng vọt ở chu kỳ đầu tiên).
  if (!now_armed) {
    _mission_ncmds = 0;
    _mission_dist_m = 0.0f;
    _tank_empty_detected = false;
    _tank_empty_ms = 0;
    _no_mission_warned = false;
    _flow_ramp_val = 0.0f;

    // Chỉ reset trạng thái đọc CẢM BIẾN THẬT (pulse/lưu lượng) khi KHÔNG
    // ở SA_SIM=1 (2026-09-07) — lý do reset (dòng chảy dư do trọng lực
    // tạo xung ảo lúc disarm) chỉ xảy ra với cảm biến thật; ở SIM,
    // _run_simulation() đã ghi thẳng dữ liệu giả vào _flow_rate_filtered
    // ngay đầu update() mỗi chu kỳ, nếu vẫn ép về 0 ở đây thì SA_DATA/GCS
    // luôn thấy 0.00 dù đang mô phỏng và không cần ARM để xem thử.
    if (_simulation.get() <= 0) {
      _last_pulse_snapshot = _pulse_count;
      _flow_rate_filtered = 0.0f;
      _flow_rate_avg = 0.0f;
      _buffer_sum = 0.0f;
      _buffer_index = 0;
      _samples_count = 0;
      for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
        _sample_buffer[i] = 0.0f;
      }
    }
  }

  switch (_spray_mode) {

  case 0: {
    // ---- MODE 0: TRUYỀN THẲNG PHẦN MỀM ----
    uint8_t rc_pump_idx = (uint8_t)constrain_int16(_flow_params.rc_pump.get() - 1, 0, 15);
    uint16_t rc_pwm = RC_Channels::get_radio_in(rc_pump_idx);
    _flow_target = 0.0f;
    _flow_ramp_val = 0.0f;
    _flow_out_of_range = false;
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (rc_pwm < 800 || rc_pwm > 2200) {
      // Chưa có tín hiệu RC hợp lệ (vd: chưa cắm/kết nối tay cầm) -> đưa
      // bơm về đúng vị trí AN TOÀN đã cấu hình (SERVOx_MIN), KHÔNG dùng
      // giá trị 1500 cứng vì có thể không phải là mức tắt bơm thực tế.
      SRV_Channel *ch0 =
          SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
      if (ch0 != nullptr) {
        _pump_pwm = ch0->get_output_min();
        _write_pump_pwm(_pump_pwm);
      }
      break;
    }
    _pump_pwm = rc_pwm;
    _write_pump_pwm(_pump_pwm);
    break;
  }

  case 1: {
    // ---- MODE 1: FLOW PID (nấc giữa — MIX_STD / béc mặc định) ----
    _flow_out_of_range = false;
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _flow_ramp_val = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch1 =
          SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
      if (ch1 != nullptr) {
        _write_pump_pwm(ch1->get_output_min());
      }
      break;
    }
    if (_flow_params.flow_mode.get() == 1 && _flow_params.tank_vol.get() > 0.0f) {
      _flow_target = _compute_visin_target(_flow_params.mix_std.get());
      if (_flow_target < 0.01f) {
        _flow_ramp_val = 0.0f;
        SRV_Channel *ch1 =
            SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
        if (ch1 != nullptr) {
          _write_pump_pwm(ch1->get_output_min());
        }
        break;
      }
      // q1 ngoài dải lưu lượng THẬT bơm đạt được (0.8-1.3) — KHÔNG ép
      // _flow_target về 0 nữa (2026-09-04): log định kỳ vẫn hiện đúng q1
      // thật + " - out range", chỉ có bơm là tắt (không chạy PID).
      if (_flow_target < 0.8f || _flow_target > 1.3f) {
        _flow_out_of_range = true;
        _flow_ramp_val = 0.0f;
        _pid_integral = 0.0f;
        _pid_output_lpf = 0.0f;
        SRV_Channel *ch1 =
            SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
        if (ch1 != nullptr) {
          _write_pump_pwm(ch1->get_output_min());
        }
        break;
      }
      // Bơm chỉ thực sự chạy khi tốc độ đã đạt đủ % tốc độ ĐẶT cho mission
      // (giống hệt cơ chế SA_SPD_START bên Module 3) - không bật ngay
      // lúc xe còn đứng yên/mới nhích bánh. _flow_target vẫn hiện đầy đủ
      // trong log như bình thường. SA_FLOW_VEL đã tự nằm trong
      // _get_dosing_ref_speed() nên không cần bypass riêng - đặt SA_FLOW_VEL
      // đủ lớn để hiệu chỉnh đứng yên là qua được ngưỡng này bình thường.
      if (_get_dosing_ref_speed() < _speed_min_start()) {
        _flow_ramp_val = 0.0f;
        _pid_integral = 0.0f;
        _pid_output_lpf = 0.0f;
        SRV_Channel *ch1 =
            SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
        if (ch1 != nullptr) {
          _write_pump_pwm(ch1->get_output_min());
        }
        break;
      }
      // Ramp từ từ lên _flow_target (0.3 L/min mỗi giây) thay vì nhảy
      // thẳng full setpoint ngay khi vừa vào WP1 - giảm bơm giật/tràn
      // lúc mới mồi (bồn đặt cao hơn bơm, không van một chiều).
      _flow_ramp_val = MIN(_flow_ramp_val + 0.3f * dt_pid, _flow_target);
    } else {
      _flow_target = _flow_params.flow_setpoint.get();
      _flow_ramp_val = MIN(_flow_ramp_val + 0.3f * dt_pid, _flow_target);
    }
    _pump_pwm = _run_flow_pid(_flow_ramp_val, dt_pid);
    _write_pump_pwm(_pump_pwm);
    break;
  }

  case 2: {
    // ---- MODE 2: FLOW PID (nấc cao — MIX_CNT / béc chống nghẹt) ----
    _flow_out_of_range = false;
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _flow_ramp_val = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch2 =
          SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
      if (ch2 != nullptr) {
        _write_pump_pwm(ch2->get_output_min());
      }
      break;
    }
    if (_flow_params.flow_mode.get() == 1 && _flow_params.tank_vol.get() > 0.0f) {
      _flow_target = _compute_visin_target(_flow_params.mix_cnt.get());
      if (_flow_target < 0.01f) {
        _flow_ramp_val = 0.0f;
        SRV_Channel *ch2 =
            SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
        if (ch2 != nullptr) {
          _write_pump_pwm(ch2->get_output_min());
        }
        break;
      }
      // Giống nấc 2: q1 ngoài dải 0.8-1.3 thì không ép _flow_target về 0.
      if (_flow_target < 0.8f || _flow_target > 1.3f) {
        _flow_out_of_range = true;
        _flow_ramp_val = 0.0f;
        _pid_integral = 0.0f;
        _pid_output_lpf = 0.0f;
        SRV_Channel *ch2 =
            SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
        if (ch2 != nullptr) {
          _write_pump_pwm(ch2->get_output_min());
        }
        break;
      }
      // Giống nấc 2: chỉ bật bơm thật khi tốc độ đã đạt đủ % tốc độ ĐẶT.
      if (_get_dosing_ref_speed() < _speed_min_start()) {
        _flow_ramp_val = 0.0f;
        _pid_integral = 0.0f;
        _pid_output_lpf = 0.0f;
        SRV_Channel *ch2 =
            SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
        if (ch2 != nullptr) {
          _write_pump_pwm(ch2->get_output_min());
        }
        break;
      }
      _flow_ramp_val = MIN(_flow_ramp_val + 0.3f * dt_pid, _flow_target);
    } else {
      float ratio =
          (_flow_params.mix_std.get() > 0.01f) ? (_flow_params.mix_cnt.get() / _flow_params.mix_std.get()) : 1.0f;
      _flow_target =
          constrain_float(_flow_params.flow_setpoint.get() * ratio, 0.0f, 200.0f);
      _flow_ramp_val = MIN(_flow_ramp_val + 0.3f * dt_pid, _flow_target);
    }
    _pump_pwm = _run_flow_pid(_flow_ramp_val, dt_pid);
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
        } else if (now - _tank_empty_ms >= 10000U) {
          _tank_empty_detected = true;
          gcs().send_text(MAV_SEVERITY_INFO,
                          "SA: TANK EMPTY - flow %.1fL/min > 1.7 for 5s",
                          (double)_flow_rate_filtered);
        }
      } else {
        _tank_empty_ms = 0;
      }
    }
  }

  // ---- 5. IN LOG LƯU LƯỢNG RA CONSOLE (SA_FLOW_LOG) ----
  // Rút gọn: chỉ 1 dòng "FM<x> N<nấc> Q:<setpoint>[ - out range]".
  //   FM<x>   = SA_FLOW_MODE (0 hoặc 1) — cách tính setpoint đang dùng.
  //   N<nấc>  = spray_mode+1 (1/2/3, khớp đúng vị trí gạt nấc RC vật lý).
  //   Q       = _flow_target: 0 ở nấc 1 (truyền thẳng tay), số cố định ở
  //             FM0 (SA_FLOW_SP), số q1 THẬT đã tính ở FM1 (công thức
  //             thùng+mission, chỉ có ý nghĩa khi đang ở nấc 2/3) — LUÔN
  //             hiện đúng số dù q1 ngoài dải bơm hay chưa đạt tốc độ %,
  //             KHÔNG ép về 0 nữa (2026-09-04).
  //   " - out range" = thêm vào cuối khi q1 (FM1) đang ngoài dải 0.8-1.3
  //             (_flow_out_of_range) — bơm KHÔNG chạy trong trường hợp
  //             này dù Q vẫn hiện số thật, thay cho cảnh báo riêng lúc ARM.
  if (_flow_params.flow_log_enable.get() > 0 &&
      now - _last_log_ms >= (uint32_t)_flow_params.flow_log_ms.get()) {
    _last_log_ms = now;
    const char *flow_pfx = (_simulation.get() > 0) ? "[SIM][FLOW]" : "[FLOW]";
    gcs().send_text(MAV_SEVERITY_INFO, "%s FM%d N%u Q: %.2f%s", flow_pfx,
                    (int)_flow_params.flow_mode.get(), (unsigned)(_spray_mode + 1),
                    (double)_flow_target,
                    _flow_out_of_range ? " - out range" : "");
  }
}

// =============================================================
// KIỂM TRA CẤU HÌNH BƠM
// Chạy trong mỗi update(). In MIN/TRIM/MAX của servo ở lần boot
// đầu hoặc khi PUMP_CHAN đổi. Cảnh báo mỗi 5s nếu function != 0.
// =============================================================
void AP_ShoesAgtech::_check_pump_config(void) {
  int8_t chan = _flow_params.pump_chan.get();
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

  _pump_config_ok = (func_val == (int32_t)SRV_Channel::k_none);
}

// RC → CHẾ ĐỘ PHUN
//   Nấc 1 (PWM < 1300) : mode 0 — passthrough
//   Nấc 2 (1300-1700)  : mode 1 — FLOW PID, tỉ lệ SA_MIX_STD
//   Nấc 3 (PWM > 1700) : mode 2 — FLOW PID, tỉ lệ SA_MIX_CNT
void AP_ShoesAgtech::_update_spray_mode(void) {
  uint8_t idx = (uint8_t)constrain_int16(_flow_params.rc_chan.get() - 1, 0, 15);
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
  SRV_Channel *ch = SRV_Channels::srv_channel((uint8_t)(_flow_params.pump_chan.get() - 1));
  uint16_t pwm_min = (ch != nullptr) ? ch->get_output_min() : 1000;
  uint16_t pwm_max = (ch != nullptr) ? ch->get_output_max() : 2000;
  uint16_t pwm_trim = (ch != nullptr) ? ch->get_trim() : 1500;

  float error = target_lmin - _flow_rate_filtered;
  float i_gain = _flow_params.pid_i.get();

  _pid_integral += error * dt;
  if (i_gain > 0.0f) {
    float ilimit = (pwm_max - pwm_min) * 0.5f / i_gain;
    _pid_integral = constrain_float(_pid_integral, -ilimit, ilimit);
  }

  float pid_raw = _flow_params.pid_p.get() * error + i_gain * _pid_integral;

  float alpha = constrain_float(_flow_params.pid_lpf.get(), 0.01f, 1.0f);
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
  uint8_t chan_idx = (uint8_t)constrain_int16(_flow_params.pump_chan.get() - 1, 0, 15);
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
  float vel = _flow_params.flow_vel.get();
  if (vel > 0.0f) {
    return vel;
  }
  return AP::ahrs().groundspeed();
}

// =============================================================
// TỐC ĐỘ THAM CHIẾU CHO CÔNG THỨC FLOW_MODE=1 — ưu tiên: SA_SIM >
// SA_FLOW_VEL > tốc độ ĐẶT cho mission (_target_speed, = WP_SPEED đã cập
// nhật qua DO_CHANGE_SPEED/GCS SET_SPEED, do Rover.cpp bơm vào qua
// set_target_speed() mỗi chu kỳ) > AHRS groundspeed (dự phòng).
//
// KHÁC với _get_spray_speed(): ưu tiên tốc độ ĐẶT thay vì tốc độ GPS TỨC
// THỜI — để q1 (và do đó setpoint bơm) không bị dao động theo từng cú
// tăng/giảm tốc, vào cua của xe (gây phun không đều dọc tuyến), chỉ đổi
// khi tốc độ ĐẶT cho mission thực sự đổi. Chỉ dùng cho
// _compute_visin_target()/log FM định kỳ — nơi khác (DOS_MODE=2 Module 3)
// vẫn dùng _get_spray_speed() (tốc độ thực) như cũ, không đổi.
//
// TẦNG DỰ PHÒNG AHRS (2026-08-20): _target_speed CHỈ được gán giá trị
// thật khi đã vào chế độ AUTO ít nhất 1 lần kể từ lúc mở nguồn
// (AR_WPNav::init() chỉ chạy trong ModeAuto::_enter()) — nếu vehicle chưa
// từng vào AUTO (vd chỉ ARM ở Manual để test bằng tay/gạt nấc),
// _target_speed sẽ luôn = 0, khiến FLOW_MODE=1 tưởng xe đứng yên mãi mãi
// dù xe đang chạy thật. Do đó nếu _target_speed vẫn đang = 0 (chưa từng
// được thiết lập), quay lại dùng AHRS groundspeed như cũ để hệ thống vẫn
// hoạt động được ngoài AUTO.
// =============================================================
float AP_ShoesAgtech::_get_dosing_ref_speed(void) {
  if (_simulation.get() > 0) {
    return _sim_speed;
  }
  float vel = _flow_params.flow_vel.get();
  if (vel > 0.0f) {
    return vel;
  }
  if (_target_speed > 0.0f) {
    return _target_speed;
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
//   1. Mission: dist <= 1m → cảnh báo 1 lần/ARM, dừng, return 0 (không có
//      q1 nào để hiện - mission chưa hợp lệ nên không tính được)
//   2. Speed:   < 0.1 m/s → reset PID, dừng, return 0 (tương tự, xe coi
//      như đứng yên, không có nghĩa để hiện 1 con số q1 nào)
//   3. Range:   q1 < 0.8 hoặc q1 > 1.3 (DẢI LƯU LƯỢNG THẬT bơm hiện tại
//      đạt được, đo thực tế 2026-08-20, chỉnh lại 2026-09-04 — KHÔNG phải
//      ngưỡng nghiệp vụ) — KHÔNG ép về 0 nữa (2026-09-04): hàm này LUÔN
//      trả về q1 thật đã tính (constrain 0-200) để log vẫn hiện đúng số,
//      caller (case 1/2 trong update()) mới là nơi quyết định có chạy
//      PID/ghi PWM ra bơm hay không dựa trên dải này (_flow_out_of_range).
// =============================================================
float AP_ShoesAgtech::_compute_visin_target(float r) {
  r = constrain_float(r, 0.01f, 1.0f);

  float speed_ms = _get_dosing_ref_speed();
  float dist = _get_mission_dist();

  if (dist <= 1.0f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (!_no_mission_warned) {
      _no_mission_warned = true;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA FM1: no mission - pump stopped");
    }
    return 0.0f;
  }

  if (speed_ms < 0.1f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    return 0.0f;
  }

  float q1 = _flow_params.tank_vol.get() * r * speed_ms * 60.0f / dist;

  return constrain_float(q1, 0.0f, 200.0f);
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

  // index 0 luôn là HOME (AP_Mission::read_cmd_from_storage() trả về
  // AP::ahrs().get_home() cho index 0, không phải WP1 thật) -> bỏ qua,
  // bắt đầu cộng dồn quãng đường từ WP1 (index 1) để không tính lố thêm
  // chặng "home -> WP1" vào tổng quãng đường mission.
  for (uint16_t i = 1; i < n; i++) {
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
// NGƯỠNG TỐC ĐỘ TỐI THIỂU ĐỂ BẮT ĐẦU BƠM/RẢI — FLOW_MODE=1 (Module 1) và
// DOS_MODE=2 (Module 3) DÙNG CHUNG tham số SA_SPD_START (không còn slot
// AP_Param trống để thêm tham số riêng cho Module 1, xem mục param) —
// đổi SA_SPD_START sẽ ảnh hưởng ngưỡng bắt đầu chạy của CẢ 2 module.
// SA_SPD_START<=0: tắt hẳn kiểm tra %, chỉ còn sàn tuyệt đối 0.05 m/s.
// SA_SPD_START>0: ngưỡng = %  × _target_speed (tốc độ ĐẶT cho mission),
// tối thiểu vẫn là 0.05 m/s.
// =============================================================
float AP_ShoesAgtech::_speed_min_start(void) {
  int8_t spd_pct_raw = _spd_start_pct.get();
  if (spd_pct_raw <= 0) {
    return 0.05f;
  }
  float spd_pct = constrain_float((float)spd_pct_raw, 1.0f, 100.0f);
  return MAX(0.05f, (spd_pct * 0.01f) * _target_speed);
}
