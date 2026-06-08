#include "AP_ShoesAgtech.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

volatile uint32_t AP_ShoesAgtech::_pulse_count = 0;

const AP_Param::GroupInfo AP_ShoesAgtech::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_ShoesAgtech, _enable_flag, 1),

    // @Param: CAL_FAC
    // @DisplayName: Flow sensor calibration (pulses/Litre) — YF-S402B
    // @Description: Số xung trên mỗi lít của cảm biến YF-S402B. Dãy hoạt động:
    // 0.3–6 L/min.
    // @User: Standard
    AP_GROUPINFO("CAL_FAC", 2, AP_ShoesAgtech, _cal_factor, 3874.5f),

    // @Param: EMA_AL
    // @DisplayName: Flow EMA smoothing alpha (0.01-1.0)
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("EMA_AL", 3, AP_ShoesAgtech, _ema_alpha, 0.1f),

    // @Param: FLOW_LOG
    // @DisplayName: Flow sensor console log enable
    // @Description: Print spray mode, flow target/actual/avg and pump PWM every
    // 1s.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FLOW_LOG", 4, AP_ShoesAgtech, _flow_log_enable, 0),

    // @Param: RC_CHAN
    // @DisplayName: RC channel for spray mode switch (1-indexed)
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_CHAN", 5, AP_ShoesAgtech, _rc_chan, 6),

    // @Param: RC_PUMP
    // @DisplayName: RC channel for manual pump control in mode 0 (1-indexed)
    // @Description: Set SERVOx_FUNCTION=0(None) on the pump channel. In mode 0
    //   this library reads SA_RC_PUMP and writes it to SA_PUMP_CHAN directly,
    //   giving identical behaviour to RC passthrough without blocking modes
    //   1/2.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_PUMP", 6, AP_ShoesAgtech, _rc_pump, 9),

    // @Param: PUMP_CHAN
    // @DisplayName: Servo output channel for pump (1-indexed)
    // @Description: Must have SERVOx_FUNCTION=0 (None). This library controls
    //   it in all modes: mode 0 passes RC_PUMP through, modes 1/2 use PID.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("PUMP_CHAN", 7, AP_ShoesAgtech, _pump_chan, 8),

    // @Param: FLOW_SP
    // @DisplayName: Flow setpoint L/min (mode 1)
    // @Range: 0 200
    // @User: Standard
    AP_GROUPINFO("FLOW_SP", 8, AP_ShoesAgtech, _flow_setpoint, 5.0f),

    // @Param: PID_P
    // @DisplayName: PID P gain (us per L/min error)
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("PID_P", 9, AP_ShoesAgtech, _pid_p, 80.0f),

    // @Param: PID_I
    // @DisplayName: PID I gain (us per L/min/s)
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("PID_I", 10, AP_ShoesAgtech, _pid_i, 20.0f),

    // @Param: PID_LPF
    // @DisplayName: PID output LPF alpha (0.01=smooth, 1.0=raw)
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("PID_LPF", 11, AP_ShoesAgtech, _pid_lpf, 0.3f),

    // @Param: APP_RATE
    // @DisplayName: Application rate L/ha (mode 2)
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("APP_RATE", 12, AP_ShoesAgtech, _app_rate, 100.0f),

    // @Param: BOOM_W
    // @DisplayName: Boom width in metres (mode 2)
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("BOOM_W", 13, AP_ShoesAgtech, _boom_width, 1.0f),

    // [AP_ShoesAgtech] pH sensor parameters — Nengshi ASPS3801D-0.5M (slots
    // 14-19)

    // @Param: PH_EN
    // @DisplayName: Enable pH sensor
    // @Description: Enable Nengshi ASPS3801D-0.5M pH sensor via Modbus RTU
    // (RS485-TTL)
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_EN", 14, AP_ShoesAgtech, _ph_en, 0),

    // @Param: PH_PORT
    // @DisplayName: UART port for pH sensor (matches SERIALx number)
    // @Description: Set to match the SERIALx port connected to the RS485-TTL
    // module.
    //   Set SERIALx_BAUD=9 (9600) and SERIALx_PROTOCOL=0 (None) on that port.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("PH_PORT", 15, AP_ShoesAgtech, _ph_port, 2),

    // @Param: PH_TOFF
    // @DisplayName: Temperature offset (°C)
    // @Description: Added to the raw sensor temperature reading after /10
    // decode.
    // @Range: -10 10
    // @User: Standard
    AP_GROUPINFO("PH_TOFF", 16, AP_ShoesAgtech, _ph_toff, -3.5f),

    // @Param: PH_OFF
    // @DisplayName: pH calibration offset
    // @Description: Added to decoded pH value. Use buffer solution to
    // determine.
    // @Range: -2.0 2.0
    // @User: Standard
    AP_GROUPINFO("PH_OFF", 17, AP_ShoesAgtech, _ph_off, 0.0f),

    // @Param: PH_KH
    // @DisplayName: Base alkalinity dKH
    // @Description: Reference alkalinity measured by test kit (dKH). Used as
    // base
    //   for the alkalinity estimation algorithm. Update when you test the pond.
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("PH_KH", 18, AP_ShoesAgtech, _ph_kh, 4.0f),

    // @Param: PH_EMA
    // @DisplayName: pH EMA smoothing alpha (0.01-1.0)
    // @Description: Exponential Moving Average coefficient. Lower = smoother
    // but slower.
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("PH_EMA", 19, AP_ShoesAgtech, _ph_ema_alpha, 0.15f),

    // @Param: PH_LOG
    // @DisplayName: pH sensor console log enable (independent of SA_LOG_EN)
    // @Description: SA_LOG_EN controls flow/spray console log. SA_PH_LOG
    // controls
    //   pH sensor console log separately. Both can be enabled at the same time.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_LOG", 20, AP_ShoesAgtech, _ph_log_enable, 0),

    // @Param: PH_TZ
    // @DisplayName: Timezone offset (hours, UTC+N)
    // @Description: Local time = UTC + PH_TZ. Vietnam is UTC+7 (default). Used
    // to
    //   classify pH readings into morning (05:00-11:59) and afternoon
    //   (12:00-16:59)
    //   slots for daily ΔpH-based alkalinity estimation.
    // @Range: -12 14
    // @User: Standard
    AP_GROUPINFO("PH_TZ", 21, AP_ShoesAgtech, _ph_tz, 7),

    // @Param: LOG_FL_MS
    // @DisplayName: Flow console log interval (ms)
    // @Description: Khoảng thời gian giữa hai lần in dữ liệu lưu lượng ra
    // console khi SA_FLOW_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("LOG_FL_MS", 22, AP_ShoesAgtech, _flow_log_ms, 1000),

    // @Param: LOG_PH_MS
    // @DisplayName: pH console log interval (ms)
    // @Description: Khoảng thời gian giữa hai lần in dữ liệu pH/nhiệt độ/kiềm
    // ra console khi SA_PH_LOG=1.
    //   Không nên đặt nhỏ hơn chu kỳ Modbus (2000ms) vì sensor chỉ trả dữ liệu
    //   mỗi 2s.
    // @Range: 500 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("LOG_PH_MS", 23, AP_ShoesAgtech, _ph_log_ms, 2000),

    // @Param: PH_TIMEOUT
    // @DisplayName: pH disconnection timeout (s)
    // @Description: Số giây không nhận được frame pH hợp lệ thì coi là
    //   "mat ket noi" - phát cảnh báo qua STATUSTEXT và reset các giá trị
    //   pH/nhiệt độ/kiềm trong gói SA_DATA (DEBUG_FLOAT_ARRAY) về 0.
    // @Range: 1 300
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("PH_TIMEOUT", 24, AP_ShoesAgtech, _ph_timeout, 2),

    // [/AP_ShoesAgtech]

    AP_GROUPEND};

AP_ShoesAgtech::AP_ShoesAgtech()
    : _last_timestamp_ms(0), _last_pulse_snapshot(0), _last_log_ms(0),
      _flow_rate_filtered(0.0f), _flow_rate_avg(0.0f), _is_initialized(false),
      _buffer_index(0), _buffer_sum(0.0f), _samples_count(0), _spray_mode(0),
      _pump_pwm(0), _flow_target(0.0f), _pid_integral(0.0f),
      _pid_output_lpf(0.0f), _pid_last_ms(0), _last_pump_chan(-1),
      _last_pump_func_val(-1), _pump_config_ok(false), _last_warn_ms(0),
      // [AP_ShoesAgtech] pH sensor initial state
      _ph_uart(nullptr), _ph_update_ms(0), _ph_req_sent_ms(0),
      _ph_req_pending(false), _ph_last_good_ms(0), _ph_nodata_warn_ms(0),
      _ph_last_log_ms(0), _ph_value(0.0f), _ph_value_ema(-1.0f),
      _ph_value_ma(0.0f), _ph_mv(0), _ph_temp(25.0f), _alk_dkh(0.0f),
      _alk_mgl(0.0f), _ph_buf_idx(0), _ph_buf_count(0), _ph_buf_sum(0.0f),
      // daily slot tracking
      _ph_morn_val(0.0f), _ph_aft_val(0.0f), _ph_morn_valid(false),
      _ph_aft_valid(false), _delta_ph(0.0f), _alk_today_dkh(0.0f),
      _alk_today_mgl(0.0f), _alk_prev_dkh(0.0f), _alk_prev_mgl(0.0f),
      _alk_slot_status(4), _rtc_last_day(0), _slot_warn_ms(0)
// [/AP_ShoesAgtech]
{
  memset(_sample_buffer, 0, sizeof(_sample_buffer));
  // [AP_ShoesAgtech]
  memset(_ph_buf, 0, sizeof(_ph_buf));
  // [/AP_ShoesAgtech]
  AP_Param::setup_object_defaults(this, var_info);
}

void AP_ShoesAgtech::init(void) {
  if (!is_enabled()) {
    return;
  }

  hal.gpio->pinMode(55, HAL_GPIO_INPUT);

  if (!hal.gpio->attach_interrupt(55, irq_handler,
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

  // [AP_ShoesAgtech] init pH sensor if enabled
  _ph_init();
  // [/AP_ShoesAgtech]
}

void AP_ShoesAgtech::irq_handler(void) { _pulse_count++; }

// =============================================================
// MAIN UPDATE — called at 10Hz from scheduler
// Flow sensor: YF-S402B, dãy hoạt động 0.3–6 L/min, GPIO pin 55
// =============================================================
void AP_ShoesAgtech::update(void) {
  if (!is_enabled()) {
    _flow_rate_filtered = 0.0f;
    _flow_rate_avg = 0.0f;
    return;
  }

  _check_pump_config();

  // [AP_ShoesAgtech] poll pH sensor over Modbus RTU
  _ph_update();
  // [/AP_ShoesAgtech]

  uint32_t now = AP_HAL::millis();
  uint32_t delta_t_ms = now - _last_timestamp_ms;

  if (!_is_initialized) {
    _last_timestamp_ms = now;
    _last_pulse_snapshot = _pulse_count;
    _pid_last_ms = now;
    _is_initialized = true;
    return;
  }

  // ---- 1. FLOW RATE CALCULATION (every 100ms) ----
  if (delta_t_ms >= 100) {
    _last_timestamp_ms = now;

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

  // ---- 2. SPRAY CONTROL ----
  float dt_pid = (now - _pid_last_ms) * 0.001f;
  if (dt_pid <= 0.0f || dt_pid > 1.0f) {
    dt_pid = 0.1f;
  }
  _pid_last_ms = now;

  _update_spray_mode();

  switch (_spray_mode) {

  case 0: {
    // ---- MODE 0: SOFTWARE PASSTHROUGH ----
    // Read SA_RC_PUMP channel and write directly to SA_PUMP_CHAN.
    // Requires SERVOx_FUNCTION = 0 (None) on the pump channel so
    // the hardware passthrough does not fight our writes in mode 1/2.
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

  case 1:
    // ---- MODE 1: FLOW PID — track SA_FLOW_SP ----
    _flow_target = _flow_setpoint.get();
    _pump_pwm = _run_flow_pid(_flow_target, dt_pid);
    _write_pump_pwm(_pump_pwm);
    break;

  case 2: {
    // ---- MODE 2: AUTO RATE — L/ha × speed × boom → target ----
    float speed_ms = AP::ahrs().groundspeed();
    if (speed_ms < 0.1f) {
      // Stopped: release override, reset integral
      _flow_target = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      // Write min to stop pump while stationary
      SRV_Channel *ch =
          SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
      if (ch != nullptr) {
        _write_pump_pwm(ch->get_output_min());
      }
    } else {
      _flow_target = _app_rate.get() * speed_ms * _boom_width.get() * 0.006f;
      _pump_pwm = _run_flow_pid(_flow_target, dt_pid);
      _write_pump_pwm(_pump_pwm);
    }
    break;
  }

  default:
    break;
  }

  // ---- 3. FLOW CONSOLE LOG (SA_LOG_EN) ----
  if (_flow_log_enable.get() > 0 &&
      now - _last_log_ms >= (uint32_t)_flow_log_ms.get()) {
    _last_log_ms = now;
    gcs().send_text(MAV_SEVERITY_INFO,
                    "[FLOW] M%u Tgt:%.1f Act:%.1f Avg:%.1f PWM:%u",
                    (unsigned)_spray_mode, (double)_flow_target,
                    (double)_flow_rate_filtered, (double)_flow_rate_avg,
                    (unsigned)_pump_pwm);
  }
}

// =============================================================
// =============================================================
// PUMP CONFIG CHECK
// Runs every update(). Prints servo MIN/TRIM/MAX on first boot
// or when PUMP_CHAN changes. Warns every 5s if function != 0.
// =============================================================
void AP_ShoesAgtech::_check_pump_config(void) {
  int8_t chan = _pump_chan.get();
  int32_t func_val = (int32_t)SRV_Channels::channel_function(
      (uint8_t)constrain_int16(chan - 1, 0, 15));

  bool changed = (chan != _last_pump_chan) || (func_val != _last_pump_func_val);

  if (!changed && _pump_config_ok) {
    return; // already OK and nothing changed
  }

  if (changed) {
    _last_pump_chan = chan;
    _last_pump_func_val = func_val;
  }

  if (func_val != (int32_t)SRV_Channel::k_none) {
    // Wrong function — print warning every 5s
    uint32_t now = AP_HAL::millis();
    if (changed || now - _last_warn_ms >= 5000) {
      _last_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA: SERVO%d_FUNCTION=%d must be 0(None)!", (int)chan,
                      (int)func_val);
    }
    _pump_config_ok = false;
  } else {
    // Correct function — print servo config
    SRV_Channel *ch = SRV_Channels::srv_channel((uint8_t)(chan - 1));
    if (ch != nullptr) {
      gcs().send_text(MAV_SEVERITY_INFO, "SA: SERVO%d OK Min:%u Trim:%u Max:%u",
                      (int)chan, (unsigned)ch->get_output_min(),
                      (unsigned)ch->get_trim(), (unsigned)ch->get_output_max());
    }
    _pump_config_ok = true;
  }
}

// RC → SPRAY MODE
//   Nấc 1 (PWM < 1300) : mode 0 — passthrough, không can thiệp
//   Nấc 2 (1300-1700)  : mode 1 — PID bám SA_FLOW_SP
//   Nấc 3 (PWM > 1700) : mode 2 — auto L/ha
// =============================================================
void AP_ShoesAgtech::_update_spray_mode(void) {
  uint8_t idx = (uint8_t)constrain_int16(_rc_chan.get() - 1, 0, 15);
  uint16_t pwm_in = RC_Channels::get_radio_in(idx);

  if (pwm_in == 0) {
    return; // no signal — keep current mode
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
// PI CONTROLLER + OUTPUT LPF
// Base PWM = servo TRIM (from SERVOx_TRIM parameter)
// Range    = servo MIN/MAX (from SERVOx_MIN/MAX parameters)
// =============================================================
uint16_t AP_ShoesAgtech::_run_flow_pid(float target_lmin, float dt) {
  SRV_Channel *ch = SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
  uint16_t pwm_min = (ch != nullptr) ? ch->get_output_min() : 1000;
  uint16_t pwm_max = (ch != nullptr) ? ch->get_output_max() : 2000;
  uint16_t pwm_trim = (ch != nullptr) ? ch->get_trim() : 1500;

  float error = target_lmin - _flow_rate_filtered;
  float i_gain = _pid_i.get();

  // Integral with anti-windup clamped to ±half PWM range
  _pid_integral += error * dt;
  if (i_gain > 0.0f) {
    float ilimit = (pwm_max - pwm_min) * 0.5f / i_gain;
    _pid_integral = constrain_float(_pid_integral, -ilimit, ilimit);
  }

  // PI raw output (in microseconds relative to trim)
  float pid_raw = _pid_p.get() * error + i_gain * _pid_integral;

  // Low-pass filter to smooth PWM commands
  float alpha = constrain_float(_pid_lpf.get(), 0.01f, 1.0f);
  _pid_output_lpf = _pid_output_lpf * (1.0f - alpha) + pid_raw * alpha;

  // Final PWM = trim + PID, clamped to servo min/max
  return (uint16_t)constrain_float((float)pwm_trim + _pid_output_lpf,
                                   (float)pwm_min, (float)pwm_max);
}

// =============================================================
// WRITE PUMP CHANNEL
// Calls set_output_pwm_chan() which sets have_pwm_mask so that
// calc_pwm() will NOT overwrite our value in subsequent loops.
// Requires SERVOx_FUNCTION = 0 (None) on the pump channel so
// output_ch() passthrough does not overwrite our output_pwm.
// =============================================================
void AP_ShoesAgtech::_write_pump_pwm(uint16_t pwm) {
  uint8_t chan_idx = (uint8_t)constrain_int16(_pump_chan.get() - 1, 0, 15);
  SRV_Channels::set_output_pwm_chan(chan_idx, pwm);
}

// =============================================================
// [AP_ShoesAgtech] pH SENSOR — Nengshi ASPS3801D-0.5M
//
// Protocol: Modbus RTU, 9600 8N1, Function Code 04
// Hardware: Sensor RS485 A/B → RS485-TTL module → TELEM port RX/TX
// Request:  [01][04][00 00][00 09][30 0C]  (9 registers from 0x0000)
// Response: [01][04][12][Reg0..Reg8 × 2B][CRC × 2B] = 23 bytes
//
// Register map (0-indexed in response buffer, data at buf[3]+):
//   0x0000 (buf[3..4])  pH × 100          unsigned
//   0x0002 (buf[7..8])  electrode mV       signed 16-bit
//   0x0008 (buf[19..20]) temperature × 10  signed 16-bit
// =============================================================

// CRC16/Modbus: polynomial 0xA001, init 0xFFFF, result LSB-first in frame
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
  _ph_value_ema = -1.0f; // sentinel: not yet initialized
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

  // ---- Send new request every 2 seconds ----
  if (!_ph_req_pending) {
    if (now - _ph_update_ms < 2000) {
      return;
    }
    _ph_update_ms = now;

    // Flush stale RX bytes before sending
    uint16_t stale = _ph_uart->available();
    while (stale > 0) {
      _ph_uart->read();
      stale--;
    }

    // Modbus FC04: read 9 registers from 0x0000, CRC = 0x0C30 (LSB first: 30
    // 0C)
    static const uint8_t req[8] = {0x01, 0x04, 0x00, 0x00,
                                   0x00, 0x09, 0x30, 0x0C};
    _ph_uart->write(req, sizeof(req));
    _ph_req_pending = true;
    _ph_req_sent_ms = now;
    return;
  }

  // ---- Wait at least 150ms for sensor to respond ----
  if (now - _ph_req_sent_ms < 150) {
    return;
  }

  uint16_t avail = _ph_uart->available();

  // Timeout: if no full frame within 500ms, give up and warn if sensor absent
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
                          "SA: pH sensor chua co du lieu - kiem tra day RS485");
        } else {
          gcs().send_text(
              MAV_SEVERITY_WARNING,
              "SA: pH sensor mat ket noi (%.0fs) - kiem tra day RS485",
              (double)((now - _ph_last_good_ms) / 1000U));
        }
      }
    }
    return;
  }

  _ph_req_pending = false;

  // ---- Read exactly 23 bytes ----
  uint8_t buf[23];
  for (uint8_t i = 0; i < 23; i++) {
    int16_t b = _ph_uart->read();
    buf[i] = (b >= 0) ? (uint8_t)b : 0;
  }

  // ---- Validate Modbus header and byte count ----
  if (buf[0] != 0x01 || buf[1] != 0x04 || buf[2] != 18) {
    return;
  }

  // ---- CRC verification (covers bytes 0..20, result compared to bytes 21-22)
  // ----
  uint16_t crc_calc = _ph_crc16(buf, 21);
  uint16_t crc_recv = (uint16_t)buf[21] | ((uint16_t)buf[22] << 8);
  if (crc_calc != crc_recv) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: pH CRC fail (noise on RS485?)");
    return;
  }

  // ---- Decode registers ----
  uint16_t raw_ph = ((uint16_t)buf[3] << 8) | buf[4];               // unsigned
  int16_t raw_mv = (int16_t)(((uint16_t)buf[7] << 8) | buf[8]);     // signed
  int16_t raw_temp = (int16_t)(((uint16_t)buf[19] << 8) | buf[20]); // signed

  float ph_cal = constrain_float(raw_ph / 100.0f + _ph_off.get(), 0.0f, 14.0f);
  _ph_mv = raw_mv;
  _ph_temp = raw_temp / 10.0f + _ph_toff.get();
  _ph_value = ph_cal;
  _ph_last_good_ms = now; // frame hợp lệ — reset bộ đếm mất kết nối

  // ---- Moving Average (PH_WINDOW = 10 samples) ----
  _ph_buf_sum -= _ph_buf[_ph_buf_idx];
  _ph_buf[_ph_buf_idx] = ph_cal;
  _ph_buf_sum += ph_cal;
  _ph_buf_idx = (_ph_buf_idx + 1) % PH_WINDOW;
  if (_ph_buf_count < PH_WINDOW) {
    _ph_buf_count++;
  }
  _ph_value_ma = _ph_buf_sum / _ph_buf_count;

  // ---- EMA (Exponential Moving Average) ----
  float alpha = constrain_float(_ph_ema_alpha.get(), 0.01f, 1.0f);
  if (_ph_value_ema < 0.0f) {
    _ph_value_ema = ph_cal; // first sample: seed EMA directly
  } else {
    _ph_value_ema = _ph_value_ema * (1.0f - alpha) + ph_cal * alpha;
  }

  // ---- Daily slot tracking + alkalinity from ΔpH ----
  _ph_update_daily_slots(ph_cal);

  // ---- PH CONSOLE LOG (SA_PH_LOG) — independent of SA_FLOW_LOG ----
  if (_ph_log_enable.get() > 0 &&
      now - _ph_last_log_ms >= (uint32_t)_ph_log_ms.get()) {
    _ph_last_log_ms = now;
    // Status tag for alkalinity data quality
    const char *slot_tag;
    switch (_alk_slot_status) {
    case 0:
      slot_tag = "FULL";
      break; // both morning + afternoon today
    case 1:
      slot_tag = "MORN";
      break; // morning only
    case 2:
      slot_tag = "AFT";
      break; // afternoon only
    case 3:
      slot_tag = "PREV";
      break; // yesterday's data
    default:
      slot_tag = "NODATA";
      break; // no data yet
    }
    gcs().send_text(MAV_SEVERITY_INFO, "[WM] pH:%.2f MA:%.2f Tmp:%.1fC mV:%d",
                    (double)_ph_value, (double)_ph_value_ma, (double)_ph_temp,
                    (int)_ph_mv);
    gcs().send_text(
        MAV_SEVERITY_INFO, "[WM] Alk:%.2fdKH %.1fmg/L dPH:%+.2f [%s]",
        (double)_alk_dkh, (double)_alk_mgl, (double)_delta_ph, slot_tag);
  }
}

// =============================================================
// [AP_ShoesAgtech] DAILY ΔpH SLOT TRACKING
//
// Classifies each pH reading into morning (05:00-11:59) or afternoon
// (12:00-16:59) local time slots. At midnight resets today's slots and
// preserves yesterday's alkalinity as fallback.
//
// Alkalinity derivation priority:
//   Both slots captured today  → ΔpH-scaled from morning pH (best)
//   One slot only              → single-point estimate + warning
//   No data today              → yesterday's value + warning
//   Never had data             → no estimate
// =============================================================
void AP_ShoesAgtech::_ph_update_daily_slots(float ph_cal) {
  uint32_t now = AP_HAL::millis();

  // ---- GPS time classification (optional — slot tracking only works with GPS
  // time) ----
  uint64_t utc_usec = 0;
  const bool have_time = AP::rtc().get_utc_usec(utc_usec);

  if (have_time) {
    int8_t tz = (int8_t)constrain_int16(_ph_tz.get(), -12, 14);
    uint32_t utc_sec = (uint32_t)(utc_usec / 1000000ULL);
    uint32_t local_sec = utc_sec + (uint32_t)((int32_t)tz * 3600);
    uint32_t day_num = local_sec / 86400U;
    uint32_t local_hour = (local_sec % 86400U) / 3600U;

    // New-day: fires on first GPS fix (0 → today) AND on midnight rollover
    if (day_num != _rtc_last_day) {
      if (_rtc_last_day != 0 && _alk_today_dkh > 0.0f) {
        _alk_prev_dkh = _alk_today_dkh;
        _alk_prev_mgl = _alk_today_mgl;
      }
      _ph_morn_valid = false;
      _ph_aft_valid = false;
      _ph_morn_val = 0.0f;
      _ph_aft_val = 0.0f;
      _delta_ph = 0.0f;
      _alk_today_dkh = 0.0f;
      _alk_today_mgl = 0.0f;
      if (_alk_prev_dkh > 0.0f) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[WM] Ngay moi: slot reset. Kiem: du lieu hom qua");
      } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[WM] Ngay moi: slot reset. Chua co du lieu kiem");
      }
      _rtc_last_day = day_num;
    }

    // Sang: 0h00–11h59 | Chieu: 12h00–23h59
    if (local_hour < 12) {
      _ph_morn_val = ph_cal;
      _ph_morn_valid = true;
    } else {
      _ph_aft_val = ph_cal;
      _ph_aft_valid = true;
    }
  }

  // ---- Alkalinity calculation — always runs, with or without GPS time ----
  if (_ph_morn_valid && _ph_aft_valid) {
    _delta_ph = _ph_aft_val - _ph_morn_val;
    float kh_scaled = _ph_kh.get() *
                      (1.0f + constrain_float(_delta_ph * 0.375f, -0.5f, 1.0f));
    _alk_today_dkh = _ph_calc_alkalinity(_ph_morn_val, kh_scaled, _ph_temp);
    _alk_today_mgl = _alk_today_dkh * 17.85f;
    _alk_dkh = _alk_today_dkh;
    _alk_mgl = _alk_today_mgl;
    _alk_slot_status = 0;
  } else if (_ph_morn_valid) {
    _delta_ph = 0.0f;
    _alk_today_dkh = _ph_calc_alkalinity(_ph_morn_val, _ph_kh.get(), _ph_temp);
    _alk_today_mgl = _alk_today_dkh * 17.85f;
    _alk_dkh = _alk_today_dkh;
    _alk_mgl = _alk_today_mgl;
    _alk_slot_status = 1;
  } else if (_ph_aft_valid) {
    _delta_ph = 0.0f;
    _alk_today_dkh = _ph_calc_alkalinity(_ph_aft_val, _ph_kh.get(), _ph_temp);
    _alk_today_mgl = _alk_today_dkh * 17.85f;
    _alk_dkh = _alk_today_dkh;
    _alk_mgl = _alk_today_mgl;
    _alk_slot_status = 2;
  } else if (_alk_prev_dkh > 0.0f) {
    // Yesterday's data still valid
    _alk_dkh = _alk_prev_dkh;
    _alk_mgl = _alk_prev_mgl;
    _alk_slot_status = 3;
  } else {
    // No slotted data at all: single-point heuristic from current pH so display
    // is not 0
    _alk_dkh = _ph_calc_alkalinity(ph_cal, _ph_kh.get(), _ph_temp);
    _alk_mgl = _alk_dkh * 17.85f;
    _alk_slot_status = 4;
  }

  // ---- Periodic status warning (every 60s when SA_PH_LOG=1) ----
  if (_ph_log_enable.get() > 0 && now - _slot_warn_ms >= 60000) {
    _slot_warn_ms = now;
    if (!have_time) {
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "[WM] Chua co GPS time, kiem tinh theo pH tuc thoi");
    } else if (_alk_slot_status == 1) {
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "[WM] Kiem: chi co du lieu sang, cho du lieu chieu");
    } else if (_alk_slot_status == 2) {
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "[WM] Kiem: chi co du lieu chieu, thieu du lieu sang");
    } else if (_alk_slot_status == 3) {
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "[WM] Kiem: dang dung du lieu hom qua");
    } else if (_alk_slot_status == 4) {
      gcs().send_text(
          MAV_SEVERITY_WARNING,
          "[WM] Kiem: chua co du lieu slot (doi GPS hoac cho khung gio)");
    }
  }
}

// =============================================================
// Alkalinity estimation algorithm
//
// Estimates relative alkalinity (KH) change based on deviation of
// the measured pH from a reference point (pH ≈ 8.0) and temperature.
//
// Physical basis:
//   - In pond water, CO2/HCO3-/CO3²- equilibrium couples pH and KH.
//   - pH > 8.3: HCO3- dominant, low free CO2 → higher effective KH.
//   - pH < 7.6: high free CO2 consumes KH → lower effective KH.
//   - Higher temperature → less dissolved CO2 → slight pH rise.
//
// This is a heuristic model, NOT a substitute for direct KH test kit
// measurement. Use to observe trends; update SA_PH_KH periodically.
// =============================================================
float AP_ShoesAgtech::_ph_calc_alkalinity(float ph, float base_kh_dkh,
                                          float temp_c) {
  ph = constrain_float(ph, 0.0f, 14.0f);
  temp_c = constrain_float(temp_c, -10.0f, 50.0f);

  // Temperature factor: each 1°C above 28°C reduces dissolved CO2 by ~0.8%
  float tf = constrain_float(1.0f - (temp_c - 28.0f) * 0.008f, 0.85f, 1.10f);

  // pH factor: linear segments around neutral reference pH 8.0
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
// [/AP_ShoesAgtech]
