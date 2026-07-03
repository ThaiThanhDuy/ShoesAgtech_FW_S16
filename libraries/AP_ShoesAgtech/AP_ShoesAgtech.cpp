#include "AP_ShoesAgtech.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Mission/AP_Mission.h>
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

    // [AP_ShoesAgtech] -------- Dosing motor (vít tải thức ăn tôm): servo xoay
    // liên tục 360° --------
    // @Param: DOS_CHAN
    // @DisplayName: Servo output channel for dosing motor (1-indexed)
    // @Description: Phải đặt SERVOx_FUNCTION=0 (None). Thư viện xuất PWM trực
    //   tiếp ra kênh này để điều khiển động cơ servo 360 độ định lượng.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_CHAN", 25, AP_ShoesAgtech, _dos_chan, 10),

    // @Param: DOS_RC
    // @DisplayName: RC channel to toggle dosing motor on/off (1-indexed)
    // @Description: PWM > 1500 -> bật động cơ (quay theo SA_DOS_SP/SA_DOS_RATE
    //   quy đổi), PWM <= 1500 -> tắt (xuất 1500, dừng). VD: nút B trên tay
    //   Skydroid T10 thường gán ở kênh 8.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_RC", 26, AP_ShoesAgtech, _dos_rc, 8),

    // @Param: DOS_RATE
    // @DisplayName: Dosing conversion ratio (gam ứng với 50us PWM lệch)
    // @Description: Tỉ lệ quy đổi lượng thức ăn (gam) sang độ lệch PWM. Giá
    //   trị là số gam tương ứng với 50us PWM lệch khỏi điểm dừng (1500). VD:
    //   nhập 100 -> cứ 100g thì lệch 50us; nhập 200 -> cứ 200g thì lệch 50us
    //   (tỉ lệ thấp hơn, motor đáp ứng "chậm" hơn theo SA_DOS_SP).
    //   offset(us) = SA_DOS_SP * 50 / SA_DOS_RATE
    // @Units: g
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("DOS_RATE", 27, AP_ShoesAgtech, _dos_rate, 100.0f),

    // @Param: DOS_SP
    // @DisplayName: Dosing setpoint (lượng thức ăn muốn cấp, gam)
    // @Description: Người dùng nhập trực tiếp lượng thức ăn mong muốn (gam).
    //   Firmware tự quy đổi ra độ lệch PWM theo tỉ lệ SA_DOS_RATE rồi cộng/trừ
    //   vào điểm dừng 1500 tuỳ chiều quay SA_DOS_REV. VD nhập 1000 (1kg) với
    //   SA_DOS_RATE=100 -> offset = 1000*50/100 = 500us.
    // @Units: g
    // @User: Standard
    AP_GROUPINFO("DOS_SP", 28, AP_ShoesAgtech, _dos_sp, 0.0f),

    // @Param: DOS_REV
    // @DisplayName: Dosing motor direction
    // @Description: Servo 360 độ, 1500=dừng. 0 = chiều thuận: PWM chạy trong
    //   dải 800-1500 (800=tốc độ cao nhất, giảm dần độ lệch về 1500). 1 =
    //   chiều ngược: PWM chạy trong dải 1500-2200 (xuất xung tăng dần từ 1500
    //   lên 2200, motor tự đảo chiều theo mức xung này).
    // @Values: 0:Thuan (800-1500), 1:Nguoc (1500-2200)
    // @User: Standard
    AP_GROUPINFO("DOS_REV", 29, AP_ShoesAgtech, _dos_rev, 0),

    // @Param: DOS_LOG
    // @DisplayName: Dosing motor console log enable
    // @Description: In ra console trạng thái ON/OFF, setpoint (SA_DOS_SP) và
    //   PWM đang xuất ra của dosing motor mỗi SA_DOS_LOG_MS mili-giây.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DOS_LOG", 30, AP_ShoesAgtech, _dos_log_enable, 0),

    // @Param: DOS_LOG_MS
    // @DisplayName: Dosing motor console log interval (ms)
    // @Description: Khoảng thời gian giữa hai lần in log dosing motor ra
    //   console khi SA_DOS_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("DOS_LOG_MS", 31, AP_ShoesAgtech, _dos_log_ms, 1000),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] Simulation mode (slot 32)
    // @Param: SIM
    // @DisplayName: Simulation mode
    // @Description: Khi bật (1), bỏ qua cảm biến thật và inject dữ liệu giả lập
    //   có biến thiên hình sin để test hiển thị GCS và logic mode 1/2. Khi tắt (0)
    //   quay về đọc sensor thật.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SIM", 32, AP_ShoesAgtech, _simulation, 0),

    // [AP_ShoesAgtech] Flow sensor GPIO pin (slot 33)
    // @Param: FLOW_PIN
    // @DisplayName: Flow sensor GPIO pin number
    // @Description: Số chân GPIO kết nối tín hiệu cảm biến lưu lượng YF-S402B.
    //   Mặc định = 55 (Pixhawk/CubeOrange AUX GPIO). Thay đổi theo phần cứng.
    // @Range: 1 200
    // @User: Standard
    AP_GROUPINFO("FLOW_PIN", 33, AP_ShoesAgtech, _flow_pin, 55),

    // [AP_ShoesAgtech] Tank volume + flow mode (slots 34-35)
    // @Param: TANK_VOL
    // @DisplayName: Tank volume (Litres)
    // @Description: Dung tích tank nước/hóa chất (lít). Dùng để:
    //   (1) tính flow_target trong mode 1 khi SA_FLOW_MODE=1
    //   (2) hiển thị cảnh báo khoảng cách còn bơm được trong mode 2.
    //   Đặt = 0 để tắt cả hai chức năng.
    // @Units: L
    // @Range: 0 2000
    // @User: Standard
    // [AP_ShoesAgtech] Dosing motor mode (slot 36)
    // @Param: DOS_MODE
    // @DisplayName: Dosing motor speed mode
    // @Description: Cách tính tốc độ động cơ định lượng khi RC bật:
    //   0 = tốc độ cố định từ SA_DOS_SP/SA_DOS_RATE (hành vi cũ).
    //   1 = tốc độ tỉ lệ theo vận tốc + tổng quãng đường mission:
    //       offset = (SA_DOS_SP × speed × 60 / mission_dist) × 50 / SA_DOS_RATE.
    //       Phân bổ SA_DOS_SP gam đều trên toàn tuyến đường.
    //       Khi không có mission hoặc speed < 0.05 m/s → dừng + cảnh báo.
    // @Values: 0:Fixed,1:MissionProportional
    // @User: Standard
    AP_GROUPINFO("DOS_MODE", 36, AP_ShoesAgtech, _dos_mode, 0),
    // [/AP_ShoesAgtech]

    AP_GROUPINFO("TANK_VOL", 34, AP_ShoesAgtech, _tank_vol, 0.0f),

    // @Param: FLOW_MODE
    // @DisplayName: Mode 1 setpoint source
    // @Description: Cách tính flow_target trong mode 1 (FLOW PID):
    //   0 = trực tiếp từ SA_FLOW_SP (L/min) — như cũ.
    //   1 = tự tính từ SA_TANK_VOL + tổng quãng đường mission:
    //       flow = (tank_vol × speed × 60) / mission_dist.
    //       Khi không có mission hoặc SA_TANK_VOL=0, fallback về SA_FLOW_SP.
    // @Values: 0:DirectSetpoint,1:TankMissionFormula
    // @User: Standard
    AP_GROUPINFO("FLOW_MODE", 35, AP_ShoesAgtech, _flow_mode, 0),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] Vi sinh mixing ratio by RC field condition (slots 37-38)
    // @Param: MIX_STD
    // @DisplayName: Vi sinh ratio — nac giua (Mac dinh van)
    // @Description: Ti le vi sinh trong tong luong phun khi chon nac giua RC.
    //   Dung trong FLOW_MODE=1: q1_target = MIX_STD * APP_RATE * speed * BOOM * 0.006.
    //   Dung trong FLOW_MODE=0: setpoint chinh la SA_FLOW_SP (khong can ratio).
    //   dist_max = TANK_VOL * 10000 / (MIX_STD * APP_RATE * BOOM).
    // @Range: 0.01 1.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MIX_STD", 37, AP_ShoesAgtech, _mix_std, 0.35f),

    // @Param: MIX_CNT
    // @DisplayName: Vi sinh ratio — nac cao (Chong nghet van)
    // @Description: Ti le vi sinh trong tong luong phun khi chon nac cao RC
    //   (van vi sinh mo nhieu hon, chong nghet). FLOW_MODE=1: dung MIX_CNT thay
    //   MIX_STD trong cong thuc. FLOW_MODE=0: flow_target = SA_FLOW_SP *
    //   (MIX_CNT / MIX_STD) de giu tong luong ra boom giong nac giua.
    // @Range: 0.01 1.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MIX_CNT", 38, AP_ShoesAgtech, _mix_cnt, 0.50f),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] Override speed for FLOW_MODE=1 calibration (slot 39)
    // @Param: FLOW_VEL
    // @DisplayName: Override speed for FLOW_MODE=1 (m/s)
    // @Description: 0 = dung van toc that tu GPS/AHRS. > 0 = ep van toc bang gia
    //   tri nay (m/s) de tinh q1_target va dist_max — dung calib FLOW_MODE=1 khi
    //   xe dung yen. Khong anh huong khi SA_SIM=1 (SA_SIM uu tien hon FLOW_VEL).
    // @Range: 0 10
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("FLOW_VEL", 39, AP_ShoesAgtech, _flow_vel, 0.0f),
    // [/AP_ShoesAgtech]

    AP_GROUPEND};

AP_ShoesAgtech::AP_ShoesAgtech()
    : _last_timestamp_ms(0), _last_pulse_snapshot(0), _last_log_ms(0),
      _flow_rate_filtered(0.0f), _flow_rate_avg(0.0f), _is_initialized(false),
      _buffer_index(0), _buffer_sum(0.0f), _samples_count(0), _spray_mode(0),
      _pump_pwm(0), _flow_target(0.0f),
      _sim_speed(0.0f),
      // [AP_ShoesAgtech] mission distance cache + tank monitor
      _mission_dist_m(0.0f), _mission_ncmds(0), _tank_warn_ms(0),
      _arm_dist_warned(false),
      _was_armed(false),
      // [/AP_ShoesAgtech]
      _pid_integral(0.0f), _pid_output_lpf(0.0f), _pid_last_ms(0),
      _last_pump_chan(-1),
      _last_pump_func_val(-1), _pump_config_ok(false), _last_warn_ms(0),
      // [AP_ShoesAgtech] dosing motor initial state
      _dos_pwm(1500), _dos_config_ok(false), _dos_warn_ms(0),
      _dos_was_ok(false), _dos_was_on(false), _dos_last_log_ms(0),
      // [/AP_ShoesAgtech]
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

  // [AP_ShoesAgtech] simulation or real sensor path
  if (_simulation.get() > 0) {
    _run_simulation();
  } else {
    // poll pH sensor over Modbus RTU (real hardware only)
    _ph_update();
  }
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] dosing motor — RC on/off + rate-to-PWM conversion
  _update_dosing_motor();
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

    if (_simulation.get() > 0) {
      // Simulation: _flow_rate_filtered/_flow_rate_avg already set by _run_simulation()
      // Still need to advance the moving-average buffer with the simulated value
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

  // ---- 2. SPRAY CONTROL ----
  float dt_pid = (now - _pid_last_ms) * 0.001f;
  if (dt_pid <= 0.0f || dt_pid > 1.0f) {
    dt_pid = 0.1f;
  }
  _pid_last_ms = now;

  _update_spray_mode();

  bool now_armed = hal.util->get_soft_armed();

  // ARM edge: in trang thai FM1 mot lan khi arm, bat ke SA_FLOW_LOG
  if (now_armed && !_was_armed) {
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f &&
        (_spray_mode == 1 || _spray_mode == 2)) {
      float r = (_spray_mode == 2) ? _mix_cnt.get() : _mix_std.get();
      _print_fm1_arm_status(r);
    }
  }

  // Khi disarm: reset warning + cache mission de ARM tiep theo tinh lai khoang cach
  if (!now_armed) {
    _arm_dist_warned = false;
    _mission_ncmds   = 0;
    _mission_dist_m  = 0.0f;
  }
  _was_armed = now_armed;

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

  case 1: {
    // ---- MODE 1: FLOW PID (nac giua — MIX_STD / Mac dinh van) ----
    // Yeu cau: vehicle phai duoc ARM truoc khi bom hoat dong
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch1 = SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
      if (ch1 != nullptr) { _write_pump_pwm(ch1->get_output_min()); }
      break;
    }
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f) {
      // FLOW_MODE=1: cong thuc L/ha × vi sinh ratio (MIX_STD)
      _flow_target = _compute_visin_target(_mix_std.get());
      if (_flow_target < 0.01f) {
        // dieu kien khong dat (mission/dist/speed/q1): force min ngay, bo qua PID
        SRV_Channel *ch1 = SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
        if (ch1 != nullptr) { _write_pump_pwm(ch1->get_output_min()); }
        break;
      }
    } else {
      // FLOW_MODE=0: setpoint truc tiep tu SA_FLOW_SP
      _flow_target = _flow_setpoint.get();
    }
    _pump_pwm = _run_flow_pid(_flow_target, dt_pid);
    _write_pump_pwm(_pump_pwm);
    break;
  }

  case 2: {
    // ---- MODE 2: FLOW PID (nac cao — MIX_CNT / Chong nghet van) ----
    // Yeu cau: vehicle phai duoc ARM truoc khi bom hoat dong
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch2 = SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
      if (ch2 != nullptr) { _write_pump_pwm(ch2->get_output_min()); }
      break;
    }
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f) {
      // FLOW_MODE=1: cong thuc L/ha × vi sinh ratio (MIX_CNT)
      _flow_target = _compute_visin_target(_mix_cnt.get());
      if (_flow_target < 0.01f) {
        // dieu kien khong dat: force min ngay, bo qua PID
        SRV_Channel *ch2 = SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
        if (ch2 != nullptr) { _write_pump_pwm(ch2->get_output_min()); }
        break;
      }
    } else {
      // FLOW_MODE=0: setpoint × ti le MIX_CNT/MIX_STD (giu tong luong ra boom)
      float ratio = (_mix_std.get() > 0.01f) ? (_mix_cnt.get() / _mix_std.get()) : 1.0f;
      _flow_target = constrain_float(_flow_setpoint.get() * ratio, 0.0f, 200.0f);
      // Tank monitor: uoc tinh khoang cach con bom duoc (moi 30s)
      if (_tank_vol.get() > 0.0f && _flow_target > 0.01f) {
        float speed_ms2 = _get_spray_speed();
        if (speed_ms2 > 0.01f && now - _tank_warn_ms >= 30000U) {
          _tank_warn_ms = now;
          float dist_m = (_tank_vol.get() / _flow_target) * speed_ms2 * 60.0f;
          gcs().send_text(MAV_SEVERITY_INFO,
                          "SA: Tank du ~%.0fm (%.1fL @%.1fL/min)",
                          (double)dist_m, (double)_tank_vol.get(),
                          (double)_flow_target);
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

  // ---- 3. FLOW CONSOLE LOG (SA_LOG_EN) ----
  if (_flow_log_enable.get() > 0 &&
      now - _last_log_ms >= (uint32_t)_flow_log_ms.get()) {
    _last_log_ms = now;
    const char *flow_pfx = (_simulation.get() > 0) ? "[SIM][FLOW]" : "[FLOW]";
    gcs().send_text(MAV_SEVERITY_INFO,
                    "%s M%u Tgt:%.1f Act:%.1f Avg:%.1f PWM:%u",
                    flow_pfx, (unsigned)_spray_mode, (double)_flow_target,
                    (double)_flow_rate_filtered, (double)_flow_rate_avg,
                    (unsigned)_pump_pwm);
    // Khi nac giua/cao + FLOW_MODE=1: in them thong tin cong thuc
    if ((_spray_mode == 1 || _spray_mode == 2) &&
        _flow_mode.get() == 1 && _tank_vol.get() > 0.0f) {
      float r     = (_spray_mode == 2) ? _mix_cnt.get() : _mix_std.get();
      float mdist = _get_mission_dist();
      float spd   = _get_spray_speed();
      float dv    = r * _app_rate.get() * _boom_width.get();
      float dmax  = (dv > 0.001f) ? (_tank_vol.get() * 10000.0f / dv) : 0.0f;
      gcs().send_text(MAV_SEVERITY_INFO,
                      "%s FM1 r:%.2f miss:%.0fm dmax:%.0fm spd:%.2fm/s",
                      flow_pfx, (double)r, (double)mdist, (double)dmax, (double)spd);
    }
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
//   Nấc 2 (1300-1700)  : mode 1 — FLOW PID, tỉ lệ SA_MIX_STD (Mặc định van)
//   Nấc 3 (PWM > 1700) : mode 2 — FLOW PID, tỉ lệ SA_MIX_CNT (Chống nghẹt van)
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
// [AP_ShoesAgtech] DOSING MOTOR — vít tải thức ăn tôm, servo xoay liên tục 360°
//
// RC SA_DOS_RC bật/tắt: PWM > 1500 -> bật (quay theo SA_DOS_SP quy đổi qua tỉ
// lệ SA_DOS_RATE), PWM <= 1500 (kể cả mất tín hiệu = 0) -> tắt, xuất 1500
// (dừng).
//
// Quy đổi lượng thức ăn (SA_DOS_SP, gam) -> độ lệch PWM:
//   offset = SA_DOS_SP * 50 / SA_DOS_RATE
//   (SA_DOS_RATE = số gam ứng với 50us lệch; vd RATE=100 -> 100g = 50us lệch)
//
// Chiều quay theo SA_DOS_REV (servo 360°, 1500 = dừng):
//   0 = thuận: pwm = constrain(1500 - offset,  800, 1500)  (800 = nhanh nhất)
//   1 = ngược: pwm = constrain(1500 + offset, 1500, 2200)
// Yêu cầu SERVOx_FUNCTION = 0 (None) trên kênh SA_DOS_CHAN.
// =============================================================

// =============================================================
// DOSING CHANNEL CONFIG CHECK
// Servo SA_DOS_CHAN BẮT BUỘC phải thoả cả 4 điều kiện mới cho phép
// motor chạy (dù bấm nút SA_DOS_RC):
//   FUNCTION = 0 (None), MIN = 800, TRIM = 1500, MAX = 2200
// Sai điều kiện nào -> chỉ báo (các) điều kiện đó, lặp lại mỗi 5 giây.
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
                      "SA: SERVO%d setup thanh cong - dosing motor san sang",
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
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d khong ton tai",
                    (int)chan);
    return;
  }
  if (!func_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA: SERVO%d FUNCTION=%d, can dat =0 (None)", (int)chan,
                    (int)func_val);
  }
  if (!min_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MIN=%u, can dat =800",
                    (int)chan, (unsigned)ch->get_output_min());
  }
  if (!trim_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d TRIM=%u, can dat =1500",
                    (int)chan, (unsigned)ch->get_trim());
  }
  if (!max_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MAX=%u, can dat =2200",
                    (int)chan, (unsigned)ch->get_output_max());
  }
}

void AP_ShoesAgtech::_update_dosing_motor(void) {
  _check_dosing_config();
  if (!_dos_config_ok) {
    // Cấu hình servo sai — không cho chạy, kể cả khi bấm SA_DOS_RC
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

  if (motor_on) {
    float ratio = _dos_rate.get();
    float pwm_f = 1500.0f;

    if (_dos_mode.get() == 0) {
      // ---- DOS_MODE 0: tốc độ cố định từ SA_DOS_SP/SA_DOS_RATE ----
      float offset = (ratio > 0.0f) ? (_dos_sp.get() * 50.0f / ratio) : 0.0f;
      if (_dos_rev.get() == 0) {
        pwm_f = constrain_float(1500.0f - offset, 800.0f, 1500.0f);
      } else {
        pwm_f = constrain_float(1500.0f + offset, 1500.0f, 2200.0f);
      }
    } else {
      // ---- DOS_MODE 1: tốc độ tỉ lệ theo speed + mission_dist ----
      float mission_dist = _get_mission_dist();
      float speed_ms = (_simulation.get() > 0) ? _sim_speed : AP::ahrs().groundspeed();
      if (mission_dist > 1.0f && speed_ms >= 0.05f) {
        float dos_gpm = (_dos_sp.get() * speed_ms * 60.0f) / mission_dist;
        float offset  = (ratio > 0.0f) ? (dos_gpm * 50.0f / ratio) : 0.0f;
        if (_dos_rev.get() == 0) {
          pwm_f = constrain_float(1500.0f - offset, 800.0f, 1500.0f);
        } else {
          pwm_f = constrain_float(1500.0f + offset, 1500.0f, 2200.0f);
        }
      } else {
        // Không đủ điều kiện → dừng motor + cảnh báo mỗi 5s
        pwm_f = 1500.0f;
        if (now - _dos_warn_ms >= 5000U) {
          _dos_warn_ms = now;
          if (mission_dist <= 1.0f) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "SA DOS1: chua co mission (dist=%.1fm) - motor dung",
                            (double)mission_dist);
          } else {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "SA DOS1: toc do qua thap (%.2fm/s) - motor dung",
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

  // ---- DOSING CONSOLE LOG (SA_DOS_LOG) ----
  if (_dos_log_enable.get() > 0) {
    if (now - _dos_last_log_ms >= (uint32_t)_dos_log_ms.get()) {
      _dos_last_log_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO, "[DOS] SERVO%d %s SP:%.0fg PWM:%u",
                      (int)_dos_chan.get(), motor_on ? "ON" : "OFF",
                      (double)_dos_sp.get(), (unsigned)_dos_pwm);
    }
  }
}
// [/AP_ShoesAgtech]

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
    const char *ph_pfx = (_simulation.get() > 0) ? "[SIM][WM]" : "[WM]";
    gcs().send_text(MAV_SEVERITY_INFO, "%s pH:%.2f MA:%.2f Tmp:%.1fC mV:%d",
                    ph_pfx, (double)_ph_value, (double)_ph_value_ma,
                    (double)_ph_temp, (int)_ph_mv);
    gcs().send_text(MAV_SEVERITY_INFO, "%s Alk:%.2fdKH %.1fmg/L dPH:%+.2f [%s]",
                    ph_pfx, (double)_alk_dkh, (double)_alk_mgl,
                    (double)_delta_ph, slot_tag);
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

// =============================================================
// SIMULATION — SA_SIM = 1
// Generates sinusoidal fake sensor data so modes 1/2 and GCS
// display can be verified without real hardware attached.
// Called from update() instead of _ph_update() when active.
// Flow values are written directly to _flow_rate_filtered and
// _sim_speed; the moving-average buffer is advanced by the
// caller (update()) as usual.
// =============================================================
void AP_ShoesAgtech::_run_simulation(void) {
  uint32_t now = AP_HAL::millis();
  float t = now * 0.001f;  // seconds since boot

  // ---- Module 1: flow (2.5 ± 1.5 L/min, 20s period) ----
  _flow_rate_filtered = 2.5f + 1.5f * sinf(2.0f * M_PI * t / 20.0f);

  // ---- Module 1: simulated groundspeed (1.0 ± 0.8 m/s, 30s period) ----
  _sim_speed = constrain_float(1.0f + 0.8f * sinf(2.0f * M_PI * t / 30.0f),
                               0.1f, 2.0f);

  // ---- Module 2: pH sensor values ----
  // pH: 7.3 ± 0.4, 60s period
  float ph_sim  = 7.3f + 0.4f * sinf(2.0f * M_PI * t / 60.0f);
  // Temperature: 28.0 ± 2.0°C, 120s period
  float temp_sim = 28.0f + 2.0f * sinf(2.0f * M_PI * t / 120.0f);
  // Electrode mV from Nernst: ~59.16 mV/pH unit relative to pH 7
  int16_t mv_sim = (int16_t)((7.0f - ph_sim) * 59.16f);
  // Alkalinity: 4.0 ± 0.8 dKH, 90s period
  float dkh_sim  = 4.0f + 0.8f * sinf(2.0f * M_PI * t / 90.0f);
  float mgl_sim  = dkh_sim * 17.85f;

  _ph_value          = ph_sim;
  _ph_value_ema      = ph_sim;
  _ph_value_ma       = ph_sim;
  _ph_mv             = mv_sim;
  _ph_temp           = temp_sim;
  _alk_dkh           = dkh_sim;
  _alk_mgl           = mgl_sim;
  _delta_ph          = 0.0f;
  _alk_slot_status   = 0;     // FULL so ph_has_data()-like checks pass

  // Keep ph_has_data() returning true
  _ph_last_good_ms = now;
}

// =============================================================
// [AP_ShoesAgtech] SPRAY SPEED — uu tien: SA_SIM > SA_FLOW_VEL > AHRS
// SA_SIM=1       : dung _sim_speed (sin wave, dung cho test man hinh)
// SA_FLOW_VEL>0  : dung gia tri co dinh (calib FLOW_MODE=1 khi xe dung yen)
// Default (=0)   : dung van toc that tu AP::ahrs().groundspeed()
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
// [/AP_ShoesAgtech]

// =============================================================
// _print_fm1_arm_status — in trang thai FLOW_MODE=1 khi ARM (bat ke FLOW_LOG).
// Goi mot lan moi ARM session khi spray_mode = 1 hoac 2.
// Kiem tra: mission, dist_max, speed, q1 range.
// =============================================================
void AP_ShoesAgtech::_print_fm1_arm_status(float r)
{
  r = constrain_float(r, 0.01f, 1.0f);
  float dist = _get_mission_dist();

  // Check 1: co mission khong?
  if (dist <= 1.0f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: chưa có mission - bơm sẽ dừng");
    return;
  }

  // Check 2: khoang cach mission co qua dai khong?
  float denom = r * _app_rate.get() * _boom_width.get();
  if (denom < 0.001f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: APP_RATE/BOOM_W = 0 - kiểm tra param");
    return;
  }
  float dist_max = _tank_vol.get() * 10000.0f / denom;
  if (dist > dist_max) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: mission %.0fm > dmax %.0fm - vẽ lại mission ngắn hơn",
                    (double)dist, (double)dist_max);
    return;
  }

  // Check 3: van toc + preview q1
  float speed = _get_spray_speed();
  if (speed <= 0.1f) {
    gcs().send_text(MAV_SEVERITY_INFO,
                    "SA FM1 SẴN SÀNG: r=%.2f miss=%.0fm/%.0fm | vận tốc=0 bơm chờ xe chạy",
                    (double)r, (double)dist, (double)dist_max);
    return;
  }

  float q1 = r * _app_rate.get() * speed * _boom_width.get() * 0.006f;
  if (q1 < 0.3f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: q1=%.2fL/ph < 0.3 @%.1fm/s - tăng APP_RATE hoặc FLOW_VEL",
                    (double)q1, (double)speed);
    return;
  }
  if (q1 > 2.0f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: q1=%.2fL/ph > 2.0 @%.1fm/s - giảm APP_RATE hoặc FLOW_VEL",
                    (double)q1, (double)speed);
    return;
  }

  // Thoi gian du kien chay het mission + luong vi sinh tieu thu
  uint32_t eta_s   = (uint32_t)(dist / speed);
  uint32_t eta_min = eta_s / 60U;
  uint32_t eta_sec = eta_s % 60U;
  float    vi_used = q1 * (eta_s / 60.0f);

  gcs().send_text(MAV_SEVERITY_INFO,
                  "SA FM1 OK: r=%.2f q1=%.2fL/ph miss=%.0fm dmax=%.0fm ~%um%02us vi~%.1fL",
                  (double)r, (double)q1,
                  (double)dist, (double)dist_max,
                  eta_min, eta_sec, (double)vi_used);
}

// =============================================================
// [AP_ShoesAgtech] VISIN TARGET — FLOW_MODE=1 (nac giua/cao)
//
// Tinh luu luong vi sinh target (L/min) theo cong thuc L/ha × ratio.
// Tat ca kiem tra xuat hien truoc khi chay PID:
//   1. Mission: dist <= 1m → warning + stop
//   2. dist_max: tank_vol × 10000 / (r × APP_RATE × BOOM) < mission_dist → stop
//   3. Speed:   < 0.1 m/s → stop (khong canh bao, chi reset PID)
//   4. Range:   q1 ∉ [0.3, 6.0] L/min → warning + stop (cam bien YF-S402B)
// Canh bao throttle chung qua _tank_warn_ms (5s min giua hai lan).
// =============================================================
float AP_ShoesAgtech::_compute_visin_target(float r) {
  uint32_t now = AP_HAL::millis();
  r = constrain_float(r, 0.01f, 1.0f);

  float speed_ms = _get_spray_speed();
  float dist = _get_mission_dist();

  // Kiem tra 1: mission da upload?
  if (dist <= 1.0f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA FM1: chua co mission - bom dung");
    }
    return 0.0f;
  }

  // Kiem tra 2: mission phai <= dist_max de vi sinh vua het khi ket thuc
  float denom = r * _app_rate.get() * _boom_width.get();
  if (denom < 0.001f) {
    return 0.0f;
  }
  float dist_max = _tank_vol.get() * 10000.0f / denom;
  if (dist > dist_max) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (!_arm_dist_warned) {
      _arm_dist_warned = true;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA FM1: mission %.0fm > dist_max %.0fm - ve lai mission ngan hon",
                      (double)dist, (double)dist_max);
    }
    return 0.0f;
  }

  // Kiem tra 3: toc do
  if (speed_ms < 0.1f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    return 0.0f;
  }

  // Tinh q1_target (L/min)
  float q1 = r * _app_rate.get() * speed_ms * _boom_width.get() * 0.006f;

  // Kiem tra 4: dai cam bien YF-S402B (0.3–6 L/min)
  if (q1 < 0.3f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA FM1: Q visin %.2fL/min < 0.3 - tang mission_dist hoac giam speed",
                      (double)q1);
    }
    return 0.0f;
  }
  if (q1 > 6.0f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA FM1: Q visin %.2fL/min > 6.0 - giam mission_dist hoac tang speed",
                      (double)q1);
    }
    return 0.0f;
  }

  return constrain_float(q1, 0.0f, 200.0f);
}
// [/AP_ShoesAgtech]

// =============================================================
// MISSION DISTANCE — SA_FLOW_MODE = 1 (mode 1) + mode 2 monitor
// Iterates AP_Mission waypoints and sums leg distances.
// Result is cached until num_commands() changes (mission edited
// or re-uploaded). Returns 0 if no mission is loaded.
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

  // Return cached value if mission hasn't changed
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
    // Only nav commands carry a meaningful location
    if (cmd.id != MAV_CMD_NAV_WAYPOINT &&
        cmd.id != MAV_CMD_NAV_LOITER_UNLIM &&
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
  _mission_ncmds  = n;
  return _mission_dist_m;
}
// [/AP_ShoesAgtech]
