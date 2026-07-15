#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_ShoesAgtech {
public:
  AP_ShoesAgtech();

  AP_ShoesAgtech(const AP_ShoesAgtech &) = delete;
  AP_ShoesAgtech &operator=(const AP_ShoesAgtech &) = delete;

  void init(void);
  void update(void);

  // ---- MODULE 1: Flow sensor getters — YF-S402B, dãy hoạt động 0.3–6 L/min ----
  float    get_flow_rate_lmin(void) const { return _flow_rate_filtered; }
  float    get_flow_rate_avg(void)  const { return _flow_rate_avg; }
  bool     is_enabled(void)         const { return _enable_flag.get() > 0; }

  // Spray control getters (for logging)
  uint8_t  get_spray_mode(void)       const { return _spray_mode; }
  uint16_t get_pump_pwm(void)         const { return _pump_pwm; }
  float    get_flow_target(void)      const { return _flow_target; }

  // ---- MODULE 2: pH sensor getters — Nengshi ASPS3801D-0.5M via Modbus RTU ----
  float    get_ph(void)              const { return _ph_value_ma; }
  float    get_ph_raw(void)          const { return _ph_value; }
  float    get_ph_temp(void)         const { return _ph_temp; }
  float    get_ph_mv(void)           const { return (float)_ph_mv; }
  float    get_ph_morn(void)         const { return _ph_morn_val; }
  float    get_ph_aft(void)          const { return _ph_aft_val; }
  int32_t  get_ph_morn_lat(void)     const { return _ph_morn_lat; }
  int32_t  get_ph_morn_lng(void)     const { return _ph_morn_lng; }
  float    get_alk_dkh(void)         const { return _alk_dkh; }
  float    get_alk_mgl(void)         const { return _alk_mgl; }
  float    get_delta_ph(void)        const { return _delta_ph; }
  bool     ph_is_enabled(void)       const { return _ph_en.get() > 0; }
  // consume_alk_log_pending — pop ONE pond pending SD write per call.
  // Sets output mirror getters (get_ph_morn, get_alk_dkh, ...) for Log.cpp.
  bool     consume_alk_log_pending(void);
  // consume_gcs_alk_pending — same pattern, independent flag for MAVLink path.
  // Does NOT interfere with consume_alk_log_pending (separate gcs_alk_pending flag).
  bool     consume_gcs_alk_pending(void);
  uint8_t  get_alk_pond_idx(void)   const { return _alk_pond_idx; }
  // Dữ liệu trực tiếp từ ao đang active (GPS-matched cycle hiện tại).
  // Trả 0 nếu ao chưa có dữ liệu đủ (alk_computed=false hoặc status<3).
  float    get_active_alk_dkh(void) const {
    return (_ponds[_active_pond_idx].alk_computed) ? _ponds[_active_pond_idx].alk_dkh : 0.0f;
  }
  float    get_active_alk_mgl(void) const {
    return (_ponds[_active_pond_idx].alk_computed) ? _ponds[_active_pond_idx].alk_mgl : 0.0f;
  }
  float    get_active_delta_ph(void) const {
    return (_ponds[_active_pond_idx].status == 3) ? _ponds[_active_pond_idx].delta_ph : 0.0f;
  }
  // pH sáng/chiều của ao active trong ngày hôm đó (0 nếu slot chưa có mẫu).
  float    get_active_ph_morn(void)  const { return _ponds[_active_pond_idx].ph_morn; }
  float    get_active_ph_aft(void)   const { return _ponds[_active_pond_idx].ph_aft; }
  // Ngày ghi nhận dữ liệu pH của ao active (ngày tính từ Unix epoch, × 86400 = Unix timestamp).
  uint32_t get_active_last_day(void)  const { return _ponds[_active_pond_idx].last_day; }
  // Index ao đang active trong cycle hiện tại (0-based nội bộ; cộng 1 trước khi hiển thị).
  uint8_t  get_active_pond_idx(void)  const { return _active_pond_idx; }
  // Lượng thức ăn (gam) của ao đang active — mỗi ao lưu riêng, khởi tạo từ SA_DOS_SP.
  float    get_active_dos_sp(void)    const { return _ponds[_active_pond_idx].dos_sp; }
  // Loại thức ăn (1-7) của ao đang active — mỗi ao lưu riêng, khởi tạo từ SA_DOS_FOOD.
  int8_t   get_active_dos_food(void)  const { return _ponds[_active_pond_idx].dos_food; }
  // true when the pH sensor has produced a valid Modbus frame within the
  // last SA_PH_TIMEOUT seconds (matches the "mất kết nối" threshold used
  // for the GCS warning)
  bool     ph_has_data(void)         const {
    const uint32_t timeout_ms = (uint32_t)((_ph_timeout.get() > 0) ? _ph_timeout.get() : 1) * 1000U;
    return (_ph_last_good_ms != 0) &&
           (AP_HAL::millis() - _ph_last_good_ms <= timeout_ms);
  }

  // ---- MODULE 3: Dosing motor getters (for logging) ----
  uint16_t get_dosing_pwm(void)     const { return _dos_pwm; }
  // Tốc độ vít tải (mL/50us) của loại thức ăn đang dùng cho ao active (SA_DOS_Fx).
  float    get_active_dos_rate(void) const {
    const int8_t food = _ponds[_active_pond_idx].valid ? _ponds[_active_pond_idx].dos_food
                                                        : (int8_t)_dos_food.get();
    return _dos_fr[_clamp_food(food) - 1].get();
  }

  static const AP_Param::GroupInfo var_info[];
  static void irq_handler(void);

private:
  // ================================================================
  // PARAMETERS
  // ================================================================

  // ---- MODULE 0: shared across modules (slots 1, 32, 59, 63) ----
  AP_Int8  _enable_flag;    // SA_ENABLE
  AP_Int16 _robot_id;       // SA_ROBOT_ID  dinh danh robot (0-999), khong dung noi bo

  // ---- MODULE 1: flow sensor YF-S402B + spray controller (slots 1-11, 22, 33-35, 37-39) ----
  AP_Float _cal_factor;     // SA_CAL_FAC   pulses/Litre
  AP_Float _ema_alpha;      // SA_EMA_AL    EMA smoothing
  AP_Int8  _flow_log_enable; // SA_FLOW_LOG  console print for flow sensor
  AP_Int8  _rc_chan;         // SA_RC_CHAN   RC channel for mode switch (1-indexed)
  AP_Int8  _rc_pump;        // SA_RC_PUMP  RC channel passed to pump in mode 0 (1-indexed)
  AP_Int8  _pump_chan;       // SA_PUMP_CHAN servo output channel (1-indexed, e.g. 8)
  AP_Float _flow_setpoint;  // SA_FLOW_SP   target L/min (mode 1)
  AP_Float _pid_p;          // SA_PID_P     P gain (us per L/min)
  AP_Float _pid_i;          // SA_PID_I     I gain (us per L/min/s)
  AP_Float _pid_lpf;        // SA_PID_LPF   output LPF alpha (0.01-1.0)
  AP_Int16 _flow_log_ms;    // SA_LOG_FL_MS   flow console log interval (ms, default 1000)
  AP_Int16 _flow_pin;       // SA_FLOW_PIN  GPIO pin number (default 55)
  AP_Float _tank_vol;       // SA_TANK_VOL  tank volume in litres (0 = disabled)
  AP_Int8  _flow_mode;      // SA_FLOW_MODE 0=direct setpoint, 1=tank+mission formula
  AP_Float _mix_std;        // SA_MIX_STD  ti le vi sinh nac giua (Mac dinh van), default 0.35
  AP_Float _mix_cnt;        // SA_MIX_CNT  ti le vi sinh nac cao (Chong nghet van), default 0.50
  AP_Float _flow_vel;       // SA_FLOW_VEL  0=dung van toc that, >0=dung gia tri nay (m/s)

  // ---- MODULE 2: pH sensor — Nengshi ASPS3801D-0.5M (slots 14-24, 55-62; slot 12 reused) ----
  AP_Int8  _ph_en;          // SA_PH_EN     enable pH sensor
  AP_Int8  _ph_port;        // SA_PH_PORT   UART port number (matches SERIALx)
  AP_Float _ph_toff;        // SA_PH_TOFF   temperature offset °C
  AP_Float _ph_off;         // SA_PH_OFF    pH calibration offset
  AP_Float _ph_kh;          // SA_PH_KH     base alkalinity dKH (from test kit)
  AP_Int8  _ph_log_enable;  // SA_PH_LOG      console print for pH sensor (independent of SA_FLOW_LOG)
  AP_Int8  _ph_tz;          // SA_PH_TZ       UTC offset hours (Vietnam = 7)
  AP_Int16 _ph_log_ms;      // SA_PH_LOG_MS   pH console log interval (ms, default 2000)
  AP_Int16 _ph_timeout;     // SA_PH_TIMEOUT  pH "mat ket noi" timeout, seconds (default 1)
  AP_Float _ph_ms;          // SA_PH_MS    gio bat dau slot sang  (0.0-23.99, default 5.0)
  AP_Float _ph_me;          // SA_PH_ME    gio ket thuc slot sang  (0.0-24.0,  default 11.0)
  AP_Float _ph_as;          // SA_PH_AS    gio bat dau slot chieu (0.0-23.99, default 12.0)
  AP_Float _ph_ae;          // SA_PH_AE    gio ket thuc slot chieu (0.0-24.0,  default 16.0)
  AP_Float _ph_pond_dist;   // SA_PH_POND_D  nguong GPS validate cung ao (m), default 300
  AP_Int16 _pond_select;    // SA_POND_IDX   ao dang do (1-based, nhap thu cong), default 1
  AP_Int16 _ph_cap_s;       // SA_PH_CAP_S   khoang thoi gian giua hai mau (giay, default 20)
  AP_Int8  _ph_cap_sam;     // SA_PH_CAP_SAM so mau tich luy de tinh trung binh (default 20)
  AP_Int8  _ph_cap_m;       // SA_PH_CAP_M   ban kinh capture (m), 1-100 - chi dung cho app, firmware khong doc

  // ---- MODULE 3: dosing motor (vit tai thuc an tom) — servo xoay lien tuc 360° (slots 25-26, 28-31, 36, 40-54; slot 27 retired) ----
  AP_Int8  _dos_chan;   // SA_DOS_CHAN   servo output channel (1-indexed)
  AP_Int8  _dos_rc;     // SA_DOS_RC     RC channel bat/tat motor (1-indexed)
  AP_Float _dos_sp;     // SA_DOS_SP     setpoint: luong thuc an muon cap, gam
  AP_Int8  _dos_rev;    // SA_DOS_REV    chieu quay: 0=thuan, 1=nguoc
  AP_Int8  _dos_log_enable; // SA_DOS_LOG     console log enable cho dosing motor
  AP_Int16 _dos_log_ms;     // SA_DOS_LOG_MS  khoang thoi gian giua hai lan in log (ms)
  AP_Int8  _dos_mode;       // SA_DOS_MODE    0=fixed PWM, 1=variable theo speed+mission
  AP_Int8  _dos_food;       // SA_DOS_FOOD    chon loai thuc an 1-7
  AP_Float _dos_fr[7];      // SA_DOS_F1..F7  the tich vit tai (mL/50us) theo tung loai thuc an
  AP_Float _dos_dr[7];      // SA_DOS_D1..D7  khoi luong rieng (g/mL) theo tung loai thuc an

  // ---- SIMULATION (slot 32) ----
  AP_Int8  _simulation;  // SA_SIM  0=real sensors, 1=simulated data

  // ================================================================
  // STATE
  // ================================================================

  // ---- MODULE 1: flow sensor state — YF-S402B ----
  uint32_t _last_timestamp_ms;
  uint32_t _last_pulse_snapshot;
  uint32_t _last_log_ms;
  float    _flow_rate_filtered;
  float    _flow_rate_avg;
  bool     _is_initialized;

  static const uint8_t WINDOW_SIZE = 10;
  float    _sample_buffer[WINDOW_SIZE];
  uint8_t  _buffer_index;
  float    _buffer_sum;
  uint8_t  _samples_count;

  static volatile uint32_t _pulse_count;

  // ---- MODULE 1: spray control state ----
  uint8_t  _spray_mode;         // 0=PASSTHROUGH 1=FLOW_PID 2=AUTO_RATE
  uint16_t _pump_pwm;
  float    _flow_target;
  float    _pid_integral;
  float    _pid_output_lpf;
  uint32_t _pid_last_ms;

  // ---- MODULE 1: pump config check state ----
  int8_t   _last_pump_chan;
  int32_t  _last_pump_func_val;
  bool     _pump_config_ok;
  uint32_t _last_warn_ms;

  // ---- MODULE 1: mission distance cache + tank monitor state ----
  float    _mission_dist_m;
  uint16_t _mission_ncmds;
  uint32_t _tank_warn_ms;
  bool     _arm_dist_warned;
  bool     _was_armed;
  bool     _tank_empty_detected;
  uint32_t _tank_empty_ms;

  // ---- SIMULATION state (sim_speed used in M1 spray calculations) ----
  float    _sim_speed;

  // ---- MODULE 2: pH sensor state — Nengshi ASPS3801D-0.5M Modbus RTU ----
  AP_HAL::UARTDriver *_ph_uart;
  uint32_t  _ph_update_ms;
  uint32_t  _ph_req_sent_ms;
  bool      _ph_req_pending;
  uint32_t  _ph_last_good_ms;
  uint32_t  _ph_nodata_warn_ms;
  uint32_t  _ph_last_log_ms;
  float     _ph_value;
  float     _ph_value_ma;
  int16_t   _ph_mv;
  float     _ph_temp;

  static const uint8_t PH_WINDOW = 10;
  float     _ph_buf[PH_WINDOW];
  uint8_t   _ph_buf_idx;
  uint8_t   _ph_buf_count;
  float     _ph_buf_sum;

  // ---- MODULE 2: per-pond GPS cluster tracking ----
  static const uint8_t MAX_PONDS = 100;

  struct PondEntry {
    float    center_lat;    // tâm ao GPS lat (degrees)
    float    center_lng;    // tâm ao GPS lng (degrees)
    float    ph_morn;       // pH buổi sáng (mẫu cuối cùng)
    float    ph_aft;        // pH buổi chiều (mẫu cuối cùng)
    float    alk_dkh;       // kiềm dKH (giữ qua ngày)
    float    alk_mgl;       // kiềm mg/L (giữ qua ngày)
    float    delta_ph;      // ph_aft - ph_morn
    float    dos_sp;        // lượng thức ăn cho ao này (gam) — khởi tạo từ SA_DOS_SP
    int8_t   dos_food;      // loại thức ăn cho ao này (1-7) — khởi tạo từ SA_DOS_FOOD
    uint32_t last_day;      // day_num lần đo gần nhất
    uint32_t morn_last_ms;  // millis mẫu sáng cuối (rate-limit)
    uint32_t aft_last_ms;   // millis mẫu chiều cuối (rate-limit)
    int32_t  morn_lat;      // GPS lat mẫu sáng cuối (deg×1e7)
    int32_t  morn_lng;      // GPS lng mẫu sáng cuối (deg×1e7)
    uint16_t gps_count;     // số mẫu GPS cho rolling centroid
    uint8_t  morn_count;    // số mẫu sáng hôm nay
    uint8_t  aft_count;     // số mẫu chiều hôm nay
    uint8_t  status;        // bit0=có_sáng bit1=có_chiều (3=FULL)
    bool     alk_pending;     // cần ghi PHAK vào SD (consumed by Log.cpp)
    bool     gcs_alk_pending; // cần gửi SA_PHK qua MAVLink (consumed by GCS)
    bool     alk_computed;   // đã tính kiềm hôm nay
    bool     morn_reported;  // đã in thông báo hoàn thành slot sáng
    bool     aft_reported;   // đã in thông báo hoàn thành slot chiều
    bool     valid;          // slot đã được khởi tạo
  };

  PondEntry  _ponds[MAX_PONDS];
  uint8_t    _pond_count;

  // Output mirrors — cập nhật từ pond active/pending, dùng bởi Log.cpp getters
  float     _ph_morn_val;
  int32_t   _ph_morn_lat;
  int32_t   _ph_morn_lng;
  float     _ph_aft_val;
  float     _delta_ph;
  float     _alk_dkh;
  float     _alk_mgl;
  uint8_t   _alk_slot_status;
  uint8_t   _alk_pond_idx;
  uint8_t   _active_pond_idx;
  uint32_t  _slot_warn_ms;
  uint32_t  _ponds_save_ms;
  bool      _pond_first_detect_done;
  bool      _ponds_dirty;
  bool      _ponds_loaded;

  // ---- MODULE 3: dosing motor state — continuous-rotation 360° servo ----
  uint16_t _dos_pwm;
  bool     _dos_config_ok;
  uint32_t _dos_warn_ms;
  bool     _dos_was_ok;
  bool     _dos_was_on;
  uint32_t _dos_last_log_ms;

  // Đồng bộ SA_DOS_SP + SA_DOS_FOOD với dos_sp/dos_food riêng của ao đang
  // active (xem _sync_dosing_setpoint)
  uint8_t  _dos_sync_pond;    // ao lần đồng bộ gần nhất, 0xFF = chưa đồng bộ
  float    _dos_sp_sync_val;  // giá trị SA_DOS_SP tại lần đồng bộ gần nhất
  int8_t   _dos_food_sync_val; // giá trị SA_DOS_FOOD tại lần đồng bộ gần nhất

  // ================================================================
  // PRIVATE METHODS
  // ================================================================

  // ---- MODULE 1: flow sensor + spray control ----
  void     _update_spray_mode(void);
  void     _check_pump_config(void);
  uint16_t _run_flow_pid(float target_lmin, float dt);
  void     _write_pump_pwm(uint16_t pwm);
  float    _get_spray_speed(void);
  float    _compute_visin_target(float r);
  void     _print_fm1_arm_status(float r);
  float    _get_mission_dist(void);

  // ---- MODULE 2: pH sensor + alkalinity + pond persistence ----
  void     _ph_init(void);
  void     _ph_update(void);
  void     _ph_update_daily_slots(float ph_cal);
  float    _ph_calc_alkalinity(float ph, float base_kh_dkh, float temp_c);
  void     _io_update(void);   // chạy trong IO thread — load/save _ponds[] an toàn với AP::FS()
  void     _pond_load(void);
  void     _pond_save(void);

  // ---- MODULE 3: dosing motor ----
  void     _check_dosing_config(void);
  void     _sync_dosing_setpoint(void);
  void     _update_dosing_motor(void);
  // Chuyển offset PWM (us, luôn dương) thành giá trị PWM xuất ra theo chiều
  // quay SA_DOS_REV, đã constrain đúng nửa dải (800-1500 hoặc 1500-2200).
  uint16_t _offset_to_dos_pwm(float offset) const;
  // Kẹp giá trị loại thức ăn (SA_DOS_FOOD/dos_food) về dải hợp lệ 1-7.
  static int8_t _clamp_food(int8_t food);

  // ---- SIMULATION ----
  void     _run_simulation(void);
};
