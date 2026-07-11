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

  // Flow sensor getters — YF-S402B, dãy hoạt động 0.3–6 L/min
  float    get_flow_rate_lmin(void) const { return _flow_rate_filtered; }
  float    get_flow_rate_avg(void)  const { return _flow_rate_avg; }
  bool     is_enabled(void)         const { return _enable_flag.get() > 0; }

  // Spray control getters (for logging)
  uint8_t  get_spray_mode(void)       const { return _spray_mode; }
  uint16_t get_pump_pwm(void)         const { return _pump_pwm; }
  float    get_flow_target(void)      const { return _flow_target; }

  // [AP_ShoesAgtech] Dosing motor getters (for logging)
  uint16_t get_dosing_pwm(void)     const { return _dos_pwm; }
  float    get_dosing_sp(void)      const { return _dos_sp.get(); }
  float    get_dosing_rate(void)    const { return _dos_rate.get(); }
  int8_t   get_dosing_food(void)    const { return _dos_food.get(); }
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] pH sensor getters — Nengshi ASPS3801D-0.5M via Modbus RTU
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
  // slot_status: 0=FULL 1=MORN 2=AFT 3=PREV(yesterday) 4=NODATA
  uint8_t  get_alk_slot_status(void) const { return _alk_slot_status; }
  bool     alk_is_yesterday(void)    const { return _alk_slot_status == 3; }
  bool     ph_is_enabled(void)       const { return _ph_en.get() > 0; }
  // Returns true while any pond has a pending PHAK SD write.
  // Each call pops ONE pending pond and loads its data into the output mirror getters
  // (get_ph_morn, get_ph_aft, get_alk_dkh, etc.) so Log.cpp can write the record.
  // Call repeatedly each cycle until it returns false (all ponds flushed).
  bool     consume_alk_log_pending(void);
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
  // true when the pH sensor has produced a valid Modbus frame within the
  // last SA_PH_TIMEOUT seconds (matches the "mất kết nối" threshold used
  // for the GCS warning)
  bool     ph_has_data(void)         const {
    const uint32_t timeout_ms = (uint32_t)((_ph_timeout.get() > 0) ? _ph_timeout.get() : 1) * 1000U;
    return (_ph_last_good_ms != 0) &&
           (AP_HAL::millis() - _ph_last_good_ms <= timeout_ms);
  }
  // [/AP_ShoesAgtech]

  static const AP_Param::GroupInfo var_info[];
  static void irq_handler(void);

private:
  // ---- Parameters: flow sensor YF-S402B + spray controller (slots 1-13)
  //      Dãy hoạt động: 0.3–6 L/min ----
  AP_Int8  _enable_flag;    // SA_ENABLE
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
  AP_Float _app_rate;       // SA_APP_RATE  L/ha (mode 2)
  AP_Float _boom_width;     // SA_BOOM_W    boom width in metres (mode 2)

  // [AP_ShoesAgtech] Parameters: pH sensor — Nengshi ASPS3801D-0.5M (slots 14-21, 55-58)
  AP_Int8  _ph_en;          // SA_PH_EN     enable pH sensor
  AP_Int8  _ph_port;        // SA_PH_PORT   UART port number (matches SERIALx)
  AP_Float _ph_toff;        // SA_PH_TOFF   temperature offset °C
  AP_Float _ph_off;         // SA_PH_OFF    pH calibration offset
  AP_Float _ph_kh;          // SA_PH_KH     base alkalinity dKH (from test kit)
  AP_Float _ph_ema_alpha;   // SA_PH_EMA    EMA smoothing alpha for pH
  AP_Int8  _ph_log_enable;  // SA_PH_LOG      console print for pH sensor (independent of SA_FLOW_LOG)
  AP_Int8  _ph_tz;          // SA_PH_TZ       UTC offset hours (Vietnam = 7)
  AP_Int16 _flow_log_ms;    // SA_LOG_FL_MS   flow console log interval (ms, default 1000)
  AP_Int16 _ph_log_ms;      // SA_PH_LOG_MS   pH console log interval (ms, default 2000)
  AP_Int16 _ph_timeout;     // SA_PH_TIMEOUT  pH "mat ket noi" timeout, seconds (default 1)
  AP_Float _ph_ms;          // SA_PH_MS    gio bat dau slot sang  (0.0-23.99, default 5.0)  vd 5.5=5h30
  AP_Float _ph_me;          // SA_PH_ME    gio ket thuc slot sang  (0.0-24.0,  default 11.0) vd 11.5=11h30
  AP_Float _ph_as;          // SA_PH_AS    gio bat dau slot chieu (0.0-23.99, default 12.0) vd 13.5=13h30
  AP_Float _ph_ae;          // SA_PH_AE    gio ket thuc slot chieu (0.0-24.0,  default 16.0) vd 16.5=16h30
  AP_Float _ph_pond_dist;   // SA_PH_POND_D nguong cung ao sang+chieu (m), default 300
  AP_Int16 _ph_cap_s;       // SA_PH_CAP_S   khoang thoi gian giua hai mau (giay, default 20)
  AP_Int8  _ph_cap_sam;     // SA_PH_CAP_SAM so mau tich luy de tinh trung binh (default 20)
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Parameters: dosing motor (vit tai thuc an tom) — servo
  // xoay lien tuc 360 do (slots 25-31, 36, 40-54)
  AP_Int8  _dos_chan;   // SA_DOS_CHAN   servo output channel (1-indexed)
  AP_Int8  _dos_rc;     // SA_DOS_RC     RC channel bat/tat motor (1-indexed, vd: nut B Skydroid T10 = 8)
  AP_Float _dos_rate;   // SA_DOS_RATE   the tich vit tai (mL/50us): bao nhieu mL ung voi 50us PWM lech
  AP_Float _dos_sp;     // SA_DOS_SP     setpoint: luong thuc an muon cap, gam (nguoi dung nhap, vd 1000)
  AP_Int8  _dos_rev;    // SA_DOS_REV    chieu quay: 0 = thuan (xung 800..1500, 800=nhanh nhat), 1 = nguoc (xung 1500..2200)
  AP_Int8  _dos_log_enable; // SA_DOS_LOG     console log enable cho dosing motor
  AP_Int16 _dos_log_ms;     // SA_DOS_LOG_MS  khoang thoi gian giua hai lan in log (ms, mac dinh 1000)
  AP_Int8  _dos_mode;       // SA_DOS_MODE    0=fixed PWM, 1=variable theo speed+mission
  AP_Int8  _dos_food;       // SA_DOS_FOOD    chon loai thuc an 1-7 (tuong ung SA_DOS_F1..F7 va SA_DOS_D1..D7)
  AP_Float _dos_fr[7];      // SA_DOS_F1..F7  the tich vit tai (mL/50us) theo tung loai thuc an (DOS_MODE=1)
  AP_Float _dos_dr[7];      // SA_DOS_D1..D7  khoi luong rieng (g/mL) theo tung loai thuc an
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Parameter: simulation mode (slot 32)
  AP_Int8  _simulation;  // SA_SIM  0=real sensors, 1=simulated data
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Parameter: flow sensor GPIO pin (slot 33)
  AP_Int16 _flow_pin;    // SA_FLOW_PIN  GPIO pin number (default 55)
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Parameters: tank volume + flow mode (slots 34-35)
  AP_Float _tank_vol;    // SA_TANK_VOL  tank volume in litres (0 = disabled)
  AP_Int8  _flow_mode;   // SA_FLOW_MODE 0=direct setpoint, 1=tank+mission formula
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Parameters: vi sinh mixing ratio by field condition (slots 37-38)
  AP_Float _mix_std;  // SA_MIX_STD  ti le vi sinh nac giua (Mac dinh van), default 0.35
  AP_Float _mix_cnt;  // SA_MIX_CNT  ti le vi sinh nac cao (Chong nghet van), default 0.50
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Parameter: override speed for FLOW_MODE=1 calibration (slot 39)
  AP_Float _flow_vel; // SA_FLOW_VEL  0=dung van toc that, >0=dung gia tri nay (m/s)
  // [/AP_ShoesAgtech]

  // ---- Flow sensor state — YF-S402B (0.3–6 L/min) ----
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

  // ---- Spray control state ----
  uint8_t  _spray_mode;         // 0=PASSTHROUGH 1=FLOW_PID 2=AUTO_RATE
  uint16_t _pump_pwm;
  float    _flow_target;

  // [AP_ShoesAgtech] Simulation state
  float    _sim_speed;           // simulated groundspeed (m/s) for mode 2 testing
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Mission distance cache + tank monitor state
  float    _mission_dist_m;      // cached total mission distance (m)
  uint16_t _mission_ncmds;       // num_commands() when distance was last cached
  uint32_t _tank_warn_ms;        // last time tank distance warning was printed
  bool     _arm_dist_warned;     // true sau khi da canh bao dist>dist_max lan nay (reset khi disarm)
  bool     _was_armed;           // trang thai arm chu ki truoc (de phat hien canh ARM)
  bool     _tank_empty_detected; // true khi da phat hien thung het vi sinh (bom hut khi)
  uint32_t _tank_empty_ms;       // thoi diem flow bat dau vuot nguong 1.7 L/min lien tuc
  // [/AP_ShoesAgtech]
  float    _pid_integral;
  float    _pid_output_lpf;
  uint32_t _pid_last_ms;

  // ---- Pump config check state ----
  int8_t   _last_pump_chan;      // last PUMP_CHAN value we checked
  int32_t  _last_pump_func_val;  // last servo function value we checked
  bool     _pump_config_ok;      // true when SERVOx_FUNCTION == 0 (None)
  uint32_t _last_warn_ms;        // last time we printed the warning

  // [AP_ShoesAgtech] Dosing motor state — continuous-rotation 360° servo
  uint16_t _dos_pwm;             // last PWM written (1500 = dung)
  // SA_DOS_CHAN chi duoc phep chay khi servo dat dung 4 dieu kien:
  // FUNCTION=0(None), MIN=800, TRIM=1500, MAX=2200 — sai 1 trong 4 thi
  // khong chay (du nhan nut SA_DOS_RC) va canh bao lien tuc moi 5s
  bool     _dos_config_ok;       // true khi ca 4 dieu kien servo dung
  uint32_t _dos_warn_ms;         // last time we printed the config warning
  bool     _dos_was_ok;          // previous _dos_config_ok (de bao "setup thanh cong" khi vua dat)
  bool     _dos_was_on;          // previous on/off state cua SA_DOS_RC (de bao khi doi trang thai)
  uint32_t _dos_last_log_ms;     // last time we printed the SA_DOS_LOG console log
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] pH sensor state — Nengshi ASPS3801D-0.5M Modbus RTU
  AP_HAL::UARTDriver *_ph_uart;      // UART driver for RS485→TTL module
  uint32_t  _ph_update_ms;           // last time a request was initiated
  uint32_t  _ph_req_sent_ms;         // timestamp of last Modbus TX
  bool      _ph_req_pending;         // waiting for response
  uint32_t  _ph_last_good_ms;        // timestamp of last valid frame (0 = never)
  uint32_t  _ph_nodata_warn_ms;      // last time "no response" warning was sent
  uint32_t  _ph_last_log_ms;         // last time pH console log was printed
  float     _ph_value;               // latest decoded pH (calibrated)
  float     _ph_value_ema;           // EMA-filtered pH (-1 = not yet init)
  float     _ph_value_ma;            // moving average pH
  int16_t   _ph_mv;                  // electrode voltage in mV (signed)
  float     _ph_temp;                // temperature in °C

  static const uint8_t PH_WINDOW = 10;
  float     _ph_buf[PH_WINDOW];      // circular buffer for pH MA
  uint8_t   _ph_buf_idx;
  uint8_t   _ph_buf_count;
  float     _ph_buf_sum;

  // ---- Per-pond GPS cluster tracking (Hướng 2) ----
  // Mỗi ao nhận diện bằng GPS cluster (tâm ± SA_PH_POND_D).
  // Dữ liệu ao giữ qua các ngày; chỉ xóa slot hôm nay khi đo lại ao đó vào ngày mới.
  // Ring buffer ghi đè ao cũ nhất khi đủ 16 ao.
  static const uint8_t MAX_PONDS = 16;

  struct PondEntry {
    float    center_lat;    // tâm ao GPS lat (degrees)
    float    center_lng;    // tâm ao GPS lng (degrees)
    float    ph_morn;       // pH buổi sáng (mẫu cuối cùng)
    float    ph_aft;        // pH buổi chiều (mẫu cuối cùng)
    float    alk_dkh;       // kiềm dKH (giữ qua ngày)
    float    alk_mgl;       // kiềm mg/L (giữ qua ngày)
    float    delta_ph;      // ph_aft - ph_morn
    uint32_t last_day;      // day_num lần đo gần nhất
    uint32_t morn_last_ms;  // millis mẫu sáng cuối (rate-limit)
    uint32_t aft_last_ms;   // millis mẫu chiều cuối (rate-limit)
    int32_t  morn_lat;      // GPS lat mẫu sáng cuối (deg×1e7)
    int32_t  morn_lng;      // GPS lng mẫu sáng cuối (deg×1e7)
    uint16_t gps_count;     // số mẫu GPS cho rolling centroid
    uint8_t  morn_count;    // số mẫu sáng hôm nay
    uint8_t  aft_count;     // số mẫu chiều hôm nay
    uint8_t  status;        // bit0=có_sáng bit1=có_chiều (3=FULL)
    bool     alk_pending;    // cần ghi PHAK vào SD
    bool     alk_computed;   // đã tính kiềm hôm nay
    bool     morn_reported;  // đã in thông báo hoàn thành slot sáng
    bool     aft_reported;   // đã in thông báo hoàn thành slot chiều
    bool     valid;          // slot đã được khởi tạo
  };

  PondEntry  _ponds[MAX_PONDS];
  uint8_t    _pond_count;     // số slot đang dùng (0–MAX_PONDS)
  uint8_t    _pond_ring_idx;  // ring buffer write head

  // Output mirrors — cập nhật từ pond active/pending, dùng bởi Log.cpp getters
  float     _ph_morn_val;
  int32_t   _ph_morn_lat;
  int32_t   _ph_morn_lng;
  float     _ph_aft_val;
  float     _delta_ph;
  float     _alk_dkh;
  float     _alk_mgl;
  // 0=FULL hôm nay  1=chỉ có sáng  2=chỉ có chiều  3=dùng dữ liệu cũ  4=chưa có
  uint8_t   _alk_slot_status;
  uint8_t   _alk_pond_idx;        // pond index của PHAK vừa ghi (logging mirror)
  uint8_t   _active_pond_idx;     // pond index đang active trong cycle hiện tại
  uint32_t  _slot_warn_ms;
  bool      _pond_first_detect_done;  // true sau khi đã in thông báo ao lần đầu

  // [/AP_ShoesAgtech]

  // Private methods — flow/spray
  void     _update_spray_mode(void);
  void     _check_pump_config(void);
  uint16_t _run_flow_pid(float target_lmin, float dt);
  void     _write_pump_pwm(uint16_t pwm);
  float    _compute_visin_target(float r);
  float    _get_spray_speed(void);
  void     _print_fm1_arm_status(float r);

  // [AP_ShoesAgtech] Private methods — dosing motor
  void     _check_dosing_config(void);
  void     _update_dosing_motor(void);
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Private method — simulation
  void     _run_simulation(void);
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Private method — mission distance
  float    _get_mission_dist(void);
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] Private methods — pH sensor
  void     _ph_init(void);
  void     _ph_update(void);
  void     _ph_update_daily_slots(float ph_cal);
  float    _ph_calc_alkalinity(float ph, float base_kh_dkh, float temp_c);
  // [/AP_ShoesAgtech]
};
