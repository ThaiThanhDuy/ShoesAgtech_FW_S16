#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

// =============================================================
// Bảng tham số riêng theo module (2026-09-07) — mỗi class dưới đây có
// var_info[] RIÊNG (64 slot riêng, độc lập với 2 class kia và với
// AP_ShoesAgtech), đăng ký làm subgroup lồng bên trong AP_ShoesAgtech
// (xem AP_SUBGROUPINFO trong AP_ShoesAgtech.cpp). Đây là cấp lồng CUỐI
// CÙNG được AP_Param cho phép (18 bit group_element = 3 cấp x 6 bit,
// AP_ShoesAgtech đã ở cấp 2 dưới Parameters->g2). Đăng ký với tiền tố
// rỗng "" nên tên tham số hiển thị KHÔNG đổi (vẫn SA_FLOW_MODE,
// SA_PH_EN... như cũ) — chỉ đổi VỊ TRÍ LƯU trên EEPROM, nên giá trị đã
// cấu hình sẽ về lại mặc định sau khi flash (cần backup/restore file
// .param, xem DOC_Design).
// =============================================================

class AP_ShoesAgtech_FlowParams {
public:
  static const AP_Param::GroupInfo var_info[];

  AP_Float cal_factor;    // SA_CAL_FAC
  AP_Float ema_alpha;     // SA_FLOW_EMA_AL
  AP_Int8  flow_log_enable; // SA_FLOW_LOG
  AP_Int8  rc_chan;       // SA_RC_CHAN
  AP_Int8  rc_pump;       // SA_RC_PUMP
  AP_Int8  pump_chan;     // SA_PUMP_CHAN
  AP_Float flow_setpoint; // SA_FLOW_SP
  AP_Float pid_p;         // SA_FLOW_PID_P
  AP_Float pid_i;         // SA_FLOW_PID_I
  AP_Float pid_lpf;       // SA_FLOW_PID_LPF
  AP_Int16 flow_log_ms;   // SA_FLOW_LOG_MS
  AP_Int16 flow_pin;      // SA_FLOW_PIN
  AP_Float tank_vol;      // SA_TANK_VOL
  AP_Int8  flow_mode;     // SA_FLOW_MODE
  AP_Float mix_std;       // SA_FLOW_MIX_STD
  AP_Float mix_cnt;       // SA_FLOW_MIX_CNT
  AP_Float flow_vel;      // SA_FLOW_VEL
};

class AP_ShoesAgtech_PHParams {
public:
  static const AP_Param::GroupInfo var_info[];

  AP_Int8  ph_en;         // SA_PH_EN
  AP_Int8  ph_port;       // SA_PH_PORT
  AP_Float ph_toff;       // SA_PH_TOFF
  AP_Float ph_off;        // SA_PH_OFF
  AP_Float ph_kh;         // SA_PH_KH
  AP_Int8  ph_log_enable; // SA_PH_LOG
  AP_Int8  ph_tz;         // SA_PH_TZ
  AP_Int16 ph_log_ms;     // SA_PH_LOG_MS
  AP_Int16 ph_timeout;    // SA_PH_TIMEOUT
  AP_Float ph_ms;         // SA_PH_MS
  AP_Float ph_me;         // SA_PH_ME
  AP_Float ph_as;         // SA_PH_AS
  AP_Float ph_ae;         // SA_PH_AE
  AP_Float ph_pond_dist;  // SA_PH_POND_D
  AP_Int16 ph_cap_s;      // SA_PH_CAP_S
  AP_Int8  ph_cap_sam;    // SA_PH_CAP_SAM
  AP_Int8  ph_cap_m;      // SA_PH_CAP_M
};

class AP_ShoesAgtech_DosingParams {
public:
  static const AP_Param::GroupInfo var_info[];

  AP_Int8  dos_chan;      // SA_DOS_CHAN
  AP_Int8  dos_rc;        // SA_DOS_RC
  AP_Float dos_sp;        // SA_DOS_SP
  AP_Int8  dos_rev;       // SA_DOS_REV
  AP_Int8  dos_log_enable; // SA_DOS_LOG
  AP_Int16 dos_log_ms;    // SA_DOS_LOG_MS
  AP_Int8  dos_mode;      // SA_DOS_MODE
  AP_Int8  dos_food;      // SA_DOS_FOOD
  AP_Float dos_v;         // SA_DOS_V
  AP_Float dos_fr[7];     // SA_DOS_F1..F7
  AP_Float dos_dr[7];     // SA_DOS_D1..D7

  // ---- Đĩa rải ly tâm (ESC riêng, quay liên tục 1 chiều — KHÁC hẳn
  // trục vít 360° đảo chiều được) — mới 2026-09-08 ----
  AP_Int8  disc_chan;     // SA_DISC_CHAN  kênh servo/ESC đĩa rải, 0=tắt tính năng
  AP_Int8  disc_pct;      // SA_DISC_PCT   % tốc độ đĩa khi chạy (0-100, 100=full PWM max)
  AP_Float disc_delay;    // SA_DISC_DLY   độ trễ (giây) giữa đĩa và trục vít khi bật/tắt
};

class AP_ShoesAgtech {
public:
  AP_ShoesAgtech();

  AP_ShoesAgtech(const AP_ShoesAgtech &) = delete;
  AP_ShoesAgtech &operator=(const AP_ShoesAgtech &) = delete;

  void init(void);
  void update(void);

  // Tốc độ ĐẶT cho mission (WP_SPEED, đã cập nhật qua DO_CHANGE_SPEED/GCS
  // SET_SPEED) — Rover.cpp gọi mỗi chu kỳ TRƯỚC update() (từ
  // g2.wp_nav.get_speed_max()). Dùng làm nguồn tốc độ tham chiếu cho công
  // thức FLOW_MODE=1 (_compute_visin_target()) thay cho tốc độ GPS tức
  // thời, để lưu lượng phun không dao động theo từng cú tăng/giảm tốc/vào
  // cua — chỉ đổi khi tốc độ ĐẶT cho mission thực sự đổi.
  void set_target_speed(float speed) { _target_speed = speed; }

  // ---- MODULE 1: Flow sensor getters — YF-S402B, dãy hoạt động 0.3–6 L/min ----
  float    get_flow_rate_lmin(void) const { return _flow_rate_filtered; }
  float    get_flow_rate_avg(void)  const { return _flow_rate_avg; }
  bool     is_enabled(void)         const { return _enable_flag.get() > 0; }
  // true sau khi đã ARM rồi DISARM đúng 1 lần kể từ lúc boot — trước đó
  // bơm (Module 1) và motor cho ăn (Module 3) đều bị ép tắt hoàn toàn.
  bool     is_system_ready(void)    const { return _system_ready; }

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
  bool     ph_is_enabled(void)       const { return _ph_params.ph_en.get() > 0; }
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
    const uint32_t timeout_ms = (uint32_t)((_ph_params.ph_timeout.get() > 0) ? _ph_params.ph_timeout.get() : 1) * 1000U;
    return (_ph_last_good_ms != 0) &&
           (AP_HAL::millis() - _ph_last_good_ms <= timeout_ms);
  }

  // ---- MODULE 3: Dosing motor getters (for logging) ----
  uint16_t get_dosing_pwm(void)     const { return _dos_pwm; }
  // Đĩa rải ly tâm — mới 2026-09-08
  uint16_t get_disc_pwm(void)       const { return _disc_pwm; }
  bool     get_disc_running(void)   const { return _disc_running; }
  // Tốc độ vít tải (mL/50us) của loại thức ăn đang dùng cho ao active (SA_DOS_Fx).
  float    get_active_dos_rate(void) const {
    const int8_t food = _ponds[_active_pond_idx].valid ? _ponds[_active_pond_idx].dos_food
                                                        : (int8_t)_dos_params.dos_food.get();
    return _dos_params.dos_fr[_clamp_food(food) - 1].get();
  }

  static const AP_Param::GroupInfo var_info[];
  static void irq_handler(void);

private:
  // ================================================================
  // PARAMETERS
  // ================================================================

  // ---- Dùng chung nhiều module, giữ trực tiếp ở cấp AP_ShoesAgtech ----
  AP_Int8  _enable_flag;    // SA_ENABLE
  AP_Int16 _robot_id;       // SA_ROBOT_ID  dinh danh robot (0-999), khong dung noi bo
  AP_Int16 _pond_select;    // SA_POND_IDX  ao dang do - dung chung Module 2 (pH) va Module 3 (dosing)
  AP_Int8  _simulation;     // SA_SIM  0=real sensors, 1=simulated data - dung chung ca 3 module
  AP_Int8  _spd_start_pct;  // SA_SPD_START  % toc do dat toi thieu de bat dau bom/rai - dung chung Module 1 (FLOW_MODE=1) va Module 3 (DOS_MODE=2)

  // ---- Tham số riêng từng module (2026-09-07) — mỗi object dưới đây có
  // var_info[] RIÊNG (64 slot riêng), đăng ký subgroup trong
  // AP_ShoesAgtech::var_info[] (xem AP_ShoesAgtech.cpp) ----
  AP_ShoesAgtech_FlowParams   _flow_params;   // Module 1 — Bơm
  AP_ShoesAgtech_PHParams     _ph_params;     // Module 2 — Giám sát nước
  AP_ShoesAgtech_DosingParams _dos_params;    // Module 3 — Cho ăn

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
  // Setpoint thực tế đưa vào PID ở FLOW_MODE=1 — tăng dần từ 0 lên
  // _flow_target (ramp) sau khi mission đã bắt đầu tới WP1, để tránh bơm
  // giật/tràn lúc mới mồi. _flow_target vẫn hiện đầy đủ trong log như cũ.
  float    _flow_ramp_val;
  float    _pid_integral;
  float    _pid_output_lpf;
  uint32_t _pid_last_ms;

  // ---- MODULE 1: pump config check state ----
  int8_t   _last_pump_chan;
  int32_t  _last_pump_func_val;
  bool     _pump_config_ok;

  // ---- MODULE 1: mission distance cache + tank monitor state ----
  float    _mission_dist_m;
  uint16_t _mission_ncmds;
  bool     _tank_empty_detected;
  uint32_t _tank_empty_ms;
  // "no mission" (dist<=1m, FLOW_MODE=1) — chỉ cảnh báo 1 LẦN mỗi phiên
  // ARM (không lặp lại mỗi 5s như TANK EMPTY), reset khi disarm.
  bool     _no_mission_warned;
  // q1 (FLOW_MODE=1) đang ngoài dải lưu lượng THẬT bơm đạt được (0.8-1.3
  // L/min, hardcode theo phần cứng bơm hiện tại) hay không — cờ theo TỪNG
  // CHU KỲ (không phải one-shot), chỉ dùng để thêm " - out range" vào
  // cuối log định kỳ; bơm vẫn tắt khi cờ này true nhưng _flow_target giữ
  // nguyên giá trị q1 thật để hiện trong log (không ép về 0 nữa).
  bool     _flow_out_of_range;
  // Tốc độ ĐẶT cho mission (WP_SPEED), do Rover.cpp bơm vào qua
  // set_target_speed() mỗi chu kỳ trước update(). 0 nếu chưa từng được set
  // (vd chưa vào Auto lần nào) — dùng cho công thức FLOW_MODE=1.
  float    _target_speed;

  // ---- "Bắt tay" an toàn lúc mới boot — DÙNG CHUNG Module 1 (bơm) và
  // Module 3 (cho ăn) — mới 2026-09-08. Ngay sau khi boot/load param,
  // RC receiver có thể chưa gửi đúng vị trí thật của nấc/nút (dial chưa
  // lăn về đúng chỗ, hoặc lỡ chạm nút) — nếu tin ngay giá trị RC lúc đó
  // thì bơm/motor cho ăn có thể tự chạy ngoài ý muốn. Bắt buộc người vận
  // hành phải ARM rồi DISARM đúng 1 lần (xác nhận đã kiểm tra hệ thống)
  // thì _system_ready mới bật — trước đó CẢ 2 module đều bị ép tắt hoàn
  // toàn, bất kể RC đang ở vị trí nào. Chỉ cần 1 lần kể từ lúc boot,
  // không lặp lại ở các lần arm/disarm sau đó.
  bool     _seen_armed_once;
  bool     _system_ready;

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
  uint32_t  _pond_save_fail_ms; // lần lưu SD thất bại gần nhất (backoff, tránh
                                // giữ semaphore filesystem chung khi thẻ SD
                                // đầy/lỗi — xem _pond_save())
  bool      _pond_first_detect_done;
  bool      _ponds_dirty;
  bool      _ponds_loaded;

  // ---- MODULE 3: dosing motor state — continuous-rotation 360° servo ----
  uint16_t _dos_pwm;
  bool     _dos_config_ok;
  uint32_t _dos_warn_ms;
  bool     _dos_was_ok;
  bool     _dos_was_on;
  // false từ lúc boot cho tới khi thấy SA_DOS_RC ở vị trí OFF ít nhất 1
  // lần - chặn motor tự chạy lại nếu FC reboot (mất điện chập chờn) trong
  // lúc switch vẫn đang ở vị trí ON từ trước; buộc phải gạt OFF rồi ON
  // lại sau mỗi lần boot mới cho chạy. Không reset khi disarm (chỉ liên
  // quan tới reboot thật, không phải chu kỳ arm/disarm bình thường).
  bool     _dos_rc_seen_off;
  uint32_t _dos_last_log_ms;

  // ---- Đĩa rải ly tâm — ESC riêng qua SA_DISC_CHAN, quay TRƯỚC khi trục
  // vít bật và tắt SAU khi trục vít tắt, cách nhau SA_DISC_DLY giây (mới
  // 2026-09-08). _dos_seq_ms = thời điểm SA_DOS_RC vừa đổi trạng thái
  // ON/OFF gần nhất (dùng chung với STATUSTEXT "Dosing motor ON/OFF" đã
  // có sẵn) — dùng để tính đã trôi qua bao lâu kể từ lúc đổi trạng thái.
  uint32_t _dos_seq_ms;
  bool     _disc_running;
  uint16_t _disc_pwm;
  // Kiểm tra cấu hình kênh đĩa rải — yêu cầu FUNCTION=0(None), MIN=1000,
  // MAX=2200 (giống mẫu _check_dosing_config() của trục vít, KHÔNG yêu
  // cầu TRIM vì đĩa chỉ quay 1 chiều, không có điểm giữa cần canh).
  bool     _disc_config_ok;
  bool     _disc_was_ok;
  uint32_t _disc_warn_ms;

  // Đồng bộ SA_DOS_SP + SA_DOS_FOOD với dos_sp/dos_food riêng của ao đang
  // active (xem _sync_dosing_setpoint)
  uint8_t  _dos_sync_pond;    // ao lần đồng bộ gần nhất, 0xFF = chưa đồng bộ
  float    _dos_sp_sync_val;  // giá trị SA_DOS_SP tại lần đồng bộ gần nhất
  int8_t   _dos_food_sync_val; // giá trị SA_DOS_FOOD tại lần đồng bộ gần nhất

  // ================================================================
  // PRIVATE METHODS
  // ================================================================

  // ---- Dùng chung: bắt tay an toàn ARM+DISARM 1 lần lúc mới boot ----
  void     _update_boot_handshake(void);

  // ---- MODULE 1: flow sensor + spray control ----
  void     _update_flow(void);
  void     _update_spray_mode(void);
  void     _check_pump_config(void);
  uint16_t _run_flow_pid(float target_lmin, float dt);
  void     _write_pump_pwm(uint16_t pwm);
  float    _get_spray_speed(void);
  float    _get_dosing_ref_speed(void);
  float    _compute_visin_target(float r);
  float    _get_mission_dist(void);
  float    _speed_min_start(void);

  // ---- MODULE 2: pH sensor + alkalinity + pond persistence ----
  void     _ph_init(void);
  void     _ph_update(void);
  void     _ph_update_daily_slots(float ph_cal);
  // Log định kỳ pH ra console (SA_PH_LOG) — tách riêng (2026-09-07) để
  // dùng chung cho cả _ph_update() (cảm biến thật) và _run_simulation()
  // (SA_SIM=1); trước đó chỉ _ph_update() có, nên bật SA_SIM thì
  // SA_PH_LOG không in được gì dù giá trị pH vẫn cập nhật bình thường.
  void     _ph_print_log(uint32_t now);
  float    _ph_calc_alkalinity(float ph, float base_kh_dkh, float temp_c);
  void     _io_update(void);   // chạy trong IO thread — load/save _ponds[] an toàn với AP::FS()
  void     _pond_load(void);
  // Trả false nếu ghi SD thất bại (vd hết dung lượng) — dùng để backoff
  // trong _io_update(), tránh giữ semaphore filesystem chung quá lâu/liên tục.
  bool     _pond_save(void);

  // ---- MODULE 3: dosing motor ----
  void     _check_dosing_config(void);
  void     _check_disc_config(void);
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
