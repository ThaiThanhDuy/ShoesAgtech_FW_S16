#include "AP_ShoesAgtech.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

volatile uint32_t AP_ShoesAgtech::_pulse_count = 0;

// =============================================================
// Tách var_info thành nhiều bảng theo module (2026-09-07) — mỗi module
// (Flow/PH/Dosing) có var_info[] RIÊNG, đăng ký làm subgroup bên dưới
// (AP_SUBGROUPINFO), mỗi bảng có ngân sách 64 slot ĐỘC LẬP thay vì dùng
// chung 64 slot như trước. Tên tham số hiển thị KHÔNG đổi (tiền tố
// subgroup để rỗng ""). Chỉ 5 tham số thật sự dùng chung nhiều module
// mới còn khai báo trực tiếp ở đây.
// =============================================================
const AP_Param::GroupInfo AP_ShoesAgtech::var_info[] = {
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

    // @Param: SPD_START
    // @DisplayName: Minimum speed to start spreading/pumping (% of target
    // mission speed) - shared by Module 1 (FLOW_MODE=1) and Module 3
    // (DOS_MODE=2)
    // @Description: FLOW_MODE=1 (Module 1) and DOS_MODE=2 (Module 3,
    //   mission-proportional) only start spreading/pumping once ground
    //   speed reaches this percentage of the target/set mission speed
    //   (WP_SPEED, updated by DO_CHANGE_SPEED/GCS SET_SPEED) — not just any
    //   nonzero speed. 0 disables this check entirely (reverts to old
    //   behaviour: any speed above the absolute 0.05 m/s floor starts
    //   spreading/pumping). The absolute 0.05 m/s floor always applies
    //   regardless of this setting.
    // @Range: 0 100
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("SPD_START", 19, AP_ShoesAgtech, _spd_start_pct, 80),

    // Module 1 (Bơm), Module 2 (Giám sát nước), Module 3 (Cho ăn) — mỗi
    // subgroup có var_info[] riêng trong file .cpp tương ứng, tiền tố
    // rỗng "" nên tên tham số không đổi (vd vẫn SA_FLOW_MODE, SA_PH_EN...).
    AP_SUBGROUPINFO(_flow_params, "", 2, AP_ShoesAgtech, AP_ShoesAgtech_FlowParams),
    AP_SUBGROUPINFO(_ph_params,   "", 3, AP_ShoesAgtech, AP_ShoesAgtech_PHParams),
    AP_SUBGROUPINFO(_dos_params,  "", 4, AP_ShoesAgtech, AP_ShoesAgtech_DosingParams),

    AP_GROUPEND};

// =============================================================
// HÀM KHỞI TẠO (CONSTRUCTOR)
// =============================================================
AP_ShoesAgtech::AP_ShoesAgtech()
    : _last_timestamp_ms(0), _last_pulse_snapshot(0), _last_log_ms(0),
      _flow_rate_filtered(0.0f), _flow_rate_avg(0.0f), _is_initialized(false),
      _buffer_index(0), _buffer_sum(0.0f), _samples_count(0), _spray_mode(0),
      _pump_pwm(0), _flow_target(0.0f), _flow_ramp_val(0.0f),
      _pid_integral(0.0f), _pid_output_lpf(0.0f), _pid_last_ms(0),
      _last_pump_chan(-1), _last_pump_func_val(-1), _pump_config_ok(false),
      _mission_dist_m(0.0f), _mission_ncmds(0),
      _tank_empty_detected(false), _tank_empty_ms(0),
      _no_mission_warned(false), _flow_out_of_range(false),
      _target_speed(0.0f), _sim_speed(0.0f), _ph_uart(nullptr),
      _ph_update_ms(0), _ph_req_sent_ms(0), _ph_req_pending(false),
      _ph_last_good_ms(0), _ph_nodata_warn_ms(0), _ph_last_log_ms(0),
      _ph_value(0.0f), _ph_value_ma(0.0f), _ph_mv(0), _ph_temp(25.0f),
      _ph_buf_idx(0), _ph_buf_count(0), _ph_buf_sum(0.0f), _pond_count(0),
      _ph_morn_val(0.0f), _ph_morn_lat(0), _ph_morn_lng(0), _ph_aft_val(0.0f),
      _delta_ph(0.0f), _alk_dkh(0.0f), _alk_mgl(0.0f), _alk_slot_status(4),
      _alk_pond_idx(0), _active_pond_idx(0), _slot_warn_ms(0),
      _ponds_save_ms(0), _pond_save_fail_ms(0), _pond_first_detect_done(false),
      _ponds_dirty(false), _ponds_loaded(false), _dos_pwm(1500),
      _dos_config_ok(false), _dos_warn_ms(0), _dos_was_ok(false),
      _dos_was_on(false), _dos_rc_seen_off(false), _dos_last_log_ms(0),
      _dos_seq_ms(0), _disc_running(false), _disc_pwm(1500),
      _disc_config_ok(false), _disc_was_ok(false), _disc_warn_ms(0),
      _dos_sync_pond(0xFF),
      _dos_sp_sync_val(0.0f), _dos_food_sync_val(0) {
  memset(_sample_buffer, 0, sizeof(_sample_buffer));
  memset(_ph_buf, 0, sizeof(_ph_buf));
  memset(_ponds, 0, sizeof(_ponds));
  // AP_Param::setup_object_defaults() KHÔNG tự đệ quy vào subgroup lồng
  // (bỏ qua entry kiểu AP_PARAM_GROUP) — phải gọi riêng thêm 1 lần cho
  // từng object con với đúng var_info của object đó, nếu không toàn bộ
  // tham số bên trong sẽ giữ giá trị khởi tạo thô = 0 thay vì default
  // thật (2026-09-07, phát hiện sau khi tách AP_ShoesAgtech_FlowParams/
  // PHParams/DosingParams làm subgroup).
  AP_Param::setup_object_defaults(this, var_info);
  AP_Param::setup_object_defaults(&_flow_params, AP_ShoesAgtech_FlowParams::var_info);
  AP_Param::setup_object_defaults(&_ph_params, AP_ShoesAgtech_PHParams::var_info);
  AP_Param::setup_object_defaults(&_dos_params, AP_ShoesAgtech_DosingParams::var_info);
}

// =============================================================
// KHỞI ĐỘNG (INIT)
// =============================================================
void AP_ShoesAgtech::init(void) {
  if (!is_enabled()) {
    return;
  }

  uint8_t flow_pin = (uint8_t)constrain_int16(_flow_params.flow_pin.get(), 1, 200);
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

  _update_flow();
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

  // In log pH định kỳ (SA_PH_LOG) — trước 2026-09-07 chỉ _ph_update() gọi,
  // nên bật SA_SIM thì log không in được gì dù giá trị pH vẫn cập nhật.
  _ph_print_log(now);
}
