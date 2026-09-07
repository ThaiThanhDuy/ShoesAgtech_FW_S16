// =============================================================
// MODULE 2 — CẢM BIẾN pH + ĐỘ KIỀM + QUẢN LÝ AO (Giám sát nước)
// Tách ra từ AP_ShoesAgtech.cpp (2026-09-07) — chỉ di chuyển vị trí định
// nghĩa hàm sang file riêng, KHÔNG đổi bất kỳ logic nào. Toàn bộ hàm ở
// đây vẫn là method của class AP_ShoesAgtech (khai báo đầy đủ trong
// AP_ShoesAgtech.h), linker gộp chung như một file duy nhất.
// =============================================================
#include "AP_ShoesAgtech.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// =============================================================
// BẢNG THAM SỐ MODULE 2 (Giám sát nước) — var_info RIÊNG, subgroup lồng
// trong AP_ShoesAgtech (xem AP_SUBGROUPINFO trong AP_ShoesAgtech.cpp),
// tiền tố rỗng nên tên tham số không đổi so với trước (vd vẫn SA_PH_EN).
// =============================================================
const AP_Param::GroupInfo AP_ShoesAgtech_PHParams::var_info[] = {
    // @Param: PH_EN
    // @DisplayName: Enable pH sensor (Nengshi ASPS3801D-0.5M)
    // @Description: Enables Modbus RTU pH sensor via RS485-TTL adapter.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_EN", 1, AP_ShoesAgtech_PHParams, ph_en, 0),

    // @Param: PH_PORT
    // @DisplayName: UART port for pH sensor (SERIALx number)
    // @Description: Set SERIALx_BAUD=9 (9600) and SERIALx_PROTOCOL=0 (None)
    //   on the matching port.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("PH_PORT", 2, AP_ShoesAgtech_PHParams, ph_port, 2),

    // @Param: PH_TOFF
    // @DisplayName: Temperature offset (°C)
    // @Description: Added to raw sensor temperature after /10 decode.
    // @Range: -10 10
    // @User: Standard
    AP_GROUPINFO("PH_TOFF", 3, AP_ShoesAgtech_PHParams, ph_toff, -3.5f),

    // @Param: PH_OFF
    // @DisplayName: pH calibration offset
    // @Description: Added to decoded pH value. Determine using a buffer
    // solution.
    // @Range: -2.0 2.0
    // @User: Standard
    AP_GROUPINFO("PH_OFF", 4, AP_ShoesAgtech_PHParams, ph_off, 0.0f),

    // @Param: PH_KH
    // @DisplayName: Base alkalinity from test kit (dKH)
    // @Description: Reference alkalinity measured with a test kit. Used as the
    //   base value for the delta-pH alkalinity estimate. Update periodically.
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("PH_KH", 5, AP_ShoesAgtech_PHParams, ph_kh, 4.0f),

    // @Param: PH_LOG
    // @DisplayName: pH console log enable (independent of SA_FLOW_LOG)
    // @Description: Prints pH, temperature, mV and alkalinity status at
    //   SA_PH_LOG_MS interval. Independent of SA_FLOW_LOG.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_LOG", 6, AP_ShoesAgtech_PHParams, ph_log_enable, 0),

    // @Param: PH_TZ
    // @DisplayName: Local timezone offset (hours, UTC+N)
    // @Description: Local time = UTC + PH_TZ. Used to classify readings into
    //   morning [PH_MS..PH_ME] or afternoon [PH_AS..PH_AE] slots.
    // @Range: -12 14
    // @User: Standard
    AP_GROUPINFO("PH_TZ", 7, AP_ShoesAgtech_PHParams, ph_tz, 7),

    // @Param: PH_LOG_MS
    // @DisplayName: pH console log interval (ms)
    // @Description: Interval between pH console prints when SA_PH_LOG=1.
    //   Keep >= 2000 to match the Modbus poll rate.
    // @Range: 500 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("PH_LOG_MS", 8, AP_ShoesAgtech_PHParams, ph_log_ms, 2000),

    // @Param: PH_TIMEOUT
    // @DisplayName: pH disconnect timeout (s)
    // @Description: Seconds without a valid frame before a warning is sent and
    //   pH fields in SA_DATA are zeroed.
    // @Range: 1 300
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("PH_TIMEOUT", 9, AP_ShoesAgtech_PHParams, ph_timeout, 2),

    // @Param: PH_MS
    // @DisplayName: Morning slot start time (fractional hours)
    // @Description: pH readings in [PH_MS, PH_ME] update the morning slot.
    //   Use decimal: 5.5 = 05:30.
    // @Range: 0 23.99
    // @User: Standard
    AP_GROUPINFO("PH_MS", 10, AP_ShoesAgtech_PHParams, ph_ms, 5.0f),

    // @Param: PH_ME
    // @DisplayName: Morning slot end time (fractional hours, inclusive)
    // @Description: Readings at exactly this time are still captured (<=).
    //   Set 24.0 to extend to end of day.
    // @Range: 0 24
    // @User: Standard
    AP_GROUPINFO("PH_ME", 11, AP_ShoesAgtech_PHParams, ph_me, 11.0f),

    // @Param: PH_AS
    // @DisplayName: Afternoon slot start time (fractional hours)
    // @Description: pH readings in [PH_AS, PH_AE] update the afternoon slot.
    //   Use decimal: 13.5 = 13:30.
    // @Range: 0 23.99
    // @User: Standard
    AP_GROUPINFO("PH_AS", 12, AP_ShoesAgtech_PHParams, ph_as, 12.0f),

    // @Param: PH_AE
    // @DisplayName: Afternoon slot end time (fractional hours, inclusive)
    // @Description: Readings at exactly this time are still captured (<=).
    //   Set 24.0 to extend to end of day.
    // @Range: 0 24
    // @User: Standard
    AP_GROUPINFO("PH_AE", 13, AP_ShoesAgtech_PHParams, ph_ae, 16.0f),

    // @Param: PH_POND_D
    // @DisplayName: Same-pond GPS distance threshold (m)
    // @Description: Maximum distance between the robot and the pond centroid
    //   to still consider it the same pond. Increase for large ponds.
    // @Range: 10 5000
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("PH_POND_D", 14, AP_ShoesAgtech_PHParams, ph_pond_dist, 300.0f),

    // @Param: PH_CAP_S
    // @DisplayName: pH sample interval within a slot (s)
    // @Description: Minimum time between samples in a slot. Samples are
    //   last-write-wins until SA_PH_CAP_SAM samples are reached.
    // @Range: 1 3600
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("PH_CAP_S", 15, AP_ShoesAgtech_PHParams, ph_cap_s, 20),

    // @Param: PH_CAP_SAM
    // @DisplayName: pH samples required to complete a slot
    // @Description: Number of samples before the slot is reported as complete.
    //   The slot value used is the last sample written (last-write-wins).
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("PH_CAP_SAM", 16, AP_ShoesAgtech_PHParams, ph_cap_sam, 20),

    // @Param: PH_CAP_M
    // @DisplayName: pH capture radius (m)
    // @Description: Capture radius in metres, used by the companion app
    //   only. Firmware initialises this value but does not read it anywhere.
    // @Range: 1 100
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("PH_CAP_M", 17, AP_ShoesAgtech_PHParams, ph_cap_m, 1),

    AP_GROUPEND};

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

  uint8_t port_num = (uint8_t)constrain_int16(_ph_params.ph_port.get(), 0, 4);
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
      const uint32_t timeout_ms = (uint32_t)MAX(_ph_params.ph_timeout.get(), 1) * 1000U;
      bool no_data =
          (_ph_last_good_ms == 0) || (now - _ph_last_good_ms > timeout_ms);
      if (no_data && now - _ph_nodata_warn_ms >= 10000) {
        _ph_nodata_warn_ms = now;
        if (_ph_last_good_ms == 0) {
          gcs().send_text(MAV_SEVERITY_WARNING,
                          "SA: pH sensor no data yet - check RS485 wiring");
        } else {
          gcs().send_text(
              MAV_SEVERITY_WARNING,
              "SA: pH sensor lost connection (%.0fs) - check RS485 wiring",
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

  float ph_cal = constrain_float(raw_ph / 100.0f + _ph_params.ph_off.get(), 0.0f, 14.0f);
  _ph_mv = raw_mv;
  _ph_temp = raw_temp / 10.0f + _ph_params.ph_toff.get();
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
  _ph_print_log(now);
}

// =============================================================
// IN LOG pH ĐỊNH KỲ RA CONSOLE (SA_PH_LOG) — tách riêng (2026-09-07) để
// dùng chung cho cả cảm biến thật (_ph_update()) và mô phỏng
// (_run_simulation(), SA_SIM=1) — trước đó chỉ _ph_update() gọi đoạn
// này, nên bật SA_SIM thì log không in dù giá trị pH vẫn cập nhật.
// =============================================================
void AP_ShoesAgtech::_ph_print_log(uint32_t now) {
  if (_ph_params.ph_log_enable.get() <= 0 ||
      now - _ph_last_log_ms < (uint32_t)_ph_params.ph_log_ms.get()) {
    return;
  }
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
    float ms = constrain_float(_ph_params.ph_ms.get(), 0.0f, 23.99f);
    float me = constrain_float(_ph_params.ph_me.get(), 0.0f, 24.0f);
    float as_ = constrain_float(_ph_params.ph_as.get(), 0.0f, 23.99f);
    float ae = constrain_float(_ph_params.ph_ae.get(), 0.0f, 24.0f);
    int ms_h = (int)ms, ms_m = (int)((ms - (int)ms) * 60.0f + 0.5f);
    int me_h = (int)me, me_m = (int)((me - (int)me) * 60.0f + 0.5f);
    int as_h = (int)as_, as_m = (int)((as_ - (int)as_) * 60.0f + 0.5f);
    int ae_h = (int)ae, ae_m = (int)((ae - (int)ae) * 60.0f + 0.5f);
    gcs().send_text(MAV_SEVERITY_INFO,
                    "%s AM:%d:%02d-%d:%02d PM:%d:%02d-%d:%02d", ph_pfx, ms_h,
                    ms_m, me_h, me_m, as_h, as_m, ae_h, ae_m);
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
    if (_ph_params.ph_log_enable.get() > 0 && now - _slot_warn_ms >= 60000) {
      _slot_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO,
                      "[WM] No GPS - alkalinity waiting for GPS/time");
    }
    return;
  }

  // ---- Yêu cầu GPS có fix 3D ----
  int32_t cur_lat_i = 0, cur_lng_i = 0;
  const AP_GPS &gps_inst = AP::gps();
  if (gps_inst.status(0) < AP_GPS::GPS_OK_FIX_3D) {
    if (_ph_params.ph_log_enable.get() > 0 && now - _slot_warn_ms >= 60000) {
      _slot_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO,
                      "[WM] No GPS - alkalinity waiting for GPS/time");
    }
    return;
  }
  {
    const Location &loc = gps_inst.location(0);
    cur_lat_i = loc.lat;
    cur_lng_i = loc.lng;
  }

  int8_t tz = (int8_t)constrain_int16(_ph_params.ph_tz.get(), -12, 14);
  uint32_t utc_sec = (uint32_t)(utc_usec / 1000000ULL);
  uint32_t local_sec = utc_sec + (uint32_t)((int32_t)tz * 3600);
  uint32_t today = local_sec / 86400U;
  float local_h = (float)(local_sec % 86400U) / 3600.0f;
  float cur_lat_f = cur_lat_i * 1.0e-7f;
  float cur_lng_f = cur_lng_i * 1.0e-7f;

  // ---- Khung giờ của slot ----
  float ms = constrain_float(_ph_params.ph_ms.get(), 0.0f, 23.99f);
  float me = constrain_float(_ph_params.ph_me.get(), 0.0f, 24.0f);
  float as_ = constrain_float(_ph_params.ph_as.get(), 0.0f, 23.99f);
  float ae = constrain_float(_ph_params.ph_ae.get(), 0.0f, 24.0f);
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
    _ponds[pond_idx].dos_sp = _dos_params.dos_sp.get();
    _ponds[pond_idx].dos_food = _clamp_food((int8_t)_dos_params.dos_food.get());
    if (pond_idx >= _pond_count)
      _pond_count = pond_idx + 1;
    _ponds_dirty = true;
  }

  PondEntry &pond = _ponds[pond_idx];
  const unsigned disp_idx = (unsigned)pond_idx + 1;

  // ---- Thông báo khi ao thay đổi (kể cả lần đầu boot) — in 1 lần ----
  if (pond_idx != _active_pond_idx || !_pond_first_detect_done) {
    _pond_first_detect_done = true;
    gcs().send_text(MAV_SEVERITY_INFO, "[SA] Switched to pond #%u%s", disp_idx,
                    pond_is_new ? " (new pond)" : "");
    if (pond.gps_count > 5) {
      const float DEG2M = 111320.0f;
      const float coslat = cosf(cur_lat_f * DEG_TO_RAD);
      float dlat_m = (cur_lat_f - pond.center_lat) * DEG2M;
      float dlng_m = (cur_lng_f - pond.center_lng) * DEG2M * coslat;
      float dist_m = sqrtf(dlat_m * dlat_m + dlng_m * dlng_m);
      float pond_thr = constrain_float(_ph_params.ph_pond_dist.get(), 10.0f, 5000.0f);
      if (dist_m <= pond_thr) {
        gcs().send_text(MAV_SEVERITY_INFO, "[SA] Pond#%u GPS OK (%.0fm)",
                        disp_idx, (double)dist_m);
      } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Pond#%u GPS off by %.0fm - check pond location",
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
        (uint32_t)constrain_int16(_ph_params.ph_cap_s.get(), 1, 3600) * 1000U;
    uint8_t cap_min = (uint8_t)constrain_int16(_ph_params.ph_cap_sam.get(), 1, 100);

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
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Pond#%u pH AM: %.2f (%u samples)", disp_idx,
                        (double)pond.ph_morn, (unsigned)cap_min);
      }
    } else if (in_aft && now - pond.aft_last_ms >= cap_ms) {
      pond.ph_aft = ph_cal;
      pond.aft_last_ms = now;
      pond.aft_count++;
      pond.status |= 2;
      _ponds_dirty = true;
      if (pond.aft_count == cap_min && !pond.aft_reported) {
        pond.aft_reported = true;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Pond#%u pH PM: %.2f (%u samples)", disp_idx,
                        (double)pond.ph_aft, (unsigned)cap_min);
      }
    }
  }

  // ---- Tính kiềm khi ao đủ cả hai slot (chỉ tính một lần mỗi ngày) ----
  if (pond.status == 3 && !pond.alk_computed) {
    pond.delta_ph = pond.ph_aft - pond.ph_morn;
    float kh_scaled =
        _ph_params.ph_kh.get() *
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
    gcs().send_text(MAV_SEVERITY_INFO, "[SA] Pond#%u AM:%.2f PM:%.2f dPH:%.2f",
                    disp_idx, (double)pond.ph_morn, (double)pond.ph_aft,
                    (double)pond.delta_ph);
    gcs().send_text(MAV_SEVERITY_INFO, "[SA] Pond#%u Alk:%.1fdKH/%.0fmgL",
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
    // Backoff 60s sau lần lưu thất bại gần nhất (thẻ SD đầy/lỗi) — tránh
    // liên tục chiếm semaphore filesystem dùng chung với AP_Logger mỗi 5s,
    // vốn có thể khiến AP_Logger không mở được file log (EBUSY) và làm
    // main loop bị treo (INTERNAL_ERROR main_loop_stuck) khi ghi SD chậm.
    uint32_t min_interval = (_pond_save_fail_ms != 0) ? 60000U : 5000U;
    if (now_ms - _ponds_save_ms >= min_interval) {
      _ponds_save_ms = now_ms;
      if (_pond_save()) {
        _ponds_dirty = false;
        _pond_save_fail_ms = 0;
      } else {
        _pond_save_fail_ms = now_ms;
      }
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

bool AP_ShoesAgtech::_pond_save(void) {
  int fd = AP::FS().open(SA_PONDS_FILE, O_WRONLY | O_CREAT | O_TRUNC);
  if (fd < 0) {
    return false;
  }

  // Chỉ ghi đúng _pond_count slot đã từng dùng tới (không phải cả MAX_PONDS)
  // để giảm I/O thẻ SD. _pond_load() vẫn đọc an toàn: phần còn lại của
  // _ponds[] giữ nguyên trạng thái zero-init (valid=false) nếu file ngắn hơn.
  PondStateHdr hdr{SA_PONDS_MAGIC, SA_PONDS_VER, _pond_count};
  const ssize_t hdr_written = AP::FS().write(fd, &hdr, sizeof(hdr));
  const size_t data_len = sizeof(PondEntry) * _pond_count;
  const ssize_t data_written = AP::FS().write(fd, _ponds, data_len);
  AP::FS().close(fd);

  // Ghi thiếu byte (vd ENOSPC — thẻ SD đầy) — không coi là đã lưu thành
  // công. Báo 1 lần mỗi 60s (dùng chung timer backoff của _io_update) để
  // người dùng biết dữ liệu ao KHÔNG được lưu, thay vì âm thầm mất dữ liệu.
  if (hdr_written != (ssize_t)sizeof(hdr) ||
      data_written != (ssize_t)data_len) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA: could not save pond data to SD (card full/error?)");
    return false;
  }
  return true;
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

  gcs().send_text(MAV_SEVERITY_INFO, "[SA] Loaded %u ponds from SD card",
                  (unsigned)_pond_count);
}
