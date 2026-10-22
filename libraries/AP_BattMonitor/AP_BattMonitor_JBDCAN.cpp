#include "AP_BattMonitor_JBDCAN.h"
#include <GCS_MAVLink/GCS.h>
#include <cstdio>
#include <cstring> // memset, memcpy

extern const AP_HAL::HAL &hal;

// Constructor: store pointer to state and params
AP_BattMonitor_JBDCAN::AP_BattMonitor_JBDCAN(
    AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state,
    AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params),
      _can_interface_index(0) // Default to CAN1 (index 0)
{}

// Initialize CAN interface (default CAN2 = hal.can[1])
void AP_BattMonitor_JBDCAN::init() {
  _iface = hal.can[1];
  if (_iface == nullptr) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "JBDCAN: CAN interface not available");
    return;
  }

  // --- JBD cũ: đăng ký register_frame_callback() — KHÔNG BAO GIỜ fire vì
  // CAN_D1_PROTOCOL=0 (cố tình, để không driver nào chiếm chung bus với
  // BMS) nên không có AP_CANDriver nào bơm iface->receive() cho interface
  // này. Đây là bug gốc khiến driver luôn Unhealthy dù dây/cổng đúng. Sửa:
  // tự gọi _iface->receive() trực tiếp trong read(), xem giải thích đầy đủ
  // trong AP_BattMonitor_JBDCAN.h.
  // if (!_iface->register_frame_callback(
  //         FUNCTOR_BIND_MEMBER(&AP_BattMonitor_JBDCAN::handle_frame_callback,
  //                             void, uint8_t, const AP_HAL::CANFrame &,
  //                             AP_HAL::CANIface::CanIOFlags),
  //         _callback_id)) {
  //   GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
  //                 "JBDCAN: Failed to register frame callback");
  //   return;
  // }

  _last_update_us = AP_HAL::micros64();
  GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JBDCAN: Interface ready");
}

// Gửi DATA frame yêu cầu đọc 1 ID — đúng DUY NHẤT lệnh hợp lệ với pin thật
// (dlc=1, data[0]=0x5A). Không gửi RTR — pin không trả lời remote frame.
void AP_BattMonitor_JBDCAN::_send_request(uint16_t id) {
  AP_HAL::CANFrame frame;
  frame.id = id; // ID chuẩn 11-bit, KHÔNG set FlagRTR/FlagEFF
  frame.dlc = 1;
  frame.data[0] = 0x5A;
  frame.setCanFD(false);

  uint64_t now = AP_HAL::micros();
  int16_t ret = _iface->send(frame, now + 100000, 0);
  if (ret < 0) {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "JBDCAN: Failed to send request 0x%X",
                  unsigned(id));
  } else if (ret == 0) {
    // 0 = hàng đợi TX đầy, frame CHƯA thực sự được gửi — hiếm khi xảy ra
    // trên bus gần như rảnh (chỉ có request 1 byte của chính driver này),
    // nhưng vẫn log để không âm thầm bỏ sót nếu có.
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                  "JBDCAN: TX queue full for request 0x%X", unsigned(id));
  }
  _pending_id = id;
  _sent_us = now;
}

// Periodic reading of battery status — tự rút frame khỏi hàng đợi RX của
// interface (xem lý do trong .h), rồi chạy state machine hỏi tuần tự
// 0x100 -> 0x101.
void AP_BattMonitor_JBDCAN::read() {
  if (_iface == nullptr)
    return;

  uint64_t now = AP_HAL::micros();

  // Mark unhealthy once no valid frame has arrived within the timeout, so a
  // disconnected/dead BMS doesn't keep reporting stale voltage/current as
  // "healthy" forever.
  if (_last_frame_us == 0 ||
      now - _last_frame_us > AP_BATTMONITOR_JBDCAN_TIMEOUT_MICROS) {
    _state.healthy = false;
    _rsoc_valid = false;
  }

  // --- JBD cũ (RTR, gửi 1 ID mỗi 200ms, không chờ trả lời trước khi gửi
  // tiếp — bản này pin thật không bao giờ trả lời vì dùng RTR) ---
  // if (now - _last_update_us < 200000)
  //   return;
  // uint16_t id = 0x100;
  // AP_HAL::CANFrame rtr_frame;
  // rtr_frame.id = id | AP_HAL::CANFrame::FlagRTR;
  // rtr_frame.dlc = 0;
  // rtr_frame.setCanFD(false);
  // int16_t ret = _iface->send(rtr_frame, now + 100000, 0);
  // if (ret > 0) {
  //   _last_update_us = now;
  // } else if (ret < 0) {
  //   GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "JBDCAN: Failed to send RTR frame");
  // }
  // -------------------------------------------------------------------

  // Rút TẤT CẢ frame đang chờ trong hàng đợi RX của interface (tối đa
  // MAX_RX_PER_TICK lần/tick) — không có AP_CANDriver nào khác làm việc
  // này cho interface bị Disabled protocol này, nên PHẢI tự làm (xem .h).
  for (uint8_t i = 0; i < MAX_RX_PER_TICK; i++) {
    AP_HAL::CANFrame frame;
    uint64_t rx_ts_us;
    AP_HAL::CANIface::CanIOFlags rx_flags{};
    int16_t res = _iface->receive(frame, rx_ts_us, rx_flags);
    if (res != 1) {
      break; // 0 = hết frame trong hàng đợi, <0 = lỗi
    }
    _process_frame(frame);
  }

  if (_pending_id != 0) {
    // Đang chờ trả lời cho _pending_id
    if (now - _sent_us < REQUEST_TIMEOUT_US) {
      return; // chưa hết timeout, chờ tiếp
    }
    // Hết timeout
    _tries++;
    if (_tries < 2) {
      // Lần hỏi đầu sau khi pin ngủ chỉ đánh thức pin — thử lại 1 lần nữa
      _send_request(_pending_id);
      return;
    }
    // Đã thử 2 lần mà vẫn không có trả lời
    if (_pending_id == JBD_ID_VOLTAGE_CURRENT) {
      // ID quan trọng nhất không trả lời -> bỏ cả chu kỳ, coi như mất kết
      // nối (chờ chu kỳ sau, _state.healthy đã được timeout tổng ở trên lo)
      _pending_id = 0;
      return;
    }
    // ID phụ (0x101) không trả lời -> bỏ qua, coi chu kỳ đã xong (vẫn còn
    // dữ liệu 0x100 vừa nhận được)
    _pending_id = 0;
    return;
  }

  // Không có request nào đang chờ — bắt đầu chu kỳ mới nếu đã đủ khoảng
  // cách UPDATE_INTERVAL_US kể từ chu kỳ trước
  if (now - _last_update_us < UPDATE_INTERVAL_US) {
    return;
  }
  _last_update_us = now;
  _tries = 0;
  _send_request(JBD_ID_VOLTAGE_CURRENT);
}

// Xử lý 1 frame vừa rút được từ _iface->receive() — chỉ chấp nhận frame
// khớp đúng ID đang chờ, chuẩn 11-bit, không phải RTR, DLC>=3.
void AP_BattMonitor_JBDCAN::_process_frame(const AP_HAL::CANFrame &frame) {
  // --- JBD cũ (callback riêng, không bao giờ fire — xem .h) ---
  // void AP_BattMonitor_JBDCAN::handle_frame_callback(...) {
  //   if ((frame.id & 0xFFFF) == 0x100) {
  //     handle_frame(frame);
  //   }
  // }
  // -----------------------------------------------------------
  if (_pending_id == 0) {
    return; // không chờ gì cả, bỏ qua
  }
  if (frame.isRemoteTransmissionRequest() || frame.isExtended()) {
    return;
  }
  if (frame.dlc < 3) {
    return;
  }
  const uint32_t id = frame.id & AP_HAL::CANFrame::MaskStdID;
  if (id != _pending_id) {
    return;
  }

  if (id == JBD_ID_VOLTAGE_CURRENT) {
    handle_frame_100(frame);
  } else if (id == JBD_ID_CAPACITY) {
    handle_frame_101(frame);
  }
}

// CRC-16/Modbus (poly 0xA001, init 0xFFFF) trên các byte trước 2 byte CRC
uint16_t AP_BattMonitor_JBDCAN::_crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// 2 byte cuối frame phải khớp CRC của phần còn lại — chấp nhận cả 2 thứ tự
// byte (pin thật luôn gửi byte cao trước, nhưng chấp nhận cả 2 cho an toàn)
bool AP_BattMonitor_JBDCAN::_crc_ok(const uint8_t *data, uint8_t dlc) {
  if (dlc < 3) {
    return false;
  }
  uint16_t crc = _crc16(data, dlc - 2);
  uint8_t hi = (uint8_t)(crc >> 8);
  uint8_t lo = (uint8_t)(crc & 0xFF);
  return (data[dlc - 2] == hi && data[dlc - 1] == lo) ||
         (data[dlc - 2] == lo && data[dlc - 1] == hi);
}

// Process 0x100: tổng áp, dòng điện, dung lượng còn lại thô (8 byte, 2 byte
// CRC cuối — KHÁC tài liệu CANBUS gốc nói 0x100 không có CRC, đã kiểm chứng
// trên pin thật là CÓ, xem AP_BattMonitor_JBDCAN.h).
void AP_BattMonitor_JBDCAN::handle_frame_100(const AP_HAL::CANFrame &frame) {
  if (frame.dlc < 8 || !_crc_ok(frame.data, frame.dlc)) {
    return;
  }
  const uint8_t *data = frame.data;

  // BYTE0~1: tổng áp, unsigned, high byte trước, đơn vị 10mV -> *0.01 = V
  uint16_t raw_voltage = ((uint16_t)data[0] << 8) | data[1];
  float voltage = raw_voltage * 0.01f;

  // BYTE2~3: dòng điện, signed, đơn vị 10mA. BMS: sạc=dương, xả=âm.
  // ArduPilot: current_amps dương khi XẢ (current draw) — NGƯỢC với BMS,
  // nên phải đảo dấu ở đây.
  int16_t raw_current = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  float current = -raw_current * 0.01f;

  _state.voltage = voltage;
  _state.current_amps = current;
  _state.healthy = true;
  _last_frame_us = AP_HAL::micros();

  // Nhận 0x100 xong -> hỏi ngay 0x101 TRONG CÙNG LƯỢT read() này — an toàn
  // vì không còn chạy trong callback riêng nữa (đã bỏ register_frame_
  // callback(), xem .h), không có nguy cơ deadlock như thiết kế trước.
  _pending_id = 0;
  _tries = 0;
  _send_request(JBD_ID_CAPACITY);
}

// Process 0x101: dung lượng đầy, số chu kỳ, RSOC (8 byte, 2 byte CRC cuối)
void AP_BattMonitor_JBDCAN::handle_frame_101(const AP_HAL::CANFrame &frame) {
  if (frame.dlc < 8 || !_crc_ok(frame.data, frame.dlc)) {
    return;
  }
  const uint8_t *data = frame.data;

  // BYTE0~1: dung lượng đầy, unsigned, high byte trước, đơn vị 10mAh.
  // Widen sang uint32_t trước khi *10 — raw có thể tới 65535.
  uint32_t full_capacity_mah =
      (uint32_t)(((uint16_t)data[0] << 8) | data[1]) * 10U;
  // BYTE2~3: số chu kỳ xả, unsigned
  _cycle_count = ((uint16_t)data[2] << 8) | data[3];
  // BYTE4~5: RSOC, unsigned, đơn vị %
  uint16_t rsoc = ((uint16_t)data[4] << 8) | data[5];

  _rsoc_pct = constrain_float((float)rsoc, 0.0f, 100.0f);
  _rsoc_valid = true;

  // Suy consumed_mah/wh từ RSOC + dung lượng đầy do chính BMS học được,
  // thay vì tự tích phân dòng điện hoặc giả định đường cong điện áp cố
  // định cho 1 loại pack cụ thể.
  if (full_capacity_mah > 0) {
    _state.consumed_mah = full_capacity_mah * (1.0f - _rsoc_pct * 0.01f);
    if (_state.voltage > 0) {
      _state.consumed_wh = _state.consumed_mah * _state.voltage * 0.001f;
    }
  }

  _last_frame_us = AP_HAL::micros();
  // Chu kỳ hoàn tất — chờ UPDATE_INTERVAL_US rồi bắt đầu chu kỳ sau
  _pending_id = 0;
}

// Return BMS-reported RSOC directly — đúng với MỌI cấu hình pack, không
// hardcode ngưỡng điện áp theo 1 loại pack cụ thể.
// --- JBD cũ: tự suy % từ đường cong điện áp hardcode cho pin 14S
// (58.8V/44.8V), rồi suy ngược consumed_mah từ đó ---
// bool AP_BattMonitor_JBDCAN::capacity_remaining_pct(uint8_t &percentage) const {
//   if (_params._pack_capacity <= 0) {
//     return false;
//   }
//   percentage = 100 * (_params._pack_capacity - _state.consumed_mah) /
//                _params._pack_capacity;
//   return true;
// }
// -----------------------------------------------------------------
bool AP_BattMonitor_JBDCAN::capacity_remaining_pct(uint8_t &percentage) const {
  if (!_rsoc_valid || !_state.healthy) {
    return false;
  }
  percentage = (uint8_t)(_rsoc_pct + 0.5f);
  return true;
}

bool AP_BattMonitor_JBDCAN::get_cycle_count(uint16_t &cycles) const {
  if (!_rsoc_valid) {
    return false;
  }
  cycles = _cycle_count;
  return true;
}
