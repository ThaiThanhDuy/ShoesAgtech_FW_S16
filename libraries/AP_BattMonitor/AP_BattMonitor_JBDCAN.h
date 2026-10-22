#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_HAL/AP_HAL.h>

// Timeout for waiting response from JBD BMS (5 seconds)
#define AP_BATTMONITOR_JBDCAN_TIMEOUT_MICROS 5000000

class AP_BattMonitor_JBDCAN : public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_JBDCAN(AP_BattMonitor& mon,
                          AP_BattMonitor::BattMonitor_State& mon_state,
                          AP_BattMonitor_Params& params);

    // Initialize backend
    void init() override;
    // Read battery voltage and current. Should be called at 10Hz
    void read() override;
    // Return remaining capacity in percentage. Returns true if percentage is valid
    bool capacity_remaining_pct(uint8_t &percentage) const override;
    // Return BMS-reported discharge cycle count (frame 0x101)
    bool get_cycle_count(uint16_t &cycles) const override;
    // Always reports current measurement capability
    bool has_current() const override { return true; }
    // Always reports consumed energy capability
    bool has_consumed_energy() const override { return true; }
private:
    // Pointer to the active CAN interface
    AP_HAL::CANIface* _iface = nullptr;
    uint8_t _can_interface_index;
    // Timestamp of last update (us)
    uint64_t _last_update_us;
    // Timestamp of last successfully received & parsed frame (us) — used to
    // detect a stale/disconnected BMS in read(), independent of the RTR
    // send-throttle timer above
    uint64_t _last_frame_us = 0;

    // ---- Đổi 2026-10-22, theo tài liệu thực đo "JBD_BMS_CAN_Pixhawk.md" ----
    // Pin thật KHÔNG trả lời remote frame (RTR) như tài liệu CANBUS gốc mô
    // tả — phải gửi DATA frame DLC=1, data=[0x5A] tới đúng ID cần đọc. MỌI
    // frame trả lời đều có CRC-16 ở 2 byte cuối, KỂ CẢ 0x100 (tài liệu gốc
    // nói 0x100 không có CRC — sai với phần cứng thật).
    //
    // ---- Đổi tiếp 2026-10-22 (bug thứ 2, QUAN TRỌNG HƠN): bỏ hẳn
    // register_frame_callback() ----
    // callback đó chỉ được gọi khi có thứ gì đó CHỦ ĐỘNG bơm frame bằng
    // iface->receive() — bình thường là thread riêng của 1 AP_CANDriver
    // (DroneCAN/PiccoloCAN...) gắn vào interface đó. Nhưng CAN_D1_PROTOCOL
    // cố tình để =0 (Disabled) để KHÔNG driver nào chiếm chung bus với BMS
    // (xem AP_CANManager::init()) — nghĩa là KHÔNG CÓ GÌ gọi receive() cả,
    // callback không bao giờ fire, dù dây/phần cứng đúng 100%. Đây là bug
    // gốc khiến driver luôn báo Unhealthy bất kể đổi cổng/đảo dây H-L.
    // Sửa: tự gọi _iface->receive() trực tiếp trong read() để rút frame ra
    // khỏi hàng đợi RX của chính interface — đúng pattern CANSensor::loop()
    // dùng cho các cảm biến CAN tự quét (AP_CANManager/AP_CANSensor.cpp).
    //
    // State machine hỏi tuần tự, 1 ID "đang chờ" tại 1 thời điểm, TẤT CẢ
    // chạy trong read() (không còn callback riêng, nên không còn lo
    // deadlock mutex — gọi _send_request() ngay sau khi parse xong 0x100
    // là an toàn vì vẫn cùng 1 lệnh gọi read(), không lồng trong callback):
    //   _pending_id == 0 (rảnh) -> đủ UPDATE_INTERVAL_US thì hỏi 0x100
    //   0x100 trả lời OK -> hỏi ngay 0x101 trong cùng lượt read()
    //   0x101 trả lời OK -> _pending_id=0, chờ chu kỳ sau
    //   Timeout (REQUEST_TIMEOUT_US) -> thử lại CÙNG ID 1 lần (lần hỏi đầu
    //   sau khi pin ngủ chỉ đánh thức pin, không có trả lời); timeout lần 2:
    //   nếu đang chờ 0x100 -> bỏ cả chu kỳ (pin mất kết nối); nếu đang chờ
    //   0x101 -> bỏ qua, coi chu kỳ xong (vẫn còn dữ liệu 0x100 vừa nhận)
    uint16_t _pending_id = 0;   // 0 = không có request nào đang chờ trả lời
    uint64_t _sent_us = 0;      // thời điểm gửi request đang chờ
    uint8_t  _tries = 0;        // số lần đã gửi cho _pending_id (0 hoặc 1)

    // RSOC (% remaining, từ frame 0x101) — SOC do chính BMS tính/học, đúng
    // với MỌI cấu hình pack (số cell, dung lượng), không cần hardcode
    // đường cong điện áp theo 1 loại pack cụ thể.
    float _rsoc_pct = 0.0f;
    bool  _rsoc_valid = false;
    uint16_t _cycle_count = 0;

    // Gửi DATA frame yêu cầu đọc 1 ID: dlc=1, data[0]=0x5A — đúng DUY NHẤT
    // lệnh hợp lệ theo tài liệu ("không gửi bất kỳ lệnh nào khác"). Cập
    // nhật _pending_id/_sent_us; KHÔNG đụng _tries (do caller tự quản lý).
    void _send_request(uint16_t id);

    // CRC-16/Modbus (poly 0xA001, init 0xFFFF) trên các byte TRƯỚC 2 byte
    // CRC cuối frame.
    static uint16_t _crc16(const uint8_t *data, uint8_t len);
    // Kiểm tra 2 byte cuối của frame khớp CRC của phần còn lại — chấp nhận
    // cả 2 thứ tự byte (tài liệu nói mơ hồ, dù pin thật luôn gửi byte cao
    // trước) để an toàn.
    static bool _crc_ok(const uint8_t *data, uint8_t dlc);

    // Xử lý 1 frame vừa rút ra được từ _iface->receive() trong read() — chỉ
    // chấp nhận frame khớp đúng ID đang chờ, chuẩn 11-bit, không phải RTR,
    // DLC>=3 (theo đúng điều kiện tài liệu), rồi dispatch theo ID.
    void _process_frame(const AP_HAL::CANFrame &frame);

    // Parse CAN frame 0x100: tổng áp / dòng điện / dung lượng còn lại thô
    void handle_frame_100(const AP_HAL::CANFrame &frame);
    // Parse CAN frame 0x101: dung lượng đầy / số chu kỳ / RSOC
    void handle_frame_101(const AP_HAL::CANFrame &frame);

    // ---- JBD cũ (RTR, trước 2026-10-22, pin thật không trả lời kiểu này) ----
    // void handle_frame(const AP_HAL::CANFrame &frame);
    // static const uint16_t JBD_REQUEST_ID  = 0x100;
    // static const uint16_t JBD_RESPONSE_ID = 0x100;
    // -------------------------------------------------------------------

    // ---- JBD cũ (bug: dựa vào register_frame_callback(), không bao giờ
    // fire vì CAN_D1_PROTOCOL=0 không có driver nào bơm receive() — xem
    // ghi chú dài phía trên _pending_id) ----
    // uint8_t _callback_id;
    // void handle_frame_callback(uint8_t iface_num,
    //                            const AP_HAL::CANFrame &frame,
    //                            AP_HAL::CANIface::CanIOFlags flags);
    // -------------------------------------------------------------------

    // Constants for JBD CAN message IDs and timing
    static const uint16_t JBD_ID_VOLTAGE_CURRENT = 0x100; // áp/dòng/dung lượng còn lại thô
    static const uint16_t JBD_ID_CAPACITY        = 0x101; // dung lượng đầy/chu kỳ/RSOC
    // Chu kỳ hỏi đầy đủ (2 ID): tài liệu khuyến nghị 1Hz là đủ, tối đa ~5Hz
    // — đổi từ 200ms (5Hz, 1 ID) sang 1s (1Hz, 2 ID) 2026-10-22.
    static const uint32_t UPDATE_INTERVAL_US = 1000000; // 1s giữa các chu kỳ
    static const uint32_t REQUEST_TIMEOUT_US = 100000;  // 100ms chờ 1 ID (khuyến nghị 50-150ms)
    // Số frame tối đa rút khỏi hàng đợi RX mỗi lượt read() — chặn trần để
    // không chiếm quá nhiều thời gian 1 tick 10Hz nếu có rác/nhiễu trên bus.
    static const uint8_t MAX_RX_PER_TICK = 10;
};
