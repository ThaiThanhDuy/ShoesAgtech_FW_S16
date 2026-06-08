# SA_DATA — truyền dữ liệu AP_ShoesAgtech qua MAVLink DEBUG_FLOAT_ARRAY

Tài liệu này mô tả cấu trúc message MAVLink `DEBUG_FLOAT_ARRAY` (msg ID 350) và
quy ước dữ liệu mà firmware Rover gửi lên (gói tên `"SA_DATA"`), để bên nhận
(GCS / web backend) có thể đọc và đẩy lên server.

Nguồn phát: [`GCS_MAVLINK_Rover::send_shoesagtech_debug_arrays()`](../../Rover/GCS_MAVLink_Rover.cpp)
(đăng ký gửi định kỳ qua `case MSG_FLOW_DATA` trong `try_send_message()`,
theo tốc độ stream `SR3_EXTRA3` của Rover).

## 1. Cấu trúc message DEBUG_FLOAT_ARRAY (MAVLink common, msg ID 350)

| Field       | Kiểu            | Ý nghĩa                                               |
|-------------|-----------------|-------------------------------------------------------|
| `time_usec` | `uint64_t`      | Mốc thời gian. **Firmware gửi giá trị `millis64()`**  |
|             |                 | (mili-giây từ lúc khởi động hệ thống — giống quy ước  |
|             |                 | `time_boot_ms` ở các message khác), KHÔNG phải micro- |
|             |                 | giây như tên field gợi ý.                             |
| `name`      | `char[10]`      | Tên gói dữ liệu — luôn là chuỗi `"SA_DATA"`.          |
| `array_id`  | `uint16_t`      | ID phân biệt mảng — luôn là `0` (chỉ có một gói gộp   |
|             |                 | chung duy nhất nên field này không mang ý nghĩa).     |
| `data`      | `float[58]`     | Mảng dữ liệu — xem layout ở mục 2. Các ô không dùng   |
|             |                 | (chỉ số 9–57) luôn được điền `0.0`.                   |

> Giới hạn: `data[58]` là **trần cứng của định dạng MAVLink** (252 byte payload,
> trừ 20 byte header `time_usec`+`array_id`+`name` còn 232 byte ÷ 4 byte/float
> = 58). Không thể tăng số lượng phần tử của một message; nếu cần gửi nhiều hơn
> 58 giá trị phải tách thành nhiều message với `array_id` khác nhau.

## 2. Layout của `data[]` (gói `"SA_DATA"`, `array_id = 0`)

| Index | Tên             | Đơn vị         | Mô tả                                              | Luôn có? |
|-------|-----------------|----------------|----------------------------------------------------|----------|
| 0     | `flow_rate`     | L/min          | Lưu lượng tức thời, lọc EMA (`SA_EMA_AL`)          | ✓        |
| 1     | `flow_rate_avg` | L/min          | Lưu lượng trung bình trượt (moving-average, 10 mẫu)| ✓        |
| 2     | `flow_target`   | L/min          | Lưu lượng mục tiêu (mode 1 = `SA_FLOW_SP`, mode 2 = tính từ `SA_APP_RATE`×tốc độ×`SA_BOOM_W`) | ✓ |
| 3     | `pump_pwm`      | µs (PWM)       | Giá trị PWM hiện đang xuất ra bơm                  | ✓        |
| 4     | `ph`            | pH             | Giá trị pH trung bình trượt (moving-average)       | chỉ khi pH bật & có dữ liệu* |
| 5     | `ph_mv`         | mV             | Điện áp điện cực pH (có dấu)                       | chỉ khi pH bật & có dữ liệu* |
| 6     | `ph_temp`       | °C             | Nhiệt độ đo được từ cảm biến pH                    | chỉ khi pH bật & có dữ liệu* |
| 7     | `alk_dkh`       | dKH            | Độ kiềm ước tính (carbonate hardness)              | chỉ khi pH bật & có dữ liệu* |
| 8     | `alk_mgl`       | mg/L CaCO₃     | Độ kiềm ước tính, quy đổi mg/L CaCO₃               | chỉ khi pH bật & có dữ liệu* |
| 9–57  | _(không dùng)_  | —              | Luôn bằng `0.0`                                    | —        |

Ghi chú:
- *"pH bật & có dữ liệu"* nghĩa là CẢ HAI điều kiện: `SA_PH_EN = 1` VÀ cảm
  biến đã nhận được ít nhất một frame Modbus hợp lệ trong vòng 30 giây gần
  nhất (`ph_has_data()`). Nếu cảm biến **tắt**, **chưa từng kết nối**, hoặc
  **mất tín hiệu quá 30s** (đứt dây RS485, lỗi nguồn...), các ô 4–8 được
  RESET về `0.0` (zero-padded) thay vì giữ giá trị cũ/rác — bên nhận nên coi
  cụm 4–8 toàn `0` là "không có dữ liệu pH hợp lệ tại thời điểm này" và
  không hiển thị/lưu trữ như phép đo thật.
- Cảnh báo "mất kết nối"/"chưa có dữ liệu" cũng được phát qua `STATUSTEXT`
  (xem `gcs().send_text(MAV_SEVERITY_WARNING, "SA: pH sensor ...")` trong
  `AP_ShoesAgtech::_ph_update()`), độc lập với gói `SA_DATA` này.
- `flow_rate` và `flow_rate_avg` đã được "khử nhiễu": các giá trị tuyệt đối
  nhỏ hơn `0.01 L/min` (dư residue của bộ lọc EMA/MA khi không có dòng chảy,
  dạng số mũ kiểu `1.2e-7`) được ép về `0.0` trước khi gửi.
- Cảm biến lưu lượng YF-S402B có dải hoạt động danh định **0.3–6 L/min**;
  giá trị ngoài dải này (kể cả 0 khi không bơm) là hợp lệ về mặt dữ liệu.

## 3. Cách đọc / đẩy lên server (gợi ý dùng pymavlink)

```python
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')

while True:
    msg = mav.recv_match(type='DEBUG_FLOAT_ARRAY', blocking=True)
    if msg.name.strip('\x00') != 'SA_DATA':
        continue

    d = msg.data
    payload = {
        'time_boot_ms':   msg.time_usec,     # thực chất là millis(), xem mục 1
        'flow_rate':      d[0],
        'flow_rate_avg':  d[1],
        'flow_target':    d[2],
        'pump_pwm':       d[3],
        'ph':             d[4],
        'ph_mv':          d[5],
        'ph_temp':        d[6],
        'alk_dkh':        d[7],
        'alk_mgl':        d[8],
    }
    # requests.post('https://your-server/api/sa-data', json=payload)
```

## 4. Xem nhanh trong Mission Planner

`Ctrl+F` → **MAVLink Inspector** → tìm message `DEBUG_FLOAT_ARRAY` → entry có
`name = SA_DATA`.
