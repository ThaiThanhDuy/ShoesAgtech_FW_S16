# SA_DATA — truyền dữ liệu AP_ShoesAgtech qua MAVLink DEBUG_FLOAT_ARRAY

Tài liệu này mô tả cấu trúc message MAVLink `DEBUG_FLOAT_ARRAY` (msg ID 350) và
quy ước dữ liệu mà firmware Rover gửi lên (gói tên `"SA_DATA"`), để bên nhận
(GCS / web backend) có thể đọc và đẩy lên server.

Nguồn phát: [`GCS_MAVLINK_Rover::send_shoesagtech_debug_arrays()`](../../Rover/GCS_MAVLink_Rover.cpp)
(đăng ký gửi định kỳ qua `case MSG_FLOW_DATA` trong `try_send_message()`,
theo tốc độ stream `SR3_EXTRA3` của Rover).

## 1. Cấu trúc message DEBUG_FLOAT_ARRAY (MAVLink common, msg ID 350)

| Field       | Kiểu        | Ý nghĩa                                               |
| ----------- | ----------- | ----------------------------------------------------- |
| `time_usec` | `uint64_t`  | Mốc thời gian. **Firmware gửi giá trị `millis64()`**  |
|             |             | (mili-giây từ lúc khởi động hệ thống — giống quy ước  |
|             |             | `time_boot_ms` ở các message khác), KHÔNG phải micro- |
|             |             | giây như tên field gợi ý.                             |
| `name`      | `char[10]`  | Tên gói dữ liệu — luôn là chuỗi `"SA_DATA"`.          |
| `array_id`  | `uint16_t`  | ID phân biệt mảng — luôn là `0` (chỉ có một gói gộp   |
|             |             | chung duy nhất nên field này không mang ý nghĩa).     |
| `data`      | `float[58]` | Mảng dữ liệu — xem layout ở mục 2. Các ô không dùng   |
|             |             | (chỉ số 9–57) luôn được điền `0.0`.                   |

> Giới hạn: `data[58]` là **trần cứng của định dạng MAVLink** (252 byte payload,
> trừ 20 byte header `time_usec`+`array_id`+`name` còn 232 byte ÷ 4 byte/float
> = 58). Không thể tăng số lượng phần tử của một message; nếu cần gửi nhiều hơn
> 58 giá trị phải tách thành nhiều message với `array_id` khác nhau.

## 2. Layout của `data[]` (gói `"SA_DATA"`, `array_id = 0`)

### Module 1 — Flow sensor + Spray controller (`data[0..4]`)

| Index | Tên             | Đơn vị | Mô tả                                                                                                                                                                                                                                     | Luôn có? |
| ----- | --------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- |
| 0     | `flow_rate`     | L/min  | Lưu lượng tức thời, lọc EMA (`SA_EMA_AL`). Noise < 0.01 L/min ép về 0.                                                                                                                                                                    | ✓        |
| 1     | `flow_rate_avg` | L/min  | Lưu lượng trung bình trượt (moving-average, 10 mẫu). Noise < 0.01 L/min ép về 0.                                                                                                                                                          | ✓        |
| 2     | `flow_target`   | L/min  | Lưu lượng mục tiêu. Mode 0 = 0.0; mode 2 = `SA_APP_RATE × speed × SA_BOOM_W × 0.006`. Mode 1 phụ thuộc `SA_FLOW_MODE`: `=0` → `SA_FLOW_SP`; `=1` → `(SA_TANK_VOL × speed × 60) / mission_dist` (= 0.0 nếu chưa có mission hoặc đứng yên). | ✓        |
| 3     | `pump_pwm`      | µs     | PWM hiện đang xuất ra kênh bơm (`SA_PUMP_CHAN`). Dải 800–2200.                                                                                                                                                                            | ✓        |
| 4     | `spray_mode`    | 0/1/2  | Chế độ phun đang chạy: **0** = PASSTHROUGH, **1** = FLOW PID, **2** = AUTO RATE.                                                                                                                                                          | ✓        |

### Module 2 — pH sensor (`data[5..11]`)

| Index | Tên           | Đơn vị     | Mô tả                                                                                          | Luôn có?                          |
| ----- | ------------- | ---------- | ---------------------------------------------------------------------------------------------- | --------------------------------- |
| 5     | `ph`          | —          | Giá trị pH trung bình trượt (moving-average, 10 mẫu).                                          | chỉ khi SA_PH_EN=1 & có dữ liệu\* |
| 6     | `ph_mv`       | mV         | Điện áp điện cực pH (signed).                                                                  | chỉ khi SA_PH_EN=1 & có dữ liệu\* |
| 7     | `ph_temp`     | °C         | Nhiệt độ nước (đã bù `SA_PH_TOFF`).                                                            | chỉ khi SA_PH_EN=1 & có dữ liệu\* |
| 8     | `alk_dkh`     | dKH        | Độ kiềm ước tính (carbonate hardness).                                                         | chỉ khi SA_PH_EN=1 & có dữ liệu\* |
| 9     | `alk_mgl`     | mg/L CaCO₃ | Độ kiềm quy đổi (`alk_dkh × 17.85`).                                                           | chỉ khi SA_PH_EN=1 & có dữ liệu\* |
| 10    | `delta_ph`    | —          | ΔpH = pH chiều − pH sáng hôm nay. 0.0 nếu chưa đủ 2 slot.                                      | chỉ khi SA_PH_EN=1 & có dữ liệu\* |
| 11    | `slot_status` | 0–4        | Chất lượng dữ liệu kiềm: **0**=FULL, **1**=MORN, **2**=AFT, **3**=PREV(hôm qua), **4**=NODATA. | chỉ khi SA_PH_EN=1 & có dữ liệu\* |

### Module 3 — Dosing motor (`data[12..14]`)

| Index | Tên        | Đơn vị   | Mô tả                                                                                                            | Luôn có? |
| ----- | ---------- | -------- | ---------------------------------------------------------------------------------------------------------------- | -------- |
| 12    | `dos_sp`   | gam      | Setpoint lượng thức ăn (`SA_DOS_SP`). 0 khi chưa đặt. Ý nghĩa phụ thuộc `SA_DOS_MODE`: `=0` → gam cấp mỗi lần bật RC; `=1` → tổng gam phân bổ đều trên toàn mission. | ✓        |
| 13    | `dos_rate` | gam/50µs | Tỉ lệ quy đổi (`SA_DOS_RATE`): số gam ứng với 50µs lệch khỏi 1500.                                                | ✓        |
| 14    | `dos_pwm`  | µs       | PWM đang xuất ra kênh định lượng (`SA_DOS_CHAN`). 1500 = dừng. `SA_DOS_MODE=0`: `offset=dos_sp×50/dos_rate` (cố định). `SA_DOS_MODE=1`: `offset=(dos_sp×speed×60/mission_dist)×50/dos_rate` (tỉ lệ tốc độ), =1500 nếu chưa có mission/đứng yên. | ✓        |

### Không dùng

| Index | Giá trị      |
| ----- | ------------ |
| 15–57 | Luôn = `0.0` |

Ghi chú:

- _"pH bật & có dữ liệu"_ nghĩa là CẢ HAI điều kiện: `SA_PH_EN = 1` VÀ cảm
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
- `SA_FLOW_PIN`, `SA_TANK_VOL`, `SA_FLOW_MODE`, `SA_DOS_MODE` (slots 33–36) chỉ
  ảnh hưởng **cách tính** `data[2]`/`data[12..14]` ở trên — không đổi layout/
  số lượng index của `SA_DATA`. Khi `SA_FLOW_MODE=1` hoặc `SA_DOS_MODE=1` mà
  thiếu mission hoặc xe đứng yên, firmware trả `flow_target=0.0` / `dos_pwm=1500`
  (KHÔNG fallback về setpoint cũ) và phát STATUSTEXT cảnh báo riêng — xem
  `AP_SHOESAGTECH_REFERENCE.md` mục 6.

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
        # Module 1 — Flow sensor + Spray controller
        'flow_rate':      d[0],
        'flow_rate_avg':  d[1],
        'flow_target':    d[2],
        'pump_pwm':       d[3],
        'spray_mode':     int(d[4]),         # 0=PASSTHROUGH 1=FLOW_PID 2=AUTO_RATE
        # Module 2 — pH sensor (0.0 khi mất kết nối hoặc SA_PH_EN=0)
        'ph':             d[5],
        'ph_mv':          d[6],
        'ph_temp':        d[7],
        'alk_dkh':        d[8],
        'alk_mgl':        d[9],
        'delta_ph':       d[10],
        'slot_status':    int(d[11]),   # 0=FULL 1=MORN 2=AFT 3=PREV 4=NODATA
        # Module 3 — Dosing motor
        'dos_sp':         d[12],
        'dos_rate':       d[13],
        'dos_pwm':        d[14],
    }

```

## 4. Xem nhanh trong Mission Planner

`Ctrl+F` → **MAVLink Inspector** → tìm message `DEBUG_FLOAT_ARRAY` → entry có
`name = SA_DATA`.
