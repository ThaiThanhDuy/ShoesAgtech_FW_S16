# SA_DATA — MAVLink DEBUG_FLOAT_ARRAY
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `Rover/GCS_MAVLink_Rover.cpp` — `send_shoesagtech_debug_arrays()`
**Loại:** `[ ] Module mới   [x] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất gửi:** theo `MAVx_EXTRA3` (khuyến nghị 2 Hz)
**Ngày hoàn thành:** 2026-05-15

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
GCS_MAVLINK scheduler [ArduPilot STREAM_EXTRA3]
    │
    └──► try_send_message(MSG_FLOW_DATA)
              │
              └──► send_shoesagtech_debug_arrays(chan)
                        │
                        ├──► shoesagtech.is_enabled()    → bool
                        ├──► HAVE_PAYLOAD_SPACE(chan, DEBUG_FLOAT_ARRAY)
                        │
                        ├──► float data[58] = {0}        -- zero-init toàn bộ
                        │
                        ├──► data[0..4] = Module 1 getters
                        │         shoesagtech.get_flow_rate_lmin()
                        │         shoesagtech.get_flow_rate_avg()
                        │         shoesagtech.get_flow_target()
                        │         shoesagtech.get_pump_pwm()
                        │         shoesagtech.get_spray_mode()
                        │
                        ├──► (SA_PH_EN=1 AND ph_has_data())
                        │         data[5..11] = Module 2 getters
                        │         shoesagtech.get_ph()
                        │         shoesagtech.get_ph_mv()
                        │         shoesagtech.get_ph_temp()
                        │         shoesagtech.get_alk_dkh()
                        │         shoesagtech.get_alk_mgl()
                        │         shoesagtech.get_delta_ph()
                        │         shoesagtech.get_alk_slot_status()
                        │
                        ├──► data[12..14] = Module 3 getters
                        │         shoesagtech.get_dosing_sp()
                        │         shoesagtech.get_dosing_rate()
                        │         shoesagtech.get_dosing_pwm()
                        │
                        └──► mavlink_msg_debug_float_array_send(
                                  chan, millis64(), "SA_DATA", 0, data)
```

### 1.2 Mô tả từng hàm

---

**`send_shoesagtech_debug_arrays(mavlink_channel_t chan)`**
- **File:** `Rover/GCS_MAVLink_Rover.cpp`
- **Được gọi bởi:** `GCS_MAVLINK_Rover::try_send_message()` khi `MSG_FLOW_DATA` được schedule theo stream EXTRA3
- **Đầu vào:**
  - `chan` — MAVLink channel (MAV_CHAN_1, v.v.)
- **Xử lý:**
  1. Nếu `!shoesagtech.is_enabled()` → return false (không gửi)
  2. Kiểm tra payload space (`HAVE_PAYLOAD_SPACE`) — nếu không đủ → return false (back-pressure)
  3. Khai báo `float data[58] = {0.0f}` — zero-init quan trọng (pH fallback = 0 khi mất kết nối)
  4. Điền Module 1 (data[0..4]): luôn luôn
  5. Điền Module 2 (data[5..11]): chỉ khi `ph_is_enabled() && ph_has_data()`; ngược lại giữ 0.0
  6. Điền Module 3 (data[12..14]): luôn luôn
  7. Gọi `mavlink_msg_debug_float_array_send(chan, millis64(), "SA_DATA", 0, data)`
- **Đầu ra / Return:** `bool` — true nếu message đã được gửi
- **Ghi chú:** `time_usec` truyền vào là `millis64()` — đơn vị **milliseconds từ boot**, KHÔNG phải microseconds. Tên field gây nhầm lẫn; phía nhận phải đọc như `time_boot_ms`.

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot / Param | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_ENABLE` | 1 | Int8 | 1 | 0 | 1 | Bật/tắt library. `=0` → không gửi SA_DATA. |
| `MAV1_EXTRA3` | ArduPilot param | Int8 | 0 | 0 | 50 | Tốc độ stream EXTRA3 (Hz). Cần ≥ 1 để nhận SA_DATA. |

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

N/A — SA_DATA không có init code riêng. Message được gửi định kỳ theo scheduler GCS_MAVLINK khi `SA_ENABLE=1` và stream EXTRA3 được bật.

### 3.2 Cấu trúc MAVLink message

| Field | Kiểu C | Giá trị / Ghi chú |
|---|---|---|
| `time_usec` | uint64_t | `millis64()` — **milliseconds** từ boot. Tên gây nhầm: KHÔNG phải microseconds. |
| `name` | char[10] | `"SA_DATA"` (7 ký tự + null padding) |
| `array_id` | uint16_t | `0` — chỉ có 1 gói, không dùng để phân biệt |
| `data[58]` | float[58] | Xem layout mục 4; index 15–57 luôn = 0.0 |

**Giới hạn cứng MAVLink:**
```
252 byte payload − 20 byte header = 232 byte ÷ 4 byte/float = 58 float tối đa
Nếu cần thêm field → dùng array_id=1 (message riêng biệt)
```

### 3.3 Luồng gửi (pseudocode)

```
send_shoesagtech_debug_arrays(chan):
    if (!SA_ENABLE) return false

    float data[58] = {0};

    // Module 1: luôn điền
    data[0] = get_flow_rate_lmin();    data[1] = get_flow_rate_avg();
    data[2] = get_flow_target();       data[3] = (float)get_pump_pwm();
    data[4] = (float)get_spray_mode();

    // Module 2: chỉ khi SA_PH_EN=1 AND có dữ liệu hợp lệ
    if (ph_is_enabled() && ph_has_data()) {
        data[5]  = get_ph();           data[6]  = get_ph_mv();
        data[7]  = get_ph_temp();      data[8]  = get_alk_dkh();
        data[9]  = get_alk_mgl();      data[10] = get_delta_ph();
        data[11] = (float)get_alk_slot_status();
    }   // else data[5..11] = 0.0 (đã zero-init)

    // Module 3: luôn điền
    data[12] = get_dosing_sp();
    data[13] = get_dosing_rate();
    data[14] = (float)get_dosing_pwm();

    mavlink_msg_debug_float_array_send(chan, millis64(), "SA_DATA", 0, data);
    return true;
```

### 3.4 Xử lý edge cases

| Điều kiện | Hành vi | Phía GCS nhận |
|---|---|---|
| `SA_ENABLE=0` | Không gửi message | Không có DEBUG_FLOAT_ARRAY từ firmware |
| `MAV1_EXTRA3=0` | Không stream EXTRA3 | Không nhận dù firmware sẵn sàng |
| `ph_has_data()=false` | data[5..11] = 0.0 | Không thể phân biệt "tắt pH" vs "mất kết nối" từ SA_DATA |
| Payload space không đủ | Back-pressure: return false, retry lần sau | GCS có thể bỏ lỡ một vài gói |

**Phân biệt `SA_PH_EN=0` vs mất kết nối:**
```
Cả hai → data[5..11] = 0.0.
Phân biệt bằng cách đọc tham số SA_PH_EN qua PARAM_REQUEST_READ.
Heuristic GCS: data[5]==0 AND data[6]==0 AND data[7]==0 → cảnh báo "pH không có dữ liệu"
```

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA — Layout data[58]

**Module 1 — Flow (data[0..4]) — luôn có:**

| Index | Tên | Đơn vị | Mô tả chi tiết |
|---|---|---|---|
| 0 | `flow_rate` | L/min | EMA tức thời; < 0.01 ép = 0.0 |
| 1 | `flow_rate_avg` | L/min | Moving avg 10 mẫu; < 0.01 ép = 0.0 |
| 2 | `flow_target` | L/min | Setpoint; =0 khi mode 0 hoặc FM1 không đủ điều kiện |
| 3 | `pump_pwm` | µs | PWM thực xuất bơm; dải MIN–MAX từ SERVOx |
| 4 | `spray_mode` | 0/1/2 | 0=PASSTHROUGH, 1=FLOW PID, 2=AUTO RATE |

**Module 2 — pH (data[5..11]) — chỉ khi `SA_PH_EN=1 AND ph_has_data()`:**

| Index | Tên | Đơn vị | Mô tả chi tiết |
|---|---|---|---|
| 5 | `ph` | — | pH moving average 10 mẫu (`_ph_value_ma`) |
| 6 | `ph_mv` | mV | Điện áp điện cực (signed int16) |
| 7 | `ph_temp` | °C | Nhiệt độ đã bù SA_PH_TOFF |
| 8 | `alk_dkh` | dKH | Kiềm ước tính (today hoặc yesterday) |
| 9 | `alk_mgl` | mg/L CaCO₃ | `alk_dkh × 17.85` |
| 10 | `delta_ph` | — | pH_aft − pH_morn hôm nay; 0 nếu chưa đủ 2 slot |
| 11 | `slot_status` | 0–4 | 0=FULL, 1=MORN, 2=AFT, 3=PREV, 4=NODATA |

**Module 3 — Dosing (data[12..14]) — luôn có:**

| Index | Tên | Đơn vị | Mô tả chi tiết |
|---|---|---|---|
| 12 | `dos_sp` | gam | `SA_DOS_SP`; ý nghĩa phụ thuộc SA_DOS_MODE |
| 13 | `dos_rate` | gam/50µs | `SA_DOS_RATE` |
| 14 | `dos_pwm` | µs | PWM thực xuất; 1500=dừng |

| 15–57 | — | — | Luôn = 0.0 (zero-padded, reserved) |

### 4.2 DataFlash Log [DATA]

N/A — SA_DATA chỉ tồn tại dưới dạng MAVLink message. DataFlash log theo từng module (FLWD, PHWD).

### 4.3 Tích hợp phía nhận

**pymavlink:**
```python
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
while True:
    msg = mav.recv_match(type='DEBUG_FLOAT_ARRAY', blocking=True)
    if msg is None or msg.name.rstrip('\x00') != 'SA_DATA':
        continue
    d = msg.data
    payload = {
        'time_boot_ms':  msg.time_usec,   # millis() — KHÔNG phải µs
        'flow_rate':     d[0],  'flow_rate_avg': d[1],
        'flow_target':   d[2],  'pump_pwm':      int(d[3]),
        'spray_mode':    int(d[4]),
        'ph':            d[5],  'ph_mv':         d[6],
        'ph_temp':       d[7],  'alk_dkh':       d[8],
        'alk_mgl':       d[9],  'delta_ph':      d[10],
        'slot_status':   int(d[11]),
        'dos_sp':        d[12], 'dos_rate':      d[13],
        'dos_pwm':       int(d[14]),
    }
    payload['ph_connected'] = not (d[5]==0.0 and d[6]==0.0 and d[7]==0.0)
```

**QGroundControl FactGroup (JSON):**
```json
{
  "messageName": "DEBUG_FLOAT_ARRAY",
  "filter": { "name": "SA_DATA" },
  "fields": [
    { "index": 0,  "name": "FlowRate",   "units": "L/min",  "decimals": 2 },
    { "index": 1,  "name": "FlowAvg",    "units": "L/min",  "decimals": 2 },
    { "index": 2,  "name": "FlowTarget", "units": "L/min",  "decimals": 2 },
    { "index": 3,  "name": "PumpPWM",    "units": "µs",     "decimals": 0 },
    { "index": 4,  "name": "SprayMode",  "units": "",       "decimals": 0 },
    { "index": 5,  "name": "pH",         "units": "",       "decimals": 2 },
    { "index": 6,  "name": "pH_mV",      "units": "mV",     "decimals": 0 },
    { "index": 7,  "name": "WaterTemp",  "units": "°C",     "decimals": 1 },
    { "index": 8,  "name": "AlkDKH",     "units": "dKH",    "decimals": 2 },
    { "index": 9,  "name": "AlkMGL",     "units": "mg/L",   "decimals": 1 },
    { "index": 10, "name": "DeltapH",    "units": "",       "decimals": 3 },
    { "index": 11, "name": "SlotStatus", "units": "",       "decimals": 0 },
    { "index": 12, "name": "DosSP",      "units": "g",      "decimals": 0 },
    { "index": 13, "name": "DosRate",    "units": "g/50µs", "decimals": 1 },
    { "index": 14, "name": "DosPWM",     "units": "µs",     "decimals": 0 }
  ]
}
```

**Logic hiển thị slot_status (data[11]):**

| Giá trị | Ý nghĩa | Màu gợi ý |
|---|---|---|
| 0 | FULL — ΔpH chính xác (sáng + chiều hôm nay) | Xanh lá |
| 1 | MORN — chỉ có slot sáng | Vàng |
| 2 | AFT — chỉ có slot chiều | Vàng |
| 3 | PREV — dùng dữ liệu kiềm hôm qua | Cam |
| 4 | NODATA — chưa có dữ liệu slot nào | Đỏ |

**Kiểm tra nhanh Mission Planner:**
```
Ctrl+F → MAVLink Inspector → DEBUG_FLOAT_ARRAY → name = "SA_DATA"
```

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
MAV1_EXTRA3 ≥ 1     → cần đặt để GCS nhận SA_DATA (khuyến nghị 2)
SA_ENABLE = 1       → bắt buộc để firmware gửi
```

**Ràng buộc phần cứng:** N/A — truyền qua telemetry / USB / UDP đã có sẵn

**Ràng buộc vận hành:** Đặt `MAV1_EXTRA3 = 2` trong Mission Planner trước khi fly

---

## 6. Kết nối phần cứng [HW]

N/A — SA_DATA truyền hoàn toàn qua MAVLink (telemetry radio, USB, hoặc UDP).

---

## 7. So sánh với Basic Design [ALL]

| Điểm | Basic Design dự kiến | Thực tế đã làm | Lý do |
|---|---|---|---|
| `time_usec` = microseconds | Giả định là µs như tên field gợi ý | Thực tế dùng `millis64()` (milliseconds) | MAVLink spec cho phép; đơn giản hóa; tên field gây nhầm — ghi chú rõ ở Detail Design |
| pH zero khi tắt | Điền 0 | Implement đúng: zero-init array + chỉ điền khi `ph_has_data()` ✓ | — |
| Giới hạn 58 float | Đề cập 58 max | Implement đúng; data[15..57]=0.0 reserved ✓ | — |

---

## 8. Tài liệu liên quan [ALL]

- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — nguồn gốc data[0..4]
- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — nguồn gốc data[5..11]
- [MODULE3_DOS_DETAIL_DESIGN.md](MODULE3_DOS_DETAIL_DESIGN.md) — nguồn gốc data[12..14]
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
