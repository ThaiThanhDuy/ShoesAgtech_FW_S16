# SA_DATA — MAVLink DEBUG_FLOAT_ARRAY
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `Rover/GCS_MAVLink_Rover.cpp` — `GCS_MAVLINK_Rover::send_shoesagtech_debug_arrays()` (dòng 279)
**Loại:** `[ ] Module mới   [x] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất gửi:** SA_DATA theo `MAVx_EXTRA3` (khuyến nghị 2 Hz); SA_PHK gửi ngay khi có ao vừa tính đủ kiềm (không theo lịch stream)
**Cập nhật lần cuối:** 2026-07-16

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
GCS_MAVLINK scheduler [ArduPilot STREAM_EXTRA3]
    │
    └──► try_send_message(MSG_FLOW_DATA)
              │
              └──► send_shoesagtech_debug_arrays()   [Rover/GCS_MAVLink_Rover.cpp : 279]
                        │
                        ├──► sa.is_enabled()  → false: return (không gửi gì)
                        ├──► HAVE_PAYLOAD_SPACE(chan, DEBUG_FLOAT_ARRAY)  → false: return
                        │
                        ├──► float data[58] = {}        -- zero-init toàn bộ
                        │
                        ├──► data[0..4] = Module 1 getters (luôn điền)
                        │         get_flow_rate_lmin(), get_flow_rate_avg()   [noise floor 0.01 ép về 0]
                        │         get_flow_target(), get_pump_pwm(), get_spray_mode()
                        │
                        ├──► (ph_is_enabled() AND ph_has_data())
                        │         data[5..14] = Module 2 getters (ao đang active)
                        │         get_ph(), get_ph_mv(), get_ph_temp()
                        │         get_active_alk_dkh(), get_active_alk_mgl(), get_active_delta_ph()
                        │         get_active_pond_idx()+1, get_active_ph_morn(), get_active_ph_aft()
                        │         get_active_last_day()
                        │
                        ├──► data[15..18] = Module 3 getters (ao đang active, luôn điền)
                        │         get_active_dos_sp(), get_active_dos_rate()
                        │         get_dosing_pwm(), get_active_dos_food()
                        │
                        ├──► mavlink_msg_debug_float_array_send(
                        │         chan, millis64(), "SA_DATA", array_id=0, data)
                        │
                        └──► (ph_is_enabled() AND consume_gcs_alk_pending())
                                  float phk[58] = {}
                                  phk[0..7] = pH sáng/chiều/ΔpH/kiềm/GPS/pond_idx của ao VỪA đạt FULL
                                  mavlink_msg_debug_float_array_send(
                                      chan, millis64(), "SA_PHK", array_id=1, phk)
```

### 1.2 Mô tả từng hàm

---

**`send_shoesagtech_debug_arrays()`**
- **File:** `Rover/GCS_MAVLink_Rover.cpp : 279`
- **Được gọi bởi:** `GCS_MAVLINK_Rover::try_send_message()` khi `MSG_FLOW_DATA` được schedule theo stream EXTRA3
- **Đầu vào:** không có tham số — dùng `chan` thành viên của `GCS_MAVLINK_Rover`
- **Xử lý:**
  1. Nếu `!sa.is_enabled()` → return (không gửi)
  2. Kiểm tra payload space (`HAVE_PAYLOAD_SPACE`) — nếu không đủ → return (back-pressure)
  3. Khai báo `float data[58] = {}` — zero-init quan trọng (pH fallback = 0 khi mất kết nối)
  4. Điền Module 1 (data[0..4]): luôn luôn; `flow_rate`/`flow_rate_avg` được "declutter" — trị tuyệt đối < 0.01 ép về 0.0 (tránh hiển thị số dạng `1.2e-7` gây nhiễu trên GCS)
  5. Điền Module 2 (data[5..14]): chỉ khi `ph_is_enabled() && ph_has_data()`; ngược lại giữ nguyên 0.0 — tất cả lấy từ **ao đang active** (`get_active_*`)
  6. Điền Module 3 (data[15..18]): luôn luôn, cũng lấy từ ao đang active
  7. Gọi `mavlink_msg_debug_float_array_send(chan, now_ms, "SA_DATA", 0, data)` với `now_ms = AP_HAL::millis64()`
  8. **Riêng biệt:** nếu `ph_is_enabled() && consume_gcs_alk_pending()` trả `true` (một ao vừa tính đủ kiềm lần đầu trong ngày) → dựng mảng `phk[58]` khác và gửi thêm 1 bản tin `DEBUG_FLOAT_ARRAY` tên `"SA_PHK"`, `array_id=1`
- **Đầu ra / Return:** `void`
- **Ghi chú:** `time_usec` truyền vào là `AP_HAL::millis64()` — đơn vị **milliseconds từ boot**, KHÔNG phải microseconds. Tên field gây nhầm lẫn; phía nhận phải đọc như `time_boot_ms`. SA_PHK dùng cùng hàm gửi, cùng chu kỳ gọi (theo EXTRA3), nhưng thực tế chỉ có nội dung khi vừa có ao FULL — không phải "stream định kỳ" như SA_DATA.

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot / Param | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_ENABLE` | 1 | Int8 | 1 | 0 | 1 | Bật/tắt library. `=0` → không gửi SA_DATA lẫn SA_PHK. |
| `SA_PH_EN` | 14 | Int8 | 0 | 0 | 1 | `=0` → data[5..14] luôn 0.0, và không bao giờ gửi SA_PHK. |
| `MAV1_EXTRA3` | ArduPilot param | Int8 | 0 | 0 | 50 | Tốc độ stream EXTRA3 (Hz). Cần ≥ 1 để nhận SA_DATA/SA_PHK. |

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

N/A — SA_DATA/SA_PHK không có init code riêng. Gửi định kỳ theo scheduler GCS_MAVLINK khi `SA_ENABLE=1` và stream EXTRA3 được bật.

### 3.2 Cấu trúc MAVLink message

| Field | Kiểu C | Giá trị / Ghi chú |
|---|---|---|
| `time_usec` | uint64_t | `AP_HAL::millis64()` — **milliseconds** từ boot. Tên gây nhầm: KHÔNG phải microseconds. |
| `name` | char[10] | `"SA_DATA"` (array_id=0) hoặc `"SA_PHK"` (array_id=1) |
| `array_id` | uint16_t | `0` = SA_DATA, `1` = SA_PHK — dùng để GCS lọc đúng message |
| `data[58]` | float[58] | SA_DATA: index 19–57 luôn 0.0. SA_PHK: chỉ dùng phk[0..7], phk[8..57]=0.0 |

**Giới hạn cứng MAVLink:**
```
252 byte payload − 20 byte header = 232 byte ÷ 4 byte/float = 58 float tối đa
SA_DATA (array_id=0) và SA_PHK (array_id=1) là 2 message riêng biệt cùng dùng giới hạn này
```

### 3.3 Luồng gửi (pseudocode)

```
send_shoesagtech_debug_arrays():
    if (!sa.is_enabled()) return
    if (!HAVE_PAYLOAD_SPACE(chan, DEBUG_FLOAT_ARRAY)) return

    now_ms = AP_HAL::millis64()
    float data[58] = {}

    // Module 1: luôn điền
    data[0] = declutter(get_flow_rate_lmin())   // |x|<0.01 -> 0.0
    data[1] = declutter(get_flow_rate_avg())
    data[2] = get_flow_target()
    data[3] = (float)get_pump_pwm()
    data[4] = (float)get_spray_mode()

    // Module 2: chỉ khi SA_PH_EN=1 AND ph_has_data() -- ao đang active
    if (ph_is_enabled() && ph_has_data()) {
        data[5]  = get_ph();                data[6]  = get_ph_mv()
        data[7]  = get_ph_temp();           data[8]  = get_active_alk_dkh()
        data[9]  = get_active_alk_mgl();    data[10] = get_active_delta_ph()
        data[11] = (float)(get_active_pond_idx()+1)
        data[12] = get_active_ph_morn();    data[13] = get_active_ph_aft()
        data[14] = (float)get_active_last_day()
    }   // else data[5..14] = 0.0 (đã zero-init)

    // Module 3: luôn điền -- ao đang active
    data[15] = get_active_dos_sp()
    data[16] = get_active_dos_rate()
    data[17] = (float)get_dosing_pwm()
    data[18] = (float)get_active_dos_food()

    mavlink_msg_debug_float_array_send(chan, now_ms, "SA_DATA", 0, data)

    // SA_PHK -- độc lập, chỉ gửi khi vừa có ao đạt FULL
    if (ph_is_enabled() && consume_gcs_alk_pending()) {
        float phk[58] = {}
        phk[0] = get_ph_morn();      phk[1] = get_ph_aft()
        phk[2] = get_delta_ph();     phk[3] = get_alk_dkh()
        phk[4] = get_alk_mgl()
        phk[5] = get_ph_morn_lat() * 1e-7;   phk[6] = get_ph_morn_lng() * 1e-7
        phk[7] = (float)(get_alk_pond_idx()+1)
        mavlink_msg_debug_float_array_send(chan, now_ms, "SA_PHK", 1, phk)
    }
```

### 3.4 Xử lý edge cases

| Điều kiện | Hành vi | Phía GCS nhận |
|---|---|---|
| `SA_ENABLE=0` | Không gửi message nào | Không có DEBUG_FLOAT_ARRAY từ firmware |
| `MAV1_EXTRA3=0` | Không stream EXTRA3 | Không nhận dù firmware sẵn sàng |
| `SA_PH_EN=0` hoặc `ph_has_data()=false` | data[5..14] = 0.0; SA_PHK không bao giờ gửi | Không thể phân biệt "tắt pH" vs "mất kết nối" chỉ từ SA_DATA |
| Payload space không đủ | Return sớm, thử lại chu kỳ sau | GCS có thể bỏ lỡ một vài gói |
| Chưa có ao nào đạt FULL | Không gửi SA_PHK | Không nhận `"SA_PHK"` cho đến khi có ao FULL |

**Phân biệt `SA_PH_EN=0` vs mất kết nối:**
```
Cả hai → data[5..14] = 0.0.
Phân biệt bằng cách đọc tham số SA_PH_EN qua PARAM_REQUEST_READ.
Heuristic GCS: data[5]==0 AND data[6]==0 AND data[7]==0 → cảnh báo "pH không có dữ liệu"
```

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA — Layout data[58] (array_id=0)

**Module 1 — Flow + Spray (data[0..4]) — luôn có:**

| Index | Tên | Đơn vị | Mô tả chi tiết |
|---|---|---|---|
| 0 | `flow_rate` | L/min | EMA tức thời; \|x\|<0.01 ép = 0.0 |
| 1 | `flow_rate_avg` | L/min | Moving avg 10 mẫu; \|x\|<0.01 ép = 0.0 |
| 2 | `flow_target` | L/min | Setpoint hiện hành; 0 khi mode 0 hoặc FLOW_MODE=1 không đủ điều kiện |
| 3 | `pump_pwm` | µs | PWM thực xuất bơm; dải MIN–MAX từ SERVOx |
| 4 | `spray_mode` | 0/1/2 | 0=PASSTHROUGH, 1=FLOW PID nấc giữa (MIX_STD), 2=FLOW PID nấc cao (MIX_CNT) |

**Module 2 — pH + ao active (data[5..14]) — chỉ khi `SA_PH_EN=1 AND ph_has_data()`:**

| Index | Tên | Đơn vị | Mô tả chi tiết |
|---|---|---|---|
| 5 | `ph` | — | pH moving average 10 mẫu (`_ph_value_ma`) |
| 6 | `ph_mv` | mV | Điện áp điện cực (signed int16 cast float) |
| 7 | `ph_temp` | °C | Nhiệt độ đã bù SA_PH_TOFF |
| 8 | `alk_dkh` | dKH | Kiềm của **ao đang active** (0 nếu chưa tính) |
| 9 | `alk_mgl` | mg/L CaCO₃ | Kiềm ao active (mg/L) |
| 10 | `delta_ph` | — | ΔpH ao active hôm nay; 0 nếu chưa đủ 2 slot |
| 11 | `pond_idx` | 1–100 | Số thứ tự ao đang active (1-based) |
| 12 | `ph_morn` | — | pH slot sáng hôm nay của ao active (0 nếu chưa có mẫu) |
| 13 | `ph_aft` | — | pH slot chiều hôm nay của ao active (0 nếu chưa có mẫu) |
| 14 | `last_day` | ngày | Ngày ghi nhận của ao active (Unix epoch ngày; ×86400 = Unix timestamp) |

**Module 3 — Dosing + ao active (data[15..18]) — luôn có:**

| Index | Tên | Đơn vị | Mô tả chi tiết |
|---|---|---|---|
| 15 | `dos_sp` | gam | Setpoint của **ao đang active** (đồng bộ 2 chiều với SA_DOS_SP) |
| 16 | `dos_rate` | mL/50µs | `SA_DOS_Fx` của loại thức ăn ao active đang dùng |
| 17 | `dos_pwm` | µs | PWM thực xuất; 1500=dừng |
| 18 | `dos_food` | 1–7 | Loại thức ăn của ao active (đồng bộ 2 chiều với SA_DOS_FOOD) |

| 19–57 | — | — | Luôn = 0.0 (zero-padded, dự phòng) |

### 4.2 MAVLink SA_PHK — Layout phk[58] (array_id=1, mới)

Gửi **độc lập** với SA_DATA (không theo lịch cố định) — chỉ khi một ao vừa tính đủ kiềm (FULL) lần đầu trong ngày. Mục đích: GCS/app nhận ngay bản ghi kiềm mới nhất mà không cần chờ tải log SD.

| Index | Tên | Đơn vị | Mô tả chi tiết |
|---|---|---|---|
| 0 | `ph_morn` | — | pH sáng của ao vừa hoàn tất |
| 1 | `ph_aft` | — | pH chiều của ao vừa hoàn tất |
| 2 | `delta_ph` | — | ΔpH = chiều − sáng |
| 3 | `alk_dkh` | dKH | Kiềm vừa tính |
| 4 | `alk_mgl` | mg/L | Kiềm vừa tính (mg/L CaCO₃) |
| 5 | `lat` | độ | GPS latitude mẫu sáng, **đã chia 1e7** (độ thập phân — khác PHAK log dùng deg×1e7 nguyên) |
| 6 | `lng` | độ | GPS longitude mẫu sáng, đã chia 1e7 |
| 7 | `pond_idx` | 1–100 | Số thứ tự ao (1-based) |

| 8–57 | — | — | Luôn = 0.0 |

### 4.3 DataFlash Log [DATA]

N/A — SA_DATA/SA_PHK chỉ tồn tại dưới dạng MAVLink message. DataFlash log theo từng module (FLWD, PHWD, PHAK) — xem MODULE1/2_DETAIL_DESIGN.md.

### 4.4 Tích hợp phía nhận

**pymavlink:**
```python
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
while True:
    msg = mav.recv_match(type='DEBUG_FLOAT_ARRAY', blocking=True)
    if msg is None:
        continue
    name = msg.name.rstrip('\x00')
    d = msg.data

    if name == 'SA_DATA':
        payload = {
            'time_boot_ms':  msg.time_usec,   # millis() — KHÔNG phải µs
            'flow_rate':     d[0],  'flow_rate_avg': d[1],
            'flow_target':   d[2],  'pump_pwm':      int(d[3]),
            'spray_mode':    int(d[4]),
            'ph':            d[5],  'ph_mv':         d[6],
            'ph_temp':       d[7],
            'alk_dkh':       d[8],  'alk_mgl':       d[9],
            'delta_ph':      d[10], 'pond_idx':      int(d[11]),
            'ph_morn':       d[12], 'ph_aft':        d[13],
            'last_day':      int(d[14]),
            'dos_sp':        d[15], 'dos_rate':      d[16],
            'dos_pwm':       int(d[17]), 'dos_food':  int(d[18]),
        }
        payload['ph_connected'] = not (d[5] == 0.0 and d[6] == 0.0 and d[7] == 0.0)

    elif name == 'SA_PHK':
        alk_event = {
            'ph_morn': d[0], 'ph_aft': d[1], 'delta_ph': d[2],
            'alk_dkh': d[3], 'alk_mgl': d[4],
            'lat': d[5], 'lng': d[6], 'pond_idx': int(d[7]),
        }
```

**QGroundControl FactGroup (JSON) — SA_DATA:**
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
    { "index": 11, "name": "PondIdx",    "units": "",       "decimals": 0 },
    { "index": 12, "name": "pHMorn",     "units": "",       "decimals": 2 },
    { "index": 13, "name": "pHAft",      "units": "",       "decimals": 2 },
    { "index": 14, "name": "LastDay",    "units": "",       "decimals": 0 },
    { "index": 15, "name": "DosSP",      "units": "g",      "decimals": 0 },
    { "index": 16, "name": "DosRate",    "units": "mL/50µs","decimals": 1 },
    { "index": 17, "name": "DosPWM",     "units": "µs",     "decimals": 0 },
    { "index": 18, "name": "DosFood",    "units": "",       "decimals": 0 }
  ]
}
```

**Logic hiển thị slot_status:** `slot_status` (0=FULL, 1=MORN, 2=AFT, 3=PREV, 4=NODATA) **không nằm trong SA_DATA** — chỉ có trong log PHAK (`SlotSt`, xem MODULE2_PH_DETAIL_DESIGN.md) và có thể suy ra phía GCS từ `ph_morn`/`ph_aft`/`delta_ph`:

| data[12] (ph_morn) | data[13] (ph_aft) | data[10] (delta_ph) | Suy ra trạng thái | Màu gợi ý |
|---|---|---|---|---|
| >0 | >0 | ≠0 | FULL — ΔpH chính xác hôm nay | Xanh lá |
| >0 | 0 | 0 | MORN — chỉ có slot sáng | Vàng |
| 0 | >0 | 0 | AFT — chỉ có slot chiều | Vàng |
| 0 | 0 | 0, nhưng `alk_dkh`>0 | PREV — dùng kiềm hôm qua | Cam |
| 0 | 0 | 0, và `alk_dkh`=0 | NODATA — chưa có gì | Đỏ |

**Kiểm tra nhanh Mission Planner:**
```
Ctrl+F → MAVLink Inspector → DEBUG_FLOAT_ARRAY → name = "SA_DATA" hoặc "SA_PHK"
```

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
MAV1_EXTRA3 ≥ 1     → cần đặt để GCS nhận SA_DATA/SA_PHK (khuyến nghị 2)
SA_ENABLE = 1       → bắt buộc để firmware gửi
SA_PH_EN = 1        → bắt buộc để data[5..14] có giá trị và để SA_PHK được gửi
```

**Ràng buộc phần cứng:** N/A — truyền qua telemetry / USB / UDP đã có sẵn

**Ràng buộc vận hành:** Đặt `MAV1_EXTRA3 = 2` trong Mission Planner trước khi fly

---

## 6. Kết nối phần cứng [HW]

N/A — SA_DATA/SA_PHK truyền hoàn toàn qua MAVLink (telemetry radio, USB, hoặc UDP).

---

## 7. So sánh với Basic Design [ALL]

| Điểm | Basic Design dự kiến | Thực tế đã làm (bản hiện tại) | Lý do |
|---|---|---|---|
| `time_usec` = microseconds | Giả định là µs như tên field gợi ý | Thực tế dùng `millis64()` (milliseconds) | MAVLink spec cho phép; đơn giản hóa; tên field gây nhầm — ghi chú rõ ở Detail Design |
| pH zero khi tắt | Điền 0 | Implement đúng: zero-init array + chỉ điền khi `ph_has_data()` ✓ | — |
| Giới hạn 58 float | Đề cập 58 max | Implement đúng; data[19..57]=0.0 reserved ✓ | — |
| Số field Module 2 | 7 field (data[5..11]) | **10 field (data[5..14])** — thêm `pond_idx`, `ph_morn`, `ph_aft`, `last_day`; đổi `alk_dkh/alk_mgl/delta_ph` sang giá trị của ao active thay vì "hệ thống nói chung" | Module 2 chuyển sang mô hình đa ao (tối đa 100), cần định danh ao trong SA_DATA |
| Số field Module 3 | 3 field (data[12..14]): dos_sp, dos_rate (g/50µs), dos_pwm | **4 field (data[15..18])**: thêm `dos_food`; `dos_rate` đổi đơn vị sang mL/50µs; toàn bộ lấy theo ao active | Đồng bộ với việc tách vol_rate/density và setpoint theo ao ở Module 3 |
| Bản ghi kiềm gửi ngay (SA_PHK) | Không có | Thêm bản tin `DEBUG_FLOAT_ARRAY` riêng `"SA_PHK"` (array_id=1), gửi độc lập khi có ao vừa FULL | GCS/app cần biết ngay khi có kiềm mới, không chờ tải log SD |
| `slot_status` | Field riêng data[11] | **Đã bỏ khỏi SA_DATA** — GCS tự suy ra từ ph_morn/ph_aft/delta_ph/alk_dkh, hoặc đọc field `SlotSt` trong log PHAK | Nhường chỗ cho `pond_idx`/`ph_morn`/`ph_aft`/`last_day` quan trọng hơn cho hiển thị đa ao |

---

## 8. Tài liệu liên quan [ALL]

- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — nguồn gốc data[0..4]
- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — nguồn gốc data[5..14] và SA_PHK
- [MODULE3_DOS_DETAIL_DESIGN.md](MODULE3_DOS_DETAIL_DESIGN.md) — nguồn gốc data[15..18]
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
