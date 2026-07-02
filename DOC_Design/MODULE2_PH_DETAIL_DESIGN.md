# Module 2 — pH Sensor (Modbus RTU)
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 0.5 Hz (Modbus poll mỗi 2000 ms)
**Ngày hoàn thành:** 2026-05-15

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
update() [Module 1, 10 Hz]
    │
    └──► (SA_SIM=0) _ph_update()
              │
              ├──► [Gửi Modbus request mỗi 2000ms]
              │         → _ph_uart->write(req[8])
              │
              ├──► [Đợi 150ms rồi đọc response]
              │         → _ph_uart->read(buf[23])
              │
              ├──► [Validate header + _ph_crc16(buf, 21)]
              │         → uint16_t crc
              │
              ├──► [Decode registers → ph_cal, _ph_mv, _ph_temp]
              │
              ├──► [MA update + EMA update]
              │
              └──► _ph_update_daily_slots(ph_cal)
                        │
                        ├──► AP::rtc().get_utc_usec()  → local time
                        │
                        ├──► [Ghi slot sáng/chiều]
                        │
                        └──► _ph_calc_alkalinity(ph, base_kh, temp)
                                  └──► return float (alk_dkh)

init() [1 lần boot]
    └──► _ph_init()
              └──► hal.serial(SA_PH_PORT)->begin(9600)
```

### 1.2 Mô tả từng hàm

---

**`_ph_init()`**
- **File:** `AP_ShoesAgtech.cpp : 906`
- **Được gọi bởi:** `init()` khi boot
- **Đầu vào:** không có (đọc `_ph_en`, `_ph_port` từ param)
- **Xử lý:**
  1. Nếu `SA_PH_EN=0` → return ngay
  2. Lấy `_ph_uart = hal.serial(SA_PH_PORT)` (0–4)
  3. Nếu nullptr → STATUSTEXT WARNING "pH sensor SERIAL\<n\> not found"; return
  4. Gọi `_ph_uart->begin(9600)`
  5. Reset toàn bộ buffer MA, EMA sentinel (-1.0), trạng thái request
- **Đầu ra / Return:** `void`
  - Thành công: STATUSTEXT INFO "SA: pH sensor on SERIAL\<n\> (Modbus RTU 9600)"
- **Ghi chú:** `SA_PH_PORT` chỉ đọc tại đây — cần reboot nếu đổi port.

---

**`_ph_update()`**
- **File:** `AP_ShoesAgtech.cpp : 936`
- **Được gọi bởi:** `update()` (SA_SIM=0) @ 10Hz; chỉ thực sự gửi/nhận mỗi 2000ms
- **Đầu vào:** không có
- **Xử lý (state machine request/response):**
  1. Nếu `_ph_uart == nullptr` → return
  2. **Gửi request** (khi `!_ph_req_pending` và Δt ≥ 2000ms):
     - Flush RX buffer còn sót
     - Ghi 8-byte Modbus FC04 request: `[01 04 00 00 00 09 30 0C]`
     - Set `_ph_req_pending = true`, ghi timestamp
  3. **Đọc response** (sau 150ms):
     - Nếu < 23 byte và Δt > 500ms → timeout, cảnh báo mỗi 10s
     - Đọc 23 byte, validate header `[01 04 12]`
     - Kiểm tra CRC bằng `_ph_crc16(buf, 21)` vs byte 21–22
     - Decode: `ph_cal`, `_ph_mv`, `_ph_temp`
     - Cập nhật MA (10 mẫu), EMA (`_ph_value_ema`)
     - Gọi `_ph_update_daily_slots(ph_cal)`
  4. Console log nếu SA_PH_LOG=1
- **Đầu ra / Return:** `void` — side effects: `_ph_value_ma`, `_ph_mv`, `_ph_temp`, `_ph_last_good_ms`

---

**`_ph_crc16(const uint8_t *buf, uint16_t len) → uint16_t`** — static file-scope
- **File:** `AP_ShoesAgtech.cpp : 895`
- **Được gọi bởi:** `_ph_update()` để verify response frame
- **Đầu vào:**
  - `buf` — con trỏ byte array
  - `len` — số byte cần tính CRC (21 cho response 23-byte)
- **Xử lý:**
  1. `crc = 0xFFFF` (init)
  2. Với mỗi byte: `crc ^= byte`; 8 lần shift: nếu LSB=1 → `crc = (crc>>1) ^ 0xA001`, ngược lại → `crc >>= 1`
- **Đầu ra / Return:** `uint16_t` — CRC16/Modbus (little-endian ready: LSB = byte[21], MSB = byte[22])
- **Ghi chú:** Algorithm: CRC16/Modbus, poly 0xA001 (bit-reversed 0x8005), init 0xFFFF.

---

**`_ph_update_daily_slots(float ph_cal)`**
- **File:** `AP_ShoesAgtech.cpp : 1098`
- **Được gọi bởi:** `_ph_update()` sau mỗi frame hợp lệ
- **Đầu vào:**
  - `ph_cal` — pH đã calibrate (sau `+ SA_PH_OFF`)
- **Xử lý:**
  1. Lấy UTC time từ `AP::rtc().get_utc_usec()`; tính `local_hour` dựa trên SA_PH_TZ
  2. Nếu `day_num` thay đổi → reset slot + lưu kiềm hôm qua làm fallback
  3. Phân slot: local_hour < 12 → MORNING; ≥ 12 → AFTERNOON (last reading wins)
  4. Tính alkalinity theo trạng thái slot:
     - MORN+AFT: `_ph_calc_alkalinity(ph_morn, kh_scaled, temp)` với kh_scaled bổ chỉnh ΔpH
     - Chỉ MORN: `_ph_calc_alkalinity(ph_morn, SA_PH_KH, temp)`
     - Chỉ AFT: `_ph_calc_alkalinity(ph_aft, SA_PH_KH, temp)`
     - Hôm qua (PREV): dùng `_alk_prev_dkh`
     - NODATA: tính thô từ `ph_cal`
  5. Cập nhật `_alk_slot_status` (0=FULL, 1=MORN, 2=AFT, 3=PREV, 4=NODATA)
  6. Console warning mỗi 60s nếu SA_PH_LOG=1
- **Đầu ra / Return:** `void` — cập nhật `_alk_dkh`, `_alk_mgl`, `_delta_ph`, `_alk_slot_status`

---

**`_ph_calc_alkalinity(float ph, float base_kh_dkh, float temp_c) → float`**
- **File:** `AP_ShoesAgtech.cpp : 1221`
- **Được gọi bởi:** `_ph_update_daily_slots()`
- **Đầu vào:**
  - `ph` — giá trị pH (clamp 0–14)
  - `base_kh_dkh` — kiềm tham chiếu từ test kit (dKH)
  - `temp_c` — nhiệt độ nước (°C)
- **Xử lý:**
  1. Temperature factor: `tf = constrain(1.0 − (temp−28) × 0.008, 0.85, 1.10)`
     - Mỗi 1°C trên 28°C → CO₂ hòa tan giảm ~0.8%
  2. pH factor (3 vùng tuyến tính):
     - pH ≥ 8.3: `pf = 1.0 + (pH−8.3) × 0.32`
     - pH ≤ 7.6: `pf = 1.0 − (7.6−pH) × 0.45`
     - 7.6 < pH < 8.3: `pf = 1.0 + (pH−8.0) × 0.15`
  3. `alk_dkh = base_kh × constrain(pf × tf, 0.55, 1.75)`
- **Đầu ra / Return:** `float` — alkalinity ước tính (dKH)
- **Ghi chú:** Đây là mô hình heuristic dựa trên cân bằng CO₂/HCO₃⁻ trong nước ngọt. Chỉ đúng gần đúng trong dải pH 6.5–8.5. Cập nhật SA_PH_KH định kỳ từ test kit thực địa.

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_PH_EN` | 14 | Int8 | 0 | 0 | 1 | Bật/tắt module pH. Tắt → không mở UART, không poll. |
| `SA_PH_PORT` | 15 | Int8 | 2 | 0 | 4 | Số SERIAL port RS485-TTL. Cần `SERIALx_BAUD=9`, `SERIALx_PROTOCOL=0`. |
| `SA_PH_TOFF` | 16 | Float | -3.5 | -10 | 10 | Offset bù nhiệt độ (°C): `T = raw/10 + SA_PH_TOFF`. |
| `SA_PH_OFF` | 17 | Float | 0.0 | -2.0 | 2.0 | Offset hiệu chuẩn pH: `pH = raw/100 + SA_PH_OFF`. |
| `SA_PH_KH` | 18 | Float | 4.0 | 0 | 30 | Kiềm tham chiếu từ test kit (dKH). Cập nhật khi test ao. |
| `SA_PH_EMA` | 19 | Float | 0.15 | 0.01 | 1.0 | Alpha EMA làm mịn pH. Nhỏ = mịn hơn, phản hồi chậm hơn. |
| `SA_PH_LOG` | 20 | Int8 | 0 | 0 | 1 | Bật (1) console log pH/nhiệt độ/kiềm theo chu kỳ `SA_LOG_PH_MS`. |
| `SA_PH_TZ` | 21 | Int8 | 7 | -12 | 14 | UTC offset (giờ). Việt Nam = 7. Dùng phân loại slot sáng/chiều. |
| `SA_LOG_PH_MS` | 23 | Int16 | 2000 | 500 | 60000 | Chu kỳ console log pH (ms). Không nên < 2000 (poll 0.5 Hz). |
| `SA_PH_TIMEOUT` | 24 | Int16 | 2 | 1 | 300 | Ngưỡng mất kết nối (giây). Quá ngưỡng → `ph_has_data()=false`. |

> **Param chỉ có hiệu lực sau reboot:** `SA_PH_PORT`

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

- Gọi `hal.serial(SA_PH_PORT)->begin(9600)` một lần khi boot
- Reset circular buffer MA (10 phần tử), EMA sentinel = -1.0
- `_ph_last_good_ms = 0` → `ph_has_data() = false` cho đến khi frame đầu tiên

### 3.2 Giao thức Modbus RTU

**Frame Request (8 bytes):**
```
[01] [04] [00 00] [00 09] [30 0C]
 ↑    ↑      ↑       ↑      ↑
Addr FC04 RegStart Count  CRC16 LSB-first

Slave address: 0x01
Function code: 0x04 (Read Input Registers)
Đọc 9 registers từ 0x0000
Gửi mỗi 2000ms; flush RX trước khi gửi
```

**Frame Response (23 bytes):**
```
[01][04][12][D0..D17][CRC_L CRC_H]
  1    1   1    18       2         = 23 bytes
```

**Register Map trong Response:**
```
buf[3..4]   → reg 0x0000: pH × 100      (uint16, big-endian)
buf[7..8]   → reg 0x0002: mV signed     (int16, big-endian)
buf[19..20] → reg 0x0008: temp × 10     (int16, big-endian)
```

**Decode:**
```
ph_cal    = constrain(raw_ph / 100.0 + SA_PH_OFF, 0.0, 14.0)
_ph_mv    = (int16_t)raw_mv
_ph_temp  = raw_temp / 10.0 + SA_PH_TOFF
```

### 3.3 Làm mịn và slot

**EMA:** `_ph_value_ema = (1−α)×prev + α×ph_cal`, α=SA_PH_EMA; first sample seeds trực tiếp

**Moving Average:** Circular buffer 10 phần tử; `_ph_value_ma = sum / count`

**Phân slot:**
```
local_sec  = UTC_epoch + SA_PH_TZ × 3600
local_hour = (local_sec % 86400) / 3600
hour < 12  → MORNING slot  (cập nhật _ph_morn_val, last reading wins)
hour ≥ 12  → AFTERNOON slot (cập nhật _ph_aft_val, last reading wins)
Midnight (day_num thay đổi) → reset cả hai slot, lưu kiềm hôm qua
```

**Tính ΔpH và kiềm (khi cả hai slot hôm nay):**
```
delta_ph   = _ph_aft_val − _ph_morn_val
kh_scaled  = SA_PH_KH × (1 + constrain(delta_ph × 0.375, −0.5, 1.0))
alk_dkh    = _ph_calc_alkalinity(_ph_morn_val, kh_scaled, _ph_temp)
alk_mgl    = alk_dkh × 17.85
```

### 3.4 Xử lý các trường hợp đặc biệt

- **CRC lỗi:** bỏ frame, không cập nhật `_ph_last_good_ms`; STATUSTEXT mỗi lần
- **Timeout response (> 500ms):** reset `_ph_req_pending`; warn mỗi 10s nếu `ph_has_data()=false`
- **Không có GPS time:** Vẫn đọc pH bình thường nhưng không phân được slot sáng/chiều; cảnh báo mỗi 60s

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

Chỉ ghi khi `SA_PH_EN=1 AND ph_has_data()`. Ngược lại → 0.0 (zero-padded).

| Index | Tên | Đơn vị | Mô tả + edge case |
|---|---|---|---|
| `data[5]` | `ph` | — | pH moving average 10 mẫu (`_ph_value_ma`) |
| `data[6]` | `ph_mv` | mV | Điện áp điện cực (signed int16 cast sang float) |
| `data[7]` | `ph_temp` | °C | Nhiệt độ đã bù SA_PH_TOFF |
| `data[8]` | `alk_dkh` | dKH | Kiềm ước tính (today hoặc yesterday) |
| `data[9]` | `alk_mgl` | mg/L CaCO₃ | `alk_dkh × 17.85` |
| `data[10]` | `delta_ph` | — | pH_aft − pH_morn hôm nay; 0 nếu chưa đủ 2 slot |
| `data[11]` | `slot_status` | 0–4 | 0=FULL, 1=MORN, 2=AFT, 3=PREV, 4=NODATA |

**Phát hiện mất kết nối phía GCS:**
```
IF data[5]==0.0 AND data[6]==0.0 AND data[7]==0.0 → hiển thị cảnh báo
```

### 4.2 DataFlash Log [DATA]

```
Message name:  "PHWD"
Format string: "QffffffB"
Điều kiện ghi: SA_PH_EN = 1
```

| Field | Tên | Kiểu | Đơn vị | Mô tả |
|---|---|---|---|---|
| 1 | `TimeUS` | uint64_t (Q) | µs | `AP_HAL::micros64()` |
| 2 | `pHRaw` | float (f) | — | pH tức thời sau calibration |
| 3 | `pHMA` | float (f) | — | pH moving average |
| 4 | `Temp` | float (f) | °C | Nhiệt độ đã bù SA_PH_TOFF |
| 5 | `AlkDKH` | float (f) | dKH | Kiềm ước tính |
| 6 | `AlkMGL` | float (f) | mg/L | Kiềm quy đổi |
| 7 | `DeltapH` | float (f) | — | ΔpH hôm nay; 0 nếu chưa đủ |
| 8 | `SlotSt` | uint8_t (B) | — | slot_status 0–4 |

### 4.3 Console Log

```
Trigger: mỗi SA_LOG_PH_MS ms khi SA_PH_LOG=1
Format:
  [WM] pH:<ph_value> MA:<ph_ma> Tmp:<temp>C mV:<ph_mv>
  [WM] Alk:<alk_dkh>dKH <alk_mgl>mg/L dPH:<delta_ph> [FULL|MORN|AFT|PREV|NODATA]

SA_SIM=1: tiền tố [SIM][WM] thay vì [WM]
```

### 4.4 STATUSTEXT — Toàn bộ thông báo

| Nội dung thông báo | Mức | Điều kiện | Tần suất |
|---|---|---|---|
| `SA: pH sensor on SERIAL<n> (Modbus RTU 9600)` | INFO | Init thành công | 1 lần |
| `SA: pH sensor SERIAL<n> not found` | WARNING | Port không tồn tại | 1 lần |
| `SA: pH sensor chua co du lieu - kiem tra day RS485` | WARNING | Chưa nhận frame nào (first timeout) | Mỗi 10s |
| `SA: pH sensor mat ket noi (<x>s) - kiem tra day RS485` | WARNING | Mất kết nối > SA_PH_TIMEOUT | Mỗi 10s |
| `SA: pH CRC fail (noise on RS485?)` | WARNING | CRC16 không khớp | Mỗi lần lỗi |
| `[WM] Ngay moi: slot reset. Kiem: du lieu hom qua` | INFO | Midnight, có prev data | 1 lần/ngày |
| `[WM] Ngay moi: slot reset. Chua co du lieu kiem` | INFO | Midnight, không có prev data | 1 lần/ngày |
| `[WM] Chua co GPS time, kiem tinh theo pH tuc thoi` | WARNING | Không có GPS fix | Mỗi 60s |
| `[WM] Kiem: chi co du lieu sang, cho du lieu chieu` | WARNING | slot_status=1 | Mỗi 60s |
| `[WM] Kiem: chi co du lieu chieu, thieu du lieu sang` | WARNING | slot_status=2 | Mỗi 60s |
| `[WM] Kiem: dang dung du lieu hom qua` | WARNING | slot_status=3 | Mỗi 60s |
| `[WM] Kiem: chua co du lieu slot (doi GPS hoac cho khung gio)` | WARNING | slot_status=4 | Mỗi 60s |

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERIALx_BAUD     = 9     (= 9600 baud)    ← x = SA_PH_PORT   → sai: không nhận frame
SERIALx_PROTOCOL = 0     (= None)         ←                   → sai: ArduPilot chiếm port
SA_PH_PORT       → chỉ đọc khi boot, cần reboot nếu đổi
SA_PH_KH cần cập nhật định kỳ bằng test kit khi độ kiềm ao thay đổi
GPS fix bắt buộc để phân slot sáng/chiều chính xác (ΔpH và FULL slot)
```

**Ràng buộc phần cứng:** Chỉ 1 thiết bị Modbus slave trên bus (address 0x01 hardcoded)

---

## 6. Kết nối phần cứng [HW]

```
Sensor Nengshi ASPS3801D:
  A+  →  RS485 A trên module converter
  B−  →  RS485 B trên module converter

Module RS485 → TTL:
  TX   →  RX cổng SERIALx FC
  RX   →  TX cổng SERIALx FC
  VCC  →  3.3V hoặc 5V (kiểm tra module)
  GND  →  GND chung

FC config (x = SA_PH_PORT):
  SERIALx_BAUD     = 9     (9600 baud)
  SERIALx_PROTOCOL = 0     (None)
```

---

## 7. So sánh với Basic Design [ALL]

| Điểm | Basic Design dự kiến | Thực tế đã làm | Lý do |
|---|---|---|---|
| Công thức alkalinity | Đơn giản: `SA_PH_KH + 16×(pH−7)` | Heuristic 3-vùng với temperature factor | Kinh nghiệm thực địa: cần bổ chỉnh nhiệt độ và đặc tính khác pH 7–8 |
| Slot thời gian | Sáng 00:00–11:59, Chiều 12:00–23:59 | Giống design, nhưng "last reading wins" trong slot | Tránh giá trị outlier đầu buổi ảnh hưởng kết quả |
| NODATA — không hiển thị 0 | Hiển thị 0 | Hiển thị kiềm ước tính thô từ pH tức thời | Tốt hơn là không có số nào để người dùng theo dõi xu hướng |
| Timeout warning | Mỗi N giây | Mỗi 10s (riêng, độc lập với SA_LOG_PH_MS) | Cần cảnh báo nhanh dù log chậm |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE2_PH_BASIC_DESIGN.md](MODULE2_PH_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA (data[5..11])
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — `update()` gọi `_ph_update()`
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
