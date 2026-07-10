# Module 2 — pH Sensor (Modbus RTU)

## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`, `Rover/Log.cpp`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 0.5 Hz (Modbus poll mỗi 2000 ms)

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
Rover::update_custom_flow()   [main loop, ~10 Hz]
    │
    ├──► g2.custom_nav.update()
    │         │
    │         ├──► (SA_SIM=0) _ph_update()
    │         │         │
    │         │         ├──► [Gửi Modbus request mỗi 2000ms]
    │         │         │       → _ph_uart->write(req[8])
    │         │         │
    │         │         ├──► [Đọc + validate response 23 bytes]
    │         │         │       → _ph_crc16(buf, 21)
    │         │         │
    │         │         ├──► [Decode: ph_cal, _ph_mv, _ph_temp]
    │         │         │
    │         │         ├──► [MA update (10 mẫu) + EMA update]
    │         │         │
    │         │         └──► _ph_update_daily_slots(ph_cal)
    │         │                   │
    │         │                   ├──► AP::rtc().get_utc_usec() → local_hour
    │         │                   ├──► AP::gps().location(0) → cur_lat, cur_lng
    │         │                   ├──► [Phân slot sáng/chiều + lưu GPS]
    │         │                   ├──► [Kiểm tra cùng ao nếu FULL]
    │         │                   │       → khoảng cách < 300m?
    │         │                   │       → _ph_calc_alkalinity() nếu OK
    │         │                   │       → set _alk_log_pending = true
    │         │                   └──► [Cập nhật _alk_slot_status]
    │         │
    │         └──► _ph_samp_update()        [distance-based sample trigger]
    │                   │
    │                   ├──► AP::gps().location(0) → cur GPS
    │                   ├──► AP::mission()->get_current_nav_index() → wp_idx
    │                   ├──► [WP thay đổi?] → _ph_samp_trigger(wp, 0, ...)
    │                   └──► [khoảng cách ≥ SA_PH_SAMP_D?] → _ph_samp_trigger(wp, sub, ...)
    │
    ├──► Log_Write_Ph_Realtime()            [PHWD — mỗi 2s]
    │         → gps.location(0) → lat, lng
    │         → logger.WriteBlock(&log_PhData, ...)
    │
    ├──► Log_Write_Ph_Alkalinity()          [PHAK — 1 lần/ngày khi FULL+cùng ao]
    │         → consume_alk_log_pending()   → chỉ ghi 1 lần rồi reset flag
    │         → logger.WriteBlock(&log_PhAlk, ...)
    │
    └──► Log_Write_Ph_Sample()              [PHSP — mỗi SA_PH_SAMP_D mét]
              → consume_ph_samp_pending()   → chỉ ghi khi flag set
              → logger.WriteBlock(&log_PhSamp, ...)

init() [1 lần boot]
    └──► _ph_init()
              └──► hal.serial(SA_PH_PORT)->begin(9600)
```

### 1.2 Mô tả từng hàm

---

**`_ph_init()`**

- **File:** `AP_ShoesAgtech.cpp`
- **Được gọi bởi:** `init()` khi boot
- **Xử lý:**
    1. Nếu `SA_PH_EN=0` → return ngay
    2. Lấy `_ph_uart = hal.serial(SA_PH_PORT)` (0–4)
    3. Nếu nullptr → STATUSTEXT WARNING; return
    4. Gọi `_ph_uart->begin(9600)`
    5. Reset buffer MA, EMA sentinel (-1.0), trạng thái request
- **Ghi chú:** `SA_PH_PORT` chỉ đọc tại đây — cần reboot nếu đổi port.

---

**`_ph_update()`**

- **File:** `AP_ShoesAgtech.cpp`
- **Được gọi bởi:** `update()` (SA_SIM=0) @ 10Hz; chỉ thực sự gửi/nhận mỗi 2000ms
- **Xử lý (state machine request/response):**
    1. **Gửi request** (khi `!_ph_req_pending` và Δt ≥ 2000ms): Flush RX → ghi 8-byte Modbus FC04
    2. **Đọc response** (sau 150ms): validate header + CRC → decode ph_cal, _ph_mv, _ph_temp
    3. Cập nhật MA (10 mẫu), EMA
    4. Gọi `_ph_update_daily_slots(ph_cal)`
    5. Console log nếu SA_PH_LOG=1

---

**`_ph_crc16(buf, len) → uint16_t`** — static

- CRC16/Modbus, poly 0xA001 (bit-reversed 0x8005), init 0xFFFF
- Dùng validate 21 byte đầu của response 23-byte

---

**`_ph_update_daily_slots(float ph_cal)`**

- **File:** `AP_ShoesAgtech.cpp`
- **Được gọi bởi:** `_ph_update()` sau mỗi frame hợp lệ
- **Xử lý:**
    1. Lấy UTC time → `local_hour` theo `SA_PH_TZ`
    2. Lấy GPS location → `cur_lat, cur_lng`
    3. `day_num` thay đổi → reset slot + GPS + `_alk_log_pending = false`; lưu kiềm hôm qua
    4. **Phân slot theo cửa sổ cài đặt:**
        - `local_hour ∈ [SA_PH_MS, SA_PH_ME]` → ghi `_ph_morn_val, _ph_morn_lat, _ph_morn_lng`
        - `local_hour ∈ [SA_PH_AS, SA_PH_AE]` → ghi `_ph_aft_val, _ph_aft_lat, _ph_aft_lng`
        - Ngoài cả hai cửa sổ → bỏ qua
    5. **Tính alkalinity (CHỈ khi FULL):**
        - Nếu cả hai slot valid → kiểm tra khoảng cách GPS sáng↔chiều
        - Nếu `dist_m > 300` → cảnh báo "khác ao", `_alk_slot_status = 2` (giữ AFT)
        - Nếu `dist_m ≤ 300` → tính ΔpH → `_ph_calc_alkalinity()` → set `_alk_log_pending = true` (lần đầu)
        - Nếu chỉ MORN: `_alk_slot_status = 1`, không tính kiềm
        - Nếu chỉ AFT: `_alk_slot_status = 2`, không tính kiềm
        - PREV: dùng `_alk_prev_dkh` (không tính lại)
        - NODATA: `_alk_dkh = 0`
- **Đầu ra:** Cập nhật `_alk_dkh, _alk_mgl, _delta_ph, _alk_slot_status, _alk_log_pending`

---

**`_ph_calc_alkalinity(float ph, float base_kh_dkh, float temp_c) → float`**

- **Được gọi bởi:** `_ph_update_daily_slots()` CHỈ khi status=FULL
- **Xử lý:**
    1. `tf = constrain(1.0 − (temp−28) × 0.008, 0.85, 1.10)` (temperature factor)
    2. `pf` = pH factor 3 vùng tuyến tính: pH≥8.3 / 7.6≤pH<8.3 / pH<7.6
    3. `return base_kh × constrain(pf × tf, 0.55, 1.75)`
- **Ghi chú:** Heuristic dựa trên cân bằng CO₂/HCO₃⁻. Cập nhật SA_PH_KH định kỳ từ test kit.

---

**`_ph_samp_update()`**

- **File:** `AP_ShoesAgtech.cpp`
- **Được gọi bởi:** `update()` mỗi vòng lặp (~10Hz)
- **Điều kiện hoạt động:** `SA_PH_SAMP_D ≥ 0.5` AND GPS fix AND `ph_has_data()`
- **Xử lý:**
    1. Đọc GPS hiện tại (`cur_lat, cur_lng`) và WP index từ `AP::mission()`
    2. **WP thay đổi** (`wp_idx != _ph_samp_wp_prev`):
        - Gọi `_ph_samp_trigger(wp_idx, 0, cur_lat, cur_lng)` → ghi điểm "WP.0"
        - Reset: `_ph_samp_sub_idx = 1`, `ref = cur GPS`
    3. **Kiểm tra khoảng cách** từ điểm mẫu trước (flat-Earth approx):
        - `dist_m = √(dlat_m² + dlng_m²)`
        - Nếu `dist_m ≥ SA_PH_SAMP_D` → `_ph_samp_trigger(wp, sub, ...)`, cập nhật ref, tăng sub
- **Đầu ra:** Set `_ph_samp_pending = true` với WP, sub, lat, lng lưu sẵn cho Logger

---

**`_ph_samp_trigger(wp, sub, lat, lng)`**

- Lưu {wp, sub, lat, lng} vào `_ph_samp_pend_*`
- Set `_ph_samp_pending = true`
- Nếu SA_PH_LOG=1 → GCS INFO "[WM] Samp WP{wp}.{sub} pH:{ma} Tmp:{temp}C"

---

**`Rover::Log_Write_Ph_Realtime()`**

- **File:** `Rover/Log.cpp`
- **Được gọi bởi:** `update_custom_flow()` mỗi vòng lặp (thực tế 0.5Hz theo Modbus)
- **Điều kiện ghi:** `logging_enabled()` AND `ph_is_enabled()`
- **Xử lý:**
    1. Đọc GPS từ `gps.location(0)` → `lat, lng` (0 nếu không có fix)
    2. Điền struct `log_PhData` từ getters của `g2.custom_nav`
    3. `logger.WriteBlock(&pkt, sizeof(pkt))`
- **Record size:** 4 (header) + 8 + 4×3 + 2 + 4 + 4 = **34 bytes**

---

**`Rover::Log_Write_Ph_Alkalinity()`**

- **File:** `Rover/Log.cpp`
- **Được gọi bởi:** `update_custom_flow()` mỗi vòng lặp
- **Điều kiện ghi:** `consume_alk_log_pending()` trả về true (chỉ 1 lần/ngày/ao)
- **Xử lý:**
    1. Kiểm tra `_alk_log_pending` — nếu false → return ngay
    2. Xóa flag (reset = false) để lần sau không ghi lại
    3. Điền struct `log_PhAlk` với pH sáng/chiều, ΔpH, kiềm, GPS ao (morning lat/lng)
    4. `logger.WriteBlock(&pkt, sizeof(pkt))`
- **Record size:** 4 + 8 + 4×5 + 4 + 4 = **40 bytes**
- **Tần suất:** Tối đa **1 lần/ngày/ao** (khi lần đầu FULL trong ngày)

---

**`Rover::Log_Write_Ph_Sample()`**

- **File:** `Rover/Log.cpp`
- **Được gọi bởi:** `update_custom_flow()` mỗi vòng lặp
- **Điều kiện ghi:** `consume_ph_samp_pending()` trả về true
- **Xử lý:**
    1. Kiểm tra `_ph_samp_pending` — nếu false → return ngay
    2. Xóa flag → reset
    3. Điền struct `log_PhSamp` từ getters: `get_ph_samp_wp/sub/lat/lng()` + `get_ph/temp/mv()`
    4. `logger.WriteBlock(&pkt, sizeof(pkt))`
- **Record size:** 4 + 8 + 2 + 1 + 4 + 4 + 2 + 4 + 4 = **33 bytes**

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_PH_EN` | 14 | Int8 | 0 | 0 | 1 | Bật/tắt module pH. Tắt → không mở UART, không poll, không ghi SD. |
| `SA_PH_PORT` | 15 | Int8 | 2 | 0 | 4 | Số SERIAL port RS485-TTL. Cần `SERIALx_BAUD=9`, `SERIALx_PROTOCOL=0`. |
| `SA_PH_TOFF` | 16 | Float | -3.5 | -10 | 10 | Offset bù nhiệt độ (°C): `T = raw/10 + SA_PH_TOFF`. |
| `SA_PH_OFF` | 17 | Float | 0.0 | -2.0 | 2.0 | Offset hiệu chuẩn pH: `pH = raw/100 + SA_PH_OFF`. |
| `SA_PH_KH` | 18 | Float | 4.0 | 0 | 30 | Kiềm tham chiếu từ test kit (dKH). Cập nhật khi test ao. |
| `SA_PH_EMA` | 19 | Float | 0.15 | 0.01 | 1.0 | Alpha EMA làm mịn pH. Nhỏ = mịn hơn, phản hồi chậm hơn. |
| `SA_PH_LOG` | 20 | Int8 | 0 | 0 | 1 | Bật (1) console log pH/nhiệt độ/mV theo chu kỳ `SA_LOG_PH_MS`. |
| `SA_PH_TZ` | 21 | Int8 | 7 | -12 | 14 | UTC offset (giờ). Việt Nam = 7. Dùng phân slot sáng/chiều + GPS thời gian. |
| `SA_LOG_PH_MS` | 23 | Int16 | 2000 | 500 | 60000 | Chu kỳ console log pH (ms). |
| `SA_PH_TIMEOUT` | 24 | Int16 | 2 | 1 | 300 | Ngưỡng mất kết nối (giây). Quá ngưỡng → `ph_has_data()=false`. |
| `SA_PH_MS` | 55 | Int8 | 5 | 0 | 23 | Giờ **bắt đầu** cửa sổ sáng (0–23). Mặc định: 5h00. |
| `SA_PH_ME` | 56 | Int8 | 11 | 0 | 24 | Giờ **kết thúc** cửa sổ sáng (0–24, inclusive). Mặc định: 11h59. |
| `SA_PH_AS` | 57 | Int8 | 12 | 0 | 23 | Giờ **bắt đầu** cửa sổ chiều (0–23). Mặc định: 12h00. |
| `SA_PH_AE` | 58 | Int8 | 16 | 0 | 24 | Giờ **kết thúc** cửa sổ chiều (0–24, inclusive). Mặc định: 16h59. |
| `SA_PH_SAMP_D` | 59 | Float | 0.0 | 0 | 500 | Khoảng cách (m) giữa các điểm mẫu PHSP. 0 = tắt. |

> **Param chỉ có hiệu lực sau reboot:** `SA_PH_PORT`
>
> **Lưu ý cửa sổ thời gian:** SA_PH_MS và SA_PH_AS clamp 0–23 (giờ bắt đầu không thể là 24). SA_PH_ME và SA_PH_AE clamp 0–24 (24 = "đến hết giờ trong ngày").

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

- Gọi `hal.serial(SA_PH_PORT)->begin(9600)` một lần khi boot
- Reset circular buffer MA (10 phần tử), EMA sentinel = -1.0
- `_ph_last_good_ms = 0` → `ph_has_data() = false` cho đến khi frame đầu tiên
- `_ph_samp_wp_prev = 0xFFFF` → lần đầu gặp WP bất kỳ sẽ trigger sample WP.0

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

**Register Map:**

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

**Phân slot theo cửa sổ cài đặt:**

```
local_sec  = UTC_epoch + SA_PH_TZ × 3600
local_hour = (local_sec % 86400) / 3600

local_hour ∈ [SA_PH_MS, SA_PH_ME] → MORNING slot
    → _ph_morn_val = ph_cal, _ph_morn_lat/lng = GPS hiện tại

local_hour ∈ [SA_PH_AS, SA_PH_AE] → AFTERNOON slot
    → _ph_aft_val = ph_cal, _ph_aft_lat/lng = GPS hiện tại

Ngoài cả hai cửa sổ → bỏ qua (không cập nhật slot nào)

Midnight (day_num thay đổi):
    → Lưu _alk_today_dkh → _alk_prev_dkh (fallback hôm sau)
    → Reset: _ph_morn/aft_val/lat/lng = 0, _alk_log_pending = false
```

**Kiểm tra cùng ao (trước khi tính kiềm):**

```
Nếu cả MORN và AFT valid AND cả hai có GPS (lat ≠ 0):
    dlat_m = (aft_lat − morn_lat) × 1e-7 × 111320
    dlng_m = (aft_lng − morn_lng) × 1e-7 × 111320 × cos(morn_lat)
    dist_m = √(dlat_m² + dlng_m²)

    dist_m > 300m → cảnh báo "khác ao", status = 2 (giữ AFT, chờ thêm)
    dist_m ≤ 300m → tính kiềm → status = 0 (FULL)
```

**Tính ΔpH và kiềm (chỉ khi FULL + cùng ao):**

```
delta_ph   = _ph_aft_val − _ph_morn_val
kh_scaled  = SA_PH_KH × (1 + constrain(delta_ph × 0.375, −0.5, 1.0))
alk_dkh    = _ph_calc_alkalinity(_ph_morn_val, kh_scaled, _ph_temp)
alk_mgl    = alk_dkh × 17.85
_alk_log_pending = true  (lần đầu đạt FULL hôm nay)
```

**Bảng trạng thái alkalinity:**

| Status | Giá trị | Điều kiện | Kiềm hiển thị |
|---|---|---|---|
| 0 FULL | 0 | MORN + AFT cùng ao | Chính xác nhất (ΔpH) |
| 1 MORN | 1 | Chỉ sáng | Không có — chờ chiều |
| 2 AFT | 2 | Chỉ chiều / hoặc khác ao | Không có — chờ thêm |
| 3 PREV | 3 | Chưa có hôm nay, có hôm qua | Hôm qua (không tính lại) |
| 4 NODATA | 4 | Chưa từng có dữ liệu | 0 |

### 3.4 Xử lý các trường hợp đặc biệt

- **CRC lỗi:** bỏ frame, không cập nhật `_ph_last_good_ms`; STATUSTEXT mỗi lần
- **Timeout response (> 500ms):** reset `_ph_req_pending`; warn mỗi 10s
- **Không có GPS time:** Vẫn đọc pH + ghi PHWD (với lat/lng=0); không phân slot; warn mỗi 60s
- **Sáng+chiều khác ao:** Giữ slot AFT (`status=2`); warn trên GCS; không ghi PHAK

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

Chỉ ghi khi `SA_PH_EN=1 AND ph_has_data()`. Ngược lại → 0.0.

| Index | Tên | Đơn vị | Mô tả |
|---|---|---|---|
| `data[5]` | `ph` | — | pH moving average 10 mẫu (`_ph_value_ma`) |
| `data[6]` | `ph_mv` | mV | Điện áp điện cực (signed int16 cast sang float) |
| `data[7]` | `ph_temp` | °C | Nhiệt độ đã bù SA_PH_TOFF |
| `data[8]` | `alk_dkh` | dKH | Kiềm (chỉ có khi status=0 FULL hoặc 3 PREV; 0 các trường hợp khác) |
| `data[9]` | `alk_mgl` | mg/L CaCO₃ | `alk_dkh × 17.85` |
| `data[10]` | `delta_ph` | — | pH_aft − pH_morn hôm nay; 0 nếu chưa đủ 2 slot cùng ao |
| `data[11]` | `slot_status` | 0–4 | 0=FULL, 1=MORN, 2=AFT/khác-ao, 3=PREV, 4=NODATA |

**Phát hiện mất kết nối phía GCS:**

```
IF data[5]==0.0 AND data[6]==0.0 AND data[7]==0.0 → hiển thị cảnh báo "mất kết nối"
```

---

### 4.2 DataFlash Log (SD card)

Ba loại message ghi độc lập vào SD card của ArduPilot theo định dạng binary `.bin`.

---

#### PHWD — pH Realtime

```
Message: "PHWD"
Format:  "QfffhLL"
Điều kiện: SA_PH_EN=1, ph_has_data()=true
Tần suất: ~0.5 Hz (1 record mỗi 2 giây)
```

| Field | Tên | Kiểu | Đơn vị | Mô tả |
|---|---|---|---|---|
| 1 | `TimeUS` | uint64 (Q) | µs | `AP_HAL::micros64()` |
| 2 | `pHRaw` | float (f) | — | pH tức thời sau calibration (latest sample) |
| 3 | `pHMA` | float (f) | — | pH moving average 10 mẫu |
| 4 | `Temp` | float (f) | °C | Nhiệt độ đã bù SA_PH_TOFF |
| 5 | `mV` | int16 (h) | mV | Điện áp điện cực Nernst (raw) |
| 6 | `Lat` | int32 (L) | deg×1e7 | GPS latitude; 0 = không có fix |
| 7 | `Lng` | int32 (L) | deg×1e7 | GPS longitude; 0 = không có fix |

**Ứng dụng:** Vẽ đường pH theo thời gian; overlay lên bản đồ GPS để xem pH từng vị trí trong ao.
**Dung lượng:** ~34 bytes × 43.200 record/ngày ≈ **1.5 MB/ngày**.

---

#### PHAK — Daily Alkalinity (Kiềm ngày)

```
Message: "PHAK"
Format:  "QfffffLL"
Điều kiện: _alk_log_pending=true (chỉ khi FULL + cùng ao ≤ 300m)
Tần suất: Tối đa 1 record/ngày/ao
```

| Field | Tên | Kiểu | Đơn vị | Mô tả |
|---|---|---|---|---|
| 1 | `TimeUS` | uint64 (Q) | µs | Timestamp khi chiều slot đủ |
| 2 | `pHMorn` | float (f) | — | pH buổi sáng (giá trị cuối trong cửa sổ SA_PH_MS..ME) |
| 3 | `pHAft` | float (f) | — | pH buổi chiều (giá trị cuối trong cửa sổ SA_PH_AS..AE) |
| 4 | `dPH` | float (f) | — | ΔpH = pHAft − pHMorn |
| 5 | `AlkDKH` | float (f) | dKH | Kiềm tính từ ΔpH + `_ph_calc_alkalinity()` |
| 6 | `AlkMGL` | float (f) | mg/L | AlkDKH × 17.85 |
| 7 | `Lat` | int32 (L) | deg×1e7 | GPS latitude khi đo sáng (định danh ao) |
| 8 | `Lng` | int32 (L) | deg×1e7 | GPS longitude khi đo sáng |

**Ứng dụng:** Theo dõi kiềm từng ao theo ngày. Filter theo Lat/Lng để xem lịch sử một ao cụ thể.
**Dung lượng:** ~40 bytes × 1 record/ngày/ao = cực nhỏ.

**Cơ chế ghi 1 lần:**

```
_ph_update_daily_slots():
    khi FULL lần đầu hôm nay → _alk_log_pending = true

Log_Write_Ph_Alkalinity():
    consume_alk_log_pending() → trả true + reset = false
    → ghi PHAK 1 lần
    → mọi lần gọi tiếp theo trong ngày → return ngay (flag = false)

Midnight reset:
    _alk_log_pending = false → sẵn sàng cho ngày mới
```

---

#### PHSP — pH Sample Points (Điểm mẫu theo khoảng cách)

```
Message: "PHSP"
Format:  "QHBffhLL"
Điều kiện: SA_PH_SAMP_D > 0, GPS fix, ph_has_data()=true
Tần suất: 1 record mỗi SA_PH_SAMP_D mét dọc tuyến đường + 1 tại mỗi WP
```

| Field | Tên | Kiểu | Đơn vị | Mô tả |
|---|---|---|---|---|
| 1 | `TimeUS` | uint64 (Q) | µs | Timestamp khi trigger |
| 2 | `WP` | uint16 (H) | — | Mission waypoint index (segment hiện tại) |
| 3 | `Sub` | uint8 (B) | — | Sub-index: 0=tại WP, 1=điểm mẫu thứ 1, 2=thứ 2, … |
| 4 | `pHMA` | float (f) | — | pH moving average tại thời điểm trigger |
| 5 | `Temp` | float (f) | °C | Nhiệt độ tại thời điểm trigger |
| 6 | `mV` | int16 (h) | mV | Điện áp điện cực |
| 7 | `Lat` | int32 (L) | deg×1e7 | GPS latitude tại điểm mẫu |
| 8 | `Lng` | int32 (L) | deg×1e7 | GPS longitude tại điểm mẫu |

**Ví dụ — WP1→WP2 = 32m, SA_PH_SAMP_D = 8m:**

| WP | Sub | Ý nghĩa | Vị trí |
|---|---|---|---|
| 1 | 0 | Đến WP1 | GPS tại WP1 |
| 1 | 1 | 8m sau WP1 | GPS +8m |
| 1 | 2 | 16m sau WP1 | GPS +16m |
| 1 | 3 | 24m sau WP1 | GPS +24m |
| 2 | 0 | Đến WP2 | GPS tại WP2 |

**Ứng dụng:** Xem phân bố pH dọc bờ ao; so sánh các vị trí trong cùng ao; phát hiện vùng pH thấp bất thường.
**Dung lượng:** ~33 bytes × (tổng_km / SA_PH_SAMP_D × 1000) record/ngày.

---

### 4.3 Console Log

```
Trigger: mỗi SA_LOG_PH_MS ms khi SA_PH_LOG=1

Dòng 1 (luôn in):
  [WM] pH:<ph_raw> MA:<ph_ma> Tmp:<temp>C mV:<mv> [FULL|MORN|AFT|PREV|NODATA]

Dòng 2 (chỉ khi status=0 FULL):
  [WM] Alk:<alk_dkh>dKH <alk_mgl>mg/L dPH:<delta_ph>

Khi trigger sample point:
  [WM] Samp WP<n>.<sub> pH:<ma> Tmp:<temp>C

SA_SIM=1: tiền tố [SIM][WM] thay vì [WM]
```

### 4.4 STATUSTEXT — Toàn bộ thông báo

| Nội dung thông báo | Mức | Điều kiện | Tần suất |
|---|---|---|---|
| `SA: pH sensor on SERIAL<n> (Modbus RTU 9600)` | INFO | Init thành công | 1 lần |
| `SA: pH sensor SERIAL<n> not found` | WARNING | Port không tồn tại | 1 lần |
| `SA: pH sensor chua co du lieu - kiem tra day RS485` | WARNING | Chưa nhận frame nào | Mỗi 10s |
| `SA: pH sensor mat ket noi (<x>s) - kiem tra day RS485` | WARNING | Mất kết nối > SA_PH_TIMEOUT | Mỗi 10s |
| `SA: pH CRC fail (noise on RS485?)` | WARNING | CRC16 không khớp | Mỗi lần lỗi |
| `[WM] Ngay moi: slot reset. Kiem: du lieu hom qua` | INFO | Midnight, có prev data | 1 lần/ngày |
| `[WM] Ngay moi: slot reset. Chua co du lieu kiem` | INFO | Midnight, không có prev | 1 lần/ngày |
| `[WM] Chua co GPS time, kiem tinh theo pH tuc thoi` | WARNING | Không có GPS time | Mỗi 60s |
| `[WM] Kiem: chi co du lieu sang, cho du lieu chieu` | WARNING | status=1 MORN | Mỗi 60s |
| `[WM] Kiem: chi co du lieu chieu, thieu du lieu sang` | WARNING | status=2 AFT | Mỗi 60s |
| `[WM] Kiem: sang/chieu khac ao (XXXm) - bo qua` | WARNING | MORN+AFT nhưng > 300m | Mỗi trigger |
| `[WM] Kiem: dang dung du lieu hom qua` | WARNING | status=3 PREV | Mỗi 60s |
| `[WM] Kiem: chua co du lieu slot (doi GPS hoac cho khung gio)` | WARNING | status=4 NODATA | Mỗi 60s |
| `[WM] Samp WP<n>.<sub> pH:<ma> Tmp:<temp>C` | INFO | Trigger PHSP (SA_PH_LOG=1) | Mỗi điểm mẫu |

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERIALx_BAUD     = 9     (= 9600 baud)    ← x = SA_PH_PORT   → sai: không nhận frame
SERIALx_PROTOCOL = 0     (= None)         ←                   → sai: ArduPilot chiếm port
SA_PH_PORT       → chỉ đọc khi boot, cần reboot nếu đổi
SA_PH_KH cần cập nhật định kỳ bằng test kit khi độ kiềm ao thay đổi
GPS fix bắt buộc để phân slot sáng/chiều và để kiểm tra cùng ao
SA_PH_MS < SA_PH_ME  và  SA_PH_AS < SA_PH_AE  (cửa sổ không rỗng)
SA_PH_ME ≤ SA_PH_AS  (cửa sổ sáng kết thúc trước khi chiều bắt đầu)
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
| Công thức alkalinity | Đơn giản: `SA_PH_KH + 16×(pH−7)` | Heuristic 3-vùng với temperature factor | Kinh nghiệm thực địa: cần bổ chỉnh nhiệt độ và đặc tính pH 7–8 |
| Slot thời gian | Sáng 00:00–11:59, Chiều 12:00–23:59 | Cửa sổ cài được (SA_PH_MS/ME/AS/AE) | Linh hoạt theo lịch đo thực tế từng trang trại |
| Kiềm realtime (NODATA) | Ước tính từ pH tức thời | Không tính — chỉ FULL mới có kiềm | Tránh nhiễu; kiềm chỉ có ý nghĩa khi có ΔpH đủ 2 slot |
| Kiềm 1 buổi (MORN/AFT) | Tính từ 1 điểm | Không tính — chờ đủ cả hai | Kiềm 1 điểm sai lệch lớn; không đáng tin cậy |
| Ghi SD card | Chưa đề cập | 3 loại PHWD/PHAK/PHSP riêng biệt | Phân tích offline + theo dõi theo vị trí GPS |
| Vị trí GPS trong log | Không có | Có trong cả 3 loại (PHWD, PHAK, PHSP) | Định danh ao, phân bố pH trong ao |
| Kiềm per-ao | Không phân | Kiểm tra khoảng cách sáng↔chiều ≤ 300m | Tránh tính kiềm nhầm khi robot đi nhiều ao/ngày |
| Điểm mẫu dọc tuyến | Không có | PHSP theo SA_PH_SAMP_D mét | Phân tích phân bố pH dọc bờ ao từng đoạn WP |
| Timeout warning | Mỗi N giây | Mỗi 10s (độc lập SA_LOG_PH_MS) | Cần cảnh báo nhanh dù log chậm |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE2_PH_BASIC_DESIGN.md](MODULE2_PH_BASIC_DESIGN.md) — yêu cầu và hành vi
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA (data[5..11])
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — `update()` gọi `_ph_update()`
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống