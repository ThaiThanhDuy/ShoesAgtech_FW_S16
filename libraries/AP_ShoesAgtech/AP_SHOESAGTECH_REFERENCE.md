# AP_ShoesAgtech — Reference Document

> **Mục đích của file này:** Tài liệu kỹ thuật đầy đủ để AI (Claude) hoặc kỹ sư có thể đọc và:
> - Cài đặt đúng tham số (`SA_*`) trên GCS / MAVLink
> - Cấu hình hiển thị trên **QGroundControl tùy chỉnh** (custom QGC)
> - Hiểu chính xác format bản tin MAVLink và log nhị phân
>
> **Phiên bản firmware:** ardupilot-jbdcan_testing_S16 / branch `ardupilot_git_tutorial`
> **Vehicle:** ArduRover (APM_BUILD_Rover)

---

## Mục lục

1. [Tổng quan hệ thống](#1-tổng-quan-hệ-thống)
2. [Tham số cài đặt SA_*](#2-tham-số-cài-đặt-sa_)
   - 2.1 [Module 1: Cảm biến lưu lượng + Điều khiển phun](#21-module-1-cảm-biến-lưu-lượng--điều-khiển-phun)
   - 2.2 [Module 2: Cảm biến pH (Modbus RTU)](#22-module-2-cảm-biến-ph-modbus-rtu)
   - 2.3 [Module 3: Động cơ định lượng (vít tải thức ăn tôm)](#23-module-3-động-cơ-định-lượng-vít-tải-thức-ăn-tôm)
   - 2.4 [Simulation (SA_SIM)](#24-simulation-sa_sim)
3. [Dữ liệu real-time MAVLink — DEBUG_FLOAT_ARRAY](#3-dữ-liệu-real-time-mavlink--debug_float_array)
4. [Dữ liệu log nhị phân DataFlash](#4-dữ-liệu-log-nhị-phân-dataflash)
5. [Cấu hình QGroundControl tùy chỉnh](#5-cấu-hình-qgroundcontrol-tùy-chỉnh)
6. [Chuẩn đoán & cảnh báo console](#6-chuẩn-đoán--cảnh-báo-console)
7. [Yêu cầu phần cứng & kết nối](#7-yêu-cầu-phần-cứng--kết-nối)

---

## 1. Tổng quan hệ thống

`AP_ShoesAgtech` là thư viện tùy chỉnh tích hợp vào **ArduRover** để quản lý thiết bị nông nghiệp thủy sản gồm 3 module độc lập:

| Module | Phần cứng | Giao tiếp | Tần suất xử lý |
|---|---|---|---|
| Lưu lượng + Bơm phun | YF-S402B + servo bơm | GPIO pin 55 (IRQ) | 10 Hz |
| Cảm biến pH | Nengshi ASPS3801D-0.5M | UART Modbus RTU 9600 8N1 (RS485-TTL) | 0.5 Hz (request mỗi 2s) |
| Động cơ định lượng | Servo 360° liên tục | PWM servo output | 10 Hz |

**Instance trong firmware:**
```
rover.g2.custom_nav   →   object AP_ShoesAgtech
```

**Prefix tham số MAVLink:** `SA_`

**Đăng ký trong ParametersG2:** slot 58, subgroup `"SA_"`

---

## 2. Tham số cài đặt SA_*

> Tất cả tham số đều đọc/ghi qua MAVLink message `PARAM_SET` / `PARAM_VALUE`.
> Tên tham số đầy đủ = `"SA_" + tên cột "Param"` bên dưới.

### 2.1 Module 1: Cảm biến lưu lượng + Điều khiển phun

| Param (tên đầy đủ) | Slot | Kiểu | Mặc định | Min | Max | Mô tả |
|---|---|---|---|---|---|---|
| `SA_ENABLE` | 1 | Int8 | **1** | 0 | 1 | Bật (1) / tắt (0) toàn bộ thư viện. Khi = 0, không có task nào chạy. |
| `SA_CAL_FAC` | 2 | Float | **3874.5** | 100 | 10000 | Hệ số hiệu chuẩn cảm biến YF-S402B: số xung trên mỗi lít (pulses/Litre). Dãy hoạt động 0.3–6 L/min. |
| `SA_EMA_AL` | 3 | Float | **0.1** | 0.01 | 1.0 | Alpha làm mịn EMA cho lưu lượng tức thời. Nhỏ = mịn hơn, phản hồi chậm hơn. |
| `SA_FLOW_LOG` | 4 | Int8 | **0** | 0 | 1 | Bật (1) in dữ liệu lưu lượng/bơm ra console GCS theo chu kỳ `SA_LOG_FL_MS`. |
| `SA_RC_CHAN` | 5 | Int8 | **6** | 1 | 16 | Kênh RC (1-indexed) chọn chế độ phun: PWM<1300→mode 0, 1300–1700→mode 1, >1700→mode 2. |
| `SA_RC_PUMP` | 6 | Int8 | **9** | 1 | 16 | Kênh RC (1-indexed) đọc PWM bơm thủ công ở mode 0 (passthrough). |
| `SA_PUMP_CHAN` | 7 | Int8 | **8** | 1 | 16 | Kênh servo đầu ra bơm (1-indexed). **Bắt buộc** `SERVOx_FUNCTION = 0 (None)`. |
| `SA_FLOW_SP` | 8 | Float | **5.0** | 0 | 200 | Setpoint lưu lượng (L/min) ở mode 1 — PID bám giá trị này. |
| `SA_PID_P` | 9 | Float | **80.0** | 0 | 500 | Hệ số P của PI controller: us PWM trên mỗi L/min sai số. |
| `SA_PID_I` | 10 | Float | **20.0** | 0 | 200 | Hệ số I của PI controller: us PWM trên mỗi L/min/giây. |
| `SA_PID_LPF` | 11 | Float | **0.3** | 0.01 | 1.0 | Alpha LPF cho đầu ra PID (0.01 = rất mịn, 1.0 = không lọc). |
| `SA_APP_RATE` | 12 | Float | **100.0** | 0 | 2000 | Tỉ lệ phun L/ha ở mode 2 — Auto Rate (tính theo tốc độ xe + boom). |
| `SA_BOOM_W` | 13 | Float | **1.0** | 0 | 30 | Chiều rộng boom phun (mét) dùng tính toán ở mode 2. |

**Logic chọn chế độ phun (SA_RC_CHAN):**

```
PWM < 1300   →  Mode 0: PASSTHROUGH — đọc SA_RC_PUMP ghi thẳng ra SA_PUMP_CHAN
1300–1700    →  Mode 1: FLOW PID    — PI bám SA_FLOW_SP, integral anti-windup
PWM > 1700   →  Mode 2: AUTO RATE  — tự tính target = SA_APP_RATE × tốc_độ × SA_BOOM_W × 0.006
```

---

### 2.2 Module 2: Cảm biến pH (Modbus RTU)

| Param (tên đầy đủ) | Slot | Kiểu | Mặc định | Min | Max | Mô tả |
|---|---|---|---|---|---|---|
| `SA_PH_EN` | 14 | Int8 | **0** | 0 | 1 | Bật (1) cảm biến pH. Khi = 0, không mở UART, không gửi request Modbus. |
| `SA_PH_PORT` | 15 | Int8 | **2** | 0 | 4 | Số port SERIAL kết nối RS485-TTL (khớp với `SERIALx`). Yêu cầu `SERIALx_BAUD=9` (9600) và `SERIALx_PROTOCOL=0` (None). |
| `SA_PH_TOFF` | 16 | Float | **-3.5** | -10 | 10 | Offset bù nhiệt độ (°C): `T_hiển_thị = T_sensor/10 + SA_PH_TOFF`. |
| `SA_PH_OFF` | 17 | Float | **0.0** | -2.0 | 2.0 | Offset hiệu chuẩn pH: `pH_cal = pH_raw/100 + SA_PH_OFF`. Dùng dung dịch buffer để đo. |
| `SA_PH_KH` | 18 | Float | **4.0** | 0 | 30 | Kiềm tham chiếu dKH đo từ test kit. Cập nhật mỗi khi test ao. |
| `SA_PH_EMA` | 19 | Float | **0.15** | 0.01 | 1.0 | Alpha EMA làm mịn pH. Nhỏ = mịn hơn, phản hồi chậm hơn. |
| `SA_PH_LOG` | 20 | Int8 | **0** | 0 | 1 | Bật (1) in dữ liệu pH/nhiệt độ/kiềm ra console GCS theo chu kỳ `SA_LOG_PH_MS`. Độc lập với `SA_FLOW_LOG`. |
| `SA_PH_TZ` | 21 | Int8 | **7** | -12 | 14 | UTC offset (giờ). Việt Nam = 7 (UTC+7). Dùng phân loại slot sáng/chiều cho tính kiềm ΔpH. |
| `SA_LOG_FL_MS` | 22 | Int16 | **1000** | 100 | 60000 | Chu kỳ in log lưu lượng ra console (ms). Chỉ hoạt động khi `SA_FLOW_LOG=1`. |
| `SA_LOG_PH_MS` | 23 | Int16 | **2000** | 500 | 60000 | Chu kỳ in log pH ra console (ms). Không nên đặt < 2000 (Modbus chỉ trả dữ liệu mỗi 2s). |
| `SA_PH_TIMEOUT` | 24 | Int16 | **2** | 1 | 300 | Ngưỡng "mất kết nối" (giây): nếu không nhận được frame pH hợp lệ quá thời gian này, phát cảnh báo STATUSTEXT và xóa `data[4..8]` về 0 trong gói SA_DATA. |

**Giao thức Modbus RTU:**

```
Baud:     9600 8N1
Request:  [01][04][00 00][00 09][30 0C]   (8 bytes, FC04, đọc 9 registers từ 0x0000)
Response: [01][04][12][R0..R8 × 2B][CRC × 2B]  = 23 bytes tổng cộng

Register map (offset từ byte 3 của response):
  0x0000  buf[3..4]   pH × 100        unsigned 16-bit  →  pH_cal = value/100 + SA_PH_OFF
  0x0002  buf[7..8]   Electrode mV    signed 16-bit    →  hiển thị trực tiếp (mV)
  0x0008  buf[19..20] Temperature ×10 signed 16-bit    →  T = value/10 + SA_PH_TOFF

CRC: CRC16/Modbus, polynomial 0xA001, init 0xFFFF, little-endian (LSB trước)
```

**Logic phân slot kiềm ΔpH:**

```
Dữ liệu pH được tích lũy theo ngày (cần GPS time):
  Slot sáng:  0h00 – 11h59 local time  (local = UTC + SA_PH_TZ)
  Slot chiều: 12h00 – 23h59 local time
  Reset lúc: nửa đêm (day_num thay đổi)

Ưu tiên tính kiềm:
  slot_status = 0 (FULL):   cả 2 slot hôm nay → ΔpH = pH_chiều − pH_sáng → kiềm chính xác nhất
  slot_status = 1 (MORN):   chỉ slot sáng hôm nay → ước tính từ pH sáng
  slot_status = 2 (AFT):    chỉ slot chiều hôm nay → ước tính từ pH chiều
  slot_status = 3 (PREV):   không có dữ liệu hôm nay → dùng dữ liệu hôm qua
  slot_status = 4 (NODATA): chưa có dữ liệu gì → ước tính trực tiếp từ pH hiện tại + SA_PH_KH
```

---

### 2.3 Module 3: Động cơ định lượng (vít tải thức ăn tôm)

| Param (tên đầy đủ) | Slot | Kiểu | Mặc định | Min | Max | Mô tả |
|---|---|---|---|---|---|---|
| `SA_DOS_CHAN` | 25 | Int8 | **10** | 1 | 16 | Kênh servo đầu ra động cơ định lượng (1-indexed). **Bắt buộc** cả 4 điều kiện: `SERVOx_FUNCTION=0`, `SERVOx_MIN=800`, `SERVOx_TRIM=1500`, `SERVOx_MAX=2200`. |
| `SA_DOS_RC` | 26 | Int8 | **8** | 1 | 16 | Kênh RC bật/tắt động cơ (1-indexed). PWM > 1500 → bật; PWM ≤ 1500 (kể cả mất tín hiệu = 0) → tắt (xuất 1500). VD: nút B Skydroid T10 thường = kênh 8. |
| `SA_DOS_RATE` | 27 | Float | **100.0** | 1 | 1000 | Tỉ lệ quy đổi: số gam ứng với 50 µs PWM lệch khỏi điểm dừng 1500. Công thức: `offset_us = SA_DOS_SP × 50 / SA_DOS_RATE`. VD: RATE=100 → 100g = 50µs; RATE=200 → 200g = 50µs. |
| `SA_DOS_SP` | 28 | Float | **0.0** | 0 | — | Setpoint lượng thức ăn muốn cấp (gam). Người dùng nhập trực tiếp. VD: 1000 = 1 kg. Offset tương ứng: `1000 × 50 / SA_DOS_RATE`. |
| `SA_DOS_REV` | 29 | Int8 | **0** | 0 | 1 | Chiều quay servo 360°: `0` = thuận (PWM dải 800–1500, 800 = nhanh nhất); `1` = ngược (PWM dải 1500–2200). |
| `SA_DOS_LOG` | 30 | Int8 | **0** | 0 | 1 | Bật (1) in log trạng thái dosing motor ra console GCS theo chu kỳ `SA_DOS_LOG_MS`. |
| `SA_DOS_LOG_MS` | 31 | Int16 | **1000** | 100 | 60000 | Chu kỳ in log dosing motor ra console (ms). Chỉ hoạt động khi `SA_DOS_LOG=1`. |

---

### 2.4 Simulation (SA_SIM)

| Param (tên đầy đủ) | Slot | Kiểu | Mặc định | Min | Max | Mô tả |
|---|---|---|---|---|---|---|
| `SA_SIM` | 32 | Int8 | **0** | 0 | 1 | **0** = sensor thật (logic cũ). **1** = giả lập sóng sin — KHÔNG đọc cảm biến, điền dữ liệu tổng hợp để test GCS + mode 1/2. |

**Dữ liệu giả lập khi `SA_SIM = 1`:**

| Đại lượng | Giá trị trung bình | Biên độ dao động | Chu kỳ |
|---|---|---|---|
| `flow_rate` (L/min) | 2.5 | ±1.5 | 20 s |
| `sim_speed` — tốc độ giả cho mode 2 (m/s) | 1.0 | ±0.8 | 30 s |
| `ph` | 7.3 | ±0.4 | 60 s |
| `ph_mv` (mV) | tính từ Nernst `(7.0 − pH) × 59.16` | — | — |
| `ph_temp` (°C) | 28.0 | ±2.0 | 120 s |
| `alk_dkh` (dKH) | 4.0 | ±0.8 | 90 s |
| `alk_mgl` (mg/L) | `alk_dkh × 17.85` | — | — |

**Lưu ý khi dùng SA_SIM:**
- Cảm biến pH Modbus **không** được thăm dò khi sim bật — UART RS485 không cần kết nối.
- `ph_has_data()` luôn trả `true` khi sim bật (firmware cập nhật `_ph_last_good_ms` mỗi chu kỳ).
- `flow_rate_avg` vẫn tính qua moving-average buffer 10 mẫu từ `flow_rate` giả lập — sẽ hội tụ sau ~10 chu kỳ (khoảng 1 giây ở 10 Hz).
- Mode 2 dùng `sim_speed` thay vì `AP::ahrs().groundspeed()` — có thể test tính toán target flow mà không cần di chuyển.
- **Tắt `SA_SIM` (= 0) trước khi deploy thực tế** — để đọc lại giá trị cảm biến thật.

**Công thức tính PWM động cơ:**

```
offset_us = SA_DOS_SP × 50.0 / SA_DOS_RATE

SA_DOS_REV = 0 (thuận):   pwm = constrain(1500 − offset_us,  800, 1500)
SA_DOS_REV = 1 (ngược):   pwm = constrain(1500 + offset_us, 1500, 2200)

Khi tắt (RC ≤ 1500):  pwm = 1500  (servo dừng)
```

**Điều kiện servo bắt buộc để motor được phép chạy:**

```
SERVOx_FUNCTION = 0    (None)
SERVOx_MIN      = 800
SERVOx_TRIM     = 1500
SERVOx_MAX      = 2200

Sai bất kỳ 1 trong 4 → firmware KHÔNG xuất PWM động cơ
                      → cảnh báo STATUSTEXT mỗi 5 giây
```

---

## 3. Dữ liệu real-time MAVLink — DEBUG_FLOAT_ARRAY

### Thông số bản tin

| Thuộc tính | Giá trị |
|---|---|
| MAVLink message ID | **350** (`MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY`) |
| Tên mảng (`name`) | **`"SA_DATA"`** (7 ký tự + null, field 10 bytes) |
| `array_id` | **0** (không dùng để phân biệt, luôn = 0) |
| `time_usec` | `millis64()` — milliseconds từ khi boot (không phải microseconds) |
| Số phần tử `data[]` | 58 floats tổng, **chỉ index 0–14 có dữ liệu**, còn lại = 0.0 |
| Stream group | **`STREAM_EXTRA3`** (MAVLink param: `MAVx_EXTRA3`) |
| Điều kiện gửi | `SA_ENABLE = 1` AND payload space đủ |

### Bảng ánh xạ data[] — theo thứ tự module

**Module 1 — Flow sensor + Spray controller (`data[0..4]`)**

| Index | Tên field | Đơn vị | Điều kiện | Mô tả |
|---|---|---|---|---|
| `data[0]` | `flow_rate` | L/min | Luôn gửi | Lưu lượng tức thời sau EMA (`SA_EMA_AL`). Noise < 0.01 ép = 0. |
| `data[1]` | `flow_rate_avg` | L/min | Luôn gửi | Moving average lưu lượng (10 mẫu). Noise < 0.01 ép = 0. |
| `data[2]` | `flow_target` | L/min | Luôn gửi | Setpoint hiện tại. Mode 0=0.0; mode 1=`SA_FLOW_SP`; mode 2=`APP_RATE×speed×BOOM_W×0.006`. |
| `data[3]` | `pump_pwm` | µs | Luôn gửi | PWM thực tế đang xuất ra kênh bơm (`SA_PUMP_CHAN`). Dải: 800–2200. |
| `data[4]` | `spray_mode` | 0/1/2 | Luôn gửi | Chế độ phun: **0**=PASSTHROUGH, **1**=FLOW PID, **2**=AUTO RATE. |

**Module 2 — pH sensor (`data[5..11]`)**

| Index | Tên field | Đơn vị | Điều kiện | Mô tả |
|---|---|---|---|---|
| `data[5]` | `ph` | — | `SA_PH_EN=1` AND `ph_has_data()` | pH đã lọc moving average (10 mẫu). Khi mất kết nối → 0.0. |
| `data[6]` | `ph_mv` | mV | `SA_PH_EN=1` AND `ph_has_data()` | Điện áp điện cực pH (signed, mV). Khi mất kết nối → 0.0. |
| `data[7]` | `ph_temp` | °C | `SA_PH_EN=1` AND `ph_has_data()` | Nhiệt độ nước (đã bù `SA_PH_TOFF`). Khi mất kết nối → 0.0. |
| `data[8]` | `alk_dkh` | dKH | `SA_PH_EN=1` AND `ph_has_data()` | Kiềm ước tính (dKH). Nguồn: FULL→ΔpH; MORN/AFT→1 slot; PREV→hôm qua; NODATA→trực tiếp. |
| `data[9]` | `alk_mgl` | mg/L | `SA_PH_EN=1` AND `ph_has_data()` | Kiềm (mg/L CaCO₃) = `alk_dkh × 17.85`. Khi mất kết nối → 0.0. |
| `data[10]` | `delta_ph` | — | `SA_PH_EN=1` AND `ph_has_data()` | ΔpH = pH chiều − pH sáng hôm nay. 0.0 nếu chưa đủ 2 slot. |
| `data[11]` | `slot_status` | 0–4 | `SA_PH_EN=1` AND `ph_has_data()` | Chất lượng dữ liệu kiềm: **0**=FULL, **1**=MORN, **2**=AFT, **3**=PREV(hôm qua), **4**=NODATA. |

**Module 3 — Dosing motor (`data[12..14]`)**

| Index | Tên field | Đơn vị | Điều kiện | Mô tả |
|---|---|---|---|---|
| `data[12]` | `dos_sp` | gam | Luôn gửi | Setpoint lượng thức ăn (`SA_DOS_SP`). 0 = chưa đặt. |
| `data[13]` | `dos_rate` | gam/50µs | Luôn gửi | Tỉ lệ quy đổi (`SA_DOS_RATE`): số gam ứng với 50µs lệch. Công thức: `offset_us = dos_sp × 50 / dos_rate`. |
| `data[14]` | `dos_pwm` | µs | Luôn gửi | PWM đang xuất ra kênh định lượng (`SA_DOS_CHAN`). 1500 = dừng. |

| `data[15..57]` | — | — | — | Luôn = 0.0 (dự phòng). |

> **Lưu ý `ph_has_data()`:**
> Trả `true` khi: `_ph_last_good_ms != 0` AND `(millis() − _ph_last_good_ms) ≤ SA_PH_TIMEOUT × 1000`.
> Khi mất kết nối, `data[5..11]` được giữ = 0.0 (không gửi giá trị cũ/rác).

---

## 4. Dữ liệu log nhị phân DataFlash

### Bản tin FLWD — Lưu lượng dòng chảy

```
Message ID:    LOG_FLOW_DATA_MSG  (enum trong Rover/defines.h)
Message name:  "FLWD"
Format string: "Qff"
```

| Field | Tên cột | Kiểu C | MAVLink format | Đơn vị | Mô tả |
|---|---|---|---|---|---|
| 1 | `TimeUS` | uint64_t | Q | µs | Thời gian hệ thống (AP_HAL::micros64()) |
| 2 | `Flow` | float | f | L/min | Lưu lượng EMA tức thời |
| 3 | `FlowAvg` | float | f | L/min | Moving average lưu lượng (10 mẫu) |

### Bản tin PHWD — Dữ liệu pH

```
Message ID:    LOG_PH_DATA_MSG  (enum trong Rover/defines.h)
Message name:  "PHWD"
Format string: "QffffffB"
Guard:         Chỉ ghi khi SA_PH_EN = 1
```

| Field | Tên cột | Kiểu C | MAVLink format | Đơn vị | Mô tả |
|---|---|---|---|---|---|
| 1 | `TimeUS` | uint64_t | Q | µs | Thời gian hệ thống |
| 2 | `pHRaw` | float | f | — | pH hiệu chuẩn tức thời (`pH/100 + SA_PH_OFF`) |
| 3 | `pHMA` | float | f | — | pH moving average (10 mẫu) |
| 4 | `Temp` | float | f | °C | Nhiệt độ nước (đã bù SA_PH_TOFF) |
| 5 | `AlkDKH` | float | f | dKH | Kiềm ước tính dKH |
| 6 | `AlkMGL` | float | f | mg/L | Kiềm ước tính mg/L CaCO₃ |
| 7 | `DeltapH` | float | f | — | ΔpH = pH chiều − pH sáng hôm nay (0 nếu chưa đủ 2 slot) |
| 8 | `SlotSt` | uint8_t | B | — | Chất lượng dữ liệu kiềm: 0=FULL, 1=MORN, 2=AFT, 3=PREV, 4=NODATA |

---

## 5. Cấu hình QGroundControl tùy chỉnh

### 5.1 Bật stream SA_DATA về GCS

Cần đặt tốc độ stream `STREAM_EXTRA3` khác 0. Thực hiện qua MAVLink hoặc Mission Planner:

```
MAV1_EXTRA3 = 2    (2 Hz, khuyến nghị — cân bằng băng thông/độ trễ)
                    hoặc
MAV1_EXTRA3 = 4    (4 Hz, nếu cần độ trơn cao hơn)
```

Kiểm tra bản tin đang tới: Mission Planner → Ctrl+F → MAVLink Inspector → lọc `DEBUG_FLOAT_ARRAY`.

### 5.2 Cấu hình Widget trong QGC tùy chỉnh

QGC đọc MAVLink message ID 350 (`DEBUG_FLOAT_ARRAY`). Lọc theo tên mảng `"SA_DATA"` và đọc theo index.

**Ví dụ cấu hình Vehicle.FactGroup (QML/JSON) cho từng field:**

```json
{
  "messageName": "DEBUG_FLOAT_ARRAY",
  "filter": { "name": "SA_DATA" },
  "fields": [
    { "index": 0,  "name": "FlowRate",    "units": "L/min",  "decimals": 2, "label": "Lưu lượng (EMA)" },
    { "index": 1,  "name": "FlowAvg",     "units": "L/min",  "decimals": 2, "label": "Lưu lượng (TB)" },
    { "index": 2,  "name": "FlowTarget",  "units": "L/min",  "decimals": 2, "label": "Setpoint" },
    { "index": 3,  "name": "PumpPWM",     "units": "µs",     "decimals": 0, "label": "PWM bơm" },
    { "index": 4,  "name": "SprayMode",   "units": "",       "decimals": 0, "label": "Mode phun (0/1/2)" },
    { "index": 5,  "name": "pH",          "units": "",       "decimals": 2, "label": "pH (MA)" },
    { "index": 6,  "name": "pH_mV",       "units": "mV",     "decimals": 0, "label": "Điện cực pH" },
    { "index": 7,  "name": "WaterTemp",   "units": "°C",     "decimals": 1, "label": "Nhiệt độ nước" },
    { "index": 8,  "name": "AlkDKH",      "units": "dKH",    "decimals": 2, "label": "Kiềm (dKH)" },
    { "index": 9,  "name": "AlkMGL",      "units": "mg/L",   "decimals": 1, "label": "Kiềm (mg/L)" },
    { "index": 10, "name": "DosSP",        "units": "g",      "decimals": 0, "label": "Thức ăn (SP)" },
    { "index": 11, "name": "DosRate",     "units": "g/50µs", "decimals": 0, "label": "Tỉ lệ quy đổi" },
    { "index": 12, "name": "DosPWM",      "units": "µs",     "decimals": 0, "label": "PWM định lượng" }
  ]
}
```

### 5.3 Xử lý trạng thái mất kết nối trong QGC

```
Quy tắc hiển thị pH/kiềm:
  IF data[5] == 0.0 AND data[6] == 0.0 AND data[7] == 0.0
      → Hiển thị "-- Mất kết nối pH --" hoặc màu đỏ
  ELSE
      → Hiển thị giá trị bình thường
```

> Lưu ý: khi pH chưa được bật (`SA_PH_EN=0`), `data[5..9]` luôn = 0.0 vì guard `ph_is_enabled() && ph_has_data()` không qua.

### 5.4 Hiển thị chế độ phun (Spray Mode)

`data[4]` (SprayMode) trực tiếp cho biết mode đang chạy:

```
data[4] == 0   → Mode 0: PASSTHROUGH — bơm theo RC tay
data[4] == 1   → Mode 1: FLOW PID   — bám SA_FLOW_SP (data[2])
data[4] == 2   → Mode 2: AUTO RATE  — bám SA_APP_RATE × tốc độ xe × SA_BOOM_W
```

`data[2]` (FlowTarget) hiển thị setpoint đang áp dụng (0.0 ở mode 0).

### 5.5 Hiển thị chất lượng dữ liệu kiềm (slot_status)

`slot_status` không nằm trong `SA_DATA` nhưng có trong log PHWD (`SlotSt`).
Real-time: dùng cảnh báo STATUSTEXT từ firmware:
- `"[WM] Kiem: chi co du lieu sang"` → slot_status = 1
- `"[WM] Kiem: chi co du lieu chieu"` → slot_status = 2
- `"[WM] Kiem: dang dung du lieu hom qua"` → slot_status = 3

### 5.6 Bảng tóm tắt tham số cần set trước khi dùng

```
# ---- BẮT BUỘC ----
SA_ENABLE      = 1          # bật thư viện
SA_PUMP_CHAN   = <n>         # kênh servo bơm (ví dụ: 8)
SERVO<n>_FUNCTION = 0       # bơm phải là None

# ---- Nếu dùng cảm biến pH ----
SA_PH_EN       = 1
SA_PH_PORT     = <x>         # số SERIAL kết nối RS485 (ví dụ: 2)
SERIAL<x>_BAUD     = 9       # = 9600 baud
SERIAL<x>_PROTOCOL = 0       # = None (không phải MAVLink)
SA_PH_KH       = <giá_trị>  # dKH đo từ test kit (cập nhật định kỳ)
SA_PH_TZ       = 7           # UTC+7 cho Việt Nam

# ---- Nếu dùng động cơ định lượng ----
SA_DOS_CHAN    = <m>          # kênh servo motor (ví dụ: 10)
SERVO<m>_FUNCTION = 0
SERVO<m>_MIN      = 800
SERVO<m>_TRIM     = 1500
SERVO<m>_MAX      = 2200
SA_DOS_SP      = <gam>       # lượng thức ăn muốn cấp
SA_DOS_RATE    = <ratio>     # gam ứng với 50µs lệch

# ---- MAVLink stream ----
MAV1_EXTRA3    = 2           # 2 Hz stream SA_DATA về GCS
```

---

## 6. Chuẩn đoán & cảnh báo console

Tất cả cảnh báo gửi qua `gcs().send_text()` → hiển thị trong **Messages** / **HUD** của QGC / Mission Planner.

| Nội dung STATUSTEXT | Mức | Ý nghĩa |
|---|---|---|
| `ShoesAgtech: IRQ attach failed` | CRITICAL | Không gắn được IRQ GPIO 55 cho cảm biến flow |
| `ShoesAgtech: Flow sensor ready` | INFO | Khởi tạo flow sensor thành công |
| `SA: SERVO<n>_FUNCTION=<x> must be 0(None)!` | WARNING | Kênh bơm chưa đặt FUNCTION=0. Lặp lại mỗi 5s. |
| `SA: SERVO<n> OK Min:<x> Trim:<y> Max:<z>` | INFO | Bơm cấu hình đúng, in ra min/trim/max |
| `SA: pH sensor on SERIAL<n> (Modbus RTU 9600)` | INFO | pH sensor khởi tạo thành công |
| `SA: pH sensor SERIAL<n> not found` | WARNING | Port SERIAL không tồn tại |
| `SA: pH sensor chua co du lieu - kiem tra day RS485` | WARNING | Chưa nhận được frame nào từ sensor |
| `SA: pH sensor mat ket noi (<x>s) - kiem tra day RS485` | WARNING | Mất kết nối, in thời gian từ lần cuối nhận frame |
| `SA: pH CRC fail (noise on RS485?)` | WARNING | Frame nhận bị lỗi CRC |
| `SA: SERVO<m> setup thanh cong - dosing motor san sang` | INFO | Dosing motor đủ 4 điều kiện, sẵn sàng chạy |
| `SA: SERVO<m> FUNCTION=<x>, can dat =0 (None)` | WARNING | Dosing servo sai FUNCTION |
| `SA: SERVO<m> MIN=<x>, can dat =800` | WARNING | Dosing servo sai MIN |
| `SA: SERVO<m> TRIM=<x>, can dat =1500` | WARNING | Dosing servo sai TRIM |
| `SA: SERVO<m> MAX=<x>, can dat =2200` | WARNING | Dosing servo sai MAX |
| `SA: Dosing motor ON` | INFO | RC bật motor |
| `SA: Dosing motor OFF` | INFO | RC tắt motor |
| `[FLOW] M<n> Tgt:<x> Act:<y> Avg:<z> PWM:<w>` | INFO | Console log lưu lượng (khi `SA_FLOW_LOG=1`) |
| `[DOS] SERVO<n> ON/OFF SP:<x>g PWM:<y>` | INFO | Console log dosing motor (khi `SA_DOS_LOG=1`) |
| `[WM] pH:<x> MA:<y> Tmp:<z>C mV:<w>` | INFO | Console log pH (khi `SA_PH_LOG=1`) |
| `[WM] Alk:<x>dKH <y>mg/L dPH:<z> [FULL/MORN/AFT/PREV/NODATA]` | INFO | Console log kiềm + chất lượng slot |
| `[WM] Ngay moi: slot reset. ...` | INFO | Reset slot khi sang ngày mới (GPS time) |
| `[WM] Chua co GPS time, kiem tinh theo pH tuc thoi` | WARNING | Không có GPS → không phân slot được |

---

## 7. Yêu cầu phần cứng & kết nối

### 7.1 Cảm biến lưu lượng YF-S402B

```
Dải hoạt động: 0.3 – 6 L/min
Giao tiếp:     GPIO, xung Hall Effect
Pin firmware:  GPIO 55 (interrupt RISING edge)
Noise floor:   0.01 L/min (dưới ngưỡng này ép về 0)
Hệ số mặc định: 3874.5 pulses/Litre (chỉnh qua SA_CAL_FAC)
```

### 7.2 Cảm biến pH Nengshi ASPS3801D-0.5M

```
Giao tiếp:   Modbus RTU, RS485 A/B
Baud:        9600 8N1
Module cầu:  RS485 → TTL (UART) → cắm vào cổng TELEMx
Địa chỉ:     0x01 (slave address = 1)
Chu kỳ poll: 2000 ms
```

### 7.3 Bơm phun

```
Kênh servo:    SA_PUMP_CHAN (mặc định 8)
Yêu cầu:       SERVOx_FUNCTION = 0 (None)
Điều khiển:    PWM từ SERVOx_MIN đến SERVOx_MAX
Điểm dừng:     SERVOx_TRIM
```

### 7.4 Động cơ servo 360° (vít tải)

```
Kênh servo:    SA_DOS_CHAN (mặc định 10)
Bắt buộc:      FUNCTION=0, MIN=800, TRIM=1500, MAX=2200
Dừng:          PWM = 1500 µs
Chiều thuận:   PWM 800–1500 (800 = nhanh nhất)
Chiều ngược:   PWM 1500–2200 (2200 = nhanh nhất ngược chiều)
```

---

*File này được tạo tự động từ source code AP_ShoesAgtech.cpp/.h và các file Rover tích hợp.*
*Cập nhật lần cuối theo commit: `ardupilot_git_tutorial` / `AP_ShoesAgtech` untracked.*
