# Module 3 — Dosing Motor (Vít Tải Thức Ăn Tôm)
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 10 Hz (`_update_dosing_motor()` gọi từ `update()`)
**Ngày hoàn thành:** 2026-05-15 | **Cập nhật lần cuối:** 2026-07-09

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
update() [Module 1, 10 Hz]
    │
    └──► _update_dosing_motor()
              │
              ├──► _check_dosing_config()
              │         │ kiểm tra 4 điều kiện servo
              │         └──► cập nhật _dos_config_ok
              │
              ├──► (config sai) → _dos_pwm = 1500, return
              │
              ├──► Đọc RC kênh SA_DOS_RC → motor_on
              │
              ├──► (motor_on thay đổi) → STATUSTEXT "ON" / "OFF"
              │
              ├──► (motor_on=false) → _dos_pwm = 1500
              │
              └──► (motor_on=true)
                        │
                        ├──► Lấy food_idx = SA_DOS_FOOD - 1  (0-indexed)
                        │
                        ├──► DOS_MODE=0:
                        │       vol_rate = SA_DOS_RATE        (mL/50us)
                        │       density  = SA_DOS_Dx[food_idx] (g/mL)
                        │       offset   = SP * 50 / (vol_rate * density)
                        │
                        └──► DOS_MODE=1: _get_mission_dist() [dùng chung Module 1]
                                         + speed (real hoặc _sim_speed)
                                         → dos_gpm = SP * speed * 60 / dist
                                         vol_rate1 = SA_DOS_Fx[food_idx]  (mL/50us)
                                         density1  = SA_DOS_Dx[food_idx]  (g/mL)
                                         offset    = dos_gpm * 50 / (vol_rate1 * density1)
                                         (không đủ điều kiện → offset=0 + warn 5s)
                              │
                              └──► Áp dụng SA_DOS_REV → _dos_pwm
                                   SRV_Channels::set_output_pwm_chan()
```

### 1.2 Mô tả từng hàm

---

**`_check_dosing_config()`**
- **File:** `AP_ShoesAgtech.cpp : ~915`
- **Được gọi bởi:** `_update_dosing_motor()` mỗi chu kỳ 10 Hz
- **Đầu vào:** không có (đọc `_dos_chan` từ param)
- **Xử lý:**
  1. Lấy `chan_idx = SA_DOS_CHAN − 1`
  2. Kiểm tra 4 điều kiện trên `SRV_Channel`:
     - `FUNCTION = k_none (0)`
     - `MIN = 800`
     - `TRIM = 1500`
     - `MAX = 2200`
  3. Nếu tất cả đúng: nếu vừa chuyển từ sai → OK, in "setup thành công" 1 lần
  4. Nếu sai bất kỳ: in từng điều kiện sai mỗi 5s (dùng `_dos_warn_ms`)
- **Đầu ra / Return:** `void` — cập nhật `_dos_config_ok` và `_dos_was_ok`
- **Ghi chú:** Không dùng `_last_warn_ms` của pump — dùng `_dos_warn_ms` riêng để không tranh chấp timer.

---

**`_update_dosing_motor()`**
- **File:** `AP_ShoesAgtech.cpp : ~1021`
- **Được gọi bởi:** `update()` mỗi chu kỳ
- **Đầu vào:** không có
- **Xử lý:**
  1. Gọi `_check_dosing_config()` → nếu `!_dos_config_ok`: `_dos_pwm=1500`, return
  2. Đọc RC: `rc_pwm = RC_Channels::get_radio_in(SA_DOS_RC − 1)`
  3. `motor_on = (rc_pwm > 1500)` — mất tín hiệu (rc_pwm=0) → tắt an toàn
  4. Nếu state thay đổi → STATUSTEXT ON/OFF
  5. Nếu `!motor_on` → `_dos_pwm = 1500`; xuất; log; return
  6. `food_idx = constrain(SA_DOS_FOOD, 1, 7) − 1` (0-indexed, dùng cho cả 2 mode)
  7. **DOS_MODE=0:**
     - `vol_rate = max(SA_DOS_RATE, 0.1)` (mL/50us)
     - `density = max(SA_DOS_Dx[food_idx], 0.01)` (g/mL)
     - `offset_us = SA_DOS_SP × 50 / (vol_rate × density)`
  8. **DOS_MODE=1:**
     - `vol_rate1 = max(SA_DOS_Fx[food_idx], 0.1)` (mL/50us)
     - `density1 = max(SA_DOS_Dx[food_idx], 0.01)` (g/mL)
     - `effective1 = vol_rate1 × density1` (g/50us)
     - `mission_dist = _get_mission_dist()`
     - `speed = (SA_SIM) ? _sim_speed : groundspeed()`
     - Điều kiện: `dist > 1.0m && speed ≥ 0.05 m/s`
     - Đủ: `dos_gpm = SA_DOS_SP × speed × 60 / dist`; `offset_us = dos_gpm × 50 / effective1`
     - Không đủ: `offset_us = 0` + STATUSTEXT mỗi 5s
  9. Áp dụng chiều quay:
     - `SA_DOS_REV=0`: `pwm = constrain(1500 − offset_us, 800, 1500)`
     - `SA_DOS_REV=1`: `pwm = constrain(1500 + offset_us, 1500, 2200)`
  10. `SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, _dos_pwm)`
  11. Console log nếu SA_DOS_LOG=1
- **Đầu ra / Return:** `void` — side effects: `_dos_pwm`, servo output

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_DOS_CHAN` | 25 | Int8 | 10 | 1 | 16 | Kênh servo đầu ra motor (1-indexed). Phải đúng 4 điều kiện SERVO. |
| `SA_DOS_RC` | 26 | Int8 | 8 | 1 | 16 | Kênh RC bật/tắt motor. PWM>1500 → bật; ≤1500 hoặc =0 (mất tín hiệu) → tắt. |
| `SA_DOS_RATE` | 27 | Float | 100.0 | 0.1 | 10000 | **Thể tích vít tải (mL/50us)** — bao nhiêu mL tống ra ứng với 50µs lệch. Thông số cơ học cố định, không đổi khi thay hạt. Backward compat: đặt SA_DOS_D1..D7=1.0 → công thức tương đương cũ. |
| `SA_DOS_SP` | 28 | Float | 0.0 | 0 | — | Setpoint lượng thức ăn (gam). 0 = không cấp (offset=0). |
| `SA_DOS_REV` | 29 | Int8 | 0 | 0 | 1 | Chiều quay: **0**=thuận (800–1500), **1**=ngược (1500–2200). |
| `SA_DOS_LOG` | 30 | Int8 | 0 | 0 | 1 | Bật (1) console log dosing motor theo chu kỳ `SA_DOS_LOG_MS`. |
| `SA_DOS_LOG_MS` | 31 | Int16 | 1000 | 100 | 60000 | Chu kỳ console log dosing (ms). |
| `SA_DOS_MODE` | 36 | Int8 | 0 | 0 | 1 | **0**=tốc độ cố định, **1**=tỉ lệ theo vận tốc + mission. |
| `SA_DOS_FOOD` | 40 | Int8 | 1 | 1 | 7 | Chọn loại thức ăn đang dùng. Hệ thống dùng SA_DOS_Fx và SA_DOS_Dx tương ứng. |
| `SA_DOS_F1..F7` | 41–47 | Float | 100.0 | 0.1 | 10000 | **Thể tích vít tải (mL/50us) theo loại thức ăn** — dùng trong DOS_MODE=1. Cho phép calib riêng nếu loại hạt khác nhau ảnh hưởng đến ma sát vít tải. |
| `SA_DOS_D1..D7` | 48–54 | Float | 1.0 | 0.1 | 5.0 | **Khối lượng riêng (g/mL) theo loại thức ăn** — mặc định 1.0 (backward compat). Đo: đổ đầy 1000mL, cân → chia 1000. Thức ăn tôm viên thực tế ≈ 0.5–0.7 g/mL. |

> **Backward compatibility:** SA_DOS_D1..D7 mặc định = 1.0 g/mL → `offset = SP × 50 / RATE` — hoàn toàn tương đương công thức cũ. Không cần đổi SA_DOS_RATE nếu chưa đo density.

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

Không có init riêng. `_check_dosing_config()` chạy lần đầu khi `_update_dosing_motor()` được gọi (chu kỳ đầu tiên của `update()`).

### 3.2 Kiểm tra điều kiện servo

```
Bắt buộc cả 4 (x = SA_DOS_CHAN):
    SERVOx_FUNCTION = 0
    SERVOx_MIN      = 800
    SERVOx_TRIM     = 1500
    SERVOx_MAX      = 2200

Kiểm tra mỗi chu kỳ (10 Hz), cảnh báo mỗi 5s nếu sai.
Sai → KHÔNG xuất PWM (giữ 1500).
```

### 3.3 Logic RC bật/tắt

```
rc_pwm = RC_Channels::get_radio_in(SA_DOS_RC − 1)   (0-indexed)
motor_on = (rc_pwm > 1500)

rc_pwm=0 (mất tín hiệu) → motor_on = false → dừng an toàn (fail-safe)
```

### 3.4 Công thức tính PWM

**Khái niệm tách biệt cơ học và hạt:**
```
vol_rate  = thể tích vít tải (mL/50us) — SA_DOS_RATE hoặc SA_DOS_Fx
           Đặc tính của vít tải: calibrate 1 lần, không đổi khi thay loại hạt.

density   = khối lượng riêng hạt (g/mL) — SA_DOS_Dx (x = SA_DOS_FOOD)
           Đặc tính của loại thức ăn: thay loại hạt thì cập nhật giá trị này.

effective = vol_rate × density   (g/50us) — "tỉ lệ hiệu dụng" = khối lượng/PWM
```

**DOS_MODE=0 (tốc độ cố định):**
```
vol_rate = max(SA_DOS_RATE, 0.1)          (mL/50us)
density  = max(SA_DOS_Dx[food_idx], 0.01) (g/mL)
offset_us = SA_DOS_SP(g) × 50 / (vol_rate × density)

Đơn vị: g × us / (mL/50us × g/mL) = g × us × 50us / (g) = 50us² ... → µs ✓
Ví dụ: SP=500g, DOS_RATE=100mL/50us, D1=0.6g/mL
  → offset = 500 × 50 / (100 × 0.6) = 416.7µs
  → pwm = 1500 - 417 = 1083µs (DOS_REV=0)
```

**DOS_MODE=1 (tỉ lệ mission):**
```
food_idx  = SA_DOS_FOOD - 1               (0-indexed)
vol_rate1 = max(SA_DOS_Fx[food_idx], 0.1) (mL/50us)
density1  = max(SA_DOS_Dx[food_idx], 0.01)(g/mL)
effective1 = vol_rate1 × density1         (g/50us)

speed_ms     = ahrs.groundspeed()   (hoặc _sim_speed nếu SA_SIM=1)
mission_dist = _get_mission_dist()  (xem MODULE1_FLOW_DETAIL_DESIGN)

Điều kiện đủ: mission_dist > 1.0m AND speed_ms ≥ 0.05m/s

dos_gpm   = (SA_DOS_SP × speed_ms × 60) / mission_dist   (g/phút)
offset_us = dos_gpm × 50 / effective1                     (µs)

Không đủ: offset_us = 0 → pwm = 1500 + warn mỗi 5s
```

**Áp dụng chiều quay:**
```
SA_DOS_REV=0 (thuận):  pwm = constrain(1500 − offset_us,  800, 1500)
SA_DOS_REV=1 (ngược):  pwm = constrain(1500 + offset_us, 1500, 2200)
→ SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, pwm)
```

### 3.5 Workflow calibrate

**Calibrate SA_DOS_RATE (thể tích vít tải) — chỉ làm 1 lần:**
```
1. Cài SA_DOS_REV đúng chiều, DOS_SP=0 → tạm dừng motor
2. Đặt SA_DOS_RATE = 100 (tạm), SA_DOS_D1 = 1.0
3. Chạy motor ở offset cố định (đặt DOS_SP thử nghiệm)
4. Thu thức ăn vào bình đong trong X giây → đo thể tích V (mL) và khối lượng M (g)
5. density_actual = M / V  → cập nhật SA_DOS_Dx
6. DOS_RATE_actual = V(mL) × 50us / offset_thực_us → cập nhật SA_DOS_RATE
```

**Calibrate SA_DOS_Dx (khối lượng riêng hạt) — làm khi đổi loại hạt:**
```
1. Đổ đầy bình đong 1000mL bằng hạt thức ăn loại x
2. Cân bình → trừ tara → được M(g)
3. SA_DOS_Dx = M / 1000
VD: cân được 620g → SA_DOS_D1 = 0.62
```

### 3.6 Xử lý các trường hợp đặc biệt

| Điều kiện | Hành vi | Output GCS |
|---|---|---|
| Bất kỳ 1 trong 4 điều kiện servo sai | Không xuất PWM (giữ 1500) | STATUSTEXT cụ thể lỗi, mỗi 5s |
| `RC_DOS PWM = 0` (mất tín hiệu) | Xuất 1500 (dừng) | data[14]=1500 |
| `SA_DOS_SP = 0` | offset_us = 0 → pwm = 1500 | data[14]=1500 |
| `SA_DOS_RATE < 0.1` hoặc `SA_DOS_Dx < 0.01` | Clamp về giá trị min → tránh chia cho 0 | PWM tính được (không crash) |
| `SA_DOS_MODE=1` & dist ≤ 1m | offset_us=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_DOS_MODE=1` & speed < 0.05 | offset_us=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_SIM=1` | `_sim_speed` thay `groundspeed()` | Tính toán vẫn chạy bình thường |
| Đổi SA_DOS_FOOD | Áp dụng ngay SA_DOS_Dx và SA_DOS_Fx mới trong chu kỳ kế | Log hiển thị F<n> và D:<x>g/mL mới |

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

`DEBUG_FLOAT_ARRAY`, name=`"SA_DATA"`, array_id=0, ghi trong `send_shoesagtech_debug_arrays()` ở `Rover/GCS_MAVLink_Rover.cpp`.

| Index | Tên | Đơn vị | Điều kiện ghi | Mô tả |
|---|---|---|---|---|
| `data[12]` | `dos_sp` | gam | Luôn | `SA_DOS_SP`; lượng thức ăn setpoint |
| `data[13]` | `dos_rate` | mL/50us | Luôn | `SA_DOS_RATE`; thể tích vít tải (đã đổi đơn vị từ g sang mL) |
| `data[14]` | `dos_pwm` | µs | Luôn | PWM thực tế đang xuất; 1500=dừng |
| `data[15]` | `dos_food` | 1–7 | Luôn | `SA_DOS_FOOD`; loại thức ăn đang chọn |

### 4.2 DataFlash Log [DATA]

N/A — Module 3 không ghi DataFlash riêng. Trạng thái theo dõi qua SA_DATA và console log.

### 4.3 Console Log

```
Trigger: mỗi SA_DOS_LOG_MS ms khi SA_DOS_LOG=1
Format:  [DOS] M<mode> F<food> SERVO<n> ON/OFF SP:<sp>g D:<density>g/mL PWM:<pwm>

Ví dụ (DOS_MODE=0, F1, D1=0.62):
  [DOS] M0 F1 SERVO10 ON SP:500g D:0.62g/mL PWM:1083
```

### 4.4 STATUSTEXT — Toàn bộ thông báo

| Nội dung thông báo | Mức | Điều kiện | Tần suất |
|---|---|---|---|
| `SA: SERVO<m> setup thanh cong - dosing motor san sang` | INFO | 4 điều kiện OK (edge rising) | 1 lần/lần vừa đúng |
| `SA: SERVO<m> FUNCTION=<x>, can dat =0 (None)` | WARNING | FUNCTION sai | Mỗi 5s |
| `SA: SERVO<m> MIN=<x>, can dat =800` | WARNING | MIN sai | Mỗi 5s |
| `SA: SERVO<m> TRIM=<x>, can dat =1500` | WARNING | TRIM sai | Mỗi 5s |
| `SA: SERVO<m> MAX=<x>, can dat =2200` | WARNING | MAX sai | Mỗi 5s |
| `SA: Dosing motor ON` | INFO | RC bật (edge rising) | 1 lần/lần bật |
| `SA: Dosing motor OFF` | INFO | RC tắt (edge falling) | 1 lần/lần tắt |
| `SA DOS1: chua co mission (dist=<x>m) - motor dung` | WARNING | DOS_MODE=1, dist ≤ 1m | Mỗi 5s |
| `SA DOS1: toc do qua thap (<x>m/s) - motor dung` | WARNING | DOS_MODE=1, speed < 0.05 | Mỗi 5s |

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERVOx_FUNCTION = 0     (x = SA_DOS_CHAN)  ┐
SERVOx_MIN      = 800                      │  Sai bất kỳ 1 → KHÔNG chạy + warn 5s
SERVOx_TRIM     = 1500                     │
SERVOx_MAX      = 2200                     ┘

SA_DOS_MODE=1: mission đã upload lên FC VÀ speed ≥ 0.05m/s
               thiếu 1 trong 2 → motor dừng (offset=0)

SA_DOS_RATE > 0.1 mL/50us   (firmware clamp, không crash)
SA_DOS_Dx   > 0.01 g/mL     (firmware clamp, không crash)
```

**Ràng buộc phần cứng:** Cần BEC riêng cho servo — không dùng nguồn FC (servo 360° tải cao)

**Ràng buộc vận hành:** Kiểm tra chiều quay đúng (`SA_DOS_REV`) trên bench trước khi lắp vào cơ cấu

---

## 6. Kết nối phần cứng [HW]

```
Servo 360° (vít tải):
  Signal  →  SERVO output SA_DOS_CHAN (default CH10)
  VCC     →  BEC 5V hoặc 6V (tách nguồn khỏi FC)
  GND     →  GND chung

FC config (x = SA_DOS_CHAN):
  SERVOx_FUNCTION = 0
  SERVOx_MIN      = 800
  SERVOx_TRIM     = 1500
  SERVOx_MAX      = 2200
```

**Lưu ý board:** Một số board (Pixhawk4) không xuất PWM khi `SERVOx_FUNCTION=0` nếu chưa armed hoặc safety switch chưa OFF. Kiểm tra Mission Planner → Servo Output trước khi lắp.

---

## 7. So sánh với Basic Design [ALL]

| Điểm | Basic Design dự kiến | Thực tế đã làm | Lý do |
|---|---|---|---|
| 4 điều kiện servo | Đề cập đủ | Implement đúng theo Basic Design ✓ | — |
| Fail-safe mất tín hiệu RC | RC=0 → dừng | `rc_pwm > 1500` (strict); rc_pwm=0 → motor_on=false | 0 là giá trị RC mất tín hiệu, đảm bảo dừng an toàn |
| Timer cảnh báo | Dùng chung timer warn | Dùng `_dos_warn_ms` riêng cho dosing | Tránh tranh chấp với `_last_warn_ms` của pump module |
| DOS_MODE=1, warn label | Đề cập chung | Warn cụ thể lý do (dist hoặc speed) với giá trị thực | Người dùng biết cụ thể vấn đề |
| Công thức DOS_RATE | g/50us (g và PWM gộp lại) | **Tách thành vol_rate (mL/50us) × density (g/mL)** | Calibrate vít tải 1 lần; đổi loại hạt chỉ cập nhật SA_DOS_Dx; SA_DOS_F1..F7 cũng đổi sang mL/50us |
| SA_DOS_F1..F7 | rate g/50us theo loại hạt | **mL/50us** theo loại hạt — cùng đơn vị với SA_DOS_RATE | Đồng nhất đơn vị, loại hạt chỉ ảnh hưởng density |
| SA_DOS_D1..D7 | Không có | Thêm mới: khối lượng riêng (g/mL) cho 7 loại thức ăn; mặc định 1.0 (backward compat) | Hoàn thiện mô hình vật lý: khối lượng = thể tích × khối lượng riêng |
| SA_DATA[15] | Không có | Thêm dos_food (loại thức ăn đang chọn) | Hiển thị trên GCS, theo dõi loại hạt đang dùng |
| Console log | `F<n> SERVO<m> ON/OFF SP:<sp>g PWM:<pwm>` | Thêm `M<mode>` và `D:<density>g/mL` | Người dùng thấy ngay khối lượng riêng đang dùng |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE3_DOS_BASIC_DESIGN.md](MODULE3_DOS_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — `_get_mission_dist()` dùng chung, `update()` gọi `_update_dosing_motor()`
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA (data[12..15])
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống