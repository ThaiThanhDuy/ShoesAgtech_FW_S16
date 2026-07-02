# Module 3 — Dosing Motor (Vít Tải Thức Ăn Tôm)
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 10 Hz (`_update_dosing_motor()` gọi từ `update()`)
**Ngày hoàn thành:** 2026-05-15

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
                        ├──► DOS_MODE=0: tính offset từ SA_DOS_SP/SA_DOS_RATE
                        │
                        └──► DOS_MODE=1: _get_mission_dist() [dùng chung Module 1]
                                         + speed (real hoặc _sim_speed)
                                         → tính dos_gpm → offset
                                         (không đủ điều kiện → offset=0 + warn 5s)
                              │
                              └──► Áp dụng SA_DOS_REV → _dos_pwm
                                   SRV_Channels::set_output_pwm_chan()
```

### 1.2 Mô tả từng hàm

---

**`_check_dosing_config()`**
- **File:** `AP_ShoesAgtech.cpp : 745`
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
- **File:** `AP_ShoesAgtech.cpp : 800`
- **Được gọi bởi:** `update()` mỗi chu kỳ
- **Đầu vào:** không có
- **Xử lý:**
  1. Gọi `_check_dosing_config()` → nếu `!_dos_config_ok`: `_dos_pwm=1500`, return
  2. Đọc RC: `rc_pwm = RC_Channels::get_radio_in(SA_DOS_RC − 1)`
  3. `motor_on = (rc_pwm > 1500)` — mất tín hiệu (rc_pwm=0) → tắt an toàn
  4. Nếu state thay đổi → STATUSTEXT ON/OFF
  5. Nếu `!motor_on` → `_dos_pwm = 1500`; xuất; log; return
  6. **DOS_MODE=0:** `offset_us = SA_DOS_SP × 50 / SA_DOS_RATE`
  7. **DOS_MODE=1:**
     - `mission_dist = _get_mission_dist()`
     - `speed = (SA_SIM) ? _sim_speed : groundspeed()`
     - Điều kiện: `dist > 1.0m && speed ≥ 0.05 m/s`
     - Đủ: `dos_gpm = (SA_DOS_SP × speed × 60) / dist`; `offset_us = dos_gpm × 50 / SA_DOS_RATE`
     - Không đủ: `offset_us = 0` + STATUSTEXT mỗi 5s (dùng `_dos_warn_ms`)
  8. Áp dụng chiều quay:
     - `SA_DOS_REV=0`: `pwm = constrain(1500 − offset_us, 800, 1500)`
     - `SA_DOS_REV=1`: `pwm = constrain(1500 + offset_us, 1500, 2200)`
  9. `SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, _dos_pwm)`
  10. Console log nếu SA_DOS_LOG=1
- **Đầu ra / Return:** `void` — side effects: `_dos_pwm`, servo output

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_DOS_CHAN` | 25 | Int8 | 10 | 1 | 16 | Kênh servo đầu ra motor (1-indexed). Phải đúng 4 điều kiện SERVO. |
| `SA_DOS_RC` | 26 | Int8 | 8 | 1 | 16 | Kênh RC bật/tắt motor. PWM>1500 → bật; ≤1500 hoặc =0 (mất tín hiệu) → tắt. |
| `SA_DOS_RATE` | 27 | Float | 100.0 | 1 | 1000 | Tỉ lệ quy đổi: số gam ứng với 50µs lệch khỏi điểm dừng 1500µs. |
| `SA_DOS_SP` | 28 | Float | 0.0 | 0 | — | Setpoint lượng thức ăn (gam). 0 = không cấp (offset=0). |
| `SA_DOS_REV` | 29 | Int8 | 0 | 0 | 1 | Chiều quay: **0**=thuận (800–1500), **1**=ngược (1500–2200). |
| `SA_DOS_LOG` | 30 | Int8 | 0 | 0 | 1 | Bật (1) console log dosing motor theo chu kỳ `SA_DOS_LOG_MS`. |
| `SA_DOS_LOG_MS` | 31 | Int16 | 1000 | 100 | 60000 | Chu kỳ console log dosing (ms). |
| `SA_DOS_MODE` | 36 | Int8 | 0 | 0 | 1 | **0**=tốc độ cố định, **1**=tỉ lệ theo vận tốc + mission. |

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

**DOS_MODE=0 (tốc độ cố định):**
```
offset_us = SA_DOS_SP × 50 / SA_DOS_RATE   (µs)

Đơn vị: gam × µs/gam = µs ✓
Ví dụ: SP=500g, RATE=100 → offset = 500×50/100 = 250µs
```

**DOS_MODE=1 (tỉ lệ mission):**
```
speed_ms     = ahrs.groundspeed()   (hoặc _sim_speed nếu SA_SIM=1)
mission_dist = _get_mission_dist()  (xem MODULE1_FLOW_DETAIL_DESIGN)

Điều kiện đủ: mission_dist > 1.0m AND speed_ms ≥ 0.05m/s

dos_gpm   = (SA_DOS_SP × speed_ms × 60) / mission_dist   (gam/phút)
offset_us = dos_gpm × 50 / SA_DOS_RATE                   (µs)

Không đủ: offset_us = 0 → pwm = 1500 + warn mỗi 5s
```

**Áp dụng chiều quay:**
```
SA_DOS_REV=0 (thuận):  pwm = constrain(1500 − offset_us,  800, 1500)
SA_DOS_REV=1 (ngược):  pwm = constrain(1500 + offset_us, 1500, 2200)
→ SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, pwm)
```

### 3.5 Xử lý các trường hợp đặc biệt

| Điều kiện | Hành vi | Output GCS |
|---|---|---|
| Bất kỳ 1 trong 4 điều kiện servo sai | Không xuất PWM (giữ 1500) | STATUSTEXT cụ thể lỗi, mỗi 5s |
| `RC_DOS PWM = 0` (mất tín hiệu) | Xuất 1500 (dừng) | data[14]=1500 |
| `SA_DOS_SP = 0` | offset_us = 0 → pwm = 1500 | data[14]=1500 |
| `SA_DOS_MODE=1` & dist ≤ 1m | offset_us=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_DOS_MODE=1` & speed < 0.05 | offset_us=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_SIM=1` | `_sim_speed` thay `groundspeed()` | Tính toán vẫn chạy bình thường |

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

| Index | Tên | Đơn vị | Điều kiện ghi | Mô tả |
|---|---|---|---|---|
| `data[12]` | `dos_sp` | gam | Luôn | `SA_DOS_SP`; ý nghĩa phụ thuộc SA_DOS_MODE |
| `data[13]` | `dos_rate` | gam/50µs | Luôn | `SA_DOS_RATE`; tỉ lệ quy đổi |
| `data[14]` | `dos_pwm` | µs | Luôn | PWM thực tế đang xuất; 1500=dừng |

### 4.2 DataFlash Log [DATA]

N/A — Module 3 không ghi DataFlash riêng. Trạng thái theo dõi qua SA_DATA và console log.

### 4.3 Console Log

```
Trigger: mỗi SA_DOS_LOG_MS ms khi SA_DOS_LOG=1
Format:  [DOS] SERVO<n> ON/OFF SP:<dos_sp>g PWM:<dos_pwm>
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

---

## 8. Tài liệu liên quan [ALL]

- [MODULE3_DOS_BASIC_DESIGN.md](MODULE3_DOS_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — `_get_mission_dist()` dùng chung, `update()` gọi `_update_dosing_motor()`
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA (data[12..14])
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
