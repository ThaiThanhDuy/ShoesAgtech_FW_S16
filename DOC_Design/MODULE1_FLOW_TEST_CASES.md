# Module 1 — Flow Sensor & Spray Controller

## Test Cases — Số đối ứng cụ thể

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Ngày viết:** 2026-07-08 | **Cập nhật lần cuối:** 2026-07-16

> **Thay đổi quan trọng so với bản trước:** `SA_APP_RATE` và `SA_BOOM_W` đã bị gỡ bỏ hoàn toàn khỏi firmware — công thức FLOW_MODE=1 cũ (`q1 = r × APP_RATE × speed × BOOM_W × 0.006`, `dist_max` cố định không đổi theo tốc độ) **không còn đúng**. Toàn bộ TC-04, TC-05, TC-06, TC-08, TC-09 trong file này đã được tính lại theo công thức hiện tại:
> ```
> q1 = SA_TANK_VOL × r × speed × 60 / mission_dist        (L/min)
> dist_max (tại speed hiện tại) = SA_TANK_VOL × r × speed × 60 / 0.3   (m — trần trên trước khi q1 < 0.3)
> dist_min (tại speed hiện tại) = SA_TANK_VOL × r × speed × 60 / 2.0   (m — trần dưới trước khi q1 > 2.0)
> vi_per_run = SA_TANK_VOL × r    (hằng số — KHÔNG đổi theo speed/mission, xem giải thích ở TC-04)
> ```
> **Khác biệt lớn nhất:** `dist_max`/`dist_min` giờ phụ thuộc tốc độ xe tại thời điểm tính (không còn là hằng số cố định như công thức cũ). Kết quả là **một kịch bản test cũ (nấc cao, tank 16L, mission 301m, v=1.3 m/s) không còn PASS** với công thức mới — xem TC-05 bên dưới để biết vì sao và cách chỉnh tốc độ vận hành cho phù hợp.

---

## Thông số phần cứng dùng trong tất cả các test

| Thông số           | Giá trị       | Ghi chú                                          |
| ------------------- | ------------- | ------------------------------------------------ |
| Cảm biến lưu lượng | YF-S402B      | Dải 0.3–6 L/min                                  |
| SA_CAL_FAC         | **1745**      | Calibrated (pulses/L)                            |
| SA_TANK_VOL        | **16** L      | Tham chiếu định lượng vi sinh cho FLOW_MODE=1 — xem lưu ý TC-04 |
| SA_FLOW_MIX_STD         | **0.35**      | Nấc giữa — van vi sinh mức 2                     |
| SA_FLOW_MIX_CNT         | **0.50**      | Nấc cao — van vi sinh mức 2 (same), chỉnh van hồ |
| SA_FLOW_SP         | **1.2** L/min | Max vi sinh vật lý tại van vi sinh mức 2 (dùng cho TC-02/03, FLOW_MODE=0) |
| SA_FLOW_PID_P           | **80**        | µs per L/min error                               |
| SA_FLOW_PID_I           | **20**        | µs per L/min/s                                   |
| SA_FLOW_PID_LPF         | **0.3**       | Output smoothing                                 |
| SERVO8_MIN         | **1000**      | pwm_min bơm                                      |
| SERVO8_TRIM        | **1500**      | pwm_trim bơm                                     |
| SERVO8_MAX         | **2000**      | pwm_max bơm                                      |
| Vận tốc thực tế    | **1.3** m/s   | Tốc độ xe phun (nấc giữa) — xem TC-05 cho nấc cao |
| Mission distance   | **301** m     | Tuyến đường đã upload                            |

> **Lưu ý:** Từ bản này, `SA_TANK_VOL` **không phải** dung tích vật lý của thùng chứa — đây là hằng số hiệu chuẩn: "tổng lượng vi sinh sẽ dùng cho **một** lần chạy hết mission, nếu tỉ lệ trộn r=1.0 (100%)". Với `r` (SA_FLOW_MIX_STD/SA_FLOW_MIX_CNT) < 1.0, lượng thực tế dùng mỗi lần chạy = `SA_TANK_VOL × r` (xem TC-04). Firmware **không** theo dõi mức vi sinh còn lại trong thùng vật lý ở FLOW_MODE=1 — người vận hành tự đối chiếu bằng cách đo trực tiếp.

---

## TC-01 — Spray mode 0 (PASSTHROUGH)

**Điều kiện:** RC nấc dưới → `_spray_mode = 0`

| Bước | Hành động                           | Kết quả mong đợi                                |
| ---- | ------------------------------------ | ------------------------------------------------ |
| 1    | Gạt RC về nấc thấp nhất             | GCS: không có SA log                            |
| 2    | Chỉnh RC_PUMP (kênh 9) lên ~1700 µs | SERVO8 output = 1700 µs (passthrough trực tiếp) |
| 3    | Chỉnh RC_PUMP xuống 1100 µs         | SERVO8 output = 1100 µs                         |
| 4    | Trả RC_PUMP về 1500 µs              | SERVO8 output = 1500 µs — bơm dừng              |

**SA_DATA debug:** `data[3]` (pump_pwm) bằng giá trị RC_PUMP; `data[4]` (spray_mode) = `0`

---

## TC-02 — FLOW_MODE=0, Spray mode 1 (FLOW PID, nấc giữa)

**Cài param trước test:**

```
SA_FLOW_MODE = 0
SA_FLOW_SP   = 1.2
```

**Điều kiện:** RC nấc giữa → `_spray_mode = 1`

### TC-02a — Khởi động PID (giây đầu tiên)

Lúc bắt đầu, lưu lượng thực = 0 L/min, integral = 0:

```
error         = 1.2 − 0.0     = 1.2 L/min
P_output      = 80 × 1.2      = 96 µs
I_output      = 0              (integral chưa tích lũy)
pid_raw       = 96 µs
lpf_output    ≈ 96 × 0.3 + 0 × 0.7 = 28.8 µs  (sau 1 chu kỳ 100ms đầu)
PWM           = 1500 + 28.8   = ~1529 µs
```

Sau khoảng 3–5 giây PID hội tụ:

```
error         → 0
integral      tích lũy giúp bù offset tĩnh
PWM ổn định  ≈ 1500 + (offset để duy trì 1.2 L/min)
```

### TC-02b — Steady-state (sau hội tụ)

| Đo                       | Giá trị                        | Cách kiểm tra         |
| ------------------------- | ------------------------------- | ---------------------- |
| SA_DATA[0] (flow EMA)    | **1.2** L/min (±0.05)          | GCS MAVLink Inspector |
| SA_DATA[2] (flow_target) | **1.2** L/min                  | GCS                   |
| SA_DATA[3] (pump_pwm)    | Ổn định, không dao động        | GCS                   |
| SA_DATA[4] (spray_mode)  | **1**                          | GCS                   |
| GCS FLOW_LOG             | `SP=1.20 actual=1.2x pwm=xxxx` | SA_FLOW_LOG=1         |

### TC-02c — Anti-windup PID

Đặt `SA_FLOW_SP = 5.0` (vượt quá khả năng vật lý):

```
I limit = (2000−1000) × 0.5 / 20 = 25 L·s/min  (integral bị clamped)
PWM sẽ lên đến 2000 µs nhưng không bị unbounded
```

**Kết quả mong đợi:** PWM = 2000, flow actual < 5.0, không treo firmware.

---

## TC-03 — FLOW_MODE=0, Spray mode 2 (FLOW PID, nấc cao/chống nghẹt)

**Cài param trước test:**

```
SA_FLOW_MODE = 0
SA_FLOW_SP   = 1.2
SA_FLOW_MIX_STD   = 0.35
SA_FLOW_MIX_CNT   = 0.50
```

**Flow target tự động tính:**

```
flow_target = SA_FLOW_SP × (FLOW_MIX_CNT / FLOW_MIX_STD)
            = 1.2 × (0.50 / 0.35)
            = 1.2 × 1.4286
            = 1.714 L/min
```

> **Lưu ý thực tế:** Van vi sinh giữ mức 2 (same mode 1), van hồ mở thêm.
> Cảm biến đo vi sinh max ~1.2 L/min → bơm sẽ cố đạt 1.714 nhưng bị giới
> hạn vật lý → pump_pwm sẽ ở mức cao (gần 2000 µs).

| Đo                       | Giá trị         | Ghi chú                       |
| ------------------------- | ---------------- | ------------------------------ |
| SA_DATA[2] (flow_target) | **1.714** L/min | Tự tính từ ratio              |
| SA_DATA[0] (flow actual) | **~1.2** L/min  | Giới hạn vật lý van vi sinh   |
| SA_DATA[3] (pump_pwm)    | **~2000** µs    | Pump max vì target > khả năng |

---

## TC-04 — FLOW_MODE=1, Spray mode 1 (nấc giữa, r = 0.35)

**Param cài trước test:**

```
SA_FLOW_MODE = 1
SA_TANK_VOL  = 16
SA_FLOW_MIX_STD   = 0.35
SA_FLOW_VEL  = 0       (dùng vận tốc thật)
```

Mission đã upload: **301 m**. Vận tốc vận hành: **1.3 m/s**.

---

### TC-04a — Công thức và giá trị tại tốc độ vận hành (1.3 m/s)

```
q1       = SA_TANK_VOL × r × speed × 60 / mission_dist
         = 16 × 0.35 × 1.3 × 60 / 301
         = 436.8 / 301
         = 1.451 L/min                    (nằm trong dải hợp lệ 0.3–2.0 ✓)

dist_max = SA_TANK_VOL × r × speed × 60 / 0.3     (trần trên mission — q1 sẽ < 0.3 nếu dài hơn)
         = 436.8 / 0.3 = 1 456 m

dist_min = SA_TANK_VOL × r × speed × 60 / 2.0     (trần dưới mission — q1 sẽ > 2.0 nếu ngắn hơn)
         = 436.8 / 2.0 = 218.4 m
```

**Mission 301 m nằm trong khoảng (218.4 m, 1 456 m) tại 1.3 m/s → PASS — bơm cho phép chạy.**

> `dist_max`/`dist_min` phụ thuộc tốc độ hiện tại — đây KHÔNG phải hằng số cố định trước khi chạy (khác với thiết kế cũ). Firmware tính lại mỗi vòng lặp theo tốc độ tức thời.

---

### TC-04b — q1 thay đổi theo vận tốc (mission cố định 301 m, r=0.35)

Công thức rút gọn tại mission=301m: `q1 = (16 × 0.35 × 60 / 301) × v = 1.1163 × v`

| Vận tốc (m/s) | q1 (L/min) | Trong dải 0.3–2.0? | Ghi chú                              |
| -------------- | ---------- | -------------------- | -------------------------------------- |
| 0.20           | 0.223      | ❌ < 0.3           | Xe đi quá chậm → bơm dừng + warning  |
| 0.26           | 0.290      | ❌ < 0.3           |                                       |
| **0.27**      | **0.301**  | ✅                 | Tốc độ tối thiểu để bơm hoạt động    |
| 0.50           | 0.558      | ✅                 |                                       |
| 0.80           | 0.893      | ✅                 |                                       |
| 1.00           | 1.116      | ✅                 |                                       |
| **1.30**      | **1.451**  | ✅                 | **Tốc độ vận hành thực tế**          |
| 1.50           | 1.674      | ✅                 |                                       |
| 1.79           | 1.998      | ✅                 | Tốc độ tối đa trước khi vượt ngưỡng  |
| **1.80**      | **2.009**  | ❌ > 2.0           | Xe đi quá nhanh → bơm dừng + warning |

---

### TC-04c — Lượng vi sinh dùng mỗi lần chạy hết mission (bất biến theo speed)

```
vi_per_run = SA_TANK_VOL × r = 16 × 0.35 = 5.6 L
```

**Vì sao là hằng số:** `vi_used = q1 × (mission_dist/speed) / 60 = [TANK_VOL×r×speed×60/dist] × (dist/speed) / 60 = TANK_VOL×r`
→ Miễn PID bám đúng q1 suốt hành trình, tổng lượng vi sinh dùng cho **một lần chạy hết mission** luôn đúng bằng `TANK_VOL × r`, bất kể tốc độ xe nhanh/chậm hay đổi giữa đường (đây là mục đích thiết kế của công thức).

**Thời gian dự kiến tại 1.3 m/s:**

```
eta_s   = 301 / 1.3 = 231.5 s → eta_min = 3, eta_sec = 51   (3 phút 51 giây)
```

**GCS message khi ARM (tất cả điều kiện OK):**

```
SA FM1 OK: r=0.35 q1=1.45L/ph miss=301m dmax=1456m ~3m51s vi/run=5.6L
```

**Console log định kỳ khi đang chạy (SA_FLOW_LOG=1):**

```
[FLOW] FM1 r:0.35 q1:1.45L/ph miss:301m dmax:1456m spd:1.30m/s vi/run:5.6L
```

---

### TC-04d — Các giá trị SA_DATA cần kiểm tra

| Field SA_DATA | Index | Giá trị mong đợi       | Cách xem          |
| -------------- | ----- | ------------------------ | ------------------ |
| flow_rate EMA | [0]   | **1.45** L/min (±0.05) | MAVLink Inspector |
| flow_rate MA  | [1]   | **1.45** L/min (±0.08) | MAVLink Inspector |
| flow_target   | [2]   | **1.451** L/min        | MAVLink Inspector |
| pump_pwm      | [3]   | Ổn định, không dao động  | MAVLink Inspector |
| spray_mode    | [4]   | **1**                  | MAVLink Inspector |

---

### TC-04e — Thủ tục test thực tế

1. Cài đủ param ở trên, upload mission 301 m.
2. ARM → đọc GCS message, xác nhận: `SA FM1 OK … r=0.35 q1=1.45L/ph … ~3m51s vi/run=5.6L`
3. Chạy xe theo mission, giữ vận tốc ~1.3 m/s.
4. DISARM sau khi kết thúc mission.
5. Đo lại lượng vi sinh thực tế đã dùng — **phải nằm trong khoảng 5.3–5.9 L** (sai số ±6% quanh 5.6 L).

---

## TC-05 — FLOW_MODE=1, Spray mode 2 (nấc cao/chống nghẹt, r = 0.50)

**Param cài trước test:**

```
SA_FLOW_MODE = 1
SA_TANK_VOL  = 16
SA_FLOW_MIX_CNT   = 0.50
SA_FLOW_VEL  = 0
```

> **Khác biệt quan trọng so với nấc giữa:** cùng `TANK_VOL=16` và mission 301 m, nấc cao (r=0.50) đòi hỏi vận tốc xe **thấp hơn** để giữ q1 trong dải 0.3–2.0. Ở tốc độ vận hành 1.3 m/s dùng cho nấc giữa (TC-04), **nấc cao sẽ FAIL** — xem TC-05a.

---

### TC-05a — Kiểm tra ở tốc độ 1.3 m/s (giống TC-04) → FAIL

```
q1       = 16 × 0.50 × 1.3 × 60 / 301 = 624 / 301 = 2.073 L/min   ❌ > 2.0
dist_min = 16 × 0.50 × 1.3 × 60 / 2.0 = 624 / 2.0  = 312 m
```

**Mission 301 m < dist_min 312 m tại 1.3 m/s → FAIL — bơm dừng khi ARM.**

**GCS message khi ARM:**

```
SA FM1: q1=2.07L/ph > 2.0 @1.3m/s dist=301m - kéo dài mission (dmin=312m)
```

**SA_DATA[2] (flow_target) = 0.0**, bơm không chạy cho đến khi giảm tốc độ hoặc kéo dài mission.

---

### TC-05b — Tốc độ vận hành an toàn cho nấc cao: 1.0 m/s

```
q1       = 16 × 0.50 × 1.0 × 60 / 301 = 480 / 301 = 1.595 L/min   ✅ (0.3–2.0)
dist_max = 16 × 0.50 × 1.0 × 60 / 0.3 = 480 / 0.3  = 1 600 m
dist_min = 16 × 0.50 × 1.0 × 60 / 2.0 = 480 / 2.0  =   240 m
```

**Mission 301 m nằm trong (240 m, 1 600 m) tại 1.0 m/s → PASS.**

**GCS message khi ARM:**

```
SA FM1 OK: r=0.50 q1=1.59L/ph miss=301m dmax=1600m ~5m01s vi/run=8.0L
```

**Console log định kỳ khi đang chạy:**

```
[FLOW] FM1 r:0.50 q1:1.59L/ph miss:301m dmax:1600m spd:1.00m/s vi/run:8.0L
```

---

### TC-05c — q1 thay đổi theo vận tốc (mission cố định 301 m, r=0.50)

Công thức rút gọn: `q1 = (16 × 0.50 × 60 / 301) × v = 1.5947 × v`

| Vận tốc (m/s) | q1 (L/min) | Trong dải 0.3–2.0? | Ghi chú                                    |
| -------------- | ---------- | -------------------- | --------------------------------------------- |
| 0.18           | 0.287      | ❌ < 0.3           |                                               |
| **0.19**      | **0.303**  | ✅                 | Tốc độ tối thiểu nấc cao                    |
| 0.50           | 0.797      | ✅                 |                                               |
| **1.00**      | **1.595**  | ✅                 | **Tốc độ vận hành khuyến nghị (TC-05b)**    |
| 1.25           | 1.993      | ✅                 | Gần trần                                     |
| **1.26**      | **2.010**  | ❌ > 2.0           | Vượt ngưỡng — thấp hơn nhiều so với nấc giữa |
| 1.30           | 2.073      | ❌ > 2.0           | Chính là TC-05a (FAIL)                        |

---

### TC-05d — Lượng vi sinh mỗi lần chạy (hằng số, xem giải thích TC-04c)

```
vi_per_run = SA_TANK_VOL × r = 16 × 0.50 = 8.0 L    (không đổi theo speed, tại v=1.0 hay bất kỳ v hợp lệ nào)
```

---

### TC-05e — Các giá trị SA_DATA cần kiểm tra (tại v=1.0 m/s)

| Field SA_DATA | Index | Giá trị mong đợi       | Cách xem          |
| -------------- | ----- | ------------------------ | ------------------ |
| flow_rate EMA | [0]   | **1.59** L/min (±0.06) | MAVLink Inspector |
| flow_rate MA  | [1]   | **1.59** L/min (±0.10) | MAVLink Inspector |
| flow_target   | [2]   | **1.595** L/min        | MAVLink Inspector |
| pump_pwm      | [3]   | Ổn định, không dao động  | MAVLink Inspector |
| spray_mode    | [4]   | **2**                  | MAVLink Inspector |

---

### TC-05f — Thủ tục test thực tế

1. Cài đủ param ở trên, upload mission 301 m.
2. Gạt RC về **nấc cao** (spray_mode = 2).
3. ARM → đọc GCS message, xác nhận: `SA FM1 OK … r=0.50 q1=1.59L/ph … ~5m01s vi/run=8.0L`
   - Nếu vô tình chạy ở tốc độ ~1.3 m/s như nấc giữa, GCS sẽ báo FAIL (xem TC-05a) — đây là hành vi đúng, không phải lỗi.
4. Chạy xe theo mission, giữ vận tốc ~**1.0 m/s** (chậm hơn nấc giữa).
5. DISARM sau khi kết thúc mission.
6. Đo lại lượng vi sinh thực tế đã dùng — **phải nằm trong khoảng 7.5–8.5 L** (sai số ±6% quanh 8.0 L).

---

## So sánh nấc giữa (TC-04) và nấc cao (TC-05) — Tank ref. 16 L, Mission 301 m

| Chỉ số                          | Nấc giữa (r=0.35)     | Nấc cao (r=0.50)       | Ghi chú |
| --------------------------------- | ----------------------- | ------------------------- | ------- |
| Tốc độ vận hành khuyến nghị    | 1.3 m/s                 | 1.0 m/s                   | Nấc cao cần **chậm hơn** để giữ q1 ≤ 2.0 với cùng tank/mission |
| q1 tại tốc độ khuyến nghị      | 1.451 L/min             | 1.595 L/min               |         |
| vi_per_run                       | **5.6 L**               | **8.0 L**                 | = TANK_VOL × r, hằng số |
| Tốc độ tối thiểu (mission 301m) | 0.27 m/s                | 0.19 m/s                  |         |
| Tốc độ tối đa (mission 301m)    | 1.79 m/s                | 1.25 m/s                  | Nấc cao có dải tốc độ hẹp hơn nhiều |
| Chạy ở 1.3 m/s?                  | ✅ PASS (q1=1.451)      | ❌ FAIL (q1=2.073 > 2.0) | Xem TC-05a |

---

## TC-06 — FAIL CASE: Tank tham chiếu quá nhỏ so với mission (q1 < 0.3)

**Cài param:**

```
SA_FLOW_MODE = 1
SA_TANK_VOL  = 3
SA_FLOW_MIX_STD   = 0.35
```

Mission 301 m, tốc độ vận hành 1.3 m/s (giống TC-04):

```
q1       = 3 × 0.35 × 1.3 × 60 / 301 = 81.9 / 301 = 0.272 L/min   ❌ < 0.3
dist_max = 3 × 0.35 × 1.3 × 60 / 0.3 = 81.9 / 0.3  = 273 m
```

**Mission 301 m > dist_max 273 m tại 1.3 m/s → FAIL → bơm dừng!**

**GCS message khi ARM:**

```
SA FM1: q1=0.27L/ph < 0.3 @1.3m/s dist=301m - rút ngắn mission (dmax=273m)
```

**SA_DATA[2]** = 0.0 (flow_target=0), bơm không chạy.

**Cách fix:** Rút ngắn mission xuống ≤ 273 m, tăng tốc độ xe, hoặc tăng `SA_TANK_VOL` (thùng tham chiếu lớn hơn).

---

## TC-07 — FAIL CASE: Speed ≤ 0.1 m/s khi đang chạy

**Điều kiện:** Xe dừng giữa chừng (kẹt, chờ), FLOW_MODE=1, spray_mode=1.

**Kết quả firmware:**

- `_compute_visin_target()` trả về 0.0
- Pump bị force về `SERVO8_MIN` = 1000 µs (qua `ch->get_output_min()`)
- Bơm dừng ngay lập tức
- Khi xe chạy lại (speed > 0.1), bơm tự tiếp tục

**Không có GCS warning** khi speed ≤ 0.1 (chỉ warning khi ARM hoặc khi q1 ngoài dải 0.3–2.0).

---

## TC-08 — SA_FLOW_VEL: Calib FLOW_MODE=1 khi xe đứng yên

**Cài param:**

```
SA_TANK_VOL  = 16
SA_FLOW_MIX_STD   = 0.35
SA_FLOW_VEL  = 1.3   (ép vận tốc = 1.3 m/s, xe đứng yên)
SA_FLOW_MODE = 1
```

**q1_target giả lập (nấc giữa, mission 301m — giống hệt TC-04a vì cùng công thức):**

```
q1 = 16 × 0.35 × 1.3 × 60 / 301 = 1.451 L/min
```

Bơm chạy như xe đang đi 1.3 m/s dù đứng yên → dùng để calib PID không cần xe chạy.

**Sau calib:** Đặt lại `SA_FLOW_VEL = 0` trước khi chạy thực tế.

---

## TC-09 — Kiểm tra cảnh báo q1 ngoài dải (0.3–2.0 L/min), khi đang chạy (không phải lúc ARM)

Cấu hình giống TC-04 (`TANK_VOL=16`, `r=SA_FLOW_MIX_STD=0.35`, mission=301m). Cảnh báo runtime này **không kèm dmax/dmin** (khác với cảnh báo lúc ARM ở TC-05a/TC-06) — dmax/dmin chỉ được tính và in trong thông báo lúc ARM.

### TC-09a — q1 < 0.3 L/min (speed quá thấp)

Speed = 0.20 m/s:

```
q1 = 16 × 0.35 × 0.20 × 60 / 301 = 67.2 / 301 = 0.223 L/min   < 0.3
```

→ Bơm dừng, GCS warning (mỗi 5s trong lúc vẫn ở trạng thái này):

```
SA FM1: q1=0.22L/ph < 0.3 - rút ngắn mission hoặc tăng speed
```

### TC-09b — q1 > 2.0 L/min (speed quá cao)

Speed = 2.5 m/s:

```
q1 = 16 × 0.35 × 2.5 × 60 / 301 = 840 / 301 = 2.791 L/min   > 2.0
```

→ Bơm dừng, GCS warning (mỗi 5s):

```
SA FM1: q1=2.79L/ph > 2.0 - kéo dài mission hoặc giảm speed
```

**Cách fix:** Giảm tốc độ xe, hoặc kéo dài mission, hoặc giảm `SA_FLOW_MIX_STD`/tăng `SA_TANK_VOL` cho phù hợp.

---

## TC-10 — Mission cache: update mission mà không cần reboot

**Vấn đề cũ:** Upload mission mới nhưng `_mission_dist_m` không cập nhật.

**Cơ chế fix:** Cache reset khi disarm (`_mission_ncmds = 0`).

**Test:**
| Bước | Hành động | Kết quả mong đợi |
|---|---|---|
| 1 | Upload mission 301 m, ARM | GCS: `miss=301m` |
| 2 | DISARM | `_mission_ncmds = 0` reset |
| 3 | Upload mission mới 500 m | (chưa ARM, dist chưa tính) |
| 4 | ARM lại | GCS: `miss=500m` — đã tính lại |

---

## TC-11 — Calib SA_CAL_FAC (quy trình bucket test)

**Công thức:**

```
new_CAL_FAC = old_CAL_FAC × (GCS_reading / actual_flow)
actual_flow  = volume_L / time_min (đo bằng bình + đồng hồ)
```

**Ví dụ đã calib (lấy từ thực tế dự án):**

| Lần      | old_CAL_FAC | Thể tích thực                 | Thời gian | Lưu lượng thực | GCS đọc    | new_CAL_FAC                  |
| -------- | ------------ | ------------------------------ | ---------- | ---------------- | ----------- | ------------------------------ |
| 1        | 3254.5      | 3 L                           | 3m00s     | 1.000 L/min    | 0.66 L/min | 3254.5×(0.66/1.0) = **2148** |
| 2        | 2148        | 3 L                           | 2m50s     | 1.059 L/min    | 0.96 L/min | 2148×(0.96/1.059) = **1947** |
| 3        | 1947        | 3 L                           | 2m30s     | 1.200 L/min    | 1.21 L/min | 1947×(1.21/1.20) = **1963**  |
| ✅ Final | **1745**    | Kết quả hội tụ sau các lần đo |            |                   |             |                                 |

**Tiêu chí đạt:** `|GCS_reading − actual_flow| / actual_flow < 2%`

---

## Checklist trước khi chạy thực địa

- [ ] `SA_CAL_FAC = 1745` (đã calib)
- [ ] `SA_FLOW_MODE = 1`, `SA_TANK_VOL = 16`
- [ ] `SA_FLOW_MIX_STD = 0.35`, `SA_FLOW_MIX_CNT = 0.50`
- [ ] `SA_FLOW_VEL = 0` (dùng vận tốc thật)
- [ ] Upload mission, ARM → kiểm tra GCS message `SA FM1 OK`
- [ ] **Nấc giữa:** vận tốc mục tiêu ~1.3 m/s, đảm bảo `301m` nằm trong `(218m, 1456m)`
- [ ] **Nấc cao:** vận tốc mục tiêu ~1.0 m/s (chậm hơn nấc giữa), đảm bảo `301m` nằm trong `(240m, 1600m)`
- [ ] Sau chạy: đo thực tế lượng vi sinh tiêu thụ, so với `vi/run=x.xL` trong GCS message
