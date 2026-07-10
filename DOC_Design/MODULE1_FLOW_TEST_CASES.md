# Module 1 — Flow Sensor & Spray Controller

## Test Cases — Số đối ứng cụ thể

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Ngày viết:** 2026-07-08

---

## Thông số phần cứng dùng trong tất cả các test

| Thông số           | Giá trị       | Ghi chú                                          |
| ------------------ | ------------- | ------------------------------------------------ |
| Cảm biến lưu lượng | YF-S402B      | Dải 0.3–6 L/min                                  |
| SA_CAL_FAC         | **1745**      | Calibrated (pulses/L)                            |
| SA_APP_RATE        | **150** L/ha  | Suất phun                                        |
| SA_BOOM_W          | **2.0** m     | Sải phun                                         |
| SA_TANK_VOL        | **16** L      | Thùng vi sinh                                    |
| SA_MIX_STD         | **0.35**      | Nấc giữa — van vi sinh mức 2                     |
| SA_MIX_CNT         | **0.50**      | Nấc cao — van vi sinh mức 2 (same), chỉnh van hồ |
| SA_FLOW_SP         | **1.2** L/min | Max vi sinh vật lý tại van vi sinh mức 2         |
| SA_PID_P           | **80**        | µs per L/min error                               |
| SA_PID_I           | **20**        | µs per L/min/s                                   |
| SA_PID_LPF         | **0.3**       | Output smoothing                                 |
| SERVO8_MIN         | **1000**      | pwm_min bơm                                      |
| SERVO8_TRIM        | **1500**      | pwm_trim bơm                                     |
| SERVO8_MAX         | **2000**      | pwm_max bơm                                      |
| Vận tốc thực tế    | **1.3** m/s   | Tốc độ xe phun                                   |
| Mission distance   | **301** m     | Tuyến đường đã upload                            |

---

## TC-01 — Spray mode 0 (PASSTHROUGH)

**Điều kiện:** RC nấc dưới → `_spray_mode = 0`

| Bước | Hành động                           | Kết quả mong đợi                                |
| ---- | ----------------------------------- | ----------------------------------------------- |
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
| ------------------------ | ------------------------------ | --------------------- |
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
SA_MIX_STD   = 0.35
SA_MIX_CNT   = 0.50
```

**Flow target tự động tính:**

```
flow_target = SA_FLOW_SP × (MIX_CNT / MIX_STD)
            = 1.2 × (0.50 / 0.35)
            = 1.2 × 1.4286
            = 1.714 L/min
```

> **Lưu ý thực tế:** Van vi sinh giữ mức 2 (same mode 1), van hồ mở thêm.
> Cảm biến đo vi sinh max ~1.2 L/min → bơm sẽ cố đạt 1.714 nhưng bị giới
> hạn vật lý → pump_pwm sẽ ở mức cao (gần 2000 µs).

| Đo                       | Giá trị         | Ghi chú                       |
| ------------------------ | --------------- | ----------------------------- |
| SA_DATA[2] (flow_target) | **1.714** L/min | Tự tính từ ratio              |
| SA_DATA[0] (flow actual) | **~1.2** L/min  | Giới hạn vật lý van vi sinh   |
| SA_DATA[3] (pump_pwm)    | **~2000** µs    | Pump max vì target > khả năng |

---

## TC-04 — FLOW_MODE=1, Spray mode 1 (nấc giữa, r = 0.35)

**Param cài trước test:**

```
SA_FLOW_MODE = 1
SA_TANK_VOL  = 16
SA_MIX_STD   = 0.35
SA_APP_RATE  = 150
SA_BOOM_W    = 2.0
SA_FLOW_VEL  = 0       (dùng vận tốc thật)
```

---

### TC-04a — Kiểm tra dist_max (trước khi ARM)

```
dist_max = TANK_VOL × 10000 / (MIX_STD × APP_RATE × BOOM_W)
         = 16 × 10000 / (0.35 × 150 × 2.0)
         = 160 000 / 105
         = 1 523.8 m
```

**Mission 301 m < 1 523.8 m → PASS — bơm cho phép chạy**

> Thùng 16 L đủ cho tuyến đường tối đa **1 524 m** ở vận tốc bất kỳ.
> Mission 301 m chỉ tiêu thụ ~21% dung tích thùng.

---

### TC-04b — q1_target thay đổi theo vận tốc

Công thức: `q1 = 0.35 × 150 × v × 2.0 × 0.006 = 0.630 × v`

| Vận tốc (m/s) | q1 (L/min) | Trong dải 0.3–2.0? | Ghi chú                              |
| ------------- | ---------- | ------------------ | ------------------------------------ |
| 0.47          | 0.296      | ❌ < 0.3           | Xe đi quá chậm → bơm dừng + warning  |
| **0.48**      | **0.302**  | ✅                 | Tốc độ tối thiểu để bơm hoạt động    |
| 0.80          | 0.504      | ✅                 |                                      |
| 1.00          | 0.630      | ✅                 |                                      |
| **1.30**      | **0.819**  | ✅                 | **Tốc độ vận hành thực tế**          |
| 1.50          | 0.945      | ✅                 |                                      |
| 2.00          | 1.260      | ✅                 |                                      |
| 3.17          | 1.997      | ✅                 | Tốc độ tối đa trước khi vượt ngưỡng  |
| **3.18**      | **2.003**  | ❌ > 2.0           | Xe đi quá nhanh → bơm dừng + warning |

**Tại v = 1.3 m/s (vận hành thực tế):**

```
q1 = 0.35 × 150 × 1.3 × 2.0 × 0.006 = 0.819 L/min
```

---

### TC-04c — Thời gian và lượng vi sinh cho 1 lần chạy 301 m

```
eta_s   = 301 / 1.3 = 231.5 s
eta_min = 231 / 60  = 3 phút
eta_sec = 231 % 60  = 51 giây

vi_used = q1 × (eta_s / 60)
        = 0.819 × (231.5 / 60)
        = 0.819 × 3.858
        = 3.16 L
```

**GCS message khi ARM (tất cả điều kiện OK):**

```
SA FM1 OK: r=0.35 q1=0.82L/ph miss=301m dmax=1524m ~3m51s vi~3.2L
```

---

### TC-04d — Bao nhiêu lần chạy với thùng 16 L?

```
Số lần chạy = TANK_VOL / vi_used = 16 / 3.16 = 5.06 lần
```

| Lần chạy  | Vi sinh tiêu thụ (L) | Còn lại trong thùng (L)     |
| --------- | -------------------- | --------------------------- |
| Bắt đầu   | —                    | **16.00**                   |
| Sau lần 1 | 3.16                 | **12.84**                   |
| Sau lần 2 | 3.16                 | **9.68**                    |
| Sau lần 3 | 3.16                 | **6.52**                    |
| Sau lần 4 | 3.16                 | **3.36**                    |
| Sau lần 5 | 3.16                 | **0.20** ← cạn, cần đổ thêm |

→ **5 lần chạy** đầy đủ 301 m với thùng 16 L. Lần thứ 6 cần đổ thêm vi sinh.

---

### TC-04e — Các giá trị SA_DATA cần kiểm tra

| Field SA_DATA | Index | Giá trị mong đợi       | Cách xem          |
| ------------- | ----- | ---------------------- | ----------------- |
| flow_rate EMA | [0]   | **0.82** L/min (±0.05) | MAVLink Inspector |
| flow_rate MA  | [1]   | **0.82** L/min (±0.08) | MAVLink Inspector |
| flow_target   | [2]   | **0.819** L/min        | MAVLink Inspector |
| pump_pwm      | [3]   | Ổn định ~1550–1700 µs  | MAVLink Inspector |
| spray_mode    | [4]   | **1**                  | MAVLink Inspector |

---

### TC-04f — Thủ tục test thực tế

1. Đổ đúng **16 L** vi sinh vào thùng, đánh dấu mực.
2. Cài đủ param ở trên, upload mission 301 m.
3. ARM → đọc GCS message, xác nhận: `SA FM1 OK … ~3m51s vi~3.2L`
4. Chạy xe theo mission, giữ vận tốc ~1.3 m/s.
5. DISARM sau khi kết thúc mission.
6. Đo lại mực vi sinh trong thùng.
7. **Lượng đã dùng phải nằm trong khoảng 3.0–3.4 L** (sai số ±6%).

---

## TC-05 — FLOW_MODE=1, Spray mode 2 (nấc cao/chống nghẹt, r = 0.50)

**Param cài trước test:**

```
SA_FLOW_MODE = 1
SA_TANK_VOL  = 16
SA_MIX_CNT   = 0.50
SA_APP_RATE  = 150
SA_BOOM_W    = 2.0
SA_FLOW_VEL  = 0
```

---

### TC-05a — Kiểm tra dist_max

```
dist_max = 16 × 10000 / (0.50 × 150 × 2.0)
         = 160 000 / 150
         = 1 066.7 m
```

**Mission 301 m < 1 066.7 m → PASS**

> Thùng 16 L đủ cho tuyến tối đa **1 067 m** ở nấc cao.
> Mission 301 m tiêu thụ ~28% dung tích.

---

### TC-05b — q1_target thay đổi theo vận tốc

Công thức: `q1 = 0.50 × 150 × v × 2.0 × 0.006 = 0.900 × v`

| Vận tốc (m/s) | q1 (L/min) | Trong dải 0.3–2.0? | Ghi chú                             |
| ------------- | ---------- | ------------------ | ----------------------------------- |
| 0.33          | 0.297      | ❌ < 0.3           | Quá chậm → bơm dừng + warning       |
| **0.34**      | **0.306**  | ✅                 | Tốc độ tối thiểu nấc cao            |
| 0.80          | 0.720      | ✅                 |                                     |
| 1.00          | 0.900      | ✅                 |                                     |
| **1.30**      | **1.170**  | ✅                 | **Tốc độ vận hành thực tế**         |
| 1.50          | 1.350      | ✅                 |                                     |
| 2.00          | 1.800      | ✅                 |                                     |
| 2.22          | 1.998      | ✅                 | Tốc độ tối đa trước khi vượt ngưỡng |
| **2.23**      | **2.007**  | ❌ > 2.0           | Quá nhanh → bơm dừng + warning      |

**Tại v = 1.3 m/s:**

```
q1 = 0.50 × 150 × 1.3 × 2.0 × 0.006 = 1.170 L/min
```

---

### TC-05c — Thời gian và lượng vi sinh cho 1 lần chạy 301 m

```
eta_s   = 301 / 1.3 = 231.5 s → 3 phút 51 giây

vi_used = 1.170 × (231.5 / 60)
        = 1.170 × 3.858
        = 4.51 L
```

**GCS message khi ARM:**

```
SA FM1 OK: r=0.50 q1=1.17L/ph miss=301m dmax=1067m ~3m51s vi~4.5L
```

**GCS message khi ARM:**

```
SA FM1 OK: r=0.50 q1=1.17L/ph miss=301m dmax=1067m ~3m51s vi~4.5L
```

---

### TC-05d — Bao nhiêu lần chạy với thùng 16 L?

```
Số lần chạy = 16 / 4.51 = 3.55 lần
```

| Lần chạy  | Vi sinh tiêu thụ (L) | Còn lại trong thùng (L)       |
| --------- | -------------------- | ----------------------------- |
| Bắt đầu   | —                    | **16.00**                     |
| Sau lần 1 | 4.51                 | **11.49**                     |
| Sau lần 2 | 4.51                 | **6.98**                      |
| Sau lần 3 | 4.51                 | **2.47** ← không đủ cho lần 4 |

→ **3 lần chạy** đầy đủ 301 m. Sau lần 3 còn **2.47 L** trong thùng — cần đổ thêm.

---

### TC-05e — Các giá trị SA_DATA cần kiểm tra

| Field SA_DATA | Index | Giá trị mong đợi       | Cách xem          |
| ------------- | ----- | ---------------------- | ----------------- |
| flow_rate EMA | [0]   | **1.17** L/min (±0.06) | MAVLink Inspector |
| flow_rate MA  | [1]   | **1.17** L/min (±0.10) | MAVLink Inspector |
| flow_target   | [2]   | **1.170** L/min        | MAVLink Inspector |
| pump_pwm      | [3]   | Ổn định ~1600–1800 µs  | MAVLink Inspector |
| spray_mode    | [4]   | **2**                  | MAVLink Inspector |

---

### TC-05f — Thủ tục test thực tế

1. Đổ đúng **16 L** vi sinh vào thùng, đánh dấu mực.
2. Cài đủ param ở trên, upload mission 301 m.
3. ARM → đọc GCS message, xác nhận: `SA FM1 OK … ~3m51s vi~4.5L`
4. Gạt RC về **nấc cao** (spray_mode = 2).
5. Chạy xe theo mission, giữ vận tốc ~1.3 m/s.
6. DISARM sau khi kết thúc mission.
7. Đo lại mực vi sinh trong thùng.
8. **Lượng đã dùng phải nằm trong khoảng 4.3–4.8 L** (sai số ±6%).

---

## So sánh nấc giữa (TC-04) và nấc cao (TC-05) — Thùng 16 L, Mission 301 m

| Chỉ số              | Nấc giữa (r=0.35)     | Nấc cao (r=0.50)      | Chênh lệch |
| ------------------- | --------------------- | --------------------- | ---------- |
| dist_max            | 1 523.8 m             | 1 066.7 m             | −30%       |
| q1 tại 1.3 m/s      | 0.819 L/min           | 1.170 L/min           | +43%       |
| Thời gian 1 lần     | 3 phút 51 giây        | 3 phút 51 giây        | Bằng nhau  |
| Vi sinh 1 lần 301 m | **3.16 L**            | **4.51 L**            | +43%       |
| Số lần chạy 301 m   | **5 lần** (dư 0.20 L) | **3 lần** (dư 2.47 L) |            |
| Tốc độ tối thiểu    | 0.48 m/s              | 0.34 m/s              |            |
| Tốc độ tối đa       | 3.17 m/s              | 2.22 m/s              |            |

---

## TC-06 — FAIL CASE: Tank 3L, mission 301 m (dist > dist_max)

**Cài param:**

```
SA_TANK_VOL = 3
SA_MIX_STD  = 0.35
```

```
dist_max = 3 × 10000 / (0.35 × 150 × 2.0)
         = 30000 / 105
         = 285.7 m
```

**Mission 301 m > 285.7 m → FAIL → bơm dừng!**

**GCS message khi ARM:**

```
SA FM1: mission 301m > dmax 286m - vẽ lại mission ngắn hơn
```

**SA_DATA[2]:** = 0.0 (pump bị force về min), bơm không chạy.

**Cách fix:** Rút ngắn mission xuống ≤ 285 m hoặc tăng TANK_VOL.

---

## TC-07 — FAIL CASE: Speed ≤ 0.1 m/s khi đang chạy

**Điều kiện:** Xe dừng giữa chừng (kẹt, chờ), FLOW_MODE=1, spray_mode=1.

**Kết quả firmware:**

- `_compute_visin_target()` trả về 0.0
- Pump bị force về `SERVO8_MIN` = 1000 µs
- Bơm dừng ngay lập tức
- Khi xe chạy lại (speed > 0.1), bơm tự tiếp tục

**Không có GCS warning** khi speed ≤ 0.1 (chỉ warning khi ARM).

---

## TC-08 — SA_FLOW_VEL: Calib FLOW_MODE=1 khi xe đứng yên

**Cài param:**

```
SA_FLOW_VEL = 1.3   (ép vận tốc = 1.3 m/s, xe đứng yên)
SA_FLOW_MODE = 1
```

**q1_target giả lập (nấc giữa):**

```
q1 = 0.35 × 150 × 1.3 × 2.0 × 0.006 = 0.819 L/min
```

Bơm chạy như xe đang đi 1.3 m/s dù đứng yên → dùng để calib PID không cần xe chạy.

**Sau calib:** Đặt lại `SA_FLOW_VEL = 0` trước khi chạy thực tế.

---

## TC-09 — Kiểm tra cảnh báo q1 ngoài dải (0.3–2.0 L/min)

### TC-09a — q1 < 0.3 L/min (speed quá thấp)

Speed = 0.2 m/s:

```
q1 = 0.35 × 150 × 0.2 × 2.0 × 0.006 = 0.126 L/min < 0.3
```

→ Bơm dừng, GCS warning:

```
SA FM1: Q visin 0.13L/min < 0.3 - tang mission_dist hoac giam speed
```

### TC-09b — q1 > 2.0 L/min (speed quá cao)

Speed = 3.5 m/s, MIX_STD = 0.35:

```
q1 = 0.35 × 150 × 3.5 × 2.0 × 0.006 = 2.205 L/min > 2.0
```

→ Bơm dừng, GCS warning:

```
SA FM1: Q visin 2.21L/min > 6.0 - giam mission_dist hoac tang speed
```

_(Lưu ý: warning text trong code dùng "> 6.0" nhưng threshold thực tế là 2.0)_

**Cách fix:** Giảm APP_RATE hoặc SA_MIX_STD, hoặc tăng BOOM_W.

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
| -------- | ----------- | ----------------------------- | --------- | -------------- | ---------- | ---------------------------- |
| 1        | 3254.5      | 3 L                           | 3m00s     | 1.000 L/min    | 0.66 L/min | 3254.5×(0.66/1.0) = **2148** |
| 2        | 2148        | 3 L                           | 2m50s     | 1.059 L/min    | 0.96 L/min | 2148×(0.96/1.059) = **1947** |
| 3        | 1947        | 3 L                           | 2m30s     | 1.200 L/min    | 1.21 L/min | 1947×(1.21/1.20) = **1963**  |
| ✅ Final | **1745**    | Kết quả hội tụ sau các lần đo |           |                |            |                              |

**Tiêu chí đạt:** `|GCS_reading − actual_flow| / actual_flow < 2%`

---

## Checklist trước khi chạy thực địa

- [ ] `SA_CAL_FAC = 1745` (đã calib)
- [ ] `SA_FLOW_MODE = 1`, `SA_TANK_VOL = 16`
- [ ] `SA_APP_RATE = 150`, `SA_BOOM_W = 2.0`
- [ ] `SA_MIX_STD = 0.35`, `SA_MIX_CNT = 0.50`
- [ ] `SA_FLOW_VEL = 0` (dùng vận tốc thật)
- [ ] Upload mission, ARM → kiểm tra GCS message `SA FM1 OK`
- [ ] Đảm bảo `dist < dist_max` (301 m < 1524 m với tank 16L)
- [ ] Sau chạy: đo thực tế lượng vi sinh tiêu thụ, so với `vi~x.xL` trong GCS message
