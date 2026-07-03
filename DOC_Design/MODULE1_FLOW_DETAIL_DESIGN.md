# Module 1 — Flow Sensor & Spray Controller
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 10 Hz (`update()` gọi từ ArduPilot scheduler)
**Ngày hoàn thành:** 2026-05-15

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
irq_handler() [ISR — mỗi RISING edge trên GPIO SA_FLOW_PIN]
    └──► _pulse_count++  (volatile uint32_t, atomic on ARM)

update() [10 Hz — ArduPilot scheduler]
    │
    ├──► _check_pump_config()
    │         └──► return void  (warn/ok mỗi 5s nếu config sai)
    │
    ├──► (SA_SIM=1) _run_simulation()    [cũng cập nhật data cho Module 2 & 3]
    │    (SA_SIM=0) _ph_update()         [Module 2 — xem MODULE2_PH_DETAIL_DESIGN]
    │
    ├──► _update_dosing_motor()          [Module 3 — xem MODULE3_DOS_DETAIL_DESIGN]
    │
    ├──► [Flow rate calc — inline trong update()]
    │         đọc _pulse_count, tính EMA → _flow_rate_filtered
    │         cập nhật moving-average buffer → _flow_rate_avg
    │
    ├──► _update_spray_mode()
    │         └──► cập nhật _spray_mode (0/1/2)
    │
    └──► switch(_spray_mode)
              mode 0 ──► _write_pump_pwm(RC_passthrough)
              mode 1 ──► [FLOW_MODE=0] flow_target = SA_FLOW_SP
              (nấc giữa)  [FLOW_MODE=1] _compute_visin_target(SA_MIX_STD)
                              └──► _get_mission_dist()
                                   kiểm tra mission / dist_max / speed / sensor range
                          _run_flow_pid(flow_target, dt)
                          └──► return pwm  ──► _write_pump_pwm(pwm)
              mode 2 ──► [FLOW_MODE=0] flow_target = SA_FLOW_SP × (MIX_CNT/MIX_STD)
              (nấc cao)  [FLOW_MODE=1] _compute_visin_target(SA_MIX_CNT)
                              └──► _get_mission_dist()
                                   kiểm tra mission / dist_max / speed / sensor range
                          _run_flow_pid(flow_target, dt)
                          └──► return pwm  ──► _write_pump_pwm(pwm)
```

### 1.2 Mô tả từng hàm

---

**`irq_handler()`** — static
- **File:** `AP_ShoesAgtech.cpp : 400`
- **Được gọi bởi:** HAL GPIO interrupt (RISING edge trên SA_FLOW_PIN)
- **Đầu vào:** không có (no-arg ISR)
- **Xử lý:**
  1. Tăng `_pulse_count` lên 1
- **Đầu ra / Return:** `void` — side effect: `_pulse_count++`
- **Ghi chú:** `_pulse_count` là `volatile uint32_t` static; ARM cortex-M bảo đảm tăng 32-bit atomic. Không dùng mutex/critical section.

---

**`init()`**
- **File:** `AP_ShoesAgtech.cpp : 371`
- **Được gọi bởi:** ArduPilot scheduler 1 lần khi boot
- **Đầu vào:** không có
- **Xử lý:**
  1. Kiểm tra `is_enabled()` — nếu SA_ENABLE=0, return ngay
  2. Đọc `SA_FLOW_PIN`, gọi `hal.gpio->pinMode()` (INPUT) rồi `attach_interrupt(irq_handler, RISING)`
  3. Reset các biến flow (buffer, integral, avg)
  4. Gọi `_ph_init()` nếu SA_PH_EN=1
- **Đầu ra / Return:** `void`
  - Thành công: STATUSTEXT INFO "ShoesAgtech: Flow sensor ready"
  - Thất bại: STATUSTEXT CRITICAL "ShoesAgtech: IRQ attach failed"
- **Ghi chú:** `SA_FLOW_PIN` chỉ đọc một lần tại đây — thay đổi sau này cần reboot.

---

**`update()`**
- **File:** `AP_ShoesAgtech.cpp : 406`
- **Được gọi bởi:** ArduPilot scheduler @ 10 Hz
- **Đầu vào:** không có
- **Xử lý:**
  1. Nếu `!is_enabled()` → zero flow, return
  2. Gọi `_check_pump_config()`
  3. Nếu SA_SIM=1 → `_run_simulation()`, ngược lại → `_ph_update()`
  4. Gọi `_update_dosing_motor()`
  5. Tính flow rate (nếu Δt ≥ 100ms): đọc `_pulse_count`, tính raw L/min → EMA → MA
  6. Tính dt PID (clamp 0 < dt ≤ 1s)
  7. Gọi `_update_spray_mode()`
  8. Switch theo `_spray_mode`: mode 1 kiểm tra `SA_SPRAY_MODE` trước khi chọn setpoint
  9. Console log nếu SA_FLOW_LOG=1
- **Đầu ra / Return:** `void` — side effects: `_flow_rate_filtered`, `_flow_rate_avg`, `_pump_pwm`, `_flow_target`

---

**`_check_pump_config()`**
- **File:** `AP_ShoesAgtech.cpp : 615`
- **Được gọi bởi:** `update()` mỗi chu kỳ
- **Đầu vào:** không có (đọc `_pump_chan` từ param)
- **Xử lý:**
  1. Đọc `SERVOx_FUNCTION` của kênh SA_PUMP_CHAN
  2. So sánh với lần trước; nếu không thay đổi và đã OK → return sớm
  3. Nếu function ≠ 0 → STATUSTEXT WARNING mỗi 5s; `_pump_config_ok = false`
  4. Nếu function = 0 → in MIN/TRIM/MAX một lần; `_pump_config_ok = true`
- **Đầu ra / Return:** `void` — cập nhật `_pump_config_ok`
- **Ghi chú:** Không block motor; chỉ cảnh báo. Pump vẫn xuất PWM ngay cả khi config sai — khác với dosing motor (vì flow module không có safety check cứng như dosing).

---

**`_update_spray_mode()`**
- **File:** `AP_ShoesAgtech.cpp : 658`
- **Được gọi bởi:** `update()`
- **Đầu vào:** không có (đọc `_rc_chan` từ param)
- **Xử lý:**
  1. Đọc PWM kênh `SA_RC_CHAN - 1` (0-indexed)
  2. Nếu PWM=0 (mất tín hiệu) → giữ nguyên mode cũ, return
  3. PWM < 1300 → mode 0 (passthrough); 1300–1699 → mode 1 (nấc giữa, MIX_STD); ≥ 1700 → mode 2 (nấc cao, MIX_CNT)
- **Đầu ra / Return:** `void` — cập nhật `_spray_mode`

---

**`_compute_visin_target(float r) → float`**
- **Được gọi bởi:** `update()` (mode 1 và mode 2 khi `FLOW_MODE=1 && TANK_VOL>0`)
- **Đầu vào:** `r` — tỉ lệ vi sinh trong tổng lưu lượng (0.01–1.0)
- **Xử lý (theo thứ tự ưu tiên):**
  1. `dist = _get_mission_dist()` — nếu ≤ 1m → warning + return 0
  2. `dist_max = SA_TANK_VOL × 10000 / (r × SA_APP_RATE × SA_BOOM_W)` — nếu `dist > dist_max` → warning + return 0
  3. `speed < 0.1 m/s` → reset PI, return 0 (không cảnh báo)
  4. `q1 = r × SA_APP_RATE × speed × SA_BOOM_W × 0.006` (L/min)
  5. `q1 < 0.3` hoặc `q1 > 6.0` (dãy YF-S402B) → warning + return 0
  6. Trả về `q1`
- **Đầu ra / Return:** `float` — lưu lượng vi sinh target (L/min); 0 nếu bất kỳ điều kiện nào không đạt
- **Ghi chú:** Warnings dùng chung timer `_tank_warn_ms`, throttle 5s giữa các lần in.

---

**`_run_flow_pid(float target_lmin, float dt) → uint16_t`**
- **File:** `AP_ShoesAgtech.cpp : 680`
- **Được gọi bởi:** `update()` (mode 1 và mode 2)
- **Đầu vào:**
  - `target_lmin` — setpoint lưu lượng (L/min)
  - `dt` — thời gian từ chu kỳ trước (giây)
- **Xử lý:**
  1. Đọc MIN/TRIM/MAX từ SRV_Channel của SA_PUMP_CHAN
  2. `error = target_lmin − _flow_rate_filtered`
  3. Tích phân: `_pid_integral += error × dt`; anti-windup clamp `±(range/2 / I_gain)`
  4. `pid_raw = P × error + I × _pid_integral`
  5. LPF: `_pid_output_lpf = (1−α)×prev + α×pid_raw`
  6. `pwm = constrain(TRIM + lpf_output, MIN, MAX)`
- **Đầu ra / Return:** `uint16_t pwm` — giá trị µs xuất ra bơm
- **Ghi chú:** Base PWM lấy từ `SERVOx_TRIM` (không hardcode 1500). Anti-windup chỉ hoạt động khi I_gain > 0.

---

**`_write_pump_pwm(uint16_t pwm)`**
- **File:** `AP_ShoesAgtech.cpp : 715`
- **Được gọi bởi:** `update()` (cả 3 mode)
- **Đầu vào:**
  - `pwm` — giá trị µs (800–2200)
- **Xử lý:**
  1. Tính `chan_idx = SA_PUMP_CHAN − 1` (0-indexed)
  2. Gọi `SRV_Channels::set_output_pwm_chan(chan_idx, pwm)`
- **Đầu ra / Return:** `void` — side effect: xuất PWM ra servo channel
- **Ghi chú:** `set_output_pwm_chan()` set `have_pwm_mask` để `calc_pwm()` không ghi đè giá trị này trong cùng loop.

---

**`_get_mission_dist() → float`**
- **File:** `AP_ShoesAgtech.cpp : 1293`
- **Được gọi bởi:** `update()` (mode 1 khi SA_FLOW_MODE=1), `_update_dosing_motor()` (DOS_MODE=1)
- **Đầu vào:** không có
- **Xử lý:**
  1. Lấy `n = AP::mission()->num_commands()`
  2. Nếu n < 2 → return 0
  3. Nếu n == `_mission_ncmds` và cache > 0 → return cache (tránh lặp mỗi 10 Hz)
  4. Duyệt tất cả command, chỉ tính các lệnh NAV (WAYPOINT, LOITER, SPLINE); bỏ qua lat/lng = 0
  5. Cộng dồn `prev_loc.get_distance(loc)` cho từng đoạn
  6. Lưu vào `_mission_dist_m` + `_mission_ncmds`
- **Đầu ra / Return:** `float` — tổng khoảng cách mission (m); 0 nếu không có mission
- **Ghi chú:** Cache vô hiệu khi `num_commands()` thay đổi (mission upload lại). Dùng chung cho cả Module 1 và Module 3.

---

**`_run_simulation()`**
- **File:** `AP_ShoesAgtech.cpp : 1251`
- **Được gọi bởi:** `update()` khi SA_SIM=1
- **Đầu vào:** không có
- **Xử lý:**
  1. `t = millis() / 1000.0` (giây từ boot)
  2. Module 1: `_flow_rate_filtered = 2.5 + 1.5×sin(2π×t/20)` (L/min, chu kỳ 20s)
  3. Module 1: `_sim_speed = constrain(1.0 + 0.8×sin(2π×t/30), 0.1, 2.0)` (m/s, chu kỳ 30s)
  4. Module 2: `ph_sim`, `temp_sim`, `mv_sim`, `dkh_sim` theo sin với các chu kỳ khác nhau
  5. Cập nhật `_ph_last_good_ms = now` để `ph_has_data()=true`
- **Đầu ra / Return:** `void` — ghi trực tiếp vào `_flow_rate_filtered`, `_sim_speed`, `_ph_value`, v.v.
- **Ghi chú:** Moving-average buffer cho flow được cập nhật bởi `update()` ngay sau khi `_run_simulation()` trả về (không trong hàm này).

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_ENABLE` | 1 | Int8 | 1 | 0 | 1 | Bật/tắt toàn bộ library. Tắt → không có task nào chạy. |
| `SA_CAL_FAC` | 2 | Float | 3874.5 | 100 | 10000 | Hệ số cảm biến YF-S402B (pulses/Litre). Đo thực nghiệm với bình chia độ. |
| `SA_EMA_AL` | 3 | Float | 0.1 | 0.01 | 1.0 | Alpha EMA lưu lượng tức thời. Nhỏ = mịn hơn, phản hồi chậm hơn. |
| `SA_FLOW_LOG` | 4 | Int8 | 0 | 0 | 1 | Bật (1) console log lưu lượng theo chu kỳ `SA_LOG_FL_MS`. |
| `SA_RC_CHAN` | 5 | Int8 | 6 | 1 | 16 | Kênh RC (1-indexed) chọn chế độ phun. |
| `SA_RC_PUMP` | 6 | Int8 | 9 | 1 | 16 | Kênh RC passthrough (mode 0). |
| `SA_PUMP_CHAN` | 7 | Int8 | 8 | 1 | 16 | Kênh servo đầu ra bơm. Bắt buộc `SERVOx_FUNCTION=0`. |
| `SA_FLOW_SP` | 8 | Float | 5.0 | 0 | 200 | Setpoint mode 1 khi `SA_FLOW_MODE=0` (L/min). |
| `SA_PID_P` | 9 | Float | 80.0 | 0 | 500 | Hệ số P: µs PWM / (L/min sai số). |
| `SA_PID_I` | 10 | Float | 20.0 | 0 | 200 | Hệ số I: µs PWM / (L/min·s). |
| `SA_PID_LPF` | 11 | Float | 0.3 | 0.01 | 1.0 | Alpha LPF đầu ra PID (1.0 = không lọc). |
| `SA_APP_RATE` | 12 | Float | 100.0 | 0 | 2000 | Tỉ lệ phun L/ha (mode 2). |
| `SA_BOOM_W` | 13 | Float | 1.0 | 0 | 30 | Chiều rộng boom phun (m), dùng mode 2. |
| `SA_LOG_FL_MS` | 22 | Int16 | 1000 | 100 | 60000 | Chu kỳ console log lưu lượng (ms). |
| `SA_SIM` | 32 | Int8 | 0 | 0 | 1 | Chế độ giả lập: 1=inject dữ liệu sin thay cảm biến thật. |
| `SA_FLOW_PIN` | 33 | Int16 | 55 | 1 | 200 | Chân GPIO cảm biến. **Chỉ đọc khi init() — cần reboot khi thay đổi.** |
| `SA_TANK_VOL` | 34 | Float | 0.0 | 0 | 2000 | Dung tích tank vi sinh (L). `=0` tắt FLOW_MODE=1. Dùng tính `dist_max`. |
| `SA_FLOW_MODE` | 35 | Int8 | 0 | 0 | 1 | Nguồn setpoint: **0**=`SA_FLOW_SP` trực tiếp, **1**=công thức L/ha × vi sinh ratio. |
| `SA_MIX_STD` | 37 | Float | 0.35 | 0.01 | 1.0 | Tỉ lệ vi sinh nấc giữa (Mặc định van). FLOW_MODE=1: `q1 = MIX_STD × APP_RATE × speed × BOOM × 0.006`. FLOW_MODE=0: setpoint = SA_FLOW_SP. |
| `SA_MIX_CNT` | 38 | Float | 0.50 | 0.01 | 1.0 | Tỉ lệ vi sinh nấc cao (Chống nghẹt van, mở van nhiều hơn). FLOW_MODE=1: dùng MIX_CNT thay MIX_STD. FLOW_MODE=0: `flow_target = SA_FLOW_SP × (MIX_CNT/MIX_STD)`. |

> **Param chỉ có hiệu lực sau reboot:** `SA_FLOW_PIN`

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

- Đọc `SA_FLOW_PIN`, gọi `hal.gpio->attach_interrupt(irq_handler, RISING)` — chỉ một lần
- Reset toàn bộ buffer EMA/MA, PID integral về 0
- Gọi `_ph_init()` cho Module 2

### 3.2 Luồng xử lý chính

**ISR đếm xung:**
```
Mỗi RISING edge trên SA_FLOW_PIN:
    _pulse_count++  (không dùng mutex — ARM 32-bit write atomic)
```

**Tính lưu lượng tức thời (mỗi 100ms):**
```
Δpulses = _pulse_count − _last_pulse_snapshot    (xử lý wrap-around uint32)
Δt      = delta_t_ms × 0.001   (giây)
raw_lpm = (Δpulses / SA_CAL_FAC) / Δt × 60      (L/min)
_flow_rate_filtered = (1−α) × prev + α × raw_lpm (EMA, α=SA_EMA_AL)
→ nếu < 0.01 L/min: ép = 0.0
```

**Moving average (10 mẫu vòng):**
```
Circular buffer 10 phần tử
_flow_rate_avg = sum / count  (count tăng dần đến 10)
→ nếu < 0.01: ép = 0.0
```

**Chọn mode từ RC:**
```
rc_pwm = RC_Channels::get_radio_in(SA_RC_CHAN − 1)
rc_pwm < 1300        →  mode 0 (PASSTHROUGH — nấc thấp)
1300 ≤ rc_pwm < 1700 →  mode 1 (FLOW PID, tỉ lệ MIX_STD — nấc giữa / Mặc định van)
rc_pwm ≥ 1700        →  mode 2 (FLOW PID, tỉ lệ MIX_CNT — nấc cao / Chống nghẹt van)
rc_pwm = 0 (mất tín hiệu) →  giữ mode cũ
```

**Tính flow_target (mode 1 — nấc giữa):**
```
SA_FLOW_MODE=0 hoặc SA_TANK_VOL=0:
    flow_target = SA_FLOW_SP  (setpoint trực tiếp)

SA_FLOW_MODE=1 và SA_TANK_VOL>0:
    flow_target = _compute_visin_target(SA_MIX_STD)
    (xem luồng _compute_visin_target bên dưới)
```

**Tính flow_target (mode 2 — nấc cao):**
```
SA_FLOW_MODE=0 hoặc SA_TANK_VOL=0:
    ratio = SA_MIX_CNT / SA_MIX_STD   (nếu MIX_STD ≤ 0.01 → ratio = 1.0)
    flow_target = constrain(SA_FLOW_SP × ratio, 0, 200)
    Ý nghĩa: giữ nguyên tổng lưu lượng ra boom, tăng phần vi sinh theo tỉ lệ van
    Tank monitor: ước tính khoảng cách còn bơm được, in mỗi 30s nếu speed > 0

SA_FLOW_MODE=1 và SA_TANK_VOL>0:
    flow_target = _compute_visin_target(SA_MIX_CNT)
    (xem luồng _compute_visin_target bên dưới)
```

**Luồng `_compute_visin_target(r)`:**
```
Đầu vào: r = tỉ lệ vi sinh (MIX_STD hoặc MIX_CNT, clamp 0.01–1.0)

1. dist = _get_mission_dist()
   dist ≤ 1m → reset PI, cảnh báo "chua co mission", return 0

2. denom = r × SA_APP_RATE × SA_BOOM_W
   denom < 0.001 → return 0 (params chưa cài)
   dist_max = SA_TANK_VOL × 10000 / denom  (m)
   dist > dist_max → reset PI, cảnh báo "tank chi du Xm", return 0

3. speed < 0.1 m/s → reset PI, return 0  (xe dừng, không cảnh báo)

4. q1 = r × SA_APP_RATE × speed × SA_BOOM_W × 0.006  (L/min)
   q1 < 0.3 → cảnh báo "Q visin Xm/L < 0.3", return 0
   q1 > 6.0 → cảnh báo "Q visin XL/min > 6.0", return 0

5. return constrain(q1, 0, 200)
   → đây là setpoint cho _run_flow_pid()
```

**PI controller:**
```
error        = flow_target − _flow_rate_filtered
I += error × dt   (anti-windup: clamp ±(range/(2×I_gain)))
pid_raw      = P × error + I_gain × I
lpf_output   = (1−α_lpf) × prev + α_lpf × pid_raw
pwm          = constrain(TRIM + lpf_output, MIN, MAX)
```

### 3.3 Xử lý các trường hợp đặc biệt

- `SERVOx_FUNCTION ≠ 0`: cảnh báo mỗi 5s, **không block** xuất PWM (khác dosing motor)
- Mode 0 passthrough: PWM ngoài 800–2200 bị clamp về 1500
- Mode 2, xe dừng (speed < 0.1): PI reset, xuất `ch->get_output_min()` (không hardcode 800)

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

`DEBUG_FLOAT_ARRAY`, name=`"SA_DATA"`, array_id=0, stream EXTRA3.

| Index | Tên field | Đơn vị | Điều kiện ghi | Mô tả + edge case |
|---|---|---|---|---|
| `data[0]` | `flow_rate` | L/min | Luôn | EMA tức thời; < 0.01 ép = 0.0 |
| `data[1]` | `flow_rate_avg` | L/min | Luôn | Moving avg 10 mẫu; < 0.01 ép = 0.0 |
| `data[2]` | `flow_target` | L/min | Luôn | Setpoint hiện hành; 0 khi mode 0 hoặc FLOW_MODE=1 không đủ điều kiện |
| `data[3]` | `pump_pwm` | µs | Luôn | PWM thực xuất ra bơm; dải MIN–MAX từ SERVOx |
| `data[4]` | `spray_mode` | 0/1/2 | Luôn | 0=PASSTHROUGH, 1=FLOW PID, 2=AUTO RATE |

### 4.2 DataFlash Log [DATA]

```
Message name:  "FLWD"
Format string: "Qff"
Điều kiện ghi: SA_FLOW_LOG = 1
```

| Field | Tên cột | Kiểu | Đơn vị | Mô tả |
|---|---|---|---|---|
| 1 | `TimeUS` | uint64_t (Q) | µs | `AP_HAL::micros64()` |
| 2 | `Flow` | float (f) | L/min | Lưu lượng EMA tức thời |
| 3 | `FlowAvg` | float (f) | L/min | Moving average 10 mẫu |

### 4.3 Console Log

```
Trigger: mỗi SA_LOG_FL_MS ms khi SA_FLOW_LOG=1
Format:  [FLOW] M<n> Tgt:<target> Act:<flow_rate> Avg:<flow_avg> PWM:<pump_pwm>

Khi mode 1 hoặc mode 2 + FLOW_MODE=1 + TANK_VOL>0 (cùng chu kỳ):
         [FLOW] FM1 r:<ratio> miss:<dist>m dmax:<dist_max>m spd:<speed>m/s

SA_SIM=1: tiền tố [SIM][FLOW] thay vì [FLOW]
```

### 4.4 STATUSTEXT — Toàn bộ thông báo

| Nội dung thông báo | Mức | Điều kiện | Tần suất |
|---|---|---|---|
| `ShoesAgtech: IRQ attach failed` | CRITICAL | GPIO attach thất bại | 1 lần init |
| `ShoesAgtech: Flow sensor ready` | INFO | GPIO attach thành công | 1 lần init |
| `SA: SERVO<n>_FUNCTION=<x> must be 0(None)!` | WARNING | Sai FUNCTION | Mỗi 5s |
| `SA: SERVO<n> OK Min:<x> Trim:<y> Max:<z>` | INFO | Config đúng | 1 lần (khi vừa đúng) |
| `SA FM1: chua co mission - bom dung` | WARNING | FLOW_MODE=1, dist ≤ 1m | Mỗi 5s |
| `SA FM1: tank chi du <x>m, mission <y>m - giam mission_dist` | WARNING | FLOW_MODE=1, dist > dist_max | Mỗi 5s |
| `SA FM1: Q visin <x>L/min < 0.3 - tang mission_dist hoac giam speed` | WARNING | FLOW_MODE=1, q1 < 0.3 | Mỗi 5s |
| `SA FM1: Q visin <x>L/min > 6.0 - giam mission_dist hoac tang speed` | WARNING | FLOW_MODE=1, q1 > 6.0 | Mỗi 5s |
| `SA: Tank du ~<x>m (<y>L @<z>L/min)` | INFO | Mode 2 FLOW_MODE=0, TANK_VOL>0 | Mỗi 30s |

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERVOx_FUNCTION = 0     (x = SA_PUMP_CHAN)  → sai: cảnh báo mỗi 5s (không block)
SA_FLOW_PIN: chỉ áp dụng khi boot          → đổi giá trị cần reboot
SA_FLOW_MODE=1 yêu cầu: SA_TANK_VOL > 0 + mission đã upload + speed ≥ 0.05 m/s
                         thiếu 1 trong 3 → flow_target = 0 (bơm dừng)
```

**Ràng buộc phần cứng:** GPIO pin phải 3.3V tolerant hoặc dùng voltage divider (YF-S402B output = 5V)

**Ràng buộc vận hành:** Với FLOW_MODE=1, phải upload mission lên FC trước khi arm và chạy

---

## 6. Kết nối phần cứng [HW]

```
YF-S402B:
  VCC  →  5V
  GND  →  GND
  SIG  →  GPIO SA_FLOW_PIN (default 55; 3.3V tolerant hoặc voltage divider)

Bơm servo:
  Signal  →  SERVO output SA_PUMP_CHAN (default CH8)
  VCC     →  BEC 5V riêng (không dùng nguồn từ FC)
  GND     →  GND chung

FC config:
  SERVOx_FUNCTION = 0   (x = SA_PUMP_CHAN)
```

**Lưu ý board:**
- CubeBlack: GPIO 55 = AUX6 (IOMCU)
- Pixhawk4: kiểm tra pinout riêng, đặt `SA_FLOW_PIN` đúng rồi reboot

---

## 7. So sánh với Basic Design [ALL]

| Điểm | Basic Design dự kiến | Thực tế đã làm | Lý do |
|---|---|---|---|
| FLOW_MODE=1, TANK_VOL=0 | Không đề cập | Fallback về SA_FLOW_SP | Tránh bơm dừng khi chưa cài tank |
| Anti-windup PI | Clamp khi chạm trần/sàn | Clamp integral `±(range/(2×I_gain))` | Chuẩn hóa theo dải PWM thực tế |
| Mode 2 (nấc cao), FLOW_MODE=0, xe dừng | Không đề cập rõ | Tank monitor mỗi 30s nếu speed > 0 | Giúp người dùng ước tính quãng đường còn lại |
| Điều kiện van (Chống nghẹt / Mặc định) | Basic Design Case 9–10, param SA_SPRAY_MODE | **Tích hợp vào nấc RC**: nấc giữa = MIX_STD, nấc cao = MIX_CNT. SA_SPRAY_MODE và SA_SP_PCT được thay thế | Loại bỏ param riêng cho điều kiện van; người dùng chọn trực tiếp bằng tay RC |
| FLOW_MODE=1 nấc cao | Không đề cập | Dùng SA_MIX_CNT: dist_max nhỏ hơn → cảnh báo sớm hơn do vi sinh nhiều hơn | Chống nghẹt = mở van vi sinh nhiều → hết tank nhanh hơn → dist_max ngắn hơn |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE1_FLOW_BASIC_DESIGN.md](MODULE1_FLOW_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA
- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — hàm `_ph_update()` gọi từ `update()`
- [MODULE3_DOS_DETAIL_DESIGN.md](MODULE3_DOS_DETAIL_DESIGN.md) — hàm `_update_dosing_motor()` gọi từ `update()`
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
