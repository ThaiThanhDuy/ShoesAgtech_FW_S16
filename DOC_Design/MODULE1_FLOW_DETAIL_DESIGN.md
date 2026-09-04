# Module 1 — Flow Sensor & Spray Controller
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 10 Hz (`update()` gọi từ ArduPilot scheduler)
**Ngày hoàn thành:** 2026-05-15 | **Cập nhật lần cuối:** 2026-08-20 (rút gọn log console)

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
irq_handler() [ISR — mỗi RISING edge trên GPIO SA_FLOW_PIN]
    └──► _pulse_count++  (volatile uint32_t, atomic on ARM)

update() [10 Hz — ArduPilot scheduler]
    │
    ├──► _check_pump_config()
    │         └──► return void  (cập nhật _pump_config_ok — KHÔNG còn in log, 2026-08-20)
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
    ├──► switch(_spray_mode)
    │         mode 0 ──► _write_pump_pwm(RC_passthrough), ramp_val=0 (reset để nấc sau ramp lại từ đầu)
    │         mode 1 ──► [FLOW_MODE=0] flow_target = SA_FLOW_SP, ramp_val = min(ramp_val+0.3×dt, flow_target)
    │         (nấc giữa)  [FLOW_MODE=1] flow_target = _compute_visin_target(SA_MIX_STD)
    │                         ├──► _get_mission_dist()
    │                         ├──► _get_dosing_ref_speed() [tốc độ ĐẶT WP_SPEED,
    │                         │      KHÔNG phải GPS tức thời — xem 1.2/3.5]
    │                         ├──► kiểm tra mission / speed / q1 trong dải 0.9-1.2 (dải lưu lượng THẬT bơm đạt được, xem 3.5)
    │                         └──► (SA_FLOW_VEL≤0) _mission_started_wp1() [mới, 2026-09-04] — false thì
    │                                bơm về MIN, ramp_val=0, flow_target vẫn hiện log bình thường
    │                                (SA_FLOW_VEL>0) bỏ qua gate — bơm chạy thật ngay để hiệu chỉnh
    │                     ramp_val = min(ramp_val + 0.3×dt, flow_target)   [ramp 0.3 L/min/s]
    │                     _run_flow_pid(ramp_val, dt)
    │                     └──► return pwm  ──► _write_pump_pwm(pwm)
    │         mode 2 ──► [FLOW_MODE=0] flow_target = SA_FLOW_SP × (MIX_CNT/MIX_STD), ramp_val = min(ramp_val+0.3×dt, flow_target)
    │         (nấc cao)  [FLOW_MODE=1] flow_target = _compute_visin_target(SA_MIX_CNT)
    │                         ├──► _get_mission_dist()
    │                         ├──► _get_dosing_ref_speed() [tốc độ ĐẶT WP_SPEED]
    │                         ├──► kiểm tra mission / speed / q1 trong dải 0.9-1.2 (dải lưu lượng THẬT bơm đạt được, xem 3.5)
    │                         └──► (SA_FLOW_VEL≤0) _mission_started_wp1() — giống mode 1
    │                     ramp_val = min(ramp_val + 0.3×dt, flow_target)
    │                     _run_flow_pid(ramp_val, dt)
    │                     └──► return pwm  ──► _write_pump_pwm(pwm)
    │
    ├──► [DISARM detection — inline trong update()]
    │         now_armed=false: reset _mission_ncmds, _mission_dist_m,
    │                          _tank_empty_detected, _tank_empty_ms, _q1_range_warned,
    │                          _flow_ramp_val (2026-09-04)
    │                          + TOÀN BỘ trạng thái đọc lưu lượng (2026-08-20):
    │                          _last_pulse_snapshot, _flow_rate_filtered,
    │                          _flow_rate_avg, buffer trung bình trượt
    │         (2026-08-20: bỏ hẳn _print_fm1_arm_status() + biến _was_armed/
    │          _arm_dist_warned — không còn in gì lúc vừa ARM, xem mục 7)
    │
    ├──► [Tank-empty detection — inline trong update(), sau switch]
    │         Điều kiện: spray_mode 1 hoặc 2 + đang ARM + !_tank_empty_detected
    │         _flow_rate_filtered > 1.7 L/min liên tục ≥ 5s → INFO + set flag
    │         Nếu flow ≤ 1.7: reset _tank_empty_ms về 0
    │
    └──► [Console log — SA_FLOW_LOG=1]
              in theo chu kỳ SA_LOG_FL_MS
```

### 1.2 Mô tả từng hàm

---

**`irq_handler()`** — static
- **File:** `AP_ShoesAgtech.cpp : 572`
- **Được gọi bởi:** HAL GPIO interrupt (RISING edge trên SA_FLOW_PIN)
- **Đầu vào:** không có (no-arg ISR)
- **Xử lý:**
  1. Tăng `_pulse_count` lên 1
- **Đầu ra / Return:** `void` — side effect: `_pulse_count++`
- **Ghi chú:** `_pulse_count` là `volatile uint32_t` static; ARM cortex-M bảo đảm tăng 32-bit atomic. Không dùng mutex/critical section.

---

**`init()`**
- **File:** `AP_ShoesAgtech.cpp : 541`
- **Được gọi bởi:** ArduPilot scheduler 1 lần khi boot
- **Đầu vào:** không có
- **Xử lý:**
  1. Kiểm tra `is_enabled()` — nếu SA_ENABLE=0, return ngay
  2. Đọc `SA_FLOW_PIN`, gọi `hal.gpio->pinMode()` (INPUT) rồi `attach_interrupt(irq_handler, RISING)`
  3. Reset các biến flow (buffer, integral, avg)
  4. Gọi `_ph_init()` nếu SA_PH_EN=1
  5. Đăng ký `hal.scheduler->register_io_process(_io_update)` — load/save dữ liệu ao (Module 2) chạy trong IO thread
- **Đầu ra / Return:** `void`
  - Thành công: STATUSTEXT INFO "ShoesAgtech: Flow sensor ready"
  - Thất bại: STATUSTEXT CRITICAL "ShoesAgtech: IRQ attach failed"
- **Ghi chú:** `SA_FLOW_PIN` chỉ đọc một lần tại đây — thay đổi sau này cần reboot.

---

**`update()`**
- **File:** `AP_ShoesAgtech.cpp : 578`
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
  8. Switch theo `_spray_mode`: tính flow_target, chạy PID
  9. ARM edge detection: phát hiện disarm→arm và arm→disarm
  10. Tank-empty detection: phát hiện bơm hút không khí
  11. Console log nếu SA_FLOW_LOG=1
- **Đầu ra / Return:** `void` — side effects: `_flow_rate_filtered`, `_flow_rate_avg`, `_pump_pwm`, `_flow_target`

---

**`_check_pump_config()`**
- **File:** `AP_ShoesAgtech.cpp : 833`
- **Được gọi bởi:** `update()` mỗi chu kỳ
- **Đầu vào:** không có (đọc `_pump_chan` từ param)
- **Xử lý:**
  1. Đọc `SERVOx_FUNCTION` của kênh SA_PUMP_CHAN
  2. So sánh với lần trước; nếu không thay đổi và đã OK → return sớm
  3. `_pump_config_ok = (function == 0)`
- **Đầu ra / Return:** `void` — cập nhật `_pump_config_ok`
- **Ghi chú:** Không block motor; hàm chỉ cập nhật cờ, không gây tác dụng phụ khác. **Đã bỏ 2 dòng STATUSTEXT (2026-08-20)** — trước đây in cảnh báo `SERVOx_FUNCTION=<x> must be 0` mỗi 5s khi sai, và `SERVOx OK Min/Trim/Max` khi đúng; cùng lúc bỏ luôn `_last_warn_ms` (không còn dùng). Vẫn kiểm tra cấu hình bình thường, chỉ không in log nữa — xem mục 7.

---

**`_update_spray_mode()`**
- **File:** `AP_ShoesAgtech.cpp : 873`
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
- **Đầu vào:** `r` — tỉ lệ vi sinh trong tổng lưu lượng (0.01–1.0, sau clamp)
- **Xử lý (theo thứ tự ưu tiên):**
  1. `dist = _get_mission_dist()` — nếu ≤ 1m → reset PI, warning + return 0
  2. `speed_ms = _get_dosing_ref_speed()` (tốc độ ĐẶT cho mission, KHÔNG phải GPS tức thời — xem 1.2/3.5) — nếu < 0.1 m/s → reset PI, return 0 (không cảnh báo)
  3. `q1 = TANK_VOL × r × speed_ms × 60 / dist` (L/min)
     - Ý nghĩa: để phân phối đúng `TANK_VOL × r` lít đều trên `dist` mét, cần lưu lượng này
  4. `q1 < 0.9` → reset PI, warning 1 lần/ARM "shorten mission or increase speed" + return 0
  5. `q1 > 1.2` → reset PI, warning 1 lần/ARM "lengthen mission or reduce speed" + return 0
  6. `return constrain(q1, 0, 200)` — dải `[0.9, 1.2]` là dải lưu lượng **THẬT** bơm hiện tại đạt được (hardcode theo phần cứng, đo thực tế 2026-08-20 — KHÔNG phải ngưỡng nghiệp vụ như 0.3/2.0 cũ trước đó). Cận trên `constrain` (200) chỉ còn là chặn an toàn tuyệt đối, không liên quan.
- **Đầu ra / Return:** `float` — lưu lượng vi sinh target (L/min); 0 nếu bất kỳ điều kiện nào không đạt
- **Ghi chú:** Warning "no mission" (dist≤1m) vẫn dùng timer `_tank_warn_ms` (lặp mỗi 5s). Warning dải q1 [0.9,1.2] dùng cờ `_q1_range_warned` riêng — chỉ in **1 LẦN mỗi phiên ARM** (khác các cảnh báo khác), reset khi disarm. `vi_per_run = TANK_VOL × r` là hằng số mỗi lần chạy mission, không phụ thuộc speed hay dist.

---

> **⚠️ Hàm đã gỡ bỏ (2026-08-20):** `_print_fm1_arm_status(float r)` — trước
> đây in 1 lần STATUSTEXT chi tiết mỗi khi vừa ARM (mission/tốc độ/q1 dự
> báo/ETA), bất kể `SA_FLOW_LOG`. Đã xóa toàn bộ hàm này + lời gọi + biến
> `_was_armed`/`_arm_dist_warned` (không còn nơi nào đọc) theo yêu cầu rút
> gọn log — Module 1 không còn in gì đặc biệt lúc vừa ARM nữa. Xem mục 7.

---

**`_run_flow_pid(float target_lmin, float dt) → uint16_t`**
- **File:** `AP_ShoesAgtech.cpp : 895`
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
- **File:** `AP_ShoesAgtech.cpp : 925`
- **Được gọi bởi:** `update()` (cả 3 mode)
- **Đầu vào:**
  - `pwm` — giá trị µs (800–2200)
- **Xử lý:**
  1. Tính `chan_idx = SA_PUMP_CHAN − 1` (0-indexed)
  2. Gọi `SRV_Channels::set_output_pwm_chan(chan_idx, pwm)`
- **Đầu ra / Return:** `void` — side effect: xuất PWM ra servo channel

---

**`_get_spray_speed() → float`**
- **Được gọi bởi:** ước tính "Tank lasts" ở mode 2 (else-branch FLOW_MODE), `_update_dosing_motor()` (Module 3, DOS_MODE=1)
- **Đầu vào:** không có
- **Xử lý (ưu tiên):**
  1. `SA_SIM=1` → return `_sim_speed` (sóng sin, test màn hình)
  2. `SA_FLOW_VEL > 0` → return SA_FLOW_VEL (override, dùng khi calib đứng yên)
  3. Ngược lại → return `AP::ahrs().groundspeed()` (m/s, tốc độ GPS TỨC THỜI)
- **Đầu ra / Return:** `float` m/s
- **Ghi chú:** Dùng cho các chỗ CẦN tốc độ thực tức thời (ước tính quãng đường còn lại, dosing Module 3). **KHÔNG** dùng trong công thức `_compute_visin_target()` nữa — xem `_get_dosing_ref_speed()` bên dưới.

---

**`_get_dosing_ref_speed() → float`** — mới, 2026-08-19; thêm tầng dự phòng AHRS 2026-08-20
- **File:** `AP_ShoesAgtech.cpp : 1027`
- **Được gọi bởi:** `_compute_visin_target()`
- **Đầu vào:** không có
- **Xử lý (ưu tiên):**
  1. `SA_SIM=1` → return `_sim_speed`
  2. `SA_FLOW_VEL > 0` → return SA_FLOW_VEL (override, dùng khi calib đứng yên)
  3. `_target_speed > 0` → return `_target_speed` (tốc độ **ĐẶT** cho mission — WP_SPEED đã cập nhật qua DO_CHANGE_SPEED/GCS SET_SPEED, do `Rover::update_custom_flow()` bơm vào qua `set_target_speed()` mỗi chu kỳ, TRƯỚC khi gọi `update()`, từ `g2.wp_nav.get_speed_max()`)
  4. Ngược lại (`_target_speed` vẫn = 0) → return `AP::ahrs().groundspeed()` (dự phòng)
- **Đầu ra / Return:** `float` m/s
- **Lý do tách riêng khỏi `_get_spray_speed()`:** công thức `q1 = TANK_VOL × r × speed × 60 / mission_dist` trước đây dùng tốc độ GPS tức thời — mỗi lần xe tăng/giảm tốc hoặc vào cua, `q1` (và setpoint bơm) đổi theo ngay, gây phun không đều dọc tuyến dù `TANK_VOL`/`mission_dist` không đổi. Dùng tốc độ **ĐẶT** (ổn định suốt 1 đoạn mission, chỉ đổi khi kỹ thuật viên chủ động đổi `WP_SPEED`/`DO_CHANGE_SPEED`) giúp setpoint bơm ổn định hơn nhiều, đánh đổi lấy sai số nhỏ nếu tốc độ thực tế lệch nhiều so với tốc độ đặt (dốc, cản gió...).
- **⚠️ Bug đã sửa (2026-08-20):** `_target_speed` (từ `g2.wp_nav.get_speed_max()`) CHỈ có giá trị thật sau khi đã vào AUTO ít nhất 1 lần kể từ lúc mở nguồn (`AR_WPNav::init()` chỉ chạy trong `ModeAuto::_enter()`) — trước đó `_base_speed_max` = 0 do zero-init tĩnh, không có giá trị mặc định nào khác. Nếu ARM/gạt nấc để test mà CHƯA từng chạy AUTO phiên đó, tầng 3 luôn trả 0 → FLOW_MODE=1 tưởng xe đứng yên mãi mãi dù xe đang chạy thật (không có cảnh báo, vì đây đúng là case "speed<0.1 im lặng" — xem mục 5). Thêm tầng 4 (AHRS groundspeed) để vẫn hoạt động được khi test ngoài AUTO.
- **Không đổi:** 2 tầng ưu tiên đầu (SIM, SA_FLOW_VEL) — vẫn dùng để hiệu chỉnh/test khi xe đứng yên, giống hệt `_get_spray_speed()`.

---

**`_get_mission_dist() → float`**
- **File:** `AP_ShoesAgtech.cpp : 1073`
- **Được gọi bởi:** `_compute_visin_target()`, `_update_dosing_motor()`
- **Đầu vào:** không có
- **Xử lý:**
  1. Lấy `n = AP::mission()->num_commands()`
  2. Nếu n < 2 → return 0
  3. Nếu n == `_mission_ncmds` và cache > 0 → return cache (tránh lặp mỗi 10 Hz)
  4. Duyệt command **từ index 1** (bỏ qua index 0), chỉ tính các lệnh NAV (WAYPOINT, LOITER, SPLINE); bỏ qua lat/lng = 0
  5. Cộng dồn `prev_loc.get_distance(loc)` cho từng đoạn
  6. Lưu vào `_mission_dist_m` + `_mission_ncmds`
- **Đầu ra / Return:** `float` — tổng khoảng cách mission (m), tính từ WP1 trở đi; 0 nếu không có mission
- **⚠️ Bug đã sửa (2026-09-04):** Trước đây duyệt từ index 0, nhưng `AP_Mission::read_cmd_from_storage(0, cmd)` **luôn luôn** trả về vị trí HOME (`AP::ahrs().get_home()`), không phải WP1 thật (xem `AP_Mission.cpp:825-829`) — do đó quãng đường bị cộng dư thêm cả chặng "home (điểm arm/xuất phát) → WP1", làm `mission_dist` bị tính lớn hơn thực tế (kéo `q1` thấp hơn thực tế). Sửa bằng cách bắt đầu vòng lặp từ index 1, chỉ tính đúng quãng đường giữa các waypoint đã upload (WP1→WP2→...→WPn).
- **Ghi chú:** Cache vô hiệu khi `num_commands()` thay đổi. `_mission_ncmds` reset về 0 khi disarm → đảm bảo tính lại nếu upload mission mới khi disarm.

---

**`_mission_started_wp1() → bool`** — mới, 2026-09-04
- **File:** `AP_ShoesAgtech.cpp : ~1184`
- **Được gọi bởi:** `update()` case 1/case 2 (nấc 2/3, chỉ khi `SA_FLOW_MODE=1`), điều kiện thực tế tại call site là `SA_FLOW_VEL.get() <= 0 && !_mission_started_wp1()` (xem ghi chú bypass bên dưới)
- **Xử lý:** `true` nếu có mission, `mission->state() == MISSION_RUNNING`, và `mission->get_current_nav_index() >= 1` (đã bỏ qua HOME ở index 0, đang thực sự navigate tới WP1 trở đi)
- **Đầu ra / Return:** `bool`
- **Lý do:** Trước đây bơm bật ngay khi vừa ARM (nếu nấc 2/3 + mission hợp lệ), kể cả khi xe còn đứng ở HOME chưa bắt đầu chạy — vì `_get_dosing_ref_speed()` ưu tiên tốc độ **ĐẶT** (`_target_speed`/WP_SPEED) thay vì tốc độ thực, nên `speed_ms` có thể >0 dù xe chưa nhúc nhích. Dùng hàm này để chỉ thực sự ghi PWM ra bơm sau khi mission đã chạy tới WP1 — `_flow_target` vẫn tính và hiện trong log bình thường bất kể kết quả hàm này.
- **⚠️ Bypass khi hiệu chỉnh (2026-09-04):** `mission->state()` chỉ RUNNING khi đang ở mode AUTO — nếu ARM ở mode khác (Manual/Hold) để hiệu chỉnh bằng `SA_FLOW_VEL` lúc xe đứng yên, hàm này luôn `false`. Do đó call site bỏ qua hẳn kết quả hàm này khi `SA_FLOW_VEL > 0`, để bơm vẫn chạy thật phục vụ hiệu chỉnh mà không cần vào AUTO.

---

**`_run_simulation()`**
- **File:** `AP_ShoesAgtech.cpp : 1978`
- **Được gọi bởi:** `update()` khi SA_SIM=1 (thay cho `_ph_update()`)
- **Xử lý:**
  1. `t = millis() / 1000.0` (giây từ boot)
  2. Module 1: `_flow_rate_filtered = 2.5 + 1.5×sin(2π×t/20)` (L/min, chu kỳ 20s)
  3. Module 1: `_sim_speed = constrain(1.0 + 0.8×sin(2π×t/30), 0.1, 2.0)` (m/s, chu kỳ 30s)
  4. Module 2: `ph_sim`, `temp_sim`, `mv_sim` (Nernst: `(7.0−ph_sim)×59.16`) theo sin với các chu kỳ khác nhau; gán thẳng vào `_ph_value`, `_ph_value_ma`, `_ph_mv`, `_ph_temp`
  5. Cập nhật `_ph_last_good_ms = now`
  6. Gọi `_ph_update_daily_slots(ph_sim)` — chạy đủ pipeline slot sáng/chiều + tính kiềm theo ao active, giống hệt đường thật, để test PHAK/SA_PHK trong SITL
- **Đầu ra / Return:** `void` — ghi trực tiếp vào `_flow_rate_filtered`, `_sim_speed`, `_ph_value`, v.v.
- **Ghi chú:** Không tự tạo dữ liệu kiềm giả — kiềm vẫn được tính thật từ `ph_sim` qua slot sáng/chiều như dữ liệu cảm biến thật.

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_ENABLE` | 1 | Int8 | 1 | 0 | 1 | Bật/tắt toàn bộ library. Tắt → không có task nào chạy. |
| `SA_CAL_FAC` | 2 | Float | 3874.5 | 100 | 10000 | Hệ số cảm biến YF-S402B (pulses/Litre). Đo thực nghiệm: xả X lít, đếm xung, CAL_FAC = pulses/X. |
| `SA_EMA_AL` | 3 | Float | 0.1 | 0.01 | 1.0 | Alpha EMA lưu lượng tức thời. Nhỏ = mịn hơn, phản hồi chậm hơn. |
| `SA_FLOW_LOG` | 4 | Int8 | 0 | 0 | 1 | Bật (1) console log lưu lượng theo chu kỳ `SA_LOG_FL_MS`. |
| `SA_RC_CHAN` | 5 | Int8 | 6 | 1 | 16 | Kênh RC (1-indexed) chọn chế độ phun. |
| `SA_RC_PUMP` | 6 | Int8 | 9 | 1 | 16 | Kênh RC passthrough (mode 0). |
| `SA_PUMP_CHAN` | 7 | Int8 | 8 | 1 | 16 | Kênh servo đầu ra bơm. Bắt buộc `SERVOx_FUNCTION=0`. |
| `SA_FLOW_SP` | 8 | Float | 5.0 | 0 | 200 | Setpoint mode 1 khi `SA_FLOW_MODE=0` (L/min). |
| `SA_PID_P` | 9 | Float | 80.0 | 0 | 500 | Hệ số P: µs PWM / (L/min sai số). |
| `SA_PID_I` | 10 | Float | 20.0 | 0 | 200 | Hệ số I: µs PWM / (L/min·s). |
| `SA_PID_LPF` | 11 | Float | 0.3 | 0.01 | 1.0 | Alpha LPF đầu ra PID (1.0 = không lọc). |
| `SA_LOG_FL_MS` | 22 | Int16 | 1000 | 100 | 60000 | Chu kỳ console log lưu lượng (ms). |
| `SA_SIM` | 32 | Int8 | 0 | 0 | 1 | Chế độ giả lập: 1=inject dữ liệu sin thay cảm biến thật. |
| `SA_FLOW_PIN` | 33 | Int16 | 55 | 1 | 200 | Chân GPIO cảm biến. **Chỉ đọc khi init() — cần reboot khi thay đổi.** |
| `SA_TANK_VOL` | 34 | Float | 0.0 | 0 | 2000 | Dung tích tank vi sinh (L). `=0` tắt FLOW_MODE=1. |
| `SA_FLOW_MODE` | 35 | Int8 | 0 | 0 | 1 | Nguồn setpoint: **0**=`SA_FLOW_SP` trực tiếp, **1**=phân phối đều theo mission+tốc độ. |
| `SA_MIX_STD` | 37 | Float | 0.35 | 0.001 | 1.0 | Tỉ lệ vi sinh nấc giữa (Mặc định van). FLOW_MODE=1: `q1 = TANK_VOL × MIX_STD × speed × 60 / dist`. FLOW_MODE=0: setpoint = SA_FLOW_SP. |
| `SA_MIX_CNT` | 38 | Float | 0.50 | 0.001 | 1.0 | Tỉ lệ vi sinh nấc cao (Chống nghẹt van). FLOW_MODE=1: `q1 = TANK_VOL × MIX_CNT × speed × 60 / dist`. FLOW_MODE=0: `flow_target = SA_FLOW_SP × (MIX_CNT/MIX_STD)`. |
| `SA_FLOW_VEL` | 39 | Float | 0.0 | 0 | 20 | Override vận tốc cho FLOW_MODE=1. `=0`: dùng vận tốc thật từ AHRS. `>0`: dùng giá trị này (m/s) — dùng khi calib đứng yên. |

> **Param chỉ có hiệu lực sau reboot:** `SA_FLOW_PIN`
>
> **Param đã bị gỡ bỏ (không còn tồn tại):** `SA_APP_RATE` (slot 12, cũ), `SA_BOOM_W` (slot 13, cũ) — thay bằng `SA_MIX_STD`/`SA_MIX_CNT`. Slot 12 nay là `SA_PH_CAP_M` (Module 2); slot 13 bỏ trống.

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
    ratio = SA_MIX_CNT / SA_MIX_STD   (nếu MIX_STD ≤ 0.001 → ratio = 1.0)
    flow_target = constrain(SA_FLOW_SP × ratio, 0, 200)

SA_FLOW_MODE=1 và SA_TANK_VOL>0:
    flow_target = _compute_visin_target(SA_MIX_CNT)
    (xem luồng _compute_visin_target bên dưới)
```

**Luồng `_compute_visin_target(r)` — FLOW_MODE=1:**
```
Đầu vào: r = tỉ lệ vi sinh (MIX_STD hoặc MIX_CNT, clamp 0.01–1.0)

1. dist = _get_mission_dist()
   dist ≤ 1m → reset PI, cảnh báo "chua co mission - bom dung", return 0

2. speed = _get_dosing_ref_speed()   [tốc độ ĐẶT WP_SPEED, KHÔNG phải GPS
   tức thời — xem 1.2 và ghi chú "Nguồn tốc độ" bên dưới]
   speed < 0.1 m/s → reset PI, return 0  (xe dừng, không cảnh báo)

3. q1 = TANK_VOL × r × speed × 60 / dist   (L/min)
   — phân phối đều TANK_VOL×r lít trên mission_dist mét —

4. q1 < 0.9 → reset PI, cảnh báo (1 lần/ARM, cờ _q1_range_warned) "shorten
   mission or increase speed (pump range)", return 0
   q1 > 1.2 → reset PI, cảnh báo (1 lần/ARM, cùng cờ) "lengthen mission or
   reduce speed (pump range)", return 0
   (0.9-1.2 = dải lưu lượng THẬT bơm hiện tại đạt được, hardcode theo phần
   cứng — đo thực tế 2026-08-20, thay cho ngưỡng nghiệp vụ 0.3/2.0 cũ đã
   gỡ bỏ trước đó)

5. return constrain(q1, 0.0, 200.0)
   → đây là setpoint cho _run_flow_pid() (cận 200 chỉ là chặn an toàn tuyệt
     đối, không phải ngưỡng nghiệp vụ)

Tính sẵn cho thông tin (không dùng trong PID):
   vi_per_run = TANK_VOL × r   (lít mỗi lần chạy mission — luôn cố định)
   dist_max = TANK_VOL × r × speed × 60 / 0.9   (m — khoảng tối đa trước khi q1 < 0.9)
   dist_min = TANK_VOL × r × speed × 60 / 1.2   (m — khoảng tối thiểu trước khi q1 > 1.2)
```

> **Nguồn tốc độ (từ 2026-08-19):** `speed` ở bước 2 lấy từ
> `_get_dosing_ref_speed()`, KHÔNG phải `_get_spray_speed()` (tốc độ GPS
> tức thời) như trước. Lý do: dùng tốc độ tức thời khiến `q1` (và setpoint
> bơm) đổi theo từng cú tăng/giảm tốc, vào cua của xe → phun không đều dọc
> tuyến dù `TANK_VOL`/mission không đổi. `_get_dosing_ref_speed()` dùng tốc
> độ **ĐẶT** cho mission (`WP_SPEED`, cập nhật qua `DO_CHANGE_SPEED`/GCS
> `SET_SPEED`, Rover.cpp bơm vào qua `set_target_speed()` từ
> `g2.wp_nav.get_speed_max()`) — ổn định suốt cả đoạn, chỉ đổi khi kỹ thuật
> viên chủ động đổi tốc độ mission. `SA_SIM`/`SA_FLOW_VEL` vẫn ưu tiên như
> cũ (không đổi) để hiệu chỉnh/test khi xe đứng yên.

> **Dải q1 [0.9, 1.2] — dải lưu lượng THẬT bơm đạt được (từ 2026-08-20):**
> trước đây ngưỡng sàn/trần là `0.3`/`2.0` (ngưỡng nghiệp vụ, sau đó bỏ hẳn
> trần). Đo thực tế trên bơm hiện tại cho thấy **toàn bộ dải PWM MIN→MAX
> chỉ tạo ra được lưu lượng thật từ 0.9 đến 1.2 L/min** — ngoài dải này PID
> chỉ kẹt ở PWM MIN/MAX, cho ra đúng 0.9 hoặc 1.2 thật chứ không đạt được
> con số `q1` yêu cầu. Vì vậy đổi hẳn ngưỡng sàn/trần thành `0.9`/`1.2`,
> hardcode thẳng trong code (không phải tham số — đặc tính riêng của bơm
> đang lắp, đổi bơm khác thì sửa lại 2 số này trong `_compute_visin_target()`
> và `_print_fm1_arm_status()`). Cảnh báo dải này dùng cờ `_q1_range_warned`
> riêng — chỉ in **1 lần mỗi phiên ARM**, KHÔNG lặp lại mỗi 5s như cảnh báo
> "no mission" (vẫn dùng `_tank_warn_ms` như cũ).

**Reset khi DISARM (inline trong update(), đầu vòng lặp):**
```
if (!hal.util->get_soft_armed()) {
    _mission_ncmds       = 0      // reset cache → tính lại khi arm lại
    _mission_dist_m      = 0.0
    _tank_empty_detected = false
    _tank_empty_ms       = 0
    _q1_range_warned     = false
    _flow_ramp_val       = 0.0    // reset ramp — lần ARM sau lại bắt đầu từ 0
    // + reset toàn bộ trạng thái đọc cảm biến lưu lượng (xem mục 8)
}
```

**Bật bơm thật chỉ sau khi mission đã tới WP1 — `_mission_started_wp1()` (mới, 2026-09-04):**
```
_mission_started_wp1():
    mission = AP::mission()
    return mission != null
        && mission->state() == MISSION_RUNNING
        && mission->get_current_nav_index() >= 1
```
Lý do cần hàm này: command index 0 của mission luôn là HOME (xem
`_get_mission_dist()` — bug đã sửa cùng ngày), nên `get_current_nav_index()`
trả về 0 nghĩa là xe **chưa thực sự bắt đầu chạy tới WP1** (mới vừa ARM,
còn ở HOME, hoặc mission chưa RUNNING). `mission->state()` chỉ chuyển
sang `MISSION_RUNNING` khi vào mode AUTO (`mission.start_or_resume()` chỉ
được gọi trong `ModeAuto::update()`, xem `mode_auto.cpp:80`) — ARM ở mode
khác (Manual/Hold...) thì mission KHÔNG RUNNING dù đã upload mission.

Ở FLOW_MODE=1 (nấc 2/3), nếu điều kiện này chưa đúng: bơm bị ép về PWM
MIN (tắt hẳn), PI reset — **nhưng `_flow_target` vẫn được tính và hiện
đầy đủ trong log như bình thường** (không ẩn đi), chỉ có việc ghi PWM ra
bơm thật là bị gate lại.

**Bỏ qua gate khi đang hiệu chỉnh — `SA_FLOW_VEL > 0` (mới, 2026-09-04):**
Điều kiện thực tế ở case 1/2 là `SA_FLOW_VEL.get() <= 0 && !_mission_started_wp1()`
— nghĩa là khi `SA_FLOW_VEL > 0` (kỹ thuật viên đang chủ động đặt tốc độ
giả để hiệu chỉnh công thức lúc xe đứng yên/không chạy AUTO), gate WP1 bị
bỏ qua hoàn toàn, bơm chạy thật ngay khi q1 hợp lệ — giống cách
`SA_FLOW_VEL` đã ưu tiên hơn tốc độ thật trong `_get_dosing_ref_speed()`.
Khi `SA_FLOW_VEL=0` (vận hành thực tế bình thường), gate WP1 hoạt động
đầy đủ như mô tả ở trên.

**Ramp lên setpoint từ từ khi bắt đầu bơm (mới, 2026-09-04):**
```
_flow_ramp_val = min(_flow_ramp_val + 0.3 × dt_pid, _flow_target)   (L/min, +0.3 L/min mỗi giây)
_run_flow_pid(_flow_ramp_val, dt_pid)     // PID bám theo giá trị ramp, KHÔNG phải _flow_target thẳng
```
Áp dụng cho **cả FLOW_MODE=0 và FLOW_MODE=1** (nấc 2/3), dùng chung một
biến `_flow_ramp_val` và một hằng số ramp — không tách riêng theo mode vì
nguyên nhân vật lý (bồn cao hơn bơm, không van một chiều) gây tràn/giật
lúc mới mồi xảy ra bất kể setpoint đến từ đâu (cố định `SA_FLOW_SP` hay
tính theo mission). `_flow_ramp_val` được reset về 0 ở 3 chỗ để lần bật
bơm kế tiếp luôn ramp lại từ đầu: disarm, mode 0 (passthrough — mỗi lần
gạt về nấc 1), và (chỉ FLOW_MODE=1) khi `_mission_started_wp1()` còn
false hoặc `_flow_target < 0.01`. Khi `_mission_started_wp1()` chuyển từ
false→true (hoặc ngay khi vào FLOW_MODE=0), `_flow_ramp_val` bắt đầu từ 0
và tăng dần 0.3 L/min mỗi giây cho tới khi bằng `_flow_target`, thay vì
PID nhận error đầy đủ ngay lập tức. Hằng số 0.3 L/min/s hardcode trực tiếp
trong code (không phải tham số — không còn slot AP_Param trống, xem mục
2), chọn sao cho đạt đủ 1.0 L/min trong ~3-4 giây.

**Tank-empty detection (inline trong update(), sau switch):**
```
Điều kiện: (spray_mode==1 || spray_mode==2) && đang_ARM && !_tank_empty_detected

if _flow_rate_filtered > 1.7 L/min:
    if _tank_empty_ms == 0:
        _tank_empty_ms = now           // bắt đầu đếm 5s
    else if now - _tank_empty_ms >= 5000ms:
        _tank_empty_detected = true
        gcs().send_text(INFO, "SA: TANK EMPTY - flow X.XL/min > 1.7 for 5s")
        — bơm KHÔNG dừng —
else:
    _tank_empty_ms = 0                 // reset nếu flow về dưới 1.7
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

- `SERVOx_FUNCTION ≠ 0`: cảnh báo mỗi 5s, **không block** xuất PWM
- Mode 0 passthrough: PWM ngoài 800–2200 (kể cả 0 = chưa có tín hiệu RC, ví dụ mới cấp điện mà chưa kết nối tay cầm) → bơm về `SERVOx_MIN` đã cấu hình (KHÔNG dùng 1500 cứng, tránh bơm tự chạy khi chưa có RC)
- Mode 2 FLOW_MODE=1, xe dừng (speed < 0.1): PI reset, return 0 → bơm dừng
- `SA_FLOW_VEL > 0`: override vận tốc, dùng khi calib đứng yên
- Tank-empty flag reset khi disarm → phát hiện lại ở lần ARM tiếp theo

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
| `data[4]` | `spray_mode` | 0/1/2 | Luôn | 0=PASSTHROUGH, 1=FLOW PID nấc giữa, 2=FLOW PID nấc cao |

> `data[5..14]` — Module 2 (pH sensor + ao active). Xem MODULE2_PH_DETAIL_DESIGN.md.
> `data[15..18]` — Module 3 (Dosing motor + ao active). Xem MODULE3_DOS_DETAIL_DESIGN.md.

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

> **Rút gọn toàn diện (2026-08-20):** log định kỳ giờ chỉ còn ĐÚNG 1 dòng,
> không còn dòng phụ "FM1 r:... q1:... miss:..." như trước. Cập nhật cùng
> ngày: tách `FM<x>` (SA_FLOW_MODE) và `N<nấc>` (vị trí gạt RC) thành 2
> trường riêng — trước đó "FM<n>" gộp chung nấc vào ký hiệu FM gây nhầm với
> ý nghĩa gốc "FM1" (FLOW_MODE=1) đã dùng ở các log khác (mục 3.5, 4.4).

```
Trigger: mỗi SA_FLOW_LOG_MS ms khi SA_FLOW_LOG=1

    [FLOW] FM<x> N<nấc> Q: <target>

FM<x>    = SA_FLOW_MODE hiện tại (0 hoặc 1) — cách tính setpoint đang dùng
N<nấc>   = _spray_mode + 1 (1/2/3 — khớp đúng vị trí gạt nấc RC vật lý)
<target> = _flow_target:
             nấc 1 (manual/passthrough)      → luôn 0.00
             nấc 2/3 + SA_FLOW_MODE=0 (FM0)  → số CỐ ĐỊNH (SA_FLOW_SP hoặc SA_FLOW_SP×ratio)
             nấc 2/3 + SA_FLOW_MODE=1 (FM1)  → số DAO ĐỘNG (q1 tính theo tank+mission+speed)

SA_SIM=1: tiền tố [SIM][FLOW] thay vì [FLOW]
```

Ví dụ:
```
[FLOW] FM0 N1 Q: 0.00      (nấc 1 manual — FM hiện đúng giá trị tham số dù nấc 1 không dùng đến)
[FLOW] FM0 N2 Q: 1.50      (nấc 2, FLOW_MODE=0, setpoint cố định 1.5 L/min)
[FLOW] FM1 N3 Q: 1.08      (nấc 3, FLOW_MODE=1, q1 tính động — số này đổi theo tốc độ/mission)
```

### 4.4 STATUSTEXT — Toàn bộ thông báo

> **Rút gọn toàn diện (2026-08-20):** bỏ hẳn log cấu hình servo (`_check_pump_config()`
> không còn in gì) và bỏ hẳn toàn bộ thông báo lúc vừa ARM (`_print_fm1_arm_status()`
> đã xóa). Bảng dưới đây là DANH SÁCH ĐẦY ĐỦ (và duy nhất) các STATUSTEXT Module 1
> còn lại sau rút gọn.

| Nội dung thông báo | Mức | Điều kiện | Tần suất |
|---|---|---|---|
| `ShoesAgtech: IRQ attach failed` | CRITICAL | GPIO attach thất bại | 1 lần init |
| `ShoesAgtech: Flow sensor ready` | INFO | GPIO attach thành công | 1 lần init |
| `SA FM1: no mission - pump stopped` | WARNING | FLOW_MODE=1, dist ≤ 1m, đang chạy | Mỗi 5s |
| `SA FM1: q1=X.XXL/min < 0.9 (pump range) - shorten mission or increase speed` | WARNING | FLOW_MODE=1, q1 < 0.9 (ngoài dải lưu lượng thật bơm đạt được) | **1 lần/phiên ARM** (cờ `_q1_range_warned`) |
| `SA FM1: q1=X.XXL/min > 1.2 (pump range) - lengthen mission or reduce speed` | WARNING | FLOW_MODE=1, q1 > 1.2 | **1 lần/phiên ARM** (cờ `_q1_range_warned`, dùng chung với dòng trên) |
| `SA: TANK EMPTY - flow X.XL/min > 1.7 for 5s` | INFO | flow > 1.7 liên tục 5s, spray_mode 1/2, đang ARM | 1 lần/phiên ARM |

**Đã gỡ bỏ hoàn toàn (2026-08-20) — không còn in nữa:**
- `SA: SERVO<n>_FUNCTION=<x> must be 0(None)!` / `SA: SERVO<n> OK Min/Trim/Max` (config servo)
- `SA FM1: no mission - pump will stay stopped` / `SA FM1 READY: ...` / `SA FM1: q1=... @...dist=...(dmax/dmin)` / `SA FM1 OK: ...` (toàn bộ 4 thông báo lúc vừa ARM)
- `SA: Tank lasts ~...` (ước tính quãng đường còn lại ở mode 2)

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERVOx_FUNCTION = 0     (x = SA_PUMP_CHAN)  → sai: KHÔNG block, KHÔNG còn cảnh báo (bỏ log 2026-08-20, tự kiểm tra cấu hình servo qua Mission Planner)
SA_FLOW_PIN: chỉ áp dụng khi boot          → đổi giá trị cần reboot
SA_FLOW_MODE=1 yêu cầu: SA_TANK_VOL > 0 + mission đã upload + speed ≥ 0.1 m/s + q1 trong [0.9, 1.2]
                         (0.9/1.2 = dải lưu lượng THẬT bơm đạt được, hardcode theo phần cứng, 2026-08-20)
                         thiếu 1 trong các điều kiện trên → flow_target = 0 (bơm dừng)
Tank-empty: chỉ cảnh báo, bơm vẫn tiếp tục — người lái tự quyết định
```

**Ràng buộc phần cứng:** GPIO pin phải 3.3V tolerant hoặc dùng voltage divider (YF-S402B output = 5V)

**Ràng buộc vận hành:** Với FLOW_MODE=1, phải upload mission lên FC trước khi arm và chạy. Nếu thay đổi mission sau khi disarm, cache sẽ tính lại tự động khi arm lần tiếp theo.

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
| Điều kiện van (Chống nghẹt / Mặc định) | Basic Design Case 9–10, param SA_SPRAY_MODE | **Tích hợp vào nấc RC**: nấc giữa = MIX_STD, nấc cao = MIX_CNT. SA_SPRAY_MODE và SA_SP_PCT được thay thế | Loại bỏ param riêng; người dùng chọn trực tiếp bằng tay RC |
| FLOW_MODE=1 công thức | `q1 = r × APP_RATE × speed × BOOM_W × 0.006` + `dist_max = TANK_VOL × 10000 / (r × APP_RATE × BOOM_W)` | `q1 = TANK_VOL × r × speed × 60 / mission_dist` | SA_APP_RATE và SA_BOOM_W loại khỏi công thức; vi_per_run = TANK_VOL×r hằng số; dist_max theo tốc độ thực tế |
| FLOW_MODE=1 dải q1 | q1 > 6.0 → dừng (giới hạn cảm biến YF-S402B) | q1 chỉ còn sàn dưới 0.3 (không còn trần trên) | Ban đầu áp trần 2.0 (YF-S402B chạy ổn nhất 0.3–2.0 L/min); từ 2026-08-19 bỏ hẳn trần trên theo yêu cầu — chấp nhận phun đậm đặc trên mission ngắn thay vì chặn bơm |
| Dải q1, ngưỡng + tần suất cảnh báo (2026-08-20) | Không có (thừa kế ngưỡng 2026-08-19 ở trên) | Đổi hẳn sàn/trần thành `0.9`/`1.2` — **dải lưu lượng THẬT bơm hiện tại đạt được**, hardcode trong code (không phải tham số); cảnh báo runtime đổi từ lặp mỗi 5s sang **1 lần/phiên ARM** (cờ `_q1_range_warned`) | Đo thực tế phát hiện bơm chỉ đạt 0.9-1.2 L/min trên toàn dải PWM MIN→MAX — ngưỡng nghiệp vụ 0.3/2.0 cũ không còn phản ánh đúng khả năng phần cứng; cảnh báo lặp mỗi 5s không cần thiết vì nguyên nhân (giới hạn phần cứng) không tự hết theo thời gian |
| Hết thùng vi sinh | Không có | Tank-empty detection: flow > 1.7 L/min liên tục 5s (tăng từ 3s ban đầu, 2026-08-19) → CRITICAL, bơm không dừng | Bơm hút không khí → bánh xe quay nhanh bất thường → cảnh báo người lái; tăng thời gian xác nhận để giảm báo động giả do dao động lưu lượng tức thời |
| ARM status (2026-05→08-19) | Không có | `_print_fm1_arm_status()` in 1 lần/ARM với q1 dự báo, thời gian, vi/run | Giúp người lái xác nhận hệ thống đúng trước khi chạy |
| Nguồn tốc độ cho công thức FLOW_MODE=1 (2026-08-19) | Không có | Tách hàm riêng `_get_dosing_ref_speed()`: dùng tốc độ **ĐẶT** (`WP_SPEED`/`g2.wp_nav.get_speed_max()`) thay vì tốc độ GPS tức thời (`_get_spray_speed()`) | Tốc độ tức thời dao động theo cua/tăng giảm tốc khiến q1 (setpoint bơm) đổi liên tục → phun không đều dọc tuyến; tốc độ ĐẶT ổn định suốt đoạn mission, giống cách AUTO_SPD đã dùng |
| Mission cache reset | Không đề cập | Reset `_mission_ncmds=0` khi disarm | Đảm bảo tính lại nếu thay đổi mission khi đất |
| Rút gọn log console (2026-08-20) | Không có | **Xóa hẳn:** `_print_fm1_arm_status()` (toàn bộ 4 STATUSTEXT lúc ARM) + `_was_armed`/`_arm_dist_warned` (không còn dùng); 2 STATUSTEXT config servo trong `_check_pump_config()` (+ `_last_warn_ms` không còn dùng); message "SA: Tank lasts" ở mode 2. **Đơn giản hoá:** log định kỳ từ 2 dòng chi tiết (`M<n> Tgt/Act/Avg/PWM` + `FM1 r/q1/miss/dmax/spd/vi_run`) còn ĐÚNG 1 dòng `FM<x> N<nấc> Q:<target>` (`FM`=SA_FLOW_MODE, `N`=nấc gạt — tách 2 trường sau khi gộp chung gây nhầm lẫn) | Theo yêu cầu người dùng — chỉ cần biết setpoint đang là gì theo nấc/mode, không cần chi tiết kỹ thuật; giữ lại đúng 3 cảnh báo thật sự cần hành động (no mission, q1 ngoài dải bơm, tank empty) |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE1_FLOW_BASIC_DESIGN.md](MODULE1_FLOW_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [MODULE1_FLOW_TEST_CASES.md](MODULE1_FLOW_TEST_CASES.md) — test cases với số đối ứng cụ thể
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA tất cả module
- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — hàm `_ph_update()` gọi từ `update()`
- [MODULE3_DOS_DETAIL_DESIGN.md](MODULE3_DOS_DETAIL_DESIGN.md) — hàm `_update_dosing_motor()` gọi từ `update()`
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
