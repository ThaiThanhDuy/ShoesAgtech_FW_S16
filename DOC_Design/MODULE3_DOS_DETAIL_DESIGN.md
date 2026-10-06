# Module 3 — Dosing Motor (Vít Tải Thức Ăn Tôm)
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 10 Hz (`_update_dosing_motor()` gọi từ `update()`)
**Ngày hoàn thành:** 2026-05-15 | **Cập nhật lần cuối:** 2026-08-20

---

## 1. Chi tiết code — Function Flow [ALL]

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
update() [Module 1, 10 Hz]
    │
    └──► _update_dosing_motor()
              │
              ├──► _sync_dosing_setpoint()
              │         │ đồng bộ 2 chiều SA_DOS_SP/SA_DOS_FOOD ↔ ao đang active
              │         │ (xem MODULE2_PH_DETAIL_DESIGN.md — pond dùng chung với Module 2)
              │         └──► cập nhật _ponds[_active_pond_idx].dos_sp / .dos_food
              │
              ├──► Lấy dos_sp_active / dos_food_active từ ao active
              │         (ao chưa hợp lệ → dùng thẳng SA_DOS_SP / SA_DOS_FOOD)
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
                        ├──► DOS_MODE=0 (PWM trực tiếp, 2026-08-20):
                        │       dos_sp_active ≤ 0 → _dos_pwm = 1500 (an toàn)
                        │       dos_sp_active > 0 → _dos_pwm = constrain(dos_sp_active, SERVO_MIN, SERVO_MAX)
                        │       (KHÔNG qua SA_DOS_Ax/Bx, KHÔNG qua SA_DOS_REV — giá trị tuyệt đối)
                        │
                        └──► DOS_MODE=1 hoặc 2:
                                  │
                                  ├──► food_idx = clamp(dos_food_active, 1, 7) − 1  (0-indexed)
                                  │
                                  ├──► DOS_MODE=1: dos_gpm = dos_sp_active
                                  │
                                  └──► DOS_MODE=2: _get_mission_dist() [dùng chung Module 1]
                                                   + _get_spray_speed() [dùng chung Module 1 —
                                                     SIM > SA_FLOW_VEL > AHRS groundspeed]
                                                   → dos_gpm = dos_sp_active × speed × 60 / dist
                                                   (không đủ điều kiện → dos_gpm=0, pwm=1500 + warn 5s)
                                        │
                                        └──► _dos_rate_to_pwm(dos_gpm, food_idx)  [mới 2026-10-06, DUY NHẤT
                                             công thức — SA_DOS_Ax=0 (chưa hiệu chuẩn) → pwm=1500 + warn;
                                             khác 0 → pwm=(dos_gpm−SA_DOS_Bx)/SA_DOS_Ax, áp SA_DOS_REV
                                             (đối xứng qua 1500 nếu REV=0)]
                                             SRV_Channels::set_output_pwm_chan()
```

> **Đổi số mode (2026-08-20):** thêm mode 0 mới (PWM trực tiếp) để hiệu chuẩn
> tại bàn. 2 mode cũ đẩy số lên: mode cũ 0 (tốc độ cố định) → **mode 1**; mode
> cũ 1 (tỉ lệ mission) → **mode 2**. **Máy đã cấu hình `SA_DOS_MODE=1` từ
> trước cần kiểm tra lại** — sau cập nhật, giá trị này sẽ tự động đổi ý
> nghĩa từ "tỉ lệ mission" sang "tốc độ cố định" nếu không chỉnh lại.

### 1.2 Mô tả từng hàm

---

**`_check_dosing_config()`**
- **File:** `AP_ShoesAgtech.cpp : 1744`
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

**`_check_disc_config()`** — mới, 2026-09-08, đổi điều kiện 2026-10-06
- **File:** `AP_ShoesAgtech_Dosing.cpp`
- **Được gọi bởi:** `_update_dosing_motor()` mỗi chu kỳ 10 Hz
- **Đầu vào:** không có (đọc `_dos_params.disc_chan` từ param)
- **Xử lý:**
  1. `SA_DISC_CHAN ≤ 0` → tính năng đang tắt, `_disc_config_ok=false`, return ngay (không cảnh báo)
  2. Lấy `chan_idx = SA_DISC_CHAN − 1`
  3. Kiểm tra 3 điều kiện trên `SRV_Channel` (**KHÔNG kiểm tra TRIM**, khác `_check_dosing_config()` của trục vít — formula vẫn đọc thẳng TRIM thật đang cấu hình làm điểm 0%, chỉ không bắt buộc đúng 1500):
     - `FUNCTION = k_none (0)`
     - `MIN = 800` (đổi từ 1500, 2026-10-06)
     - `MAX = 2200`
  4. Nếu tất cả đúng: nếu vừa chuyển từ sai → OK, in "setup thành công" 1 lần
  5. Nếu sai bất kỳ: in từng điều kiện sai mỗi 5s (dùng `_disc_warn_ms` riêng, không tranh chấp `_dos_warn_ms` của trục vít)
- **Đầu ra / Return:** `void` — cập nhật `_disc_config_ok` và `_disc_was_ok`
- **Ghi chú:** Cấu hình sai không chặn trục vít chạy — chỉ chặn riêng việc ghi PWM ra đĩa rải (xem bước 14b trong `_update_dosing_motor()`); độ trễ `SA_DISC_DLY` giữa đĩa/trục vít vẫn áp dụng bình thường dựa theo `SA_DISC_CHAN > 0`, không phụ thuộc kết quả kiểm tra này.

---

**`_clamp_food(int8_t food) → int8_t`** — static
- **File:** `AP_ShoesAgtech_Dosing.cpp`
- Kẹp giá trị loại thức ăn về dải hợp lệ 1–7 (khớp `SA_DOS_FOOD` `@Range`). Dùng ở mọi nơi truy cập `SA_DOS_Ax[]`/`SA_DOS_Bx[]` để tránh index ngoài mảng.

---

**`_dos_rate_to_pwm(float rate_gpm, uint8_t food_idx) → uint16_t`** — mới 2026-10-05, DUY NHẤT công thức từ 2026-10-06 (không còn fallback, không còn `const` vì cần in STATUSTEXT)
- **File:** `AP_ShoesAgtech_Dosing.cpp`
- **Được gọi bởi:** `_update_dosing_motor()`, cả DOS_MODE=1 và DOS_MODE=2
- **Đầu vào:** `rate_gpm` — tốc độ cấp mong muốn (g/phút); `food_idx` — chỉ số loại thức ăn 0-based
- **Xử lý:**
  1. Đọc `a = SA_DOS_Ax[food_idx]`
  2. **Nếu `|a| < 0.0001` (mặc định, CHƯA hiệu chuẩn):** STATUSTEXT WARNING `"SA: SA_DOS_A%d not calibrated - feeder stopped"` (rate-limit 5s, dùng chung `_dos_warn_ms`), return `1500` (dừng an toàn) — **không có fallback nào khác**
  3. **Nếu `|a| ≥ 0.0001` (đã hiệu chuẩn bằng cân thực tế):**
     - `pwm_calib = (rate_gpm − SA_DOS_Bx[food_idx]) / a` — PWM **tuyệt đối**, không phải offset cộng/trừ vào 1500
     - `SA_DOS_REV = 0` (thuận) → lấy đối xứng qua 1500: `pwm_calib = 3000 − pwm_calib`
     - `SA_DOS_REV = 1` (ngược) → giữ nguyên `pwm_calib`
     - Return `constrain(pwm_calib, 800, 2200)`
- **Đầu ra / Return:** `uint16_t` PWM cuối cùng ghi ra servo
- **Ghi chú:** Tách theo **từng loại thức ăn** (`food_idx`) — mỗi loại phải tự hiệu chuẩn `SA_DOS_Ax`/`SA_DOS_Bx` riêng trước khi dùng được ở DOS_MODE=1/2, không có cách nào chạy "tạm" khi chưa đo. Lý do hồi quy trực tiếp chính xác hơn công thức cũ (đã gỡ bỏ): **không ép buộc đường thẳng phải đi qua gốc tọa độ** — dữ liệu đo thật thường có offset/deadband cơ khí đáng kể (vd dữ liệu hiệu chuẩn loại 1 thực tế: `a=1.520350`, `b=−2196.423077`, R²=0.997 — ở PWM=1500 công thức cũ giả định lưu lượng=0, nhưng hồi quy thật cho ra ≈83.6 g/phút, chênh lệch đáng kể).

---

**`_sync_dosing_setpoint()`** — mới, chưa có ở bản thiết kế trước
- **File:** `AP_ShoesAgtech.cpp : 1826`
- **Được gọi bởi:** `_update_dosing_motor()` — đầu tiên, mỗi chu kỳ 10 Hz
- **Mục đích:** `SA_DOS_SP`/`SA_DOS_FOOD` là tham số người dùng đọc/sửa qua GCS, nhưng giá trị điều khiển motor thực tế lấy từ `dos_sp`/`dos_food` riêng của từng ao (`PondEntry`, dùng chung với Module 2). Hàm này đồng bộ 2 chiều giữa hai nơi lưu trữ đó.
- **Xử lý:**
  1. Nếu ao đang active (`_ponds[_active_pond_idx]`) chưa hợp lệ (`!valid`, vd chưa có GPS) → return ngay, `_update_dosing_motor()` sẽ dùng thẳng `SA_DOS_SP`/`SA_DOS_FOOD` làm giá trị chung
  2. **Vừa đổi ao** (`_dos_sync_pond != _active_pond_idx`): nạp `dos_sp`/`dos_food` đã lưu của ao đó lên `SA_DOS_SP`/`SA_DOS_FOOD` (ghi đè giá trị hiển thị trên GCS), cập nhật `_dos_sync_pond = _active_pond_idx`, return
  3. **Cùng ao như lần đồng bộ trước:** so `SA_DOS_SP`/`SA_DOS_FOOD` hiện tại với giá trị tại lần đồng bộ gần nhất (`_dos_sp_sync_val`/`_dos_food_sync_val`) — nếu người dùng vừa sửa, ghi giá trị mới xuống ao đang active (`pond.dos_sp`/`pond.dos_food`), đặt `_ponds_dirty=true` (sẽ được `_io_update()` lưu xuống SD)
- **Đầu ra / Return:** `void` — side effect: `_ponds[].dos_sp/.dos_food`, hoặc `SA_DOS_SP`/`SA_DOS_FOOD` (qua `AP_Param::set()`)
- **Ghi chú:** `_dos_sync_pond = 0xFF` ban đầu (chưa đồng bộ lần nào) đảm bảo lần active-pond đầu tiên luôn nạp giá trị ao xuống param, không bị hiểu nhầm là "người dùng vừa sửa".

---

**`_update_dosing_motor()`**
- **File:** `AP_ShoesAgtech.cpp : 1856`
- **Được gọi bởi:** `update()` mỗi chu kỳ
- **Đầu vào:** không có
- **Xử lý:**
  1. Gọi `_sync_dosing_setpoint()` — đồng bộ setpoint/loại thức ăn với ao active
  2. Lấy `dos_sp_active`/`dos_food_active`: nếu ao active hợp lệ → lấy từ `PondEntry`; ngược lại → dùng thẳng `SA_DOS_SP`/`SA_DOS_FOOD`
  3. Gọi `_check_dosing_config()` → nếu `!_dos_config_ok`: `_dos_pwm=1500`, return
  4. Đọc RC: `rc_pwm = RC_Channels::get_radio_in(SA_DOS_RC − 1)`
  5. `motor_on = (rc_pwm > 1500)` — mất tín hiệu (rc_pwm=0) → tắt an toàn
  5b/5c. **An toàn khởi động — thay ARM/DISARM (đổi 2026-09-09, gộp chung 1 cơ chế với chặn-tự-chạy-lại-sau-reboot cũ):** nếu `_dos_rc_seen_off` còn `false` thì: mỗi 3 GIÂY (`now - _dos_rc_check_ms >= 3000`, không phải mỗi chu kỳ 10Hz) kiểm tra `rc_pwm` — trong khoảng `(0, 1500]` → set `_dos_rc_seen_off = true` + STATUSTEXT INFO `"SA: Feeder RC ready - feeder can start"`; ngược lại → STATUSTEXT WARNING `"SA: Move SA_DOS_RC to OFF to start feeder"`, lặp lại mỗi 3s cho tới khi đạt. Trong lúc `_dos_rc_seen_off == false`, luôn ép `motor_on = false` bất kể `rc_pwm`. `_dos_rc_seen_off` không reset khi disarm (chỉ reset khi FC reboot thật) — chỉ cần đạt 1 lần/phiên nguồn. Nếu bật nguồn mà `SA_DOS_RC` đã sẵn ở OFF → tự động sẵn sàng ngay từ lần kiểm tra đầu tiên (3s sau boot), không cần gạt gì thêm.
  6. Nếu state thay đổi (`motor_on != _dos_was_on`) → STATUSTEXT ON/OFF, ghi mốc `_dos_seq_ms = now`
  6b. **Đĩa rải ly tâm — sequencing (mới 2026-09-08):** `disc_enabled = SA_DISC_CHAN > 0`; `elapsed = now − _dos_seq_ms`; `disc_delay_ms = disc_enabled ? SA_DISC_DLY×1000 : 0`. Tính:
      - `auger_allowed = motor_on && (elapsed ≥ disc_delay_ms)` — trục vít chỉ được chạy SAU khi đã đủ độ trễ kể từ lúc bật
      - `_disc_running = _dos_rc_seen_off && disc_enabled && (motor_on || elapsed < disc_delay_ms)` — đĩa chạy NGAY khi bật, và còn chạy thêm `SA_DISC_DLY` giây sau khi tắt trước khi dừng hẳn
      - `disc_enabled=false` (mặc định `SA_DISC_CHAN=0`) → `auger_allowed = motor_on` (giống hệt hành vi cũ, không có độ trễ nào)
      - **⚠️ Lý do thêm `_dos_rc_seen_off &&` vào `_disc_running` (2026-09-08, đổi cờ nguồn từ `_system_ready` sang `_dos_rc_seen_off` ngày 2026-09-09 khi bỏ ARM/DISARM):** nếu chỉ ép `motor_on=false` mà không chặn thêm công thức này, đĩa vẫn có thể quay giả do `_dos_seq_ms` mặc định = 0 lúc boot → `elapsed` nhỏ → nhánh `elapsed < disc_delay_ms` vô tình đúng trong `disc_delay_ms` mili-giây đầu tiên sau boot dù `motor_on` đã bị ép false — phải chặn tận gốc bằng cờ boot-ready hiện có (`_dos_rc_seen_off`).
  7. Nếu `!auger_allowed` (bao gồm cả `!motor_on` VÀ giai đoạn đang chờ đĩa quay lên trước khi cho trục vít chạy) → `_dos_pwm = 1500` (KHÔNG return — hàm vẫn chạy tiếp để ghi PWM đĩa rải và in log như bình thường)
  8. **DOS_MODE=0 (PWM trực tiếp, mới 2026-08-20):** `dos_sp_active ≤ 0` → `_dos_pwm = 1500` (an toàn — tránh chạy full tốc nếu quên set); ngược lại `_dos_pwm = constrain(dos_sp_active, SERVOx_MIN, SERVOx_MAX)` — ghi thẳng, KHÔNG qua `SA_DOS_Ax`/`SA_DOS_Bx`/`SA_DOS_REV`. Nhảy thẳng xuống bước 14 (bỏ qua 9-13).
  9. **DOS_MODE=1 hoặc 2** — `food_idx = clamp(dos_food_active, 1, 7) − 1` (0-indexed, dùng chung cho cả 2 mode)
  10. **DOS_MODE=1:** `dos_rate_gpm = dos_sp_active` (SP CHÍNH LÀ tốc độ, g/phút — nhập THẲNG tốc độ cấp liên tục, không có khái niệm tổng khối lượng hay quãng đường)
  11. **DOS_MODE=2:**
      - `mission_dist = _get_mission_dist()`
      - `speed = _get_spray_speed()` — **từ bản này, dùng chung hàm với Module 1** (trước đây tự viết inline `SIM ? _sim_speed : groundspeed()`, thiếu lớp `SA_FLOW_VEL`; nay đủ 3 cấp: SIM > SA_FLOW_VEL > AHRS groundspeed)
      - `SA_SPD_START=0` → `speed_min_start = 0.05 m/s` (tắt hẳn kiểm tra %, giống hành vi cũ). `SA_SPD_START>0` → `speed_min_start = max(0.05 m/s, (SA_SPD_START/100) × _target_speed)` — `_target_speed` = tốc độ ĐẶT cho mission (`WP_SPEED`, Rover.cpp bơm vào qua `set_target_speed()`, dùng chung với Module 1)
      - Điều kiện: `dist > 1.0m && speed ≥ speed_min_start` (2026-08-19: không rải khi xe mới nhích/đang tăng tốc — mặc định phải đạt ≥80% (`SA_SPD_START` mặc định 80) tốc độ đặt mới bắt đầu rải, tránh dồn thức ăn vào đoạn xe đi chậm lúc bắt đầu/qua cua)
      - Đủ: `dos_rate_gpm = dos_sp_active × speed × 60 / dist` — `dos_sp_active` ở mode này là **TỔNG số gam** cho cả `mission_dist` mét, firmware tự chia theo tốc độ/quãng đường ra tốc độ tức thời tương ứng
      - Không đủ: `dos_rate_gpm = 0`, `_dos_pwm = 1500` + STATUSTEXT mỗi 5s
  12. `_dos_pwm = _dos_rate_to_pwm(dos_rate_gpm, food_idx)` — DUY NHẤT công thức (2026-10-06), xem function-doc riêng bên dưới (SA_DOS_Ax=0 chưa hiệu chuẩn → dừng an toàn, không có fallback)
  14. `SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, _dos_pwm)`
  14b. **Đĩa rải ly tâm (mới 2026-09-08, đổi công thức 2026-10-06):** gọi `_check_disc_config()` đầu hàm (yêu cầu FUNCTION=0/MIN=800/MAX=2200 trên `SA_DISC_CHAN` — KHÔNG kiểm tra TRIM, xem mục 1.2). Nếu `disc_enabled && _disc_running && _disc_config_ok`: `pct = SA_DISC_PCT/100`; `SA_DISC_REV=0` (thuận) → `_disc_pwm = TRIM + pct×(MAX−TRIM)`; `SA_DISC_REV=1` (ngược) → `_disc_pwm = TRIM − pct×(TRIM−MIN)`. Không chạy/cấu hình sai → `_disc_pwm = TRIM` (1500, **đổi từ dừng ở MIN trước đây**). Ghi qua `SRV_Channels::set_output_pwm_chan()`. Cấu hình sai → PWM luôn ở TRIM (không quay) dù `_disc_running=true`, đồng thời in cảnh báo (xem `_check_disc_config()`). Độc lập với `_dos_pwm`/`SA_DOS_REV` (của trục vít) — đĩa có `SA_DISC_REV` riêng, không dùng chung với trục vít.
  - `dos_rate_gpm` (biến cục bộ, khởi tạo 0.0f đầu hàm, luôn =0 ở mode 0) được giữ lại đến bước in log (15) để hiển thị tốc độ tức thời — không có getter public, chỉ dùng nội bộ cho console log
  15. Console log nếu SA_DOS_LOG=1 — **không đổi**, vẫn chỉ hiện `FM<x> Q:<y>` của trục vít, không có thông tin đĩa rải trong dòng log này (xem `get_disc_pwm()`/`get_disc_running()` nếu cần đọc qua code khác)
- **Đầu ra / Return:** `void` — side effects: `_dos_pwm`, `_disc_pwm`, 2 servo output riêng biệt
- **Ghi chú:** `DOS_MODE=1` và `DOS_MODE=2` dùng chung một nguồn thông số (`SA_DOS_V` dùng chung + `SA_DOS_Fx`/`SA_DOS_Dx` theo loại thức ăn) — không còn tham số `SA_DOS_RATE` riêng. `DOS_MODE=0` (PWM trực tiếp) không dùng bất kỳ thông số nào trong nhóm này. `SA_DOS_Fx` đổi ý nghĩa 2026-08-19 (xem mục 2 và 3.5); số thứ tự 3 mode đổi 2026-08-20 (xem mục 1.1).

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_DOS_CHAN` | 1 | Int8 | 10 | 1 | 16 | Kênh servo đầu ra motor (1-indexed). Phải đúng 4 điều kiện SERVO. |
| `SA_DOS_RC` | 2 | Int8 | 8 | 1 | 16 | Kênh RC bật/tắt motor. PWM>1500 → bật; ≤1500 hoặc =0 (mất tín hiệu) → tắt. |
| `SA_DOS_SP` | 3 | Float | 0.0 | — | — | **Ý nghĩa đổi theo `SA_DOS_MODE`:** mode 0 = xung PWM (µs) ghi thẳng ra servo; mode 1 = tốc độ cố định (g/phút); mode 2 = tổng gam cho cả mission. Luôn theo **ao đang chọn** (`SA_POND_IDX`) — đổi ao nạp lại giá trị đã lưu của ao đó; sửa giá trị này lưu lại cho ao đang chọn. |
| `SA_DOS_REV` | 4 | Int8 | 0 | 0 | 1 | Chiều quay: **0**=thuận (800–1500), **1**=ngược (1500–2200). Chỉ áp dụng mode 1/2 — mode 0 (PWM trực tiếp) không qua REV. |
| `SA_SPD_START` | 19 | Int8 | 80 | 0 | 100 | **% tốc độ ĐẶT cho mission (`WP_SPEED`) tối thiểu để bắt đầu rải/bơm — DÙNG CHUNG DOS_MODE=2 (Module 3) và FLOW_MODE=1 (Module 1, xem MODULE1_FLOW_DETAIL_DESIGN.md).** `0` = tắt hẳn kiểm tra này (quay về hành vi cũ: chỉ cần vượt sàn tuyệt đối 0.05 m/s là chạy). Sàn 0.05 m/s luôn áp dụng dù giá trị này là bao nhiêu. Thêm 2026-08-19 (tên cũ `SA_DOS_SPD_PCT`, mặc định 50, chỉ Module 3); đổi tên + mặc định 80 + dùng chung Module 1, 2026-09-04. |
| `SA_DOS_LOG` | 5 | Int8 | 0 | 0 | 1 | Bật (1) console log dosing motor theo chu kỳ `SA_DOS_LOG_MS`. |
| `SA_DOS_LOG_MS` | 6 | Int16 | 1000 | 100 | 60000 | Chu kỳ console log dosing (ms). |
| `SA_DOS_MODE` | 7 | Int8 | 0 | 0 | 2 | **0**=PWM trực tiếp (mới, 2026-08-20 — SA_DOS_SP là xung PWM, dùng để hiệu chuẩn tại bàn), **1**=tốc độ cố định (số cũ = 0), **2**=tỉ lệ theo vận tốc + mission (số cũ = 1). Mode 1/2 dùng chung `SA_DOS_Ax`/`SA_DOS_Bx` (xem bên dưới); mode 0 không dùng. |
| `SA_DOS_FOOD` | 8 | Int8 | 1 | 1 | 7 | Loại thức ăn đang dùng **cho ao đang chọn**. Đổi ao sẽ nạp lại giá trị đã lưu của ao đó; sửa giá trị này sẽ lưu lại cho ao đang chọn. Quyết định dùng cặp `SA_DOS_Ax`/`SA_DOS_Bx` nào. Không dùng ở mode 0. |

Slot 9-23 (`SA_DOS_V`, `SA_DOS_F1-7`, `SA_DOS_D1-7`) **đã gỡ bỏ 2026-10-06** — xem ghi chú bên dưới bảng.

| `SA_DISC_CHAN` | 24 | Int8 | 0 | 0 | 16 | **Mới 2026-09-08.** Kênh servo/ESC đĩa rải ly tâm (1-indexed) — ESC RIÊNG với `SA_DOS_CHAN` (trục vít). `0` = tắt hẳn tính năng đĩa rải, không ghi PWM ra bất kỳ kênh nào. |
| `SA_DISC_PCT` | 25 | Int8 | 100 | 0 | 100 | **Đổi ý nghĩa 2026-10-06.** % tốc độ đĩa khi đang chạy, tính từ **TRIM (1500, điểm dừng)**: `0` = TRIM (dừng), `100` = lệch hết cỡ về phía `SA_DISC_REV` quy định (MAX/2200 nếu REV=0, MIN/800 nếu REV=1). Lúc không chạy/cấu hình sai luôn là TRIM (trước đây là MIN). |
| `SA_DISC_DLY` | 26 | Float | 2.0 | 0 | 10 | **Mới 2026-09-08.** Độ trễ (giây) giữa đĩa và trục vít lúc bật/tắt `SA_DOS_RC`: bật → đĩa quay ngay, trục vít chờ đủ `SA_DISC_DLY` giây mới chạy; tắt → trục vít dừng ngay, đĩa quay thêm `SA_DISC_DLY` giây rồi mới dừng. Chỉ có tác dụng khi `SA_DISC_CHAN > 0`. |
| `SA_DISC_REV` | 41 | Int8 | 0 | 0 | 1 | **Mới 2026-10-06.** Chiều lệch khỏi TRIM khi tăng `SA_DISC_PCT`: `0`=thuận (lệch về MAX/2200), `1`=ngược (lệch về MIN/800). Đổi chiều quay thật cần đảo dây động cơ/ESC; tham số này chỉ chỉnh hướng PWM cho khớp đúng chiều đã đấu. |
| `SA_DOS_A1..A7` | 27–33 | Float | 0.0 | -50 | 50 | **Mới 2026-10-05, DUY NHẤT công thức từ 2026-10-06.** Hệ số góc `a` (g/phút trên mỗi µs) từ hồi quy tuyến tính `Q=a×PWM+b` đo trực tiếp bằng cân — RIÊNG theo loại thức ăn. `=0` (mặc định) → loại đó **motor dừng an toàn (1500), không chạy** — bắt buộc phải hiệu chuẩn trước khi dùng, không còn fallback nào khác. Xem mục 3.6 quy trình đo. |
| `SA_DOS_B1..B7` | 34–40 | Float | 0.0 | -5000 | 5000 | **Mới 2026-10-05.** Hệ số chặn `b` (g/phút) từ cùng hồi quy với `SA_DOS_Ax` — chỉ có ý nghĩa khi `SA_DOS_Ax≠0` tương ứng. |

> **Param đã bị gỡ bỏ (không còn tồn tại):** `SA_DOS_RATE` (slot 27, cũ) — trước đây là thể tích vít tải dùng riêng cho mode tốc độ cố định, đã gộp vào `SA_DOS_Fx`/`SA_DOS_Dx` từ 2026-08-19 rồi chính `SA_DOS_Fx`/`SA_DOS_Dx` cũng bị gỡ bỏ luôn 2026-10-06 (xem ngay dưới). Slot 27 hiện bỏ trống, không tái sử dụng.
>
> **⚠️ Đã gỡ bỏ HOÀN TOÀN `SA_DOS_V`/`SA_DOS_F1-7`/`SA_DOS_D1-7` (slot 9-23, 2026-10-06):** công thức gián tiếp `V×Fx×Dx` (mô tả lịch sử phát triển ở đoạn cũ bên dưới) đã bị thay thế hẳn bằng hồi quy tuyến tính đo trực tiếp `SA_DOS_A1-7`/`SA_DOS_B1-7` — xem mục 3.5/3.6. Slot 9-23 bỏ trống vĩnh viễn, không tái sử dụng. Đoạn mô tả lịch sử "đổi ý nghĩa 2026-08-19" bên dưới **chỉ còn giá trị tham khảo lịch sử**, các param nhắc tới trong đó không còn tồn tại.
>
> <details><summary>Lịch sử (2026-08-19, trước khi gỡ bỏ hoàn toàn 2026-10-06)</summary>
>
> `SA_DOS_Fx` trước đây LÀ trực tiếp thể tích vít tải hiệu dụng (mL/50us, mặc định 100.0, đo bằng cách chạy motor + cân sản lượng). Từ bản 2026-08-19, `SA_DOS_Fx` đổi thành **hệ số điền đầy hạt** (không thứ nguyên, mặc định 1.0), và phần hình học tách riêng ra tham số mới `SA_DOS_V` (mL/50us, mặc định 100.0). Firmware từng có cảnh báo tự động qua STATUSTEXT nếu phát hiện `SA_DOS_Fx > 5.0` — cảnh báo này cũng đã bị gỡ cùng lúc với `SA_DOS_Fx`.
>
> </details>
>
> **⚠️ Đổi số thứ tự mode + thêm mode 0 mới (từ bản 2026-08-20):** thêm mode
> **PWM trực tiếp** (số 0) để hiệu chuẩn tại bàn (nhập thẳng xung PWM vào
> `SA_DOS_SP`, xuất thẳng ra servo, không qua bất kỳ công thức nào ở trên).
> 2 mode cũ đẩy số lên: tốc độ cố định (cũ = 0) → **1**; tỉ lệ mission (cũ
> = 1) → **2**. **Máy đã cấu hình sẵn `SA_DOS_MODE=1` (mission cũ) sẽ tự
> động đổi ý nghĩa thành "tốc độ cố định" sau khi cập nhật firmware — bắt
> buộc kiểm tra/chỉnh lại giá trị này trên máy đã triển khai.**
>
> **Tham số dùng chung với Module 2:** `SA_POND_IDX` (slot 59) chọn ao — quyết định `SA_DOS_SP`/`SA_DOS_FOOD` đang áp dụng (xem `_sync_dosing_setpoint()` và MODULE2_PH_DETAIL_DESIGN.md).
>
> **⚠️ Đổi hệ đánh số slot (2026-09-07):** Module 3 được tách thành class
> `AP_ShoesAgtech_DosingParams` riêng (xem MODULE1_FLOW_DETAIL_DESIGN.md
> mục kiến trúc, hoặc code `AP_ShoesAgtech.h`), có bảng 64 slot **riêng**
> thay vì dùng chung 64 slot với toàn bộ thư viện như trước — cột "Slot"
> trong bảng trên là số MỚI (1-26), không còn là 13/19/25-54 như tài liệu
> cũ. Tên tham số hiển thị (`SA_DOS_CHAN`...) không đổi. Đĩa rải ly tâm
> (`SA_DISC_*`, slot 24-26) là tính năng mới thêm ngay sau khi tách,
> chiếm 3 trong số ~38 slot còn trống của bảng riêng này.

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

Không có init riêng. `_check_dosing_config()` và `_sync_dosing_setpoint()` chạy lần đầu khi `_update_dosing_motor()` được gọi (chu kỳ đầu tiên của `update()`).

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

### 3.4 Đồng bộ setpoint/loại thức ăn theo ao

```
Nguồn sự thật: _ponds[_active_pond_idx].dos_sp / .dos_food  (lưu trên SD, xem Module 2)
Giao diện người dùng: SA_DOS_SP / SA_DOS_FOOD (param, hiển thị/sửa trên GCS)

Mỗi chu kỳ _sync_dosing_setpoint():
    ao chưa hợp lệ         → không đồng bộ gì, dùng thẳng SA_DOS_SP/SA_DOS_FOOD
    vừa đổi ao             → nạp dos_sp/dos_food của ao mới LÊN SA_DOS_SP/SA_DOS_FOOD
    cùng ao, giá trị đổi   → GHI XUỐNG dos_sp/dos_food của ao đang active, đánh dấu lưu SD
```

### 3.5 Công thức tính PWM

> **⚠️ Thay hoàn toàn công thức tính PWM (2026-10-06):** công thức gián
> tiếp cũ dựa trên `SA_DOS_V × SA_DOS_Fx × SA_DOS_Dx` (3 hằng số nhân
> nhau, giả định đường thẳng luôn đi qua gốc tọa độ PWM=1500⇔lưu
> lượng=0) đã **bị gỡ bỏ hoàn toàn**, cùng với `SA_DOS_V`/`SA_DOS_F1-7`/
> `SA_DOS_D1-7` (slot 9-23, bỏ trống vĩnh viễn — xem mục 2). Thay bằng
> **hồi quy tuyến tính đo trực tiếp bằng cân** `SA_DOS_A1-7`/`SA_DOS_B1-7`
> — xem lý do và quy trình đo ở mục 3.6.

**DOS_MODE=0 (PWM trực tiếp — không đổi):**
```
dos_sp_active ≤ 0   → _dos_pwm = 1500                                  (an toàn, giá trị mặc định chưa chỉnh)
dos_sp_active > 0   → _dos_pwm = constrain(dos_sp_active, SERVOx_MIN, SERVOx_MAX)

Ví dụ: SA_DOS_SP = 1300  → _dos_pwm = 1300 (giả sử nằm trong SERVOx_MIN..MAX)
```
Không qua `SA_DOS_Ax`/`SA_DOS_Bx` (không có khái niệm tốc độ/thức ăn ở
mode này) và không qua `SA_DOS_REV` (giá trị tuyệt đối, không phải PWM
suy ra từ hồi quy) — `SA_DOS_SP` ở mode này chính là con số PWM sẽ xuất
ra, dùng để hiệu chuẩn tại bàn (đo khối lượng ở PWM biết trước — chính
là bước đo dữ liệu cho mục 3.6) mà không cần công cụ test servo riêng
của GCS.

**DOS_MODE=1 (tốc độ cố định — `SA_DOS_SP` LÀ tốc độ, g/phút):**
```
food_idx = clamp(dos_food_active, 1, 7) − 1
dos_gpm  = dos_sp_active
pwm      = _dos_rate_to_pwm(dos_gpm, food_idx)     (xem công thức bên dưới)
```

**DOS_MODE=2 (tỉ lệ mission — `SA_DOS_SP` là TỔNG gam cho cả tuyến):**
```
speed_ms     = _get_spray_speed()   (dùng chung Module 1: SIM > SA_FLOW_VEL > AHRS groundspeed)
mission_dist = _get_mission_dist()  (dùng chung Module 1, xem MODULE1_FLOW_DETAIL_DESIGN)

SA_SPD_START == 0:
    speed_min_start = 0.05 m/s                        (tắt hẳn kiểm tra %, y hệt hành vi cũ)
SA_SPD_START > 0 (mặc định 80):
    speed_min_start = max(0.05 m/s, (SA_SPD_START/100) × _target_speed)
    (_target_speed = tốc độ ĐẶT cho mission, WP_SPEED, dùng chung với Module 1 — xem set_target_speed())

Điều kiện đủ: mission_dist > 1.0m AND speed_ms ≥ speed_min_start

Đủ:    dos_gpm = (dos_sp_active × speed_ms × 60) / mission_dist   (g/phút)
       pwm     = _dos_rate_to_pwm(dos_gpm, food_idx)
Không đủ: dos_gpm = 0 → pwm = 1500 + warn mỗi 5s
```

> **Ngưỡng bắt đầu rải (từ 2026-08-19):** trước đây chỉ cần `speed_ms ≥
> 0.05m/s` (gần như bất kỳ chuyển động nào) là bắt đầu rải. Từ nay mặc định
> phải đạt **≥ `SA_SPD_START`% tốc độ ĐẶT cho mission** (mặc định 80%)
> mới bắt đầu — tránh rải dồn thức ăn vào đoạn xe còn đang tăng tốc từ lúc
> dừng hoặc mới qua khúc cua (xe đi chậm ở đoạn đó lâu hơn nên nếu rải ngay
> sẽ bị dồn liều). Đặt `SA_SPD_START=0` để tắt hẳn kiểm tra này, quay về
> đúng hành vi cũ (chỉ cần vượt sàn tuyệt đối 0.05 m/s). Sàn 0.05 m/s luôn
> áp dụng bất kể `SA_SPD_START`, kể cả khi `_target_speed` chưa có (vd
> chưa từng vào Auto). Vẫn dùng `speed_ms` (tốc độ GPS tức thời) cho công
> thức `dos_gpm`, chỉ đổi NGƯỠNG bắt đầu, không đổi công thức tính tốc độ
> rải.

**`_dos_rate_to_pwm(dos_gpm, food_idx)` — công thức DUY NHẤT cho mode 1/2 (2026-10-06):**
```
a = SA_DOS_Ax[food_idx]

|a| < 0.0001 (CHƯA hiệu chuẩn, mặc định):
    pwm = 1500                                    (dừng an toàn) + warn mỗi 5s

|a| ≥ 0.0001 (đã hiệu chuẩn bằng hồi quy tuyến tính thật):
    b          = SA_DOS_Bx[food_idx]
    pwm_calib  = (dos_gpm − b) / a                (PWM TUYỆT ĐỐI, không phải offset)
    SA_DOS_REV=0 (thuận): pwm_calib = 3000 − pwm_calib    (đối xứng qua 1500)
    SA_DOS_REV=1 (ngược): pwm_calib giữ nguyên
    pwm = constrain(pwm_calib, 800, 2200)

→ SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, pwm)
```

Ví dụ bằng đúng số liệu hiệu chuẩn thật loại 1 (`a=1.520350`, `b=−2196.423077`, REV=1):
```
dos_gpm = 500 g/phút
pwm_calib = (500 − (−2196.423077)) / 1.520350 ≈ 1774.5
REV=1 → pwm = constrain(1774.5, 800, 2200) = 1775µs
```

### 3.6 Workflow calibrate

> **⚠️ Quy trình cũ (2 bước: calib `SA_DOS_V` hình học + calib `SA_DOS_Fx`
> hệ số điền đầy) đã bị loại bỏ hoàn toàn 2026-10-06**, cùng với việc gỡ
> bỏ `SA_DOS_V`/`SA_DOS_Fx`/`SA_DOS_Dx`. Quy trình duy nhất còn lại là
> hồi quy tuyến tính đo trực tiếp bằng cân dưới đây — **bắt buộc** phải
> làm cho MỌI loại thức ăn muốn dùng DOS_MODE=1/2 (không còn fallback).

**Hiệu chuẩn `SA_DOS_Ax`/`SA_DOS_Bx` (hồi quy tuyến tính thật) — làm RIÊNG
cho từng loại thức ăn:**

```
1. SA_DOS_MODE=0 (PWM trực tiếp)
2. Với ≥ 10 mức PWM khác nhau trong dải servo thật (vd 1650→2200, bước
   50us) — BỎ QUA mức nào bị kẹt hạt/dính cơ khí (không phải lỗi đo, chỉ
   là điểm dữ liệu xấu, loại khỏi hồi quy):
   a. SA_DOS_SP = mức PWM đó
   b. Chạy ĐÚNG 1 phút, cân khối lượng thức ăn loại x thu được (gam)
   c. Ghi lại cặp (PWM, gam/phút)
3. Hồi quy tuyến tính Q = a×PWM + b trên toàn bộ các cặp đã đo (Excel/
   Google Sheets: SLOPE()/INTERCEPT(), hoặc LINEST()) — kiểm tra R² đủ
   cao (≥0.95) mới tin dùng, càng gần 1.0 càng tốt
4. SA_DOS_Ax = a (hệ số góc), SA_DOS_Bx = b (hệ số chặn)
5. Xong → đổi SA_DOS_MODE về 1 hoặc 2 để vận hành thật (mode 0 chỉ dùng
   để hiệu chuẩn, không dùng khi cho ăn thật)
```

Ví dụ dữ liệu thật đã đo cho loại 1: `a=1.520350`, `b=−2196.423077`,
R²=0.997251 (12 điểm đo, dải PWM 1650-2200, bỏ 2 mức 1550/1600 do kẹt
hạt). Xem công thức sử dụng kết quả này ở mục 3.5.

> **Vì sao chính xác hơn công thức cũ:** công thức cũ (`V×Fx×Dx`) giả
> định quan hệ PWM↔lưu lượng là tỉ lệ thuận tuyệt đối qua gốc tọa độ
> (PWM=1500 ⇔ lưu lượng=0). Thực tế cơ khí thường có deadband/offset —
> với số liệu loại 1 ở trên, ở PWM=1500 công thức hồi quy cho lưu lượng
> ≈83.6 g/phút chứ không phải 0, nghĩa là công thức cũ (ép qua gốc) sẽ
> sai hệ thống ở vùng PWM thấp. Hồi quy trực tiếp theo đúng dữ liệu đo
> không mắc lỗi này.
>
> **Loại thức ăn nào chưa đo = motor dừng hẳn:** `SA_DOS_Ax=0` (mặc định,
> chưa hiệu chuẩn) → `_dos_rate_to_pwm()` trả về PWM=1500 (dừng an toàn)
> + STATUSTEXT cảnh báo mỗi 5s, KHÔNG còn fallback nào khác — bắt buộc
> phải hoàn thành quy trình trên trước khi dùng loại thức ăn đó ở
> DOS_MODE=1/2.

### 3.7 Xử lý các trường hợp đặc biệt

| Điều kiện | Hành vi | Output GCS |
|---|---|---|
| Bất kỳ 1 trong 4 điều kiện servo sai | Không xuất PWM (giữ 1500) | STATUSTEXT cụ thể lỗi, mỗi 5s |
| `RC_DOS PWM = 0` (mất tín hiệu) | Xuất 1500 (dừng) | data[17]=1500 |
| `SA_DOS_SP = 0` (của ao active), mode 1/2 | dos_rate_gpm = 0 → pwm = (0−SA_DOS_Bx)/SA_DOS_Ax (có thể ≠1500 nếu Bx≠0!) | data[17] tùy Ax/Bx |
| `SA_DOS_SP ≤ 0`, mode 0 | pwm = 1500 (an toàn, không kẹp về pwm_min) | data[17]=1500 |
| `SA_DOS_Ax = 0` (chưa hiệu chuẩn, mode 1/2) — **đổi 2026-10-06** | `_dos_rate_to_pwm()` trả 1500 (dừng an toàn), KHÔNG còn fallback | STATUSTEXT cảnh báo, mỗi 5s |
| `SA_DOS_MODE=2` & dist ≤ 1m | dos_rate_gpm=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_DOS_MODE=2` & speed < speed_min_start | dos_rate_gpm=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_SIM=1` (mode 1/2) | `_get_spray_speed()` trả `_sim_speed` thay `groundspeed()` | Tính toán vẫn chạy bình thường |
| `SA_FLOW_VEL > 0` (DOS_MODE=2) | `_get_spray_speed()` trả giá trị ép này thay `groundspeed()` — dùng để calib DOS_MODE=2 khi xe đứng yên, giống cách calib FLOW_MODE=1 ở Module 1 | Motor chạy như xe đang đi tốc độ đó dù đứng yên |
| Đổi SA_DOS_FOOD (mode 1/2) | Áp dụng ngay cặp SA_DOS_Ax/Bx mới trong chu kỳ kế; lưu lại cho ao active | Log/data[16] đổi theo loại mới |
| Đổi SA_POND_IDX | Nạp lại dos_sp/dos_food đã lưu của ao mới lên SA_DOS_SP/SA_DOS_FOOD | GCS thấy SA_DOS_SP/SA_DOS_FOOD tự đổi theo ao |
| Ao đang active chưa hợp lệ (chưa có GPS) | Dùng thẳng SA_DOS_SP/SA_DOS_FOOD làm giá trị chung, không đồng bộ theo ao | Không có thông báo riêng |

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

`DEBUG_FLOAT_ARRAY`, name=`"SA_DATA"`, array_id=0, ghi trong `send_shoesagtech_debug_arrays()` ở `Rover/GCS_MAVLink_Rover.cpp`. Cả 4 field đều lấy từ **ao đang active** (qua các getter `get_active_dos_sp()`/`get_active_dos_rate()`/`get_active_dos_food()`), trừ `dos_pwm` là giá trị PWM thực tế đang xuất — không phụ thuộc ao.

| Index | Tên | Đơn vị | Điều kiện ghi | Mô tả |
|---|---|---|---|---|
| `data[15]` | `dos_sp` | gam | Luôn | `get_active_dos_sp()`; setpoint của ao đang active (đồng bộ 2 chiều với SA_DOS_SP) |
| `data[16]` | `dos_rate` | g/phút trên mỗi µs ⚠️ đổi ý nghĩa LẦN 2 | Luôn | `get_active_dos_rate()`; trả thẳng `SA_DOS_Ax` (hệ số góc hiệu chuẩn) của loại thức ăn ao active đang dùng |
| `data[17]` | `dos_pwm` | µs | Luôn | PWM thực tế đang xuất; 1500=dừng |
| `data[18]` | `dos_food` | 1–7 | Luôn | `get_active_dos_food()`; loại thức ăn của ao active (đồng bộ 2 chiều với SA_DOS_FOOD) |

> **⚠️ Cần xác nhận lại với app/web (chưa xử lý ở bản này):** `get_active_dos_rate()` (nguồn của `data[16]`) đã đổi ý nghĩa **2 lần**: (1) 2026-08-19 — từ thể tích vít tải mL/50us (~100) sang hệ số điền đầy không thứ nguyên (~1.0); (2) **2026-10-06 — từ hệ số điền đầy sang hệ số góc `SA_DOS_Ax`** (g/phút trên mỗi µs, cỡ số hoàn toàn khác, vd `1.52` theo dữ liệu hiệu chuẩn thật loại 1) khi gỡ bỏ `SA_DOS_Fx`. Field/tên getter không đổi nhưng **giá trị số và đơn vị đã đổi hoàn toàn lần nữa** — nếu companion app/trang web đang hiển thị field này theo ý nghĩa cũ (hệ số điền đầy ~0.05-2.0) hoặc dùng để tính toán gì khác, cần cập nhật lại nhãn/công thức phía app. **Chưa chọn hướng xử lý nào ở bản này** (vẫn chưa cập nhật phía app/web).

### 4.2 DataFlash Log [DATA]

N/A — Module 3 không ghi DataFlash riêng. Trạng thái theo dõi qua SA_DATA và console log.

### 4.3 Console Log

> **Rút gọn (2026-09-04):** đồng bộ theo đúng kiểu Module 1 — đúng 1 dòng
> `[DOS] FM<x> Q:<y>` cho mọi `SA_DOS_MODE`, bỏ hẳn `SERVO<n>`/`ON-OFF`/
> `PWM`/`F<food>`/`D:<density>`/`SP:<sp>` khỏi log định kỳ (vẫn xem đầy đủ
> qua `SA_DATA` nếu cần). Format cũ (3 dòng khác nhau theo mode) xem lịch
> sử ở mục 7.

```
Trigger: mỗi SA_DOS_LOG_MS ms khi SA_DOS_LOG=1

[DOS] FM<x> Q:<y>

<x> = SA_DOS_MODE hiện tại (0/1/2)
<y> = dos_rate_gpm (g/phút):
        mode 0 (PWM trực tiếp) — luôn 0.00, không có khái niệm tốc độ
        mode 1 (tốc độ cố định) — chính dos_sp_active
        mode 2 (tỉ lệ mission) — dos_sp_active × speed × 60 / mission_dist
                                  (0 nếu chưa đủ điều kiện chạy: chưa đủ
                                  quãng đường hoặc chưa đạt SA_SPD_START)
```

Ví dụ:
```
[DOS] FM0 Q:0.00        (PWM trực tiếp, không có khái niệm lưu lượng)
[DOS] FM1 Q:500.00      (tốc độ cố định, đặt 500 g/phút)
[DOS] FM2 Q:1000.27     (tỉ lệ mission, tốc độ tức thời suy ra ≈1000 g/phút)
```

### 4.4 STATUSTEXT — Toàn bộ thông báo

> **Lưu ý:** toàn bộ STATUSTEXT trong code đã được dịch sang tiếng Anh (bản
> 2026-08-15, xem `DEBUG_LOG_GIAI_THICH.md`) — bảng dưới đây liệt kê đúng
> câu chữ tiếng Anh hiện có trong code, kèm giải thích tiếng Việt ở cột Điều
> kiện.

| Nội dung thông báo | Mức | Điều kiện | Tần suất |
|---|---|---|---|
| `SA: SERVO<m> setup OK - dosing motor ready` | INFO | 4 điều kiện OK (edge rising) | 1 lần/lần vừa đúng |
| `SA: SERVO<m> does not exist` | WARNING | Không tìm thấy kênh servo | Mỗi 5s |
| `SA: SERVO<m> FUNCTION=<x>, must set =0 (None)` | WARNING | FUNCTION sai | Mỗi 5s |
| `SA: SERVO<m> MIN=<x>, must set =800` | WARNING | MIN sai | Mỗi 5s |
| `SA: SERVO<m> TRIM=<x>, must set =1500` | WARNING | TRIM sai | Mỗi 5s |
| `SA: SERVO<m> MAX=<x>, must set =2200` | WARNING | MAX sai | Mỗi 5s |
| `SA: SERVO<m> setup OK - spreader disc ready` | INFO | **Đĩa rải (mới 2026-09-08):** FUNCTION/MIN/MAX OK (edge rising, KHÔNG kiểm tra TRIM), chỉ khi `SA_DISC_CHAN>0` | 1 lần/lần vừa đúng |
| `SA: SERVO<m> does not exist` | WARNING | **Đĩa rải:** không tìm thấy kênh servo — dùng chung câu chữ với trục vít, phân biệt qua số kênh `<m>` (`SA_DISC_CHAN` khác `SA_DOS_CHAN`) | Mỗi 5s |
| `SA: SERVO<m> FUNCTION=<x>, must set =0 (None)` | WARNING | **Đĩa rải:** FUNCTION sai | Mỗi 5s |
| `SA: SERVO<m> MIN=<x>, must set =800` | WARNING | **Đĩa rải:** MIN sai (đổi từ 1500→800, 2026-10-06 — đĩa giờ dùng đủ dải 800-2200, điểm dừng là TRIM THẬT đang cấu hình, không bắt buộc đúng 1500) | Mỗi 5s |
| `SA: SERVO<m> MAX=<x>, must set =2200` | WARNING | **Đĩa rải:** MAX sai | Mỗi 5s |
| `SA: Dosing motor ON` | INFO | RC bật (edge rising) | 1 lần/lần bật |
| `SA: Dosing motor OFF` | INFO | RC tắt (edge falling) | 1 lần/lần tắt |
| `SA DOS2: no mission (dist=<x>m) - motor stopped` | WARNING | DOS_MODE=2, dist ≤ 1m (đổi tên từ "SA DOS1" 2026-08-20, khớp số mode mới) | Mỗi 5s |
| `SA DOS2: speed too low (<x>m/s < <y>m/s min) - motor stopped` | WARNING | DOS_MODE=2, speed < max(0.05, `SA_SPD_START`% tốc độ đặt) — cập nhật 2026-08-19, trước đây ngưỡng cố định 0.05 | Mỗi 5s |
| `SA: SA_DOS_A<n> not calibrated - feeder stopped` | WARNING | **Đổi 2026-10-06, thay cảnh báo "Fx uncalibrated" cũ (đã gỡ bỏ):** `SA_DOS_Ax` của loại thức ăn active vẫn =0 (chưa hiệu chuẩn) — motor dừng hẳn (xem mục 3.6) | Mỗi 5s |
| `SA: Feeder RC ready - feeder can start` | INFO | **Gate an toàn khởi động (đổi 2026-09-09, thay ARM/DISARM):** `SA_DOS_RC` đã về vị trí OFF (≤1500) | 1 lần duy nhất mỗi phiên boot |
| `SA: Move SA_DOS_RC to OFF to start feeder` | WARNING | Cùng gate trên: `SA_DOS_RC` chưa về OFF | Mỗi 3s, lặp lại cho tới khi đạt |

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERVOx_FUNCTION = 0     (x = SA_DOS_CHAN)  ┐
SERVOx_MIN      = 800                      │  Sai bất kỳ 1 → KHÔNG chạy + warn 5s
SERVOx_TRIM     = 1500                     │
SERVOx_MAX      = 2200                     ┘

SA_DOS_MODE=2: mission đã upload lên FC VÀ speed ≥ max(0.05m/s, SA_SPD_START% tốc độ đặt WP_SPEED)
               (SA_SPD_START=0 → chỉ còn sàn 0.05m/s, y hệt hành vi cũ)
               thiếu 1 trong 2 → motor dừng (dos_rate_gpm=0)

SA_DOS_Ax (của loại thức ăn active) PHẢI ≠ 0 mới chạy được mode 1/2 — (đổi
               2026-10-06, bắt buộc, không còn clamp/fallback như V/Fx/Dx
               cũ đã gỡ bỏ) =0 → motor dừng hẳn (1500) + warn 5s, xem mục 3.6

Đĩa rải ly tâm (nếu SA_DISC_CHAN > 0, đổi 2026-10-06):
SERVOx_FUNCTION = 0     (x = SA_DISC_CHAN) ┐  Sai bất kỳ 1 → đĩa không quay
SERVOx_MIN      = 800                      │  (PWM giữ nguyên TRIM thật đang
SERVOx_MAX      = 2200                     ┘  cấu hình) + warn 5s. KHÔNG kiểm
                                               tra TRIM (khác trục vít) —
                                               formula vẫn tự đọc TRIM thật
                                               làm điểm 0%, không bắt buộc 1500.
                                               (KHÔNG chặn trục vít chạy)

SA_DOS_MODE=0: SA_DOS_SP là PWM tuyệt đối, tự constrain về đúng dải SERVOx_MIN..MAX
               SA_DOS_SP ≤ 0 → 1500 (an toàn), không kẹp về pwm_min

SA_DOS_SP/SA_DOS_FOOD hiển thị trên GCS luôn phản ánh ao đang chọn (SA_POND_IDX)
— đổi ao sẽ tự đổi giá trị hiển thị, không phải lỗi đồng bộ

An toàn khởi động (thay ARM/DISARM, đổi 2026-09-09): SA_DOS_RC phải về
OFF (≤1500) — kiểm tra mỗi 3 giây kể từ boot cho tới khi đạt
(_dos_rc_seen_off, dùng LUÔN cờ có sẵn từ cơ chế chặn-tự-chạy-sau-reboot
2026-09-04, không thêm cờ mới). Chưa đạt → WARNING mỗi 3s + motor/đĩa ép
tắt hoàn toàn. Đã sẵn OFF lúc bật nguồn → tự khởi động sau đúng 3 giây,
không cần thao tác gì. Chỉ cần đạt 1 lần/phiên nguồn — không lặp lại ở
các lần arm/disarm sau đó (độc lập hoàn toàn với ARM/DISARM, không còn
liên quan tới Module 1 nữa — mỗi module tự kiểm tra RC của mình).
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

| Điểm | Basic Design dự kiến | Thực tế đã làm (bản hiện tại) | Lý do |
|---|---|---|---|
| 4 điều kiện servo | Đề cập đủ | Implement đúng theo Basic Design ✓ | — |
| Fail-safe mất tín hiệu RC | RC=0 → dừng | `rc_pwm > 1500` (strict); rc_pwm=0 → motor_on=false | 0 là giá trị RC mất tín hiệu, đảm bảo dừng an toàn |
| Timer cảnh báo | Dùng chung timer warn | Dùng `_dos_warn_ms` riêng cho dosing | Tránh tranh chấp với `_last_warn_ms` của pump module |
| DOS_MODE=2 (tỉ lệ mission), warn label | Đề cập chung | Warn cụ thể lý do (dist hoặc speed) với giá trị thực | Người dùng biết cụ thể vấn đề |
| Công thức tốc độ cố định (DOS_MODE=0) | `g/50us` gộp chung, sau đó tách `vol_rate(mL/50us) × density(g/mL)` với `SA_DOS_RATE` riêng | **Gỡ bỏ hoàn toàn `SA_DOS_RATE`** — DOS_MODE=0 dùng chung `SA_DOS_Fx`/`SA_DOS_Dx` với DOS_MODE=1 | Không còn lý do calib 2 bộ thông số khác nhau cho cùng 1 vít tải; đơn giản hoá cấu hình |
| SA_DOS_F1..F7 | mL/50us theo loại hạt, chỉ dùng ở DOS_MODE=1 | **Dùng cho CẢ 2 mode** | Thống nhất công thức, giảm tham số cần nhớ |
| SA_DOS_Fx — cấu trúc calib (2026-08-19) | Không có — 1 con số/loại thức ăn gộp cả hình học vít và điền đầy hạt | **Tách thành `SA_DOS_V` (mL/50us, DÙNG CHUNG mọi loại thức ăn, đặc tính cơ khí trục vít) × `SA_DOS_Fx` (hệ số điền đầy, không thứ nguyên, RIÊNG theo loại thức ăn)** | Bù đúng bản chất vật lý: trục vít không lấp đầy 100% hạt thức ăn (có khoảng trống không khí) — tách hình học (đo 1 lần) khỏi hệ số điền đầy (đo lại mỗi loại hạt mới) giúp hiệu chuẩn nhanh hơn khi thêm loại thức ăn |
| SA_DOS_D1..D7 | Khối lượng riêng (g/mL) cho 7 loại thức ăn | Không đổi — vẫn 7 loại, mặc định 1.0 | — |
| Setpoint/loại thức ăn theo ao | Không có — 1 giá trị SA_DOS_SP/SA_DOS_FOOD chung toàn hệ thống | **Lưu riêng theo từng ao** (`PondEntry.dos_sp/.dos_food`), đồng bộ 2 chiều qua `_sync_dosing_setpoint()`, tồn tại qua reboot (SD) | Mỗi ao có thể cần lượng/loại thức ăn khác nhau; tránh phải nhập lại tay mỗi lần đổi ao |
| SA_DATA index | data[12..15] | **data[15..18]** (dịch do Module 2 tăng từ 7 lên 10 field) | Module 2 (pond, ph_morn/aft/last_day) chiếm thêm chỗ trong mảng |
| Console log | `F<n> SERVO<m> ON/OFF SP:<sp>g PWM:<pwm>` | Thêm `M<mode>` và `D:<density>g/mL`; SP hiển thị là setpoint của ao active | Người dùng thấy ngay khối lượng riêng và ao đang áp dụng |
| Ngưỡng bắt đầu rải, DOS_MODE=2 (2026-08-19) | Ngưỡng cố định 0.05 m/s (gần như bất kỳ chuyển động nào) | `speed_min_start = max(0.05 m/s, (SA_SPD_START/100) × _target_speed)`, tham số `SA_SPD_START` (0–100, mặc định 50) — `=0` tắt hẳn kiểm tra %, về đúng hành vi cũ | Tránh rải dồn thức ăn vào đoạn xe còn đang tăng tốc từ lúc dừng/qua khúc cua, giống lý do đổi nguồn tốc độ ở Module 1; để dạng tham số vì mỗi xe/mission có thể cần ngưỡng khác nhau |
| Mode PWM trực tiếp + đổi số mode (2026-08-20) | Không có | Thêm `SA_DOS_MODE=0` (PWM trực tiếp — `SA_DOS_SP` xuất thẳng ra servo, bỏ qua V/Fx/Dx/REV); 2 mode cũ đẩy số lên 1 (tốc độ cố định, cũ=0) và 2 (tỉ lệ mission, cũ=1) | Hiệu chuẩn tại bàn (đo RPM/sản lượng ở PWM biết trước) không cần công cụ test servo riêng của GCS; tái sử dụng `SA_DOS_SP` thay vì thêm tham số mới (hết slot — xem mục 2) |
| Chặn motor tự chạy lại sau reboot (2026-09-04) | Không có — motor chạy ngay theo vị trí RC hiện tại, không phân biệt vừa boot hay đang chạy sẵn | Thêm cờ `_dos_rc_seen_off`: sau mỗi lần FC reboot, motor bị ép OFF cho tới khi thấy `SA_DOS_RC` ở vị trí OFF ít nhất 1 lần, dù switch đang ở ON | Bug thật: pin chập chờn khiến FC brown-out/reboot trong lúc switch cho ăn đang ở ON → RC receiver (boot nhanh hơn FC) vẫn báo đúng vị trí ON → motor tự quay lại ngay khi có điện, không ai chủ động bật. Chọn hướng "gạt lại OFF→ON" thay vì bắt buộc ARM để vẫn test/xả thức ăn được lúc DISARM |
| `SA_DOS_SPD_PCT` → `SA_SPD_START`, dùng chung Module 1 (2026-09-04) | Không có | Đổi tên tham số + mặc định `50`→`80`; Module 1 (FLOW_MODE=1, nấc 2/3) giờ dùng CHUNG tham số này để gate bơm bắt đầu chạy (thay cho gate WP1 cũ) qua hàm dùng chung `_speed_min_start()` | Không còn slot AP_Param trống để thêm tham số riêng cho Module 1; đổi tên bỏ tiền tố `DOS_` vì tham số không còn là của riêng Module 3 nữa — đổi `SA_SPD_START` giờ ảnh hưởng cả 2 module |
| Rút gọn log console (2026-09-04) | Không có | Đổi từ 3 format khác nhau theo mode (`M0 SERVO/PWM`, `M1 F/SERVO/Rate/D/PWM`, `M2 F/SERVO/SP/Rate/D/PWM`) còn ĐÚNG 1 dòng `FM<x> Q:<y>` cho mọi mode — `x`=SA_DOS_MODE, `y`=dos_rate_gpm (0 ở mode 0) | Đồng bộ hình thức với log Module 1 (`[FLOW] FM<x> N<nấc> Q:<target>`) theo yêu cầu người dùng; chi tiết SERVO/PWM/mật độ/ao vẫn xem được qua `SA_DATA` khi cần |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE3_DOS_BASIC_DESIGN.md](MODULE3_DOS_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — `_get_mission_dist()` dùng chung, `update()` gọi `_update_dosing_motor()`
- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — `PondEntry`/`SA_POND_IDX` dùng chung, lưu trữ SD qua `_io_update()`
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA (data[15..18])
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
