# Module 3 — Dosing Motor (Vít Tải Thức Ăn Tôm)
## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 10 Hz (`_update_dosing_motor()` gọi từ `update()`)
**Ngày hoàn thành:** 2026-05-15 | **Cập nhật lần cuối:** 2026-08-19

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
                        ├──► food_idx = clamp(dos_food_active, 1, 7) − 1  (0-indexed)
                        ├──► fill_k   = max(SA_DOS_Fx[food_idx], 0.01)    (hệ số điền đầy, không thứ nguyên, RIÊNG theo loại thức ăn)
                        ├──► v_const  = max(SA_DOS_V, 0.1)                (mL/50us, hằng số hình học, DÙNG CHUNG mọi loại thức ăn)
                        ├──► vol_rate = v_const × fill_k                   (mL/50us hiệu dụng)
                        ├──► density  = max(SA_DOS_Dx[food_idx], 0.01)   (g/mL)
                        ├──► effective = vol_rate × density               (g/50us — dùng chung CẢ 2 mode)
                        │
                        ├──► DOS_MODE=0:
                        │       offset = dos_sp_active × 50 / effective
                        │
                        └──► DOS_MODE=1: _get_mission_dist() [dùng chung Module 1]
                                         + _get_spray_speed() [dùng chung Module 1 —
                                           SIM > SA_FLOW_VEL > AHRS groundspeed]
                                         → dos_gpm = dos_sp_active × speed × 60 / dist
                                         offset    = dos_gpm × 50 / effective
                                         (không đủ điều kiện → offset=0 + warn 5s)
                              │
                              └──► Áp dụng SA_DOS_REV → _dos_pwm
                                   SRV_Channels::set_output_pwm_chan()
```

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

**`_clamp_food(int8_t food) → int8_t`** — static
- **File:** `AP_ShoesAgtech.cpp : 1800`
- Kẹp giá trị loại thức ăn về dải hợp lệ 1–7 (khớp `SA_DOS_FOOD` `@Range`). Dùng ở mọi nơi truy cập `_dos_fr[]`/`_dos_dr[]` để tránh index ngoài mảng.

---

**`_offset_to_dos_pwm(float offset) const → uint16_t`**
- **File:** `AP_ShoesAgtech.cpp : 1806`
- **Đầu vào:** `offset` — độ lệch PWM (µs, luôn dương)
- **Xử lý:** Áp dụng chiều quay `SA_DOS_REV`:
  - `= 0` (thuận): `constrain(1500 − offset, 800, 1500)`
  - `= 1` (ngược): `constrain(1500 + offset, 1500, 2200)`
- **Đầu ra / Return:** `uint16_t` PWM đã constrain đúng nửa dải servo

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
  6. Nếu state thay đổi → STATUSTEXT ON/OFF
  7. Nếu `!motor_on` → `_dos_pwm = 1500`; xuất; log; return
  8. `food_idx = clamp(dos_food_active, 1, 7) − 1` (0-indexed, dùng chung cho cả 2 mode)
  9. `fill_k = max(SA_DOS_Fx[food_idx], 0.01)` (hệ số điền đầy hạt, không thứ nguyên, RIÊNG theo loại thức ăn); nếu `fill_k > 5.0` → STATUSTEXT cảnh báo nghi chưa hiệu chuẩn lại theo công thức mới (rate-limit 5s, dùng chung `_dos_warn_ms`); `v_const = max(SA_DOS_V, 0.1)` (mL/50us, hằng số hình học vít tải, DÙNG CHUNG mọi loại thức ăn); `vol_rate = v_const × fill_k`; `density = max(SA_DOS_Dx[food_idx], 0.01)` (g/mL); `effective = vol_rate × density` — **giống hệt công thức cho cả DOS_MODE=0 và 1**
  10. **DOS_MODE=0:** `dos_rate_gpm = dos_sp_active` (SP CHÍNH LÀ tốc độ, g/phút); `offset_us = dos_rate_gpm × 50 / effective`
  11. **DOS_MODE=1:**
      - `mission_dist = _get_mission_dist()`
      - `speed = _get_spray_speed()` — **từ bản này, dùng chung hàm với Module 1** (trước đây tự viết inline `SIM ? _sim_speed : groundspeed()`, thiếu lớp `SA_FLOW_VEL`; nay đủ 3 cấp: SIM > SA_FLOW_VEL > AHRS groundspeed)
      - Điều kiện: `dist > 1.0m && speed ≥ 0.05 m/s`
      - Đủ: `dos_rate_gpm = dos_sp_active × speed × 60 / dist`; `offset_us = dos_rate_gpm × 50 / effective`
      - Không đủ: `dos_rate_gpm = 0`, `offset_us = 0` (pwm=1500) + STATUSTEXT mỗi 5s
  12. `_dos_pwm = _offset_to_dos_pwm(offset_us)` (áp dụng chiều quay SA_DOS_REV)
  13. `SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, _dos_pwm)`
  - `dos_rate_gpm` (biến cục bộ, khởi tạo 0.0f đầu hàm) được giữ lại đến bước in log (14) để hiển thị tốc độ tức thời — không có getter public, chỉ dùng nội bộ cho console log
  14. Console log nếu SA_DOS_LOG=1
- **Đầu ra / Return:** `void` — side effects: `_dos_pwm`, servo output
- **Ghi chú:** Từ bản này, **DOS_MODE=0 và DOS_MODE=1 dùng chung một nguồn thông số** (`SA_DOS_V` dùng chung + `SA_DOS_Fx`/`SA_DOS_Dx` theo loại thức ăn) — không còn tham số `SA_DOS_RATE` riêng cho mode cố định. `SA_DOS_Fx` đổi ý nghĩa từ bản này (xem mục 2 và 3.5).

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_DOS_CHAN` | 25 | Int8 | 10 | 1 | 16 | Kênh servo đầu ra motor (1-indexed). Phải đúng 4 điều kiện SERVO. |
| `SA_DOS_RC` | 26 | Int8 | 8 | 1 | 16 | Kênh RC bật/tắt motor. PWM>1500 → bật; ≤1500 hoặc =0 (mất tín hiệu) → tắt. |
| `SA_DOS_SP` | 28 | Float | 0.0 | 0 | — | Setpoint lượng thức ăn (gam) **cho ao đang chọn** (`SA_POND_IDX`). Đổi ao sẽ nạp lại giá trị đã lưu của ao đó; sửa giá trị này sẽ lưu lại cho ao đang chọn. |
| `SA_DOS_REV` | 29 | Int8 | 0 | 0 | 1 | Chiều quay: **0**=thuận (800–1500), **1**=ngược (1500–2200). |
| `SA_DOS_LOG` | 30 | Int8 | 0 | 0 | 1 | Bật (1) console log dosing motor theo chu kỳ `SA_DOS_LOG_MS`. |
| `SA_DOS_LOG_MS` | 31 | Int16 | 1000 | 100 | 60000 | Chu kỳ console log dosing (ms). |
| `SA_DOS_MODE` | 36 | Int8 | 0 | 0 | 1 | **0**=tốc độ cố định, **1**=tỉ lệ theo vận tốc + mission. Cả 2 mode đều dùng chung `SA_DOS_Fx`/`SA_DOS_Dx`. |
| `SA_DOS_FOOD` | 40 | Int8 | 1 | 1 | 7 | Loại thức ăn đang dùng **cho ao đang chọn**. Đổi ao sẽ nạp lại giá trị đã lưu của ao đó; sửa giá trị này sẽ lưu lại cho ao đang chọn. |
| `SA_DOS_V` | 13 | Float | 100.0 | 0.1 | 10000 | **Hằng số hình học vít tải (mL/50us, ở fill=100%)** — DÙNG CHUNG cho MỌI loại thức ăn, chỉ đổi khi thay trục vít vật lý khác. Tính từ đường kính cánh vít, đường kính trục và bước ren, hoặc đo thực nghiệm (xem mục 3.5, 3.6). |
| `SA_DOS_F1..F7` | 41–47 | Float | 1.0 | 0.05 | 2.0 | **Hệ số điền đầy hạt (không thứ nguyên) theo loại thức ăn** — dùng cho CẢ DOS_MODE=0 và 1. Bù cho khoảng trống không khí giữa các hạt thức ăn trong vít (hạt to/không đều → hệ số thấp hơn). Lưu lượng hiệu dụng = `SA_DOS_V × SA_DOS_Fx`. |
| `SA_DOS_D1..D7` | 48–54 | Float | 1.0 | 0.1 | 5.0 | **Khối lượng riêng (g/mL) theo loại thức ăn** — mặc định 1.0. Đo: đổ đầy 1000mL, cân → chia 1000. Thức ăn tôm viên thực tế ≈ 0.5–0.7 g/mL. |

> **Param đã bị gỡ bỏ (không còn tồn tại):** `SA_DOS_RATE` (slot 27, cũ) — trước đây là thể tích vít tải dùng riêng cho DOS_MODE=0. Từ bản này, DOS_MODE=0 dùng chung `SA_DOS_Fx`/`SA_DOS_Dx` (theo `SA_DOS_FOOD`) với DOS_MODE=1 — không còn công thức/param riêng cho từng mode. Slot 27 hiện bỏ trống, không tái sử dụng.
>
> **⚠️ Đổi ý nghĩa tham số (từ bản 2026-08-19):** `SA_DOS_Fx` trước đây LÀ trực tiếp thể tích vít tải hiệu dụng (mL/50us, mặc định 100.0, đo bằng cách chạy motor + cân sản lượng). Từ bản này, `SA_DOS_Fx` đổi thành **hệ số điền đầy hạt** (không thứ nguyên, mặc định 1.0), và phần hình học tách riêng ra tham số mới `SA_DOS_V` (mL/50us, mặc định 100.0 — giữ đúng hành vi mặc định `V × Fx = 100 × 1.0 = 100` như số mặc định cũ, nên máy MỚI/CHƯA hiệu chuẩn không bị ảnh hưởng). **Máy đã hiệu chuẩn `SA_DOS_Fx` theo công thức CŨ (giá trị ~100) bắt buộc phải hiệu chuẩn lại** sau khi cập nhật firmware này, nếu không lượng thức ăn cấp ra sẽ sai (thường là quá ít, vì lưu lượng hiệu dụng bị tính vọt lên gấp nhiều lần thực tế) — xem quy trình hiệu chuẩn mới ở mục 3.6. Firmware có cảnh báo tự động qua STATUSTEXT nếu phát hiện `SA_DOS_Fx > 5.0` (xem mục 3.7, 4.4) nhưng **không thay thế được việc hiệu chuẩn lại thủ công**, chỉ là lưới an toàn phát hiện trường hợp rõ ràng chưa cập nhật.
>
> **Tham số dùng chung với Module 2:** `SA_POND_IDX` (slot 59) chọn ao — quyết định `SA_DOS_SP`/`SA_DOS_FOOD` đang áp dụng (xem `_sync_dosing_setpoint()` và MODULE2_PH_DETAIL_DESIGN.md).

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

**Tách biệt hình học vít tải, hệ số điền đầy hạt, và mật độ hạt (dùng chung cho cả 2 mode):**
```
v_const   = hằng số hình học vít tải (mL/50us, ở fill=100%) — SA_DOS_V
           Đặc tính THUẦN CƠ KHÍ của trục vít: tính/đo 1 lần, DÙNG CHUNG cho
           MỌI loại thức ăn, chỉ đổi khi thay trục vít vật lý khác.

fill_k    = hệ số điền đầy hạt (không thứ nguyên, ~0.05–2.0) — SA_DOS_Fx
           (x = SA_DOS_FOOD của ao active)
           Đặc tính RIÊNG của từng loại thức ăn: bù cho khoảng trống không
           khí giữa các hạt thức ăn trong vít — trục vít KHÔNG THỂ lấp đầy
           100% hạt thức ăn thực tế như v_const giả định (lý thuyết). Thay
           loại hạt thì cập nhật giá trị này.

vol_rate  = v_const × fill_k   (mL/50us hiệu dụng, đã trừ khoảng trống hạt)

density   = khối lượng riêng hạt (g/mL) — SA_DOS_Dx
           Đặc tính của loại thức ăn: thay loại hạt thì cập nhật giá trị này.

effective = vol_rate × density   (g/phút — lượng dispensed khi offset đúng bằng 50µs)
```

> **Lưu ý đơn vị:** dù tên gọi "mL/50us" gợi ý một hằng số hình học thuần túy, `vol_rate` (và trước đây là `SA_DOS_Fx` khi còn mang trực tiếp ý nghĩa này) thực chất được calib để `effective = vol_rate × density` ra đúng **gam/phút** đạt được ở offset=50µs (xem `dos_gpm` bên dưới, đơn vị g/phút, dùng chung `effective` này). Vì vậy khi calib (mục 3.6) phải đo trong đúng 1 phút.

**Cách tính `SA_DOS_V` từ phần cứng trục vít** (đo/tính lại 1 lần mỗi khi
thay trục vít khác — hoàn toàn không phụ thuộc loại thức ăn):

1. **Thể tích quét lý thuyết mỗi vòng quay** (thuần hình học vít tải):
   ```
   V_per_rev(mL/vòng) = (π/4) × (D_ngoài² − D_trục²) × pitch
   ```
   - `D_ngoài` = đường kính cánh vít (cm, đo trực tiếp trên trục vít).
   - `D_trục` = đường kính trục/lõi giữa vít (cm); = 0 nếu vít không có
     trục giữa (vít dạng lò xo/shaftless).
   - `pitch` = bước ren — khoảng cách DỌC TRỤC vít di chuyển được sau đúng
     1 vòng quay (cm/vòng, đo trên chính trục vít).
   - (1 cm³ = 1 mL nên công thức trên ra thẳng đơn vị mL/vòng.)

2. **Hệ số vòng quay theo PWM offset** của riêng bộ động cơ/servo đang lắp
   (`RPM_per_50us`, vòng/phút ứng với mỗi 50µs PWM lệch tâm 1500) — phải
   **đo thực tế** bằng máy đo vòng quay (tachometer) ở vài mức PWM offset
   khác nhau rồi lấy độ dốc trung bình. Đây là đặc tính của bộ động
   cơ/servo, KHÔNG suy được từ hình học vít.

3. **`SA_DOS_V (mL/50us) = V_per_rev × RPM_per_50us`**

**Ví dụ bằng số** (khớp với giá trị mặc định 100.0): vít
`D_ngoài=3.0cm, D_trục=0.8cm, pitch=2.5cm`
```
V_per_rev = 0.785 × (3.0² − 0.8²) × 2.5 = 0.785 × 8.36 × 2.5 ≈ 16.4 mL/vòng
```
Đo được động cơ quay ~6.1 vòng/phút ứng với mỗi 50µs PWM offset:
```
SA_DOS_V = 16.4 × 6.1 ≈ 100 mL/50us
```

`SA_DOS_V` là giá trị **lý thuyết ở fill=100%** (vít lấp đầy hoàn toàn,
không khoảng trống) — sai lệch giữa số này và thực tế đo được (do khoảng
trống không khí giữa các hạt thức ăn) chính là phần `SA_DOS_Fx` xử lý riêng
theo từng loại thức ăn (xem quy trình hiệu chuẩn ở mục 3.6).

**DOS_MODE=0 (tốc độ cố định):**
```
food_idx = clamp(dos_food_active, 1, 7) − 1
fill_k   = max(SA_DOS_Fx[food_idx], 0.01)  (hệ số điền đầy, không thứ nguyên)
v_const  = max(SA_DOS_V, 0.1)              (mL/50us, hằng số hình học, DÙNG CHUNG)
vol_rate = v_const × fill_k                (mL/50us hiệu dụng, ngầm định /phút — xem lưu ý trên)
density  = max(SA_DOS_Dx[food_idx], 0.01)  (g/mL)
offset_us = dos_sp_active(g) × 50 / (vol_rate × density)

Ví dụ: SP=500, V=100mL/50us, F1=1.0 (fill_k), D1=0.6g/mL
  → vol_rate = 100 × 1.0 = 100
  → offset = 500 × 50 / (100 × 0.6) = 416.7µs
  → pwm = 1500 - 417 = 1083µs (DOS_REV=0)
```

> **Ý nghĩa thực sự của `SA_DOS_SP` ở mode này:** công thức KHÔNG có bước quy đổi "tổng gam → tốc độ" như DOS_MODE=1 (không chia cho mission_dist/speed) — `dos_sp_active` được đưa thẳng vào đúng vị trí mà `dos_gpm` (g/phút) chiếm ở DOS_MODE=1. Vì `effective` có đơn vị g/phút (ở offset=50µs), để `offset_us` ra đúng đơn vị µs thì `dos_sp_active` **bắt buộc phải được hiểu là một tốc độ liên tục, gam/phút** — KHÔNG phải "tổng gam rồi motor tự dừng" như mô tả `@Description` gợi ý. Motor ở mode 0 quay liên tục ở offset không đổi cho đến khi RC tắt; không có cơ chế "đã cấp đủ SP gam thì dừng". Ví dụ trên: SP=500 nghĩa là "cấp liên tục ~500g/phút", không phải "cấp 500g rồi ngừng".

**DOS_MODE=1 (tỉ lệ mission) — cùng vol_rate/density như trên:**
```
speed_ms     = _get_spray_speed()   (dùng chung Module 1: SIM > SA_FLOW_VEL > AHRS groundspeed)
mission_dist = _get_mission_dist()  (dùng chung Module 1, xem MODULE1_FLOW_DETAIL_DESIGN)

Điều kiện đủ: mission_dist > 1.0m AND speed_ms ≥ 0.05m/s

dos_gpm   = (dos_sp_active × speed_ms × 60) / mission_dist   (g/phút)
offset_us = dos_gpm × 50 / (vol_rate × density)               (µs)

Không đủ: offset_us = 0 → pwm = 1500 + warn mỗi 5s
```

**Áp dụng chiều quay:**
```
SA_DOS_REV=0 (thuận):  pwm = constrain(1500 − offset_us,  800, 1500)
SA_DOS_REV=1 (ngược):  pwm = constrain(1500 + offset_us, 1500, 2200)
→ SRV_Channels::set_output_pwm_chan(SA_DOS_CHAN − 1, pwm)
```

### 3.6 Workflow calibrate

**Bước 1 — Calibrate `SA_DOS_V` (hằng số hình học vít tải) — làm 1 LẦN khi
lắp hoặc thay trục vít khác, KHÔNG phụ thuộc loại thức ăn:**

Cách A — tính từ hình học (công thức đầy đủ ở mục 3.5):
```
1. Đo D_ngoài, D_trục, pitch trực tiếp trên trục vít (cm)
2. Tính V_per_rev = (π/4) × (D_ngoài² − D_trục²) × pitch   (mL/vòng)
3. Đo RPM_per_50us bằng tachometer ở vài mức PWM offset, lấy độ dốc trung bình
4. SA_DOS_V = V_per_rev × RPM_per_50us
```

Cách B — đo thực nghiệm bằng vật liệu chảy tự do gần lấp đầy 100% (hạt mịn,
đều, không kết dính — coi tạm `fill_k ≈ 1.0`):
```
1. Đặt tạm SA_DOS_Fx = 1.0 (mọi loại), SA_DOS_REV đúng chiều, DOS_SP=0
2. DOS_MODE=0, đặt DOS_SP = giá trị thử nghiệm bất kỳ (vd 500 g/phút)
3. Chạy motor ĐÚNG 1 PHÚT ở offset đó, dùng vật liệu chảy tự do trên
   → đo thể tích thu được Vt (mL) trong 1 phút đó
4. SA_DOS_V = Vt(mL/phút) × 50us / offset_thực_us
```

**Bước 2 — Calibrate `SA_DOS_Fx` (hệ số điền đầy hạt) — làm RIÊNG cho MỖI
loại thức ăn mới, làm SAU khi đã có `SA_DOS_V` đúng ở Bước 1:**
```
1. SA_DOS_V đã có giá trị đúng (Bước 1), SA_DOS_REV đúng chiều, DOS_SP=0
2. Chọn SA_DOS_FOOD = x (loại đang calib), đặt tạm SA_DOS_Fx = 1.0, SA_DOS_Dx = 1.0
3. DOS_MODE=0, đặt DOS_SP = giá trị thử nghiệm bất kỳ (vd 500) — nhớ: SP ở mode 0
   là TỐC ĐỘ (g/phút), không phải tổng gam, nên chỉ dùng để tạo ra 1 offset_thực_us cố định
4. Chạy motor ĐÚNG 1 PHÚT ở offset đó → thu thức ăn thật (loại x) vào bình đong
   → đo thể tích Vx (mL) và khối lượng M (g) thu được TRONG 1 PHÚT đó
   (bắt buộc đúng 1 phút vì hiệu dụng được định nghĩa theo đơn vị g/phút — xem mục 3.5)
5. density_actual = M / Vx  → cập nhật SA_DOS_Dx
6. Fx_actual = [Vx(mL/phút) × 50us / offset_thực_us] / SA_DOS_V → cập nhật SA_DOS_Fx
```

> **Lợi ích của việc tách 2 bước:** trước đây mỗi loại thức ăn mới phải đo
> lại TOÀN BỘ `SA_DOS_Fx` (thể tích vít tải tuyệt đối) từ đầu. Từ nay,
> Bước 1 chỉ làm 1 lần cho cả trục vít; thêm loại thức ăn mới chỉ cần lặp
> lại Bước 2 (nhanh hơn, ít bước hơn).

**Calibrate SA_DOS_Dx (khối lượng riêng hạt) — làm khi đổi loại hạt (thuộc
Bước 2, không đổi ở Bước 1):**
```
1. Đổ đầy bình đong 1000mL bằng hạt thức ăn loại x
2. Cân bình → trừ tara → được M(g)
3. SA_DOS_Dx = M / 1000
VD: cân được 620g → SA_DOS_D1 = 0.62
```

### 3.7 Xử lý các trường hợp đặc biệt

| Điều kiện | Hành vi | Output GCS |
|---|---|---|
| Bất kỳ 1 trong 4 điều kiện servo sai | Không xuất PWM (giữ 1500) | STATUSTEXT cụ thể lỗi, mỗi 5s |
| `RC_DOS PWM = 0` (mất tín hiệu) | Xuất 1500 (dừng) | data[17]=1500 |
| `SA_DOS_SP = 0` (của ao active) | offset_us = 0 → pwm = 1500 | data[17]=1500 |
| `SA_DOS_Fx < 0.01` hoặc `SA_DOS_V < 0.1` hoặc `SA_DOS_Dx < 0.01` | Clamp về giá trị min → tránh chia cho 0 | PWM tính được (không crash) |
| `SA_DOS_Fx > 5.0` (nghi vẫn còn giá trị cũ, chưa hiệu chuẩn lại theo công thức `V × Fx`) | Vẫn tính PWM bình thường theo giá trị hiện tại — KHÔNG tự sửa | STATUSTEXT cảnh báo, mỗi 5s |
| `SA_DOS_MODE=1` & dist ≤ 1m | offset_us=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_DOS_MODE=1` & speed < 0.05 | offset_us=0 → motor dừng | STATUSTEXT warning mỗi 5s |
| `SA_SIM=1` | `_get_spray_speed()` trả `_sim_speed` thay `groundspeed()` | Tính toán vẫn chạy bình thường |
| `SA_FLOW_VEL > 0` (DOS_MODE=1) | `_get_spray_speed()` trả giá trị ép này thay `groundspeed()` — dùng để calib DOS_MODE=1 khi xe đứng yên, giống cách calib FLOW_MODE=1 ở Module 1 | Motor chạy như xe đang đi tốc độ đó dù đứng yên |
| Đổi SA_DOS_FOOD | Áp dụng ngay SA_DOS_Dx và SA_DOS_Fx mới trong chu kỳ kế; lưu lại cho ao active | Log hiển thị F<n> và D:<x>g/mL mới |
| Đổi SA_POND_IDX | Nạp lại dos_sp/dos_food đã lưu của ao mới lên SA_DOS_SP/SA_DOS_FOOD | GCS thấy SA_DOS_SP/SA_DOS_FOOD tự đổi theo ao |
| Ao đang active chưa hợp lệ (chưa có GPS) | Dùng thẳng SA_DOS_SP/SA_DOS_FOOD làm giá trị chung, không đồng bộ theo ao | Không có thông báo riêng |

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

`DEBUG_FLOAT_ARRAY`, name=`"SA_DATA"`, array_id=0, ghi trong `send_shoesagtech_debug_arrays()` ở `Rover/GCS_MAVLink_Rover.cpp`. Cả 4 field đều lấy từ **ao đang active** (qua các getter `get_active_dos_sp()`/`get_active_dos_rate()`/`get_active_dos_food()`), trừ `dos_pwm` là giá trị PWM thực tế đang xuất — không phụ thuộc ao.

| Index | Tên | Đơn vị | Điều kiện ghi | Mô tả |
|---|---|---|---|---|
| `data[15]` | `dos_sp` | gam | Luôn | `get_active_dos_sp()`; setpoint của ao đang active (đồng bộ 2 chiều với SA_DOS_SP) |
| `data[16]` | `dos_rate` | — (hệ số điền đầy, không thứ nguyên) ⚠️ đổi ý nghĩa | Luôn | `get_active_dos_rate()`; trả thẳng `SA_DOS_Fx` (hệ số điền đầy) của loại thức ăn ao active đang dùng |
| `data[17]` | `dos_pwm` | µs | Luôn | PWM thực tế đang xuất; 1500=dừng |
| `data[18]` | `dos_food` | 1–7 | Luôn | `get_active_dos_food()`; loại thức ăn của ao active (đồng bộ 2 chiều với SA_DOS_FOOD) |

> **⚠️ Cần xác nhận lại với app/web (chưa xử lý ở bản này):** `get_active_dos_rate()` (nguồn của `data[16]`) trước đây trả `SA_DOS_Fx` ở đơn vị mL/50us (~100); từ bản 2026-08-19 trả hệ số điền đầy không thứ nguyên (~1.0) — **giá trị số thay đổi hoàn toàn dù tên trường và getter không đổi**. Nếu companion app/trang web đang hiển thị field này với nhãn "mL/50us" hoặc dùng nó để tính toán gì khác, cần cập nhật theo — không phải lỗi firmware, chỉ là đổi ý nghĩa con số. Có 2 hướng xử lý, cần người phụ trách app/web quyết định: (a) sửa `get_active_dos_rate()` để trả thẳng `SA_DOS_V × SA_DOS_Fx` (giữ nguyên đơn vị mL/50us hiệu dụng như cũ cho app), hoặc (b) giữ nguyên getter, chỉ cập nhật cách app hiển thị/diễn giải con số này. **Chưa chọn hướng nào ở bản này.**

### 4.2 DataFlash Log [DATA]

N/A — Module 3 không ghi DataFlash riêng. Trạng thái theo dõi qua SA_DATA và console log.

### 4.3 Console Log

Format khác nhau theo `SA_DOS_MODE` — vì ý nghĩa của `SA_DOS_SP` khác nhau giữa 2 mode (xem mục 3.5): mode 0 thì `SA_DOS_SP` CHÍNH LÀ tốc độ (g/phút) nên chỉ in `Rate`; mode 1 thì `SA_DOS_SP` là tổng gam cho cả mission, nên in thêm `Rate` (tốc độ tức thời suy ra từ speed/mission_dist hiện tại) bên cạnh `SP` (tổng).

```
Trigger: mỗi SA_DOS_LOG_MS ms khi SA_DOS_LOG=1

DOS_MODE=0:  [DOS] M0 F<food> SERVO<n> ON/OFF Rate:<rate>g/ph D:<density>g/mL PWM:<pwm>
DOS_MODE=1:  [DOS] M1 F<food> SERVO<n> ON/OFF SP:<sp>g Rate:<rate>g/ph D:<density>g/mL PWM:<pwm>

<sp>   = tổng setpoint của ao đang active (dos_sp_active) — chỉ có ở mode 1
<rate> = tốc độ cấp tức thời (g/phút): mode 0 = chính dos_sp_active;
         mode 1 = dos_gpm tính từ dos_sp_active × speed × 60 / mission_dist (0 nếu chưa đủ điều kiện chạy)
<food> = dos_food_active
```

Ví dụ (DOS_MODE=0, F1, D1=0.62, SP=500 → chạy liên tục 500g/ph):
```
[DOS] M0 F1 SERVO10 ON Rate:500g/ph D:0.62g/mL PWM:1083
```

Ví dụ (DOS_MODE=1, F1, D1=0.62, SP=3860g cho mission 301m @1.3m/s → tốc độ tức thời ≈1000g/ph):
```
[DOS] M1 F1 SERVO10 ON SP:3860g Rate:1000.27g/ph D:0.62g/mL PWM:1083
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
| `SA: Dosing motor ON` | INFO | RC bật (edge rising) | 1 lần/lần bật |
| `SA: Dosing motor OFF` | INFO | RC tắt (edge falling) | 1 lần/lần tắt |
| `SA DOS1: no mission (dist=<x>m) - motor stopped` | WARNING | DOS_MODE=1, dist ≤ 1m | Mỗi 5s |
| `SA DOS1: speed too low (<x>m/s) - motor stopped` | WARNING | DOS_MODE=1, speed < 0.05 | Mỗi 5s |
| `SA: SA_DOS_F<n>=<x> looks uncalibrated for new V x fill-factor formula (expected ~0.05-2.0)` | WARNING | `SA_DOS_Fx` của loại thức ăn active > 5.0 — nghi vẫn còn giá trị cũ (thang mL/50us, thường ~100) từ trước khi tách `SA_DOS_V × SA_DOS_Fx`, chưa được hiệu chuẩn lại (xem mục 2, 3.6) | Mỗi 5s |

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERVOx_FUNCTION = 0     (x = SA_DOS_CHAN)  ┐
SERVOx_MIN      = 800                      │  Sai bất kỳ 1 → KHÔNG chạy + warn 5s
SERVOx_TRIM     = 1500                     │
SERVOx_MAX      = 2200                     ┘

SA_DOS_MODE=1: mission đã upload lên FC VÀ speed ≥ 0.05m/s
               thiếu 1 trong 2 → motor dừng (offset=0)

SA_DOS_V  > 0.1            (firmware clamp, không crash) — mL/50us, DÙNG CHUNG mọi loại thức ăn
SA_DOS_Fx > 0.01           (firmware clamp, không crash) — hệ số điền đầy, RIÊNG theo loại thức ăn
SA_DOS_Dx > 0.01 g/mL      (firmware clamp, không crash)

SA_DOS_SP/SA_DOS_FOOD hiển thị trên GCS luôn phản ánh ao đang chọn (SA_POND_IDX)
— đổi ao sẽ tự đổi giá trị hiển thị, không phải lỗi đồng bộ
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
| DOS_MODE=1, warn label | Đề cập chung | Warn cụ thể lý do (dist hoặc speed) với giá trị thực | Người dùng biết cụ thể vấn đề |
| Công thức tốc độ cố định (DOS_MODE=0) | `g/50us` gộp chung, sau đó tách `vol_rate(mL/50us) × density(g/mL)` với `SA_DOS_RATE` riêng | **Gỡ bỏ hoàn toàn `SA_DOS_RATE`** — DOS_MODE=0 dùng chung `SA_DOS_Fx`/`SA_DOS_Dx` với DOS_MODE=1 | Không còn lý do calib 2 bộ thông số khác nhau cho cùng 1 vít tải; đơn giản hoá cấu hình |
| SA_DOS_F1..F7 | mL/50us theo loại hạt, chỉ dùng ở DOS_MODE=1 | **Dùng cho CẢ 2 mode** | Thống nhất công thức, giảm tham số cần nhớ |
| SA_DOS_Fx — cấu trúc calib (2026-08-19) | Không có — 1 con số/loại thức ăn gộp cả hình học vít và điền đầy hạt | **Tách thành `SA_DOS_V` (mL/50us, DÙNG CHUNG mọi loại thức ăn, đặc tính cơ khí trục vít) × `SA_DOS_Fx` (hệ số điền đầy, không thứ nguyên, RIÊNG theo loại thức ăn)** | Bù đúng bản chất vật lý: trục vít không lấp đầy 100% hạt thức ăn (có khoảng trống không khí) — tách hình học (đo 1 lần) khỏi hệ số điền đầy (đo lại mỗi loại hạt mới) giúp hiệu chuẩn nhanh hơn khi thêm loại thức ăn |
| SA_DOS_D1..D7 | Khối lượng riêng (g/mL) cho 7 loại thức ăn | Không đổi — vẫn 7 loại, mặc định 1.0 | — |
| Setpoint/loại thức ăn theo ao | Không có — 1 giá trị SA_DOS_SP/SA_DOS_FOOD chung toàn hệ thống | **Lưu riêng theo từng ao** (`PondEntry.dos_sp/.dos_food`), đồng bộ 2 chiều qua `_sync_dosing_setpoint()`, tồn tại qua reboot (SD) | Mỗi ao có thể cần lượng/loại thức ăn khác nhau; tránh phải nhập lại tay mỗi lần đổi ao |
| SA_DATA index | data[12..15] | **data[15..18]** (dịch do Module 2 tăng từ 7 lên 10 field) | Module 2 (pond, ph_morn/aft/last_day) chiếm thêm chỗ trong mảng |
| Console log | `F<n> SERVO<m> ON/OFF SP:<sp>g PWM:<pwm>` | Thêm `M<mode>` và `D:<density>g/mL`; SP hiển thị là setpoint của ao active | Người dùng thấy ngay khối lượng riêng và ao đang áp dụng |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE3_DOS_BASIC_DESIGN.md](MODULE3_DOS_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — `_get_mission_dist()` dùng chung, `update()` gọi `_update_dosing_motor()`
- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — `PondEntry`/`SA_POND_IDX` dùng chung, lưu trữ SD qua `_io_update()`
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA (data[15..18])
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
