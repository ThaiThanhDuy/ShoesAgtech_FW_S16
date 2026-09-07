# Module 2 — pH Sensor (Modbus RTU)

## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `libraries/AP_ShoesAgtech/AP_ShoesAgtech.cpp/.h`, `Rover/Log.cpp`, `Rover/GCS_MAVLink_Rover.cpp`
**Loại:** `[x] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`
**Tần suất update:** 0.5 Hz (Modbus poll mỗi 2000 ms); `_io_update()` chạy trong IO thread độc lập
**Cập nhật lần cuối:** 2026-07-18

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
    │         │         ├──► [Moving-average update (10 mẫu) → _ph_value_ma]
    │         │         │       (không còn EMA riêng — SA_PH_EMA đã bị gỡ bỏ)
    │         │         │
    │         │         └──► _ph_update_daily_slots(ph_cal)
    │         │                   │
    │         │                   ├──► AP::rtc().get_utc_usec() — bắt buộc, nếu chưa có → return
    │         │                   ├──► AP::gps().status(0) >= GPS_OK_FIX_3D — bắt buộc, nếu chưa → return
    │         │                   ├──► Chọn ao theo SA_POND_IDX (1-100, nhập tay) → pond_idx
    │         │                   ├──► Ao mới? → khởi tạo PondEntry, tâm GPS = vị trí hiện tại
    │         │                   ├──► Đổi ao (khác _active_pond_idx)? → thông báo + so tâm GPS
    │         │                   │       (chỉ cảnh báo, KHÔNG chặn đo/tính kiềm)
    │         │                   ├──► Cập nhật centroid GPS ao (rolling average)
    │         │                   ├──► Ngày mới cho ao này? → reset slot hôm nay (giữ kiềm hôm qua)
    │         │                   ├──► Đang ARM + trong khung sáng/chiều + đủ SA_PH_CAP_S giây
    │         │                   │       kể từ mẫu trước → ghi đè slot (last-write-wins),
    │         │                   │       đủ SA_PH_CAP_SAM mẫu → báo "hoàn thành slot"
    │         │                   ├──► Ao đủ CẢ hai slot hôm nay lần đầu → tính ΔpH + kiềm,
    │         │                   │       set alk_pending (SD) + gcs_alk_pending (MAVLink)
    │         │                   └──► Cập nhật _alk_slot_status (0 FULL .. 4 NODATA)
    │         │
    │         └──► (SA_SIM=1) _run_simulation()  — chạy CẢ pipeline slot/kiềm với ph_sim
    │                          (xem MODULE1_FLOW_DETAIL_DESIGN.md)
    │
    ├──► Log_Write_Ph_Realtime()            [PHWD — mỗi 2s]
    │         → gps.location(0) → lat, lng
    │         → logger.WriteBlock(&log_PhData, ...)
    │
    ├──► Log_Write_Ph_Alkalinity()          [PHAK — 1 lần/ngày/ao khi FULL]
    │         → consume_alk_log_pending()   → lấy 1 ao đang chờ mỗi lần gọi
    │         → logger.WriteBlock(&log_PhAlk, ...)
    │
    └──► GCS_MAVLINK_Rover::send_shoesagtech_debug_arrays()  [Rover/GCS_MAVLink_Rover.cpp]
              → SA_DATA (array_id=0): data[5..14] pH + ao active
              → SA_PHK  (array_id=1): consume_gcs_alk_pending() → gửi khi có ao vừa FULL

init() [1 lần boot]
    ├──► _ph_init()
    │         └──► hal.serial(SA_PH_PORT)->begin(9600)
    └──► hal.scheduler->register_io_process(_io_update)
              │  chạy trong IO thread (an toàn cho AP::FS())
              ├──► lần gọi đầu tiên: _pond_load()  — đọc /APM/SA_PONDS.bin
              └──► mỗi lần _ponds_dirty=true: _pond_save()
                        rate-limit 5s bình thường, hoặc 60s nếu lần lưu
                        gần nhất thất bại (SD đầy/lỗi) — xem _pond_save_fail_ms
```

### 1.2 Mô tả từng hàm

---

**`_ph_init()`**

- **File:** `AP_ShoesAgtech.cpp : 1146`
- **Được gọi bởi:** `init()` khi boot
- **Xử lý:**
    1. Nếu `SA_PH_EN=0` → return ngay
    2. Lấy `_ph_uart = hal.serial(SA_PH_PORT)` (0–4)
    3. Nếu nullptr → STATUSTEXT WARNING; return
    4. Gọi `_ph_uart->begin(9600)`
    5. Reset buffer MA, trạng thái request, `_ph_last_good_ms=0`
- **Ghi chú:** `SA_PH_PORT` chỉ đọc tại đây — cần reboot nếu đổi port.

---

**`_ph_update()`**

- **File:** `AP_ShoesAgtech.cpp : 1175`
- **Được gọi bởi:** `update()` (SA_SIM=0) @ 10Hz; chỉ thực sự gửi/nhận mỗi 2000ms
- **Xử lý (state machine request/response):**
    1. **Gửi request** (khi `!_ph_req_pending` và Δt ≥ 2000ms): flush RX → ghi 8-byte Modbus FC04
    2. **Đọc response** (sau tối thiểu 150ms, timeout 500ms nếu chưa đủ 23 byte): validate header + CRC → decode `ph_cal`, `_ph_mv`, `_ph_temp`
    3. Cập nhật moving-average 10 mẫu → `_ph_value_ma`
    4. Gọi `_ph_update_daily_slots(ph_cal)`
    5. Gọi `_ph_print_log(now)` — console log nếu SA_PH_LOG=1 (theo chu kỳ SA_PH_LOG_MS)
- **Ghi chú:** Không còn bước làm mịn EMA — tham số `SA_PH_EMA` đã bị gỡ bỏ khỏi hệ thống; `get_ph()` trả về moving-average, `get_ph_raw()` trả về mẫu calib mới nhất (không lọc).

---

**`_ph_crc16(buf, len) → uint16_t`** — static

- **File:** `AP_ShoesAgtech.cpp : 1135`
- CRC16/Modbus, poly 0xA001, init 0xFFFF
- Dùng validate 21 byte đầu của response 23-byte

---

**`_ph_print_log(uint32_t now)`** — mới, 2026-09-07

- **File:** `AP_ShoesAgtech_PH.cpp`
- **Được gọi bởi:** `_ph_update()` (SA_SIM=0) **và** `_run_simulation()` (SA_SIM=1)
- **Xử lý:** In log pH định kỳ (`[WM]`/`[SIM][WM]` pH/MA/Tmp/mV + slot tag, dòng Alk khi ao FULL, dòng khung giờ AM/PM), gate theo `SA_PH_LOG`/`SA_PH_LOG_MS` — y hệt nội dung log cũ, chỉ tách ra thành hàm riêng.
- **⚠️ Bug đã sửa (2026-09-07):** Trước đây đoạn in log này nằm **trực tiếp trong** `_ph_update()`, nên khi `SA_SIM=1` (không bao giờ gọi `_ph_update()`) thì **console không in được dòng pH định kỳ nào cả**, dù `_ph_value`/`_ph_value_ma`/... vẫn được `_run_simulation()` cập nhật đúng và các getter (`get_ph()`...) vẫn trả đúng số. Người dùng bật `SA_PH_LOG=1` khi test SIM sẽ không thấy log nào — dễ nhầm là lỗi mô phỏng trong khi giá trị pH thực ra vẫn đúng, chỉ là log không được gọi tới.

---

**`_ph_update_daily_slots(float ph_cal)`**

- **File:** `AP_ShoesAgtech.cpp : 1340`
- **Được gọi bởi:** `_ph_update()` sau mỗi frame hợp lệ (và `_run_simulation()` khi SA_SIM=1)
- **Xử lý:**
    1. **Bắt buộc có giờ UTC** (`AP::rtc().get_utc_usec()`) và **GPS fix 3D** (`AP_GPS::GPS_OK_FIX_3D`) — thiếu 1 trong 2 → cảnh báo mỗi 60s (nếu SA_PH_LOG=1), return ngay, không làm gì thêm
    2. `local_sec = utc_sec + SA_PH_TZ×3600`; `today = local_sec/86400`; `local_h` = giờ địa phương dạng thập phân
    3. **Chọn ao:** `pond_idx = constrain(SA_POND_IDX, 1, 100) − 1` (nhập tay, không tự động theo GPS)
    4. **Ao mới** (`!_ponds[pond_idx].valid`): khởi tạo `PondEntry`, `center_lat/lng` = GPS hiện tại, `dos_sp`/`dos_food` = giá trị `SA_DOS_SP`/`SA_DOS_FOOD` hiện tại, đánh dấu `_ponds_dirty=true`
    5. **Đổi ao** (`pond_idx != _active_pond_idx` hoặc lần đầu sau boot): in `[SA] Chuyen sang ao #n`; nếu ao đã có > 5 mẫu GPS lịch sử, so khoảng cách tâm ao đã lưu với vị trí hiện tại — lệch quá `SA_PH_POND_D` mét thì in cảnh báo tham khảo (**không chặn** đo/tính kiềm)
    6. Cập nhật `_active_pond_idx = pond_idx`
    7. **Centroid GPS** ao cập nhật kiểu rolling-average, trọng số `1/gps_count` (tăng dần đến trần 1000 mẫu)
    8. **Ngày mới cho ao này** (`pond.last_day != today`): reset `ph_morn/ph_aft/status/delta_ph/alk_pending/gcs_alk_pending/alk_computed/...` về 0/false — **giữ nguyên** `alk_dkh/alk_mgl` (kiềm hôm qua) để dùng làm PREV
    9. **Lấy mẫu** — chỉ khi `(in_morn || in_aft) && hal.util->get_soft_armed()`:
       - Rate-limit `SA_PH_CAP_S` giây/mẫu theo từng slot (`morn_last_ms`/`aft_last_ms`)
       - Ghi đè giá trị mới nhất vào `ph_morn`/`ph_aft` (last-write-wins), tăng `morn_count`/`aft_count`, set bit `status` (bit0=sáng, bit1=chiều)
       - Khi vừa đạt đủ `SA_PH_CAP_SAM` mẫu và chưa từng báo → in 1 lần "Ao#n pH sang/chieu: X (N mau)"
    10. **Tính kiềm** — khi `status == 3` (FULL) lần đầu trong ngày (`!alk_computed`):
        - `delta_ph = ph_aft − ph_morn`
        - `kh_scaled = SA_PH_KH × (1 + constrain(delta_ph×0.375, −0.5, 1.0))`
        - `alk_dkh = _ph_calc_alkalinity(ph_morn, kh_scaled, _ph_temp)`; `alk_mgl = alk_dkh × 17.85`
        - Set `alk_computed=true`, `alk_pending=true` (chờ ghi SD), `gcs_alk_pending=true` (chờ gửi MAVLink), `_ponds_dirty=true`
        - In 2 dòng INFO: `Ao#n S:x C:x dPH:x` và `Ao#n kiem:xdKH/xmgL`
    11. Cập nhật `_alk_slot_status` hiển thị: `status==3`→0(FULL), `status==1`→1(MORN), `status==2`→2(AFT), else nếu `alk_dkh>0`→3(PREV, dùng dữ liệu ngày trước), else→4(NODATA)
- **Đầu ra:** Cập nhật `_ponds[pond_idx]`, các mirror `_alk_dkh/_alk_mgl/_delta_ph/_alk_slot_status`, `_active_pond_idx`

---

**`_ph_calc_alkalinity(float ph, float base_kh_dkh, float temp_c) → float`**

- **File:** `AP_ShoesAgtech.cpp : 1546`
- **Được gọi bởi:** `_ph_update_daily_slots()` khi ao vừa đạt FULL
- **Xử lý:**
    1. `tf = constrain(1.0 − (temp−28) × 0.008, 0.85, 1.10)` (temperature factor)
    2. `pf` = pH factor 3 vùng tuyến tính: pH≥8.3 / 7.6≤pH<8.3 / pH<7.6
    3. `return base_kh × constrain(pf × tf, 0.55, 1.75)`
- **Ghi chú:** Heuristic dựa trên cân bằng CO₂/HCO₃⁻. Cập nhật SA_PH_KH định kỳ từ test kit.

---

**`consume_alk_log_pending() → bool`**

- **File:** `AP_ShoesAgtech.cpp : 1571`
- **Được gọi bởi:** `Rover::Log_Write_Ph_Alkalinity()` mỗi vòng lặp
- **Xử lý:** Duyệt `_ponds[0.._pond_count)`, tìm ao đầu tiên có `alk_pending=true` → xoá flag, gán các mirror `_ph_morn_val/_ph_aft_val/_ph_morn_lat/_ph_morn_lng/_delta_ph/_alk_dkh/_alk_mgl` và `_alk_pond_idx` từ ao đó, trả `true`. Không còn ao nào đang chờ → trả `false`.
- **Ghi chú:** Chỉ lấy **một** ao mỗi lần gọi — nếu nhiều ao cùng đạt FULL trong 1 vòng lặp, các ao còn lại được ghi ở những lần gọi kế tiếp (10 Hz nên không đáng kể độ trễ).

---

**`consume_gcs_alk_pending() → bool`**

- **File:** `AP_ShoesAgtech.cpp : 1595`
- **Được gọi bởi:** `GCS_MAVLINK_Rover::send_shoesagtech_debug_arrays()`
- **Xử lý:** Giống hệt `consume_alk_log_pending()` nhưng dùng cờ độc lập `gcs_alk_pending` — **không** ảnh hưởng/tranh chấp với đường ghi SD.

---

**`_io_update()`**

- **File:** `AP_ShoesAgtech.cpp : 1619`
- **Được gọi bởi:** `hal.scheduler->register_io_process()` — chạy trong IO thread riêng của ArduPilot (không phải scheduler task 10Hz), an toàn để gọi `AP::FS()`
- **Xử lý:**
    1. Lần đầu tiên (`!_ponds_loaded`): gọi `_pond_load()`
    2. Nếu `_ponds_dirty=true` và đã đủ chu kỳ chờ kể từ lần lưu trước: gọi `_pond_save()`
       - Chu kỳ chờ bình thường: **5000ms**
       - Nếu lần lưu **gần nhất thất bại** (`_pond_save_fail_ms != 0`): chu kỳ giãn ra **60000ms** (60s)
    3. `_pond_save()` trả `true` → xoá `_ponds_dirty`, reset `_pond_save_fail_ms=0`. Trả `false` → giữ nguyên `_ponds_dirty` (thử lại ở chu kỳ sau) và set `_pond_save_fail_ms=now` để kích hoạt backoff 60s
- **Ghi chú:** Backoff 60s khi ghi thất bại nhằm giảm số lần `_pond_save()` chiếm giữ semaphore filesystem dùng chung với `AP_Logger` (`AP_Filesystem_FATFS.cpp` dùng 1 semaphore toàn cục cho mọi thao tác file) — nếu thẻ SD đầy/lỗi khiến `write()` chậm bất thường trong khi giữ semaphore, thử lại liên tục mỗi 5s có thể góp phần gây `INTERNAL_ERROR main_loop_stuck` hoặc khiến `AP_Logger` không mở được file log (EBUSY). Backoff không giải quyết tận gốc thẻ SD hỏng/đầy — chỉ giảm mức độ ảnh hưởng.

---

**`_pond_save() → bool`**

- **File:** `AP_ShoesAgtech.cpp : 1660`
- **File lưu:** `/APM/SA_PONDS.bin`
- **Format:** header `{magic=0x504F4E44 ('POND'), version=3, count}` + mảng `PondEntry[count]` (chỉ ghi đúng số ao đã từng dùng, không ghi cả 100 slot để giảm hao mòn thẻ SD)
- **Xử lý:** mở `O_WRONLY|O_CREAT|O_TRUNC`; ghi header rồi `count` phần tử `PondEntry`; **kiểm tra số byte thực sự ghi được** của cả 2 lần `write()` so với kích thước mong đợi
- **Đầu ra / Return:** `bool` — `true` nếu ghi đủ byte cả header lẫn dữ liệu; `false` nếu thiếu (vd `ENOSPC` — thẻ đầy) hoặc không mở được file
- **Khi trả `false`:** gửi STATUSTEXT WARNING `"SA: khong the luu du lieu ao xuong SD (the day/loi?)"` — trước đây lỗi ghi bị bỏ qua hoàn toàn (không ai biết dữ liệu ao đang mất); giờ luôn có cảnh báo mỗi lần thất bại
- **Ghi chú:** Trước bản này hàm là `void`, không kiểm tra kết quả `write()` — sửa cùng đợt với cơ chế backoff ở `_io_update()` để tránh giữ semaphore filesystem chung quá lâu khi SD lỗi (xem STATUSTEXT `main_loop_stuck`/`EBUSY` liên quan)

---

**`_pond_load()`**

- **File:** `AP_ShoesAgtech.cpp : 1686`
- **Xử lý:** đọc header, kiểm tra `magic` và `version` khớp — sai (kể cả file version cũ hơn) → bỏ qua, coi như chưa có dữ liệu; đúng → đọc `_ponds[]`, đặt `_pond_count`, xoá các cờ tạm thời (`morn_last_ms/aft_last_ms/alk_pending/gcs_alk_pending`) để tránh ghi lại dữ liệu cũ ngay sau boot; in `[SA] Load N ao tu SD card`
- **Ghi chú version:** `SA_PONDS_VER` tăng mỗi khi đổi layout `PondEntry` (v2 thêm `gcs_alk_pending`, v3 thêm `dos_food`) — file cũ hơn bị bỏ qua thay vì đọc nhầm layout lệch.
- **Lưu ý:** không có cơ chế backoff cho `_pond_load()` — hàm này chỉ chạy đúng 1 lần lúc boot (`!_ponds_loaded`). Nếu bản thân thao tác đọc thẻ SD bị treo (thẻ hỏng nặng ở tầng phần cứng, không chỉ đầy dung lượng), không có retry/backoff nào ở tầng ứng dụng có thể xử lý được — cần thay thẻ SD.

---

## 2. Tham số cài đặt [ALL]

| Tham số | Slot | Kiểu | Mặc định | Min | Max | Mô tả đầy đủ |
|---|---|---|---|---|---|---|
| `SA_PH_EN` | 14 | Int8 | 0 | 0 | 1 | Bật/tắt module pH. Tắt → không mở UART, không poll, không ghi SD. |
| `SA_PH_PORT` | 15 | Int8 | 2 | 0 | 4 | Số SERIAL port RS485-TTL. Cần `SERIALx_BAUD=9`, `SERIALx_PROTOCOL=0`. |
| `SA_PH_TOFF` | 16 | Float | -3.5 | -10 | 10 | Offset bù nhiệt độ (°C): `T = raw/10 + SA_PH_TOFF`. |
| `SA_PH_OFF` | 17 | Float | 0.0 | -2.0 | 2.0 | Offset hiệu chuẩn pH: `pH = raw/100 + SA_PH_OFF`. |
| `SA_PH_KH` | 18 | Float | 4.0 | 0 | 30 | Kiềm tham chiếu từ test kit (dKH). Cập nhật khi test ao. |
| `SA_PH_LOG` | 20 | Int8 | 0 | 0 | 1 | Bật (1) console log pH/nhiệt độ/mV theo chu kỳ `SA_PH_LOG_MS`. |
| `SA_PH_TZ` | 21 | Int8 | 7 | -12 | 14 | UTC offset (giờ). Việt Nam = 7. Dùng phân slot sáng/chiều + GPS thời gian. |
| `SA_PH_LOG_MS` | 23 | Int16 | 2000 | 500 | 60000 | Chu kỳ console log pH (ms). |
| `SA_PH_TIMEOUT` | 24 | Int16 | 2 | 1 | 300 | Ngưỡng mất kết nối (giây). Quá ngưỡng → `ph_has_data()=false`. |
| `SA_PH_MS` | 55 | Float | 5.0 | 0 | 23.99 | Giờ **bắt đầu** cửa sổ sáng, thập phân (vd 5.5 = 5h30). |
| `SA_PH_ME` | 56 | Float | 11.0 | 0 | 24 | Giờ **kết thúc** cửa sổ sáng (inclusive `<=`). 24.0 = đến hết ngày. |
| `SA_PH_AS` | 57 | Float | 12.0 | 0 | 23.99 | Giờ **bắt đầu** cửa sổ chiều, thập phân. |
| `SA_PH_AE` | 58 | Float | 16.0 | 0 | 24 | Giờ **kết thúc** cửa sổ chiều (inclusive `<=`). 24.0 = đến hết ngày. |
| `SA_POND_IDX` | 59 | Int16 | 1 | 0 | 100 | Ao đang đo (1-based, nhập tay). Chung cho Module 2 (pH) và Module 3 (dosing setpoint/loại thức ăn theo ao). 0 = tắt chọn ao (không phân slot). |
| `SA_PH_POND_D` | 60 | Float | 300.0 | 10 | 5000 | Ngưỡng khoảng cách GPS (m) so với tâm ao đã lưu để cảnh báo "lệch vị trí". Chỉ tham khảo — không chặn đo/tính kiềm. |
| `SA_PH_CAP_S` | 61 | Int16 | 20 | 1 | 3600 | Khoảng thời gian tối thiểu (giây) giữa 2 mẫu tích lũy trong 1 slot. |
| `SA_PH_CAP_SAM` | 62 | Int8 | 20 | 1 | 100 | Số mẫu cần đạt để slot được báo "hoàn thành" (giá trị dùng là mẫu cuối — last-write-wins). |
| `SA_PH_CAP_M` | 12 | Int8 | 1 | 1 | 100 | Bán kính capture (m) — **chỉ dùng cho app đồng hành**, firmware khởi tạo giá trị nhưng không đọc lại. |

> **Param chỉ có hiệu lực sau reboot:** `SA_PH_PORT`
>
> **Param đã bị gỡ bỏ (không còn tồn tại):** `SA_PH_EMA` (slot 19, cũ — làm mịn EMA cho pH đã bỏ, chỉ còn moving-average 10 mẫu), `SA_PH_SAMP_D` (điểm mẫu PHSP theo khoảng cách — tính năng đã bị loại bỏ hoàn toàn).
>
> **Lưu ý cửa sổ thời gian:** `SA_PH_MS`/`SA_PH_AS` clamp 0–23.99 (giờ bắt đầu). `SA_PH_ME`/`SA_PH_AE` clamp 0–24 (24 = "đến hết giờ trong ngày"). Giờ là số thập phân (không còn Int8) nên có thể đặt phút lẻ.

---

## 3. Mô tả kỹ thuật [ALL]

### 3.1 Khởi tạo

- Gọi `hal.serial(SA_PH_PORT)->begin(9600)` một lần khi boot
- Reset circular buffer moving-average (10 phần tử)
- `_ph_last_good_ms = 0` → `ph_has_data() = false` cho đến khi frame đầu tiên
- Đăng ký `_io_update()` làm IO process — lần chạy đầu tiên sẽ load `_ponds[]` từ thẻ SD

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

### 3.3 Làm mịn, ao và slot

**Moving Average (duy nhất — không còn EMA):** circular buffer 10 phần tử; `_ph_value_ma = sum / count`. `get_ph()` → MA; `get_ph_raw()` → mẫu calib mới nhất chưa lọc.

**Chọn ao (thủ công, không tự động theo GPS):**

```
pond_idx = constrain(SA_POND_IDX, 1, 100) − 1     (0-based nội bộ)

Ao chưa tồn tại (_ponds[pond_idx].valid == false):
    → khởi tạo: center_lat/lng = GPS hiện tại, dos_sp = SA_DOS_SP,
      dos_food = SA_DOS_FOOD, valid = true, last_day = today

Ao vừa đổi (khác _active_pond_idx, hoặc lần đầu sau boot):
    → in "[SA] Chuyen sang ao #n" (+ "(ao moi)" nếu vừa khởi tạo)
    → nếu gps_count > 5: so khoảng cách GPS hiện tại với tâm ao đã lưu
        <= SA_PH_POND_D  → "[SA] Ao#n GPS OK (Xm)"
        >  SA_PH_POND_D  → "[SA] Ao#n GPS lech Xm - can check lai vi tri ao"
      (CHỈ CẢNH BÁO — không chặn đo/tính kiềm dù lệch bao xa)

Centroid GPS ao: rolling average, trọng số 1/gps_count, trần 1000 mẫu
```

**Phân slot theo cửa sổ cài đặt (chỉ khi đang ARM):**

```
local_sec  = UTC_epoch + SA_PH_TZ × 3600
local_h    = (local_sec % 86400) / 3600.0   (giờ thập phân)

in_morn = local_h ∈ [SA_PH_MS, SA_PH_ME]
in_aft  = local_h ∈ [SA_PH_AS, SA_PH_AE]

(in_morn || in_aft) && dang_ARM && đủ SA_PH_CAP_S giây kể từ mẫu trước của slot đó:
    ghi đè pond.ph_morn hoặc pond.ph_aft = ph_cal   (last-write-wins)
    tăng morn_count/aft_count, set bit status (1=sáng, 2=chiều)
    đạt đúng SA_PH_CAP_SAM mẫu lần đầu → báo "hoàn thành slot" 1 lần

Ngoài cả hai cửa sổ, hoặc đang DISARM → không lấy mẫu

Ngày mới cho ao này (last_day != today):
    reset ph_morn/ph_aft/status/delta_ph/alk_*_pending/alk_computed = 0/false
    GIỮ NGUYÊN alk_dkh/alk_mgl (kiềm hôm qua) → dùng làm PREV
```

**Tính ΔpH và kiềm (khi status == 3, tức FULL, lần đầu trong ngày):**

```
delta_ph   = pond.ph_aft − pond.ph_morn
kh_scaled  = SA_PH_KH × (1 + constrain(delta_ph × 0.375, −0.5, 1.0))
alk_dkh    = _ph_calc_alkalinity(pond.ph_morn, kh_scaled, _ph_temp)
alk_mgl    = alk_dkh × 17.85
alk_computed = true
alk_pending = true       (chờ Log.cpp ghi PHAK)
gcs_alk_pending = true    (chờ MAVLink gửi SA_PHK — độc lập với alk_pending)
```

**Bảng trạng thái alkalinity (`slot_status`):**

| Status | Giá trị | Điều kiện | Kiềm hiển thị |
|---|---|---|---|
| 0 FULL | 0 | Ao active có đủ sáng + chiều hôm nay | Chính xác nhất (ΔpH) |
| 1 MORN | 1 | Chỉ có sáng hôm nay | Không có — chờ chiều |
| 2 AFT | 2 | Chỉ có chiều hôm nay | Không có — chờ sáng |
| 3 PREV | 3 | Chưa có dữ liệu hôm nay, có kiềm hôm qua (`alk_dkh>0`) | Hôm qua (không tính lại) |
| 4 NODATA | 4 | Ao chưa từng có dữ liệu kiềm | 0 |

### 3.4 Xử lý các trường hợp đặc biệt

- **CRC lỗi:** bỏ frame, không cập nhật `_ph_last_good_ms`; STATUSTEXT mỗi lần
- **Timeout response (> 500ms):** reset `_ph_req_pending`; warn mỗi 10s
- **Không có GPS 3D fix hoặc chưa có giờ UTC:** vẫn đọc pH + ghi PHWD (với lat/lng=0); **không** chọn ao, **không** phân slot, **không** tính kiềm; warn mỗi 60s (nếu SA_PH_LOG=1)
- **Đang DISARM:** vẫn đọc/ghi PHWD bình thường, nhưng KHÔNG tích lũy mẫu vào slot sáng/chiều (chỉ lấy mẫu khi ARM)
- **Lệch vị trí GPS so với tâm ao:** chỉ cảnh báo tham khảo, không chặn — người vận hành tự quyết định có đúng ao hay không
- **`SA_POND_IDX = 0`:** vô hiệu hoá việc chọn ao — `_ph_update_daily_slots()` vẫn chạy nhưng `constrain(0,1,100)=1` nên thực chất luôn dùng ao #1; đặt về 0 không "tắt" theo nghĩa dừng lấy mẫu

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

`DEBUG_FLOAT_ARRAY`, name=`"SA_DATA"`, array_id=0, stream EXTRA3. Chỉ ghi khi `SA_PH_EN=1 AND ph_has_data()`. Ngược lại → toàn bộ `data[5..14] = 0.0`. Các trường `alk_*`/`delta_ph`/`ph_morn`/`ph_aft`/`last_day` đều lấy từ **ao đang active** (`_active_pond_idx`), không phải tổng toàn hệ thống.

| Index | Tên | Đơn vị | Mô tả |
|---|---|---|---|
| `data[5]` | `ph` | — | pH moving average 10 mẫu (`_ph_value_ma`) |
| `data[6]` | `ph_mv` | mV | Điện áp điện cực (signed int16 cast sang float) |
| `data[7]` | `ph_temp` | °C | Nhiệt độ đã bù SA_PH_TOFF |
| `data[8]` | `alk_dkh` | dKH | Kiềm của ao active (0 nếu `alk_computed=false`) |
| `data[9]` | `alk_mgl` | mg/L CaCO₃ | Kiềm của ao active (0 nếu chưa tính) |
| `data[10]` | `delta_ph` | — | ΔpH ao active hôm nay; 0 nếu `status != 3` |
| `data[11]` | `pond_idx` | 1–100 | Số thứ tự ao đang active (hiển thị bắt đầu từ 1) |
| `data[12]` | `ph_morn` | — | pH slot sáng hôm nay của ao active (0 nếu chưa có mẫu) |
| `data[13]` | `ph_aft` | — | pH slot chiều hôm nay của ao active (0 nếu chưa có mẫu) |
| `data[14]` | `last_day` | ngày | Ngày ghi nhận (Unix epoch ngày; ×86400 = Unix timestamp) |

**Phát hiện mất kết nối phía GCS:**

```
IF data[5]==0.0 AND data[6]==0.0 AND data[7]==0.0 → hiển thị cảnh báo "mất kết nối"
```

### 4.2 MAVLink SA_PHK (bản ghi kiềm/ao — mới)

`DEBUG_FLOAT_ARRAY`, name=`"SA_PHK"`, **array_id=1** (phân biệt với SA_DATA array_id=0). Gửi **độc lập** với SA_DATA, chỉ khi `sa.ph_is_enabled() && sa.consume_gcs_alk_pending()` trả `true` — tức đúng thời điểm một ao vừa tính đủ kiềm (FULL) lần đầu trong ngày. Mục đích: cho GCS/app hiển thị "chất lượng nước" ngay, không cần tải lại log SD.

| Index | Tên | Đơn vị | Mô tả |
|---|---|---|---|
| `phk[0]` | `ph_morn` | — | pH sáng của ao vừa hoàn tất |
| `phk[1]` | `ph_aft` | — | pH chiều của ao vừa hoàn tất |
| `phk[2]` | `delta_ph` | — | ΔpH = chiều − sáng |
| `phk[3]` | `alk_dkh` | dKH | Kiềm vừa tính |
| `phk[4]` | `alk_mgl` | mg/L | Kiềm vừa tính (mg/L CaCO₃) |
| `phk[5]` | `lat` | độ | GPS latitude mẫu sáng, đã chia 1e7 (độ thập phân, không phải deg×1e7 như log) |
| `phk[6]` | `lng` | độ | GPS longitude mẫu sáng, đã chia 1e7 |
| `phk[7]` | `pond_idx` | 1–100 | Số thứ tự ao (1-based) |

---

### 4.3 DataFlash Log (SD card)

Hai loại message ghi độc lập vào SD card của ArduPilot theo định dạng binary `.bin`.

---

#### PHWD — pH Realtime

```
Message: "PHWD"
Format:  "QfffhLL"
Điều kiện: SA_PH_EN=1 (logger.logging_enabled()); KHÔNG yêu cầu ph_has_data()
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

**Ứng dụng:** Vẽ đường pH theo thời gian; overlay lên bản đồ GPS để xem pH từng vị trí.
**Dung lượng:** ~34 bytes × 43.200 record/ngày ≈ **1.5 MB/ngày**.
**Lưu ý:** Không mang theo số hiệu ao — muốn biết record thuộc ao nào phải đối chiếu thời gian/GPS với PHAK hoặc log vị trí robot.

---

#### PHAK — Daily Alkalinity (Kiềm ngày/ao)

```
Message: "PHAK"
Format:  "QfffffLLB"
Điều kiện: consume_alk_log_pending()=true (ao vừa đạt FULL lần đầu trong ngày)
Tần suất: Tối đa 1 record/ngày/ao — nhưng có thể nhiều record/vòng lặp nếu nhiều ao cùng FULL
```

| Field | Tên | Kiểu | Đơn vị | Mô tả |
|---|---|---|---|---|
| 1 | `TimeUS` | uint64 (Q) | µs | Timestamp khi ao đạt FULL |
| 2 | `pHMorn` | float (f) | — | pH buổi sáng (mẫu cuối trong slot, last-write-wins) |
| 3 | `pHAft` | float (f) | — | pH buổi chiều (mẫu cuối trong slot) |
| 4 | `dPH` | float (f) | — | ΔpH = pHAft − pHMorn |
| 5 | `AlkDKH` | float (f) | dKH | Kiềm tính từ ΔpH + `_ph_calc_alkalinity()` |
| 6 | `AlkMGL` | float (f) | mg/L | AlkDKH × 17.85 |
| 7 | `Lat` | int32 (L) | deg×1e7 | GPS latitude mẫu sáng cuối (định danh ao) |
| 8 | `Lng` | int32 (L) | deg×1e7 | GPS longitude mẫu sáng cuối |
| 9 | `PondIdx` | uint8 (B) | — | Số thứ tự ao, 1-based (khớp `SA_POND_IDX`, tối đa 100) |

**Ứng dụng:** Theo dõi kiềm từng ao theo ngày. Filter theo `PondIdx` (chính xác hơn Lat/Lng) để xem lịch sử một ao cụ thể.
**Dung lượng:** ~41 bytes × (số ao đạt FULL)/ngày — cực nhỏ.

**Cơ chế ghi (queue nhiều ao):**

```
_ph_update_daily_slots(): mỗi ao đạt FULL lần đầu hôm nay → set alk_pending=true

Log_Write_Ph_Alkalinity(): mỗi vòng lặp
    consume_alk_log_pending() → lấy MỘT ao đang chờ, reset flag ao đó
    → ghi 1 record PHAK cho ao đó
    → ao khác còn chờ sẽ được ghi ở (các) lần gọi kế tiếp
```

---

### 4.4 Console Log

```
Trigger: mỗi SA_PH_LOG_MS ms khi SA_PH_LOG=1

Dòng 1 (luôn in):
  [WM] pH:<ph_raw> MA:<ph_ma> Tmp:<temp>C mV:<mv> [FULL|MORN|AFT|PREV|NODATA]

Dòng 2 (chỉ khi status=0 FULL):
  [WM] Alk:<alk_dkh>dKH <alk_mgl>mg/L dPH:<delta_ph>

Dòng 3 (luôn in — hiển thị khung giờ đang cấu hình):
  [WM] Sang:<h>:<mm>-<h>:<mm> Chieu:<h>:<mm>-<h>:<mm>

SA_SIM=1: tiền tố [SIM][WM] thay vì [WM]
```

### 4.5 STATUSTEXT — Toàn bộ thông báo

| Nội dung thông báo | Mức | Điều kiện | Tần suất |
|---|---|---|---|
| `SA: pH sensor on SERIAL<n> (Modbus RTU 9600)` | INFO | Init thành công | 1 lần |
| `SA: pH sensor SERIAL<n> not found` | WARNING | Port không tồn tại | 1 lần |
| `SA: pH sensor chưa có dữ liệu - kiểm tra dây RS485` | WARNING | Chưa nhận frame nào | Mỗi 10s |
| `SA: pH sensor mất kết nối (<x>s) - kiểm tra dây RS485` | WARNING | Mất kết nối > SA_PH_TIMEOUT | Mỗi 10s |
| `SA: pH CRC fail (noise on RS485?)` | WARNING | CRC16 không khớp | Mỗi lần lỗi |
| `[WM] Chưa GPS - kiềm đợi GPS/giờ` | INFO | Chưa có giờ UTC hoặc chưa GPS fix 3D (chỉ khi SA_PH_LOG=1) | Mỗi 60s |
| `[SA] Chuyen sang ao #<n>` (+ `(ao moi)` nếu vừa tạo) | INFO | Đổi `SA_POND_IDX` hoặc lần đầu sau boot | 1 lần/lần đổi |
| `[SA] Ao#<n> GPS OK (<x>m)` | INFO | Vừa đổi ao, ao có > 5 mẫu lịch sử, lệch ≤ SA_PH_POND_D | 1 lần/lần đổi |
| `[SA] Ao#<n> GPS lech <x>m - can check lai vi tri ao` | INFO | Vừa đổi ao, lệch > SA_PH_POND_D | 1 lần/lần đổi |
| `[SA] Ao#<n> pH sang: <x> (<N> mau)` | INFO | Slot sáng vừa đủ SA_PH_CAP_SAM mẫu | 1 lần/ngày/ao |
| `[SA] Ao#<n> pH chieu: <x> (<N> mau)` | INFO | Slot chiều vừa đủ SA_PH_CAP_SAM mẫu | 1 lần/ngày/ao |
| `[SA] Ao#<n> S:<x> C:<x> dPH:<x>` | INFO | Ao vừa đạt FULL — dòng 1 | 1 lần/ngày/ao |
| `[SA] Ao#<n> kiem:<x>dKH/<x>mgL` | INFO | Ao vừa đạt FULL — dòng 2 | 1 lần/ngày/ao |
| `[SA] Load <N> ao tu SD card` | INFO | Boot xong, đọc lại `/APM/SA_PONDS.bin` | 1 lần khi boot |
| `SA: khong the luu du lieu ao xuong SD (the day/loi?)` | WARNING | `_pond_save()` ghi thiếu byte (thẻ SD đầy/lỗi) | Mỗi lần thử lưu thất bại (tối đa mỗi 60s do backoff) |
| `[WM] pH:<x> MA:<y> Tmp:<z>C mV:<w> [...]` | INFO | Console log (SA_PH_LOG=1) | Mỗi SA_PH_LOG_MS |
| `[WM] Alk:<x>dKH <y>mg/L dPH:<z>` | INFO | Console log, chỉ khi FULL | Mỗi SA_PH_LOG_MS |
| `[WM] Sang:H:MM-H:MM Chieu:H:MM-H:MM` | INFO | Console log, luôn in | Mỗi SA_PH_LOG_MS |

---

## 5. Yêu cầu / Ràng buộc [ALL]

```
SERIALx_BAUD     = 9     (= 9600 baud)    ← x = SA_PH_PORT   → sai: không nhận frame
SERIALx_PROTOCOL = 0     (= None)         ←                   → sai: ArduPilot chiếm port
SA_PH_PORT       → chỉ đọc khi boot, cần reboot nếu đổi
SA_PH_KH cần cập nhật định kỳ bằng test kit khi độ kiềm ao thay đổi
GPS fix 3D + giờ UTC bắt buộc để chọn ao và phân slot sáng/chiều
Phải đang ARM để mẫu pH được tích lũy vào slot (đứng yên/disarm vẫn đọc/log PHWD nhưng không tính kiềm)
SA_PH_MS < SA_PH_ME  và  SA_PH_AS < SA_PH_AE  (cửa sổ không rỗng)
Tối đa 100 ao (SA_POND_IDX 1-100); dữ liệu ao lưu trên thẻ SD, còn nguyên qua reboot
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

| Điểm | Basic Design dự kiến | Thực tế đã làm (bản hiện tại) | Lý do |
|---|---|---|---|
| Công thức alkalinity | Đơn giản: `SA_PH_KH + 16×(pH−7)` | Heuristic 3-vùng với temperature factor | Kinh nghiệm thực địa: cần bổ chỉnh nhiệt độ và đặc tính pH 7–8 |
| Slot thời gian | Sáng 00:00–11:59, Chiều 12:00–23:59 | Cửa sổ cài được, dạng giờ thập phân (SA_PH_MS/ME/AS/AE) | Linh hoạt theo lịch đo thực tế từng trang trại, kể cả phút lẻ |
| Chọn ao | Tự động theo khoảng cách GPS sáng↔chiều (≤300m) | **Chọn tay bằng `SA_POND_IDX`** (1-100); GPS chỉ cảnh báo lệch vị trí, không chặn | Thực địa: robot có thể chưa đủ fix GPS chính xác lúc mới đến ao; chọn tay chắc chắn hơn |
| Số lượng ao | Không giới hạn rõ ràng | Tối đa **100 ao**, mỗi ao lưu riêng pH/kiềm/ngày/dos_sp/dos_food | Đáp ứng trang trại nhiều ao, luân phiên đo trong ngày |
| Lưu trữ ao qua reboot | Không đề cập | Lưu toàn bộ `_ponds[]` vào `/APM/SA_PONDS.bin`, load lại khi boot | Không mất lịch sử đo khi FC mất điện/reset |
| Làm mịn pH | EMA + moving average song song | Chỉ còn **moving average** (10 mẫu) | Đơn giản hoá, EMA không mang lại lợi ích thêm cho tần suất đo 2s |
| Kiềm realtime (NODATA) | Ước tính từ pH tức thời | Không tính — chỉ FULL mới có kiềm mới | Tránh nhiễu; kiềm chỉ có ý nghĩa khi có ΔpH đủ 2 slot |
| Điểm mẫu dọc tuyến (PHSP) | Ghi theo khoảng cách dọc mission | **Đã loại bỏ hoàn toàn** — không còn `SA_PH_SAMP_D`/PHSP | Trùng lặp với nhu cầu thực tế; đơn giản hoá theo hướng multi-pond |
| Ghi SD card | Chưa đề cập | 2 loại PHWD/PHAK + 1 file trạng thái toàn bộ ao (SA_PONDS.bin) | Phân tích offline + khôi phục trạng thái qua reboot |
| Gửi kiềm về GCS ngay | Chưa đề cập | Bản tin MAVLink riêng `SA_PHK` (array_id=1), độc lập SA_DATA | GCS/app hiển thị ngay khi vừa tính xong, không cần tải log |
| Bắt buộc ARM để lấy mẫu | Không đề cập | Chỉ tích lũy mẫu slot khi đang ARM | Tránh lẫn mẫu đo lúc đứng yên/thử nghiệm vào dữ liệu ao thật |
| Timeout warning | Mỗi N giây | Mỗi 10s (độc lập SA_PH_LOG_MS) | Cần cảnh báo nhanh dù log chậm |

---

## 8. Tài liệu liên quan [ALL]

- [MODULE2_PH_BASIC_DESIGN.md](MODULE2_PH_BASIC_DESIGN.md) — yêu cầu và hành vi
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA (data[5..14])
- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — `update()` gọi `_ph_update()`/`_run_simulation()`
- [MODULE3_DOS_DETAIL_DESIGN.md](MODULE3_DOS_DETAIL_DESIGN.md) — `dos_sp`/`dos_food` riêng theo ao, đồng bộ qua `_sync_dosing_setpoint()`
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
