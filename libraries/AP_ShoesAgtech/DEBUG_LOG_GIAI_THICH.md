# ShoesAgtech — Giải thích Debug Log (Tiếng Việt)

> Toàn bộ chuỗi log (`gcs().send_text(...)`) trong `AP_ShoesAgtech.cpp` đã được
> chuyển sang **tiếng Anh** để tương thích tốt hơn với GCS/QGC/Mission Planner
> (tránh lỗi font, dễ tìm kiếm/log lại). File này là bảng tra cứu song ngữ +
> giải thích ý nghĩa, dành cho người đọc tiếng Việt. Comment trong code vẫn
> giữ nguyên tiếng Việt, không đổi.

---

## 1. Quy ước tiền tố (prefix)

| Prefix             | Module                      | Ý nghĩa                                                         |
| ------------------ | --------------------------- | --------------------------------------------------------------- |
| `ShoesAgtech: ...` | Khởi tạo                    | Log lúc `init()`, trước khi có tiền tố `SA:`                    |
| `SA: ...`          | Chung / Module 1 / Module 3 | Cảnh báo, trạng thái chung                                      |
| `SA FM1: ...`      | Module 1 (Flow Mode 1)      | Cảnh báo riêng cho `SA_FLOW_MODE=1` (công thức tank+mission)    |
| `SA DOS2: ...`     | Module 3 (Dosing Mode 2)    | Cảnh báo riêng cho `SA_DOS_MODE=2` (đổi tên từ "SA DOS1" 2026-08-20, khớp số mode mới — xem mục 7) |
| `[FLOW] ...`       | Module 1                    | Log định kỳ lưu lượng (bật bằng `SA_FLOW_LOG`)                  |
| `[DOS] ...`        | Module 3                    | Log định kỳ motor cho ăn (bật bằng `SA_DOS_LOG`)                |
| `[WM] ...`         | Module 2                    | Log pH / độ kiềm (bật bằng `SA_PH_LOG`)                         |
| `[SA] ...`         | Module 2 mở rộng            | Theo dõi ao (pond tracking), lưu/đọc SD card                    |
| `[SIM][...]`       | Bất kỳ                      | Thêm khi `SA_SIM=1` — dữ liệu giả lập, không phải cảm biến thật |

---

## 2. Module 1 — Điều khiển bơm vi sinh (Flow)

> **⚠️ Rút gọn toàn diện (2026-08-20):** đã xóa log cấu hình servo, toàn bộ
> thông báo lúc vừa ARM, và "SA: Tank lasts"; log định kỳ rút từ 2 dòng chi
> tiết còn đúng 1 dòng `FM<x> N<nấc> Q:<target>` (`FM`=SA_FLOW_MODE, `N`=nấc gạt — tách 2 trường 2026-09-04 để tránh nhầm lẫn). Bảng dưới đây là danh sách ĐẦY
> ĐỦ VÀ DUY NHẤT các log Module 1 còn lại — xem mục 8 để biết chi tiết
> những gì đã bỏ.

| Log tiếng Anh (mới)                                                          | Mức       | Giải thích                                                                                                                                                                              |
| ----------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ShoesAgtech: IRQ attach failed`                                             | CRITICAL  | Không gắn được ngắt (interrupt) cho chân `SA_FLOW_PIN` — cảm biến flow sẽ không đếm xung được.                                                                                          |
| `ShoesAgtech: Flow sensor ready`                                             | INFO      | Khởi tạo cảm biến flow (YF-S402B) thành công.                                                                                                                                           |
| `SA: TANK EMPTY - flow %.1fL/min > 1.7 for 5s`                               | INFO      | Phát hiện hết vi sinh trong thùng: lưu lượng thực tế đo được > 1.7 L/ph liên tục 5 giây dù đang bơm — dấu hiệu bơm chạy không tải (thùng cạn). 1 lần/phiên ARM.                          |
| `SA FM1: no mission - pump stopped`                                          | WARNING   | **`SA_FLOW_MODE=1`**: `mission_dist ≤ 1m` — chưa upload mission lên FC, hoặc mission không có ≥2 waypoint NAV hợp lệ (tọa độ khác 0,0). `flow_target=0`, bơm dừng. Lặp mỗi 5s.          |
| `SA FM1: q1=<x>L/min < 0.9 (pump range) - shorten mission or increase speed` | WARNING   | q1 tính ra thấp hơn 0.9 L/ph — **dải lưu lượng THẬT bơm hiện tại đạt được** (đo thực tế, không phải ngưỡng nghiệp vụ), mission quá dài so với `SA_TANK_VOL × SA_MIX_STD × tốc độ`. Bơm dừng. **Chỉ in 1 lần/phiên ARM**, không lặp mỗi 5s. |
| `SA FM1: q1=<x>L/min > 1.2 (pump range) - lengthen mission or reduce speed`  | WARNING   | q1 tính ra cao hơn 1.2 L/ph — vượt dải lưu lượng THẬT bơm đạt được, mission quá ngắn so với tank/tốc độ. Bơm dừng. **Chỉ in 1 lần/phiên ARM**, không lặp mỗi 5s.                          |
| `%s FM<m> N<n> Q: <x>`                                                       | INFO      | Log định kỳ duy nhất (`SA_FLOW_LOG`): `%s`=`[FLOW]`/`[SIM][FLOW]`, `m`=`SA_FLOW_MODE` hiện tại (0/1), `n`=nấc gạt (spray_mode+1, 1/2/3), `<x>`=`_flow_target` — 0 ở nấc 1 (manual), số cố định ở `SA_FLOW_MODE=0`, số dao động ở `SA_FLOW_MODE=1`. |

---

## 3. Module 2 — pH / độ kiềm / theo dõi ao

| Log tiếng Anh (mới)                                         | Mức     | Giải thích                                                                                                                                |
| ----------------------------------------------------------- | ------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `SA: pH sensor on SERIAL<n> (Modbus RTU 9600)`              | INFO    | Khởi tạo cảm biến pH (Modbus RTU) thành công trên cổng SERIAL<n>.                                                                         |
| `SA: pH sensor SERIAL<n> not found`                         | WARNING | Không tìm thấy cổng SERIAL đã cấu hình trong `SA_PH_PORT`.                                                                                |
| `SA: pH sensor no data yet - check RS485 wiring`            | WARNING | Chưa từng nhận được frame hợp lệ nào từ lúc khởi động — kiểm tra dây RS485 A/B.                                                           |
| `SA: pH sensor lost connection (<x>s) - check RS485 wiring` | WARNING | Đã từng nhận dữ liệu nhưng mất kết nối `<x>` giây — kiểm tra dây/nguồn cảm biến.                                                          |
| `SA: pH CRC fail (noise on RS485?)`                         | WARNING | Frame Modbus nhận được bị lỗi CRC — nghi nhiễu tín hiệu trên đường truyền RS485.                                                          |
| `%s pH:<x> MA:<y> Tmp:<z>C mV:<w> [<slot>]`                 | INFO    | Log định kỳ pH: giá trị thô, trung bình trượt (MA), nhiệt độ, điện thế mV, và slot hiện tại (AM/PM/...).                                  |
| `%s Alk:<x>dKH <y>mg/L dPH:<z>`                             | INFO    | Log định kỳ độ kiềm đã tính (nếu đủ dữ liệu cả 2 slot sáng/chiều).                                                                        |
| `%s AM:<h>:<mm>-<h>:<mm> PM:<h>:<mm>-<h>:<mm>`              | INFO    | Khung giờ lấy mẫu sáng (AM)/chiều (PM) đang cấu hình (`SA_PH_MS/ME/AS/AE`).                                                               |
| `[WM] No GPS - alkalinity waiting for GPS/time`             | INFO    | Chưa có giờ UTC từ GPS (hoặc GPS chưa fix 3D) — không thể phân loại mẫu vào slot sáng/chiều, tính năng kiềm tạm dừng. Lặp tối đa mỗi 60s. |
| `[SA] Switched to pond #<n>` / `... (new pond)`             | INFO    | Đổi sang ao khác theo `SA_POND_IDX` (in 1 lần mỗi lần đổi). `(new pond)` nếu ao này chưa từng ghi nhận.                                   |
| `[SA] Pond#<n> GPS OK (<x>m)`                               | INFO    | Vị trí GPS hiện tại khớp với tâm ao đã lưu (trong ngưỡng `SA_PH_POND_DIST`).                                                              |
| `[SA] Pond#<n> GPS off by <x>m - check pond location`       | INFO    | Vị trí GPS lệch xa so với tâm ao đã lưu — có thể chọn nhầm `SA_POND_IDX` hoặc vị trí ao lưu sai.                                          |
| `[SA] Pond#<n> pH AM: <x> (<n> samples)`                    | INFO    | Đã đủ số mẫu tối thiểu cho slot sáng của ao, in giá trị pH slot sáng.                                                                     |
| `[SA] Pond#<n> pH PM: <x> (<n> samples)`                    | INFO    | Tương tự cho slot chiều.                                                                                                                  |
| `[SA] Pond#<n> AM:<x> PM:<y> dPH:<z>`                       | INFO    | Đủ cả 2 slot, tính được ΔpH (chênh lệch chiều − sáng).                                                                                    |
| `[SA] Pond#<n> Alk:<x>dKH/<y>mgL`                           | INFO    | Độ kiềm tính được cho ao, theo cả 2 đơn vị dKH và mg/L.                                                                                   |
| `SA: could not save pond data to SD (card full/error?)`     | WARNING | Ghi dữ liệu ao xuống SD thất bại (thiếu byte ghi) — nghi thẻ SD đầy hoặc lỗi. Lặp mỗi 60s để không mất dữ liệu âm thầm.                   |
| `[SA] Loaded <n> ponds from SD card`                        | INFO    | Đọc lại thành công `<n>` ao đã lưu từ SD lúc khởi động.                                                                                   |

---

## 4. Module 3 — Motor cho ăn (Dosing)

| Log tiếng Anh (mới)                                                       | Mức     | Giải thích                                                                                                     |
| ------------------------------------------------------------------------- | ------- | -------------------------------------------------------------------------------------------------------------- |
| `SA: SERVO<m> setup OK - dosing motor ready`                              | INFO    | Servo motor cho ăn đủ 4 điều kiện cấu hình (FUNCTION/MIN/TRIM/MAX), sẵn sàng chạy.                             |
| `SA: SERVO<m> does not exist`                                             | WARNING | Kênh servo cấu hình trong `SA_DOS_CHAN` không tồn tại trên board.                                              |
| `SA: SERVO<m> FUNCTION=<x>, must set =0 (None)`                           | WARNING | Sai `SERVOx_FUNCTION`, cần đặt về 0 (None) để code ghi PWM trực tiếp.                                          |
| `SA: SERVO<m> MIN=<x>, must set =800`                                     | WARNING | Sai `SERVOx_MIN`.                                                                                              |
| `SA: SERVO<m> TRIM=<x>, must set =1500`                                   | WARNING | Sai `SERVOx_TRIM`.                                                                                             |
| `SA: SERVO<m> MAX=<x>, must set =2200`                                    | WARNING | Sai `SERVOx_MAX`.                                                                                              |
| `SA: Dosing motor ON` / `OFF`                                             | INFO    | RC bật/tắt motor cho ăn thủ công.                                                                              |
| `SA DOS2: no mission (dist=<x>m) - motor stopped`                         | WARNING | **`SA_DOS_MODE=2`**: chưa upload mission (tương tự lỗi FM1 ở Module 1) — motor dừng (PWM=1500). Lặp mỗi 5s.    |
| `SA DOS2: speed too low (<x>m/s < <y>m/s min) - motor stopped`            | WARNING | **`SA_DOS_MODE=2`**: tốc độ chưa đạt `<y>` = max(0.05 m/s, `SA_DOS_SPD_PCT`% tốc độ ĐẶT cho mission, mặc định 50%) — motor chưa rải (tránh dồn liều lúc xe mới tăng tốc/qua cua). Đặt `SA_DOS_SPD_PCT=0` để tắt kiểm tra này, về hành vi cũ (chỉ cần vượt 0.05 m/s). Cập nhật 2026-08-19, trước đây ngưỡng cố định 0.05 m/s. Lặp mỗi 5s. |
| `[DOS] M0 SERVO<c> <ON/OFF> PWM:<w>`                                      | INFO    | Log định kỳ `SA_DOS_MODE=0` (PWM trực tiếp, mới 2026-08-20): không có thức ăn/tốc độ, chỉ in PWM đang xuất — dùng khi hiệu chuẩn tại bàn. |
| `[DOS] M1 F<n> SERVO<c> <ON/OFF> Rate:<x>g/min D:<y>g/mL PWM:<w>`         | INFO    | Log định kỳ `SA_DOS_MODE=1` (đổi số từ 0 cũ, 2026-08-20): F=loại thức ăn active, Rate=tốc độ cấp cố định (g/phút), D=tỷ trọng, PWM=xung ra. |
| `[DOS] M2 F<n> SERVO<c> <ON/OFF> SP:<x>g Rate:<y>g/min D:<z>g/mL PWM:<w>` | INFO    | Log định kỳ `SA_DOS_MODE=2` (đổi số từ 1 cũ, 2026-08-20): SP=tổng gam cho cả mission, Rate=tốc độ tức thời suy ra từ speed/mission_dist.    |
| `SA: SA_DOS_F<n>=<x> looks uncalibrated for new V x fill-factor formula (expected ~0.05-2.0)` | WARNING | **Chỉ xuất hiện sau khi đổi công thức hiệu chuẩn (xem mục 6)**: `SA_DOS_Fx` đang lớn hơn 5.0 — nghi vẫn còn giá trị cũ (thang mL/50us, thường ~100) từ trước khi tách `SA_DOS_V x SA_DOS_Fx`, chưa được đo/hiệu chuẩn lại theo công thức mới. Nếu không sửa, lượng thức ăn cấp ra sẽ sai (thường là quá ít). Lặp mỗi 5s. |

---

## 5. Phân tích lỗi đã gặp: setpoint = 0 khi `SA_FLOW_MODE=1` lúc chạy AUTO

> **⚠️ Đã thay đổi (2026-08-19):** ngưỡng trần trên `q1 > 2.0` mô tả dưới đây
> **đã được gỡ bỏ khỏi code** theo yêu cầu — bơm không còn tự khóa khi mission
> quá ngắn nữa. Mục này giữ lại làm lịch sử/tham khảo cơ chế cũ; xem ghi chú
> cuối mục để biết hành vi hiện tại.

**Triệu chứng:** Đặt `SA_FLOW_MODE=1`, khi xe chạy AUTO thì `flow_target` (setpoint bơm) về 0, bơm không chạy dù xe đang di chuyển theo mission.

**Nguyên nhân xác nhận qua GCS:** log cảnh báo `SA FM1: q1=<x>L/min > 2.0 ...` xuất hiện.

### Cơ chế

Khi `SA_FLOW_MODE=1` và `SA_TANK_VOL > 0`, setpoint mode 1 **không** lấy từ `SA_FLOW_SP` nữa mà tính theo công thức phân bố đều vi sinh trên toàn tuyến mission
(`_compute_visin_target()`, [AP_ShoesAgtech.cpp:967-1018](AP_ShoesAgtech.cpp#L967-L1018)):

```
q1 = SA_TANK_VOL × SA_MIX_STD × speed × 60 / mission_dist        (L/phút)
```

Đây là **lưu lượng bơm cần thiết** để toàn bộ `SA_TANK_VOL × SA_MIX_STD` lít vi sinh
được phun đều hết trên quãng đường `mission_dist`, với tốc độ xe hiện tại `speed`.

Vì lý do an toàn/thực tế phần cứng (bơm màng có dải lưu lượng hoạt động ổn định
giới hạn), code chỉ cho bơm chạy khi:

```
0.3 L/phút  ≤  q1  ≤  2.0 L/phút
```

Nếu `q1 > 2.0` (như trường hợp của bạn) nghĩa là: **mission quá ngắn** so với
lượng vi sinh cần phun (`SA_TANK_VOL × SA_MIX_STD`) ở tốc độ đang chạy — nếu cứ
bơm sẽ phun quá đậm đặc trong thời gian ngắn. Code chủ động **khóa bơm về 0**
thay vì bơm sai liều lượng.

### Cách tính khoảng cách tối thiểu cần thiết

```
dist_min = SA_TANK_VOL × SA_MIX_STD × speed × 60 / 2.0
```

Giá trị `dist_min` này được in kèm ngay trong log lúc ARM:
`"SA FM1: q1=... @<speed>m/s dist=<dist>m - lengthen mission (dmin=<dist_min>m)"`
([AP_ShoesAgtech.cpp:1058-1061](AP_ShoesAgtech.cpp#L1058-L1061)) — so `dist_min`
này với độ dài mission hiện tại để biết cần kéo dài thêm bao nhiêu mét.

### Cách khắc phục (chọn 1 hoặc kết hợp)

1. **Kéo dài mission** — thêm waypoint/quãng đường để tổng khoảng cách giữa
   các chặng NAV (WP/LOITER/SPLINE) ≥ `dist_min`.
2. **Giảm tốc độ AUTO** (`WP_SPEED` / `CRUISE_SPEED`) — `q1` tỉ lệ thuận với
   `speed`, giảm tốc độ sẽ giảm `q1`.
3. **Giảm `SA_TANK_VOL` hoặc `SA_MIX_STD`** — nếu giá trị đang đặt cao hơn
   thực tế bình chứa / tỷ lệ pha mong muốn ở nấc giữa.

### Ghi chú thêm

- Trường hợp `speed < 0.1 m/s` (xe đứng yên) cũng khiến `flow_target = 0`
  nhưng **không in cảnh báo nào** ra GCS (khác với 2 case còn lại) — nếu sau
  này gặp bơm dừng mà không rõ nguyên nhân và không thấy log `SA FM1: ...`,
  nhiều khả năng đây là do tốc độ dùng trong công thức (`SA_FLOW_VEL`, hoặc
  từ 2026-08-19 là **tốc độ ĐẶT `WP_SPEED`** — xem mục dưới, không phải GPS
  tức thời nữa) đang dưới 0.1 m/s.
- Nếu `SA_TANK_VOL = 0` (giá trị mặc định), code **không** vào công thức
  trên — setpoint sẽ fallback về `SA_FLOW_SP` (mặc định 5.0 L/ph), không
  bao giờ về 0 trong trường hợp đó.
- **Hành vi hiện tại (từ 2026-08-20, thay cho mô tả 2026-08-19 ở trên):**
  sau khi đo thực tế phát hiện bơm chỉ đạt lưu lượng thật 0.9–1.2 L/phút
  trên toàn dải PWM MIN→MAX, đã **thêm lại** cả sàn và trần cho `q1`,
  nhưng đổi số thành đúng dải phần cứng thật: `q1 < 0.9` hoặc `q1 > 1.2`
  đều khiến bơm dừng (không phải ngưỡng nghiệp vụ 0.3/2.0 cũ). Khác biệt
  quan trọng: cảnh báo này **chỉ in 1 lần mỗi phiên ARM** (cờ
  `_q1_range_warned`), KHÔNG lặp lại mỗi 5s như cảnh báo "no mission". Xem
  mục 2 và `MODULE1_FLOW_DETAIL_DESIGN.md` mục 3.5.
- **Nguồn `speed` trong công thức đổi từ 2026-08-19:** trước đây `speed` là
  tốc độ GPS TỨC THỜI (`AP::ahrs().groundspeed()`), khiến `q1`/setpoint bơm
  dao động theo từng cú tăng/giảm tốc, vào cua — gây phun không đều dọc
  tuyến. Từ nay dùng tốc độ **ĐẶT** cho mission (`WP_SPEED`, cập nhật qua
  `DO_CHANGE_SPEED`/GCS `SET_SPEED`) — ổn định suốt cả đoạn, chỉ đổi khi kỹ
  thuật viên chủ động đổi tốc độ. `SA_SIM`/`SA_FLOW_VEL` vẫn ưu tiên như cũ
  để hiệu chỉnh/test khi xe đứng yên. Xem `MODULE1_FLOW_DETAIL_DESIGN.md`
  mục 1.2 (`_get_dosing_ref_speed()`) để biết chi tiết.
- **⚠️ Bug đã gặp và sửa (2026-08-20) — FLOW_MODE=1 không bao giờ chạy dù
  ARM/gạt nấc đúng:** tốc độ ĐẶT (`_target_speed`) ở trên chỉ có giá trị
  thật SAU KHI đã vào chế độ AUTO ít nhất 1 lần kể từ lúc mở nguồn — nếu
  chỉ ARM ở Manual để test tay/gạt nấc mà chưa từng chạy AUTO phiên đó,
  `_target_speed` sẽ luôn = 0, khiến hệ thống tưởng xe đứng yên mãi mãi dù
  xe đang chạy thật (rơi đúng vào case "speed<0.1 im lặng, không cảnh
  báo" ở trên — rất khó nhận ra). Đã thêm tầng dự phòng: nếu `_target_speed`
  vẫn = 0, quay lại dùng AHRS groundspeed như trước đây, để hệ thống vẫn
  hoạt động được khi test ngoài AUTO.

---

## 6. Đổi công thức hiệu chuẩn Module 3: tách `SA_DOS_Fx` thành `SA_DOS_V x SA_DOS_Fx`

**Trước đây:** `SA_DOS_Fx` (x = loại thức ăn 1-7) là một con số lưu lượng vít
tải (mL/50us) đo trực tiếp — chạy motor, cân sản lượng, suy ngược ra một con
số duy nhất cho mỗi loại thức ăn (mặc định 100.0).

**Bây giờ:** `SA_DOS_Fx` **đổi ý nghĩa** thành hệ số điền đầy hạt (fill
factor, không thứ nguyên, dải hợp lệ ~0.05–2.0, mặc định 1.0) — bù cho
khoảng trống không khí giữa các hạt thức ăn trong vít tải, khác nhau theo
từng loại thức ăn. Thêm tham số mới `SA_DOS_V` (mL/50us, mặc định 100.0,
**dùng chung cho mọi loại thức ăn**) — hằng số hình học của trục vít (chỉ
phụ thuộc đường kính/bước ren vít + tương quan PWM↔tốc độ quay), chỉ cần đổi
khi thay trục vít vật lý khác.

Công thức mới: `lưu lượng hiệu dụng = SA_DOS_V x SA_DOS_Fx x SA_DOS_Dx`
(trước đây là `SA_DOS_Fx x SA_DOS_Dx`).

**Cách hiệu chuẩn từ nay:**
1. Đo/tính `SA_DOS_V` **một lần** cho trục vít đang lắp (theo hình học vít,
   hoặc chạy thử với 1 loại vật liệu chảy tự do gần như lấp đầy 100%).
2. Với mỗi loại thức ăn mới, **chỉ cần điều chỉnh `SA_DOS_Fx`** (chạy motor,
   cân sản lượng thực tế, so với sản lượng lý thuyết nếu điền đầy 100% để
   suy ra hệ số) — không cần đo lại từ đầu như trước.

**⚠️ Bắt buộc khi cập nhật firmware lên máy đã hiệu chuẩn từ trước:** vì
`SA_DOS_Fx` vẫn giữ nguyên tên/tham số (chỉ đổi Ý NGHĨA con số), máy nào đã
có `SA_DOS_Fx` ~100 theo công thức cũ sẽ tự động bị hiểu nhầm thành hệ số
điền đầy = 100 (rất vô lý), dẫn đến cho ăn sai (thường là quá ít, do lưu
lượng hiệu dụng bị tính vọt lên gấp trăm lần thực tế). Hệ thống có cảnh báo
`SA: SA_DOS_F<n>=<x> looks uncalibrated for new V x fill-factor formula` khi
phát hiện giá trị > 5.0 (xem mục 4), nhưng **mọi máy đã hiệu chuẩn trước khi
đổi công thức đều cần hiệu chuẩn lại `SA_DOS_Fx` theo quy trình mới ở trên**
trước khi dùng, không chỉ dựa vào cảnh báo tự động này.

---

## 7. Thêm mode PWM trực tiếp + đổi số 2 mode cũ ở Module 3 (2026-08-20)

**Thêm mới:** `SA_DOS_MODE=0` — mode PWM trực tiếp. `SA_DOS_SP` ở mode này
được hiểu là **xung PWM tuyệt đối (µs)**, ghi thẳng ra servo sau khi
constrain về đúng dải `SERVOx_MIN..MAX` — hoàn toàn không qua công thức
`SA_DOS_V`/`SA_DOS_Fx`/`SA_DOS_Dx` và không qua `SA_DOS_REV`. Mục đích:
hiệu chuẩn tại bàn (đo RPM/sản lượng ở 1 mức PWM biết trước — xem mục 6 và
`MODULE3_DOS_DETAIL_DESIGN.md` mục 3.6) mà không cần công cụ test servo
riêng của GCS. An toàn: `SA_DOS_SP ≤ 0` (giá trị mặc định, chưa từng chỉnh)
→ xuất PWM=1500 (dừng), KHÔNG kẹp về PWM tối thiểu (có thể là tốc độ tối
đa) — tránh trường hợp bật RC dosing mà quên set `SA_DOS_SP` khiến motor
chạy full tốc ngoài ý muốn.

**⚠️ Đổi số 2 mode cũ:** tốc độ cố định (cũ = `SA_DOS_MODE=0`) → **1**; tỉ
lệ theo mission (cũ = `SA_DOS_MODE=1`) → **2**. Các câu lệnh cảnh báo
`"SA DOS1: ..."` đổi tên thành `"SA DOS2: ..."` cho khớp số mode mới (xem
mục 4). **Máy đã cấu hình sẵn `SA_DOS_MODE=1` (nghĩa cũ = tỉ lệ mission) sẽ
tự động đổi ý nghĩa thành "tốc độ cố định" (nghĩa mới của số 1) sau khi cập
nhật firmware — bắt buộc kiểm tra/chỉnh lại giá trị tham số này trên mọi
máy đã triển khai trước khi dùng.**

**Tham số tái sử dụng, không thêm mới:** vì bảng tham số `AP_ShoesAgtech`
đã hết chỗ trống (64/64 slot), mode PWM trực tiếp tái sử dụng `SA_DOS_SP`
(đã sẵn có, chỉ đổi ý nghĩa theo mode) thay vì tạo tham số riêng.

---

## 8. Rút gọn toàn diện log Module 1 (2026-08-20)

Theo yêu cầu: log Module 1 (bơm vi sinh) được rút gọn tối đa, chỉ giữ lại
những gì thật sự cần hành động. Bảng đầy đủ hiện tại xem mục 2.

**Đã xóa hoàn toàn (không còn in nữa):**
- `SA: SERVO<n>_FUNCTION=<x> must be 0(None)!` / `SA: SERVO<n> OK Min/Trim/Max`
  — log cấu hình servo trong `_check_pump_config()`. Hàm vẫn kiểm tra bình
  thường (`_pump_config_ok`), chỉ không in log — muốn kiểm tra cấu hình
  servo thì xem trực tiếp trên Mission Planner (Servo Output).
- Toàn bộ 4 STATUSTEXT lúc vừa ARM (`SA FM1: no mission - pump will stay
  stopped`, `SA FM1 READY: ...`, 2 cảnh báo `q1=... @...dist=...(dmax/dmin)`,
  `SA FM1 OK: ...`) — hàm `_print_fm1_arm_status()` đã bị xóa hoàn toàn
  khỏi code (không phải tắt, mà xóa hẳn function + lời gọi + biến
  `_was_armed`/`_arm_dist_warned` không còn dùng).
- `SA: Tank lasts ~<x>m (<y>L @<z>L/min)` — ước tính quãng đường còn lại ở
  mode 2, cùng với nhánh code tính toán nó (không còn tác dụng gì khác).

**Đơn giản hoá log định kỳ (`SA_FLOW_LOG`):** từ 2 dòng
(`M<n> Tgt/Act/Avg/PWM` + `FM1 r/q1/miss/dmax/spd/vi_run`) rút còn **đúng 1
dòng**: `FM<m> N<n> Q: <target>` — `<m>` = `SA_FLOW_MODE` hiện tại (0/1,
tách riêng 2026-09-04 vì ban đầu gộp chung với nấc gây nhầm lẫn), `<n>` =
nấc gạt (spray_mode+1, hiển thị đúng 1/2/3 khớp vị trí gạt vật lý thay vì
spray_mode nội bộ 0/1/2), `<target>` = `_flow_target` (0 ở manual, cố định
ở FLOW_MODE=0, dao động ở FLOW_MODE=1).

**Vẫn giữ nguyên 3 cảnh báo có hành động cần làm:** `SA: TANK EMPTY`,
`SA FM1: no mission`, `SA FM1: q1=... (pump range)` (2 chiều) — đây là các
trường hợp người vận hành cần biết để xử lý, khác với các log đã xóa vốn
chỉ mang tính thông tin/xác nhận.

---

## 9. Lưu ý về tài liệu liên quan

File [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) (mục 6 —
"Chuẩn đoán & cảnh báo console") có bảng log tương tự nhưng được viết từ
trước và **chưa cập nhật theo bản dịch tiếng Anh mới nhất**, đồng thời thiếu
một số dòng log (vd. cảnh báo `q1 < 0.3` / `q1 > 2.0`, log `FM1 READY`/`FM1 OK`,
các log theo dõi ao `[SA] Pond#...`). Coi file này (`DEBUG_LOG_GIAI_THICH.md`)
là nguồn tham chiếu log mới nhất; nếu cần, cập nhật lại bảng trong
`AP_SHOESAGTECH_REFERENCE.md` cho khớp.
