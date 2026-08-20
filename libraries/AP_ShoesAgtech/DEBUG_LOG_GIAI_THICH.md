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
| `SA DOS1: ...`     | Module 3 (Dosing Mode 1)    | Cảnh báo riêng cho `SA_DOS_MODE=1`                              |
| `[FLOW] ...`       | Module 1                    | Log định kỳ lưu lượng (bật bằng `SA_FLOW_LOG`)                  |
| `[DOS] ...`        | Module 3                    | Log định kỳ motor cho ăn (bật bằng `SA_DOS_LOG`)                |
| `[WM] ...`         | Module 2                    | Log pH / độ kiềm (bật bằng `SA_PH_LOG`)                         |
| `[SA] ...`         | Module 2 mở rộng            | Theo dõi ao (pond tracking), lưu/đọc SD card                    |
| `[SIM][...]`       | Bất kỳ                      | Thêm khi `SA_SIM=1` — dữ liệu giả lập, không phải cảm biến thật |

---

## 2. Module 1 — Điều khiển bơm vi sinh (Flow)

| Log tiếng Anh (mới)                                                         | Mức                                       | Giải thích                                                                                                                                                                              |
| --------------------------------------------------------------------------- | ----------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `ShoesAgtech: IRQ attach failed`                                            | CRITICAL                                  | Không gắn được ngắt (interrupt) cho chân `SA_FLOW_PIN` — cảm biến flow sẽ không đếm xung được.                                                                                          |
| `ShoesAgtech: Flow sensor ready`                                            | INFO                                      | Khởi tạo cảm biến flow (YF-S402B) thành công.                                                                                                                                           |
| `SA: SERVO<n>_FUNCTION=<x> must be 0(None)!`                                | WARNING                                   | Kênh servo bơm chưa đặt `SERVOx_FUNCTION=0`, PWM ghi trực tiếp sẽ bị hệ thống servo ghi đè. Lặp mỗi 5s.                                                                                 |
| `SA: SERVO<n> OK Min:<x> Trim:<y> Max:<z>`                                  | INFO                                      | Cấu hình servo bơm hợp lệ.                                                                                                                                                              |
| `SA: TANK EMPTY - flow %.1fL/min > 1.7 for 5s`                              | CRITICAL                                  | Phát hiện hết vi sinh trong thùng: lưu lượng thực tế đo được > 1.7 L/ph liên tục 5 giây (tăng từ 3s, 2026-08-19) dù đang bơm — dấu hiệu bơm chạy không tải (thùng cạn).                 |
| `SA: Tank lasts ~<x>m (<y>L @<z>L/min)`                                     | INFO                                      | (Chỉ Mode 2, khi `SA_TANK_VOL>0`) Ước tính còn bơm được bao nhiêu mét với tốc độ/lưu lượng hiện tại. Lặp mỗi 30s.                                                                       |
| `SA FM1: no mission - pump stopped`                                         | WARNING                                   | **Mode 1 + `SA_FLOW_MODE=1`**: `mission_dist ≤ 1m` — chưa upload mission lên FC, hoặc mission không có ≥2 waypoint NAV hợp lệ (tọa độ khác 0,0). `flow_target=0`, bơm dừng. Lặp mỗi 5s. |
| `SA FM1: q1=<x>L/min < 0.3 - shorten mission or increase speed`             | WARNING                                   | q1 tính ra quá thấp (dưới ngưỡng an toàn 0.3 L/ph) — mission quá dài so với `SA_TANK_VOL × SA_MIX_STD × tốc độ`. Bơm dừng để tránh phun quá loãng.                                      |
| `SA FM1 READY: r=<x> miss=<y>m bio/run=<z>L                                 | speed=0 pump waiting for vehicle to move` | INFO                                                                                                                                                                                    | In một lần khi ARM: mission hợp lệ nhưng xe đang đứng yên, bơm chờ xe di chuyển. |
| `SA FM1 OK: r=<x> q1=<y>L/min miss=<z>m dmax=<w>m ~<mm>m<ss>s bio/run=<v>L` | INFO                                      | In một lần khi ARM: mọi điều kiện hợp lệ, kèm ETA hoàn thành mission.                                                                                                                   |
| `%s M<n> Tgt:<x> Act:<y> Avg:<z> PWM:<w>`                                   | INFO                                      | Log định kỳ (`SA_FLOW_LOG`): `%s`=`[FLOW]`/`[SIM][FLOW]`, `n`=spray_mode, Tgt=setpoint, Act=lưu lượng thực đo (đã lọc), Avg=trung bình trượt, PWM=xung ra bơm.                          |
| `%s FM1 r:<x> q1:<y>L/min miss:<z>m dmax:<w>m spd:<v>m/s bio/run:<u>L`      | INFO                                      | Log định kỳ bổ sung khi Mode 1/2 + `SA_FLOW_MODE=1`: chi tiết công thức q1 hiện tại.                                                                                                    |

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
| `SA DOS1: no mission (dist=<x>m) - motor stopped`                         | WARNING | **`SA_DOS_MODE=1`**: chưa upload mission (tương tự lỗi FM1 ở Module 1) — motor dừng (PWM=1500). Lặp mỗi 5s.    |
| `SA DOS1: speed too low (<x>m/s < <y>m/s min) - motor stopped`            | WARNING | **`SA_DOS_MODE=1`**: tốc độ chưa đạt `<y>` = max(0.05 m/s, `SA_DOS_SPD_PCT`% tốc độ ĐẶT cho mission, mặc định 50%) — motor chưa rải (tránh dồn liều lúc xe mới tăng tốc/qua cua). Đặt `SA_DOS_SPD_PCT=0` để tắt kiểm tra này, về hành vi cũ (chỉ cần vượt 0.05 m/s). Cập nhật 2026-08-19, trước đây ngưỡng cố định 0.05 m/s. Lặp mỗi 5s. |
| `[DOS] M0 F<n> SERVO<c> <ON/OFF> Rate:<x>g/min D:<y>g/mL PWM:<w>`         | INFO    | Log định kỳ `SA_DOS_MODE=0`: F=loại thức ăn active, Rate=tốc độ cấp cố định (g/phút), D=tỷ trọng, PWM=xung ra. |
| `[DOS] M1 F<n> SERVO<c> <ON/OFF> SP:<x>g Rate:<y>g/min D:<z>g/mL PWM:<w>` | INFO    | Log định kỳ `SA_DOS_MODE=1`: SP=tổng gam cho cả mission, Rate=tốc độ tức thời suy ra từ speed/mission_dist.    |
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
- **Hành vi hiện tại (từ 2026-08-19):** cảnh báo `q1 > 2.0` và việc khóa bơm
  khi mission quá ngắn **không còn tồn tại**. `q1` chỉ còn bị chặn ở sàn dưới
  `0.3 L/phút` (mission quá dài/xe quá chậm); không còn trần trên — bơm sẽ
  chạy dù `q1` tính ra rất cao (phun đậm đặc trên mission ngắn), không tự
  dừng và không cảnh báo cho trường hợp này nữa. Nếu cần giới hạn liều lượng
  tối đa, phải tự theo dõi thủ công (log `[FLOW]`/`[SIM][FLOW]` định kỳ vẫn
  in `q1` thực tế, xem mục 2).
- **Nguồn `speed` trong công thức đổi từ 2026-08-19:** trước đây `speed` là
  tốc độ GPS TỨC THỜI (`AP::ahrs().groundspeed()`), khiến `q1`/setpoint bơm
  dao động theo từng cú tăng/giảm tốc, vào cua — gây phun không đều dọc
  tuyến. Từ nay dùng tốc độ **ĐẶT** cho mission (`WP_SPEED`, cập nhật qua
  `DO_CHANGE_SPEED`/GCS `SET_SPEED`) — ổn định suốt cả đoạn, chỉ đổi khi kỹ
  thuật viên chủ động đổi tốc độ. `SA_SIM`/`SA_FLOW_VEL` vẫn ưu tiên như cũ
  để hiệu chỉnh/test khi xe đứng yên. Xem `MODULE1_FLOW_DETAIL_DESIGN.md`
  mục 1.2 (`_get_dosing_ref_speed()`) để biết chi tiết.

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

## 7. Lưu ý về tài liệu liên quan

File [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) (mục 6 —
"Chuẩn đoán & cảnh báo console") có bảng log tương tự nhưng được viết từ
trước và **chưa cập nhật theo bản dịch tiếng Anh mới nhất**, đồng thời thiếu
một số dòng log (vd. cảnh báo `q1 < 0.3` / `q1 > 2.0`, log `FM1 READY`/`FM1 OK`,
các log theo dõi ao `[SA] Pond#...`). Coi file này (`DEBUG_LOG_GIAI_THICH.md`)
là nguồn tham chiếu log mới nhất; nếu cần, cập nhật lại bảng trong
`AP_SHOESAGTECH_REFERENCE.md` cho khớp.
