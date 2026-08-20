# Module 1 — Flow Sensor & Spray Controller
## Basic Design Document

> **Đây là tài liệu TRƯỚC KHI code.**
> Mô tả YÊU CẦU và HÀNH VI — không liên quan đến code, tham số hay kỹ thuật bên trong.
> Người không biết lập trình cũng đọc được và hiểu hệ thống làm gì.

**Dự án:** `ardupilot-jbdcan_testing_S16`
**Ngày tạo:** 2026-05-01 | **Cập nhật lần cuối:** 2026-07-16
**Người viết:** ThaiThanhDuy
**Trạng thái:** `[x] Draft   [ ] Review   [ ] Approved`

---

## 1. Loại thay đổi

```
[x] Tính năng mới hoàn toàn
[ ] Bổ sung vào hệ thống có sẵn
[ ] Sửa lỗi / thay đổi hành vi
```

> Thêm mới toàn bộ hệ thống điều khiển phun nước tự động dựa trên cảm biến lưu lượng — không có chức năng này trong ArduPilot gốc.

---

## 2. Mục đích

Module 1 điều khiển bơm phun vi sinh trên robot nông nghiệp.
Người lái chỉ cần **chọn chế độ bằng một nút RC** — hệ thống tự đọc lưu lượng thực tế và điều chỉnh bơm để đạt mục tiêu.

Dùng khi: phun vi sinh, thuốc, phân bón trên đồng/ao — cần phân phối đều và đúng lượng theo tuyến đường mission.

---

## 3. Phạm vi thay đổi

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| Điều khiển bơm phun | Có | Toàn bộ logic xuất tín hiệu bơm theo 3 chế độ |
| Giao tiếp GCS / MAVLink | Có | Gửi lưu lượng, mode, tình trạng bơm lên màn hình |
| Phần cứng / kết nối | Có | Thêm cảm biến lưu lượng và kênh servo điều khiển bơm |
| Điều hướng (navigation) | Không | Không thay đổi |
| Module pH, Dosing | Không | Không liên quan |

---

## 4. Phần cứng / Giao tiếp sử dụng

| Thiết bị | Vai trò | Kết nối | Ghi chú |
|---|---|---|---|
| Cảm biến lưu lượng YF-S402B | Đo lưu lượng vi sinh thực tế | Chân GPIO của FC | Dải 0.3–2.0 L/min thực tế |
| Bơm (servo PWM 360°) | Bơm vi sinh | Kênh servo của FC | Phải cài FUNCTION=0 |
| Tay lái / Remote RC | Người lái chọn chế độ | Kênh RC 3 nấc | Nấc thấp / giữa / cao |

---

## 5. Flow hoạt động

```
Người lái gạt nút RC chọn chế độ phun
    ↓
Cảm biến đếm xung liên tục → hệ thống tính lưu lượng thực (L/min)
    ↓
┌────────────────────────────────────────────────────────────────────┐
│  Nấc thấp → Chế độ 0 (LÁI TAY)                                   │
│    Tín hiệu joystick thứ 2 truyền thẳng ra bơm                    │
│                                                                    │
│  Nấc giữa → PID bám lưu lượng vi sinh (van Mặc định)             │
│    SA_FLOW_MODE=0: bám SA_FLOW_SP (người dùng calib tay)          │
│    SA_FLOW_MODE=1: bám công thức phân phối đều vi sinh            │
│        q1 = TANK_VOL × SA_MIX_STD × vận_tốc × 60 / mission_dist  │
│        → vi sinh phân phối đều toàn tuyến, luôn dùng đúng         │
│           TANK_VOL × SA_MIX_STD lít mỗi lần chạy                  │
│                                                                    │
│  Nấc cao → PID bám lưu lượng vi sinh (van Chống nghẹt)            │
│    SA_FLOW_MODE=0: bám SA_FLOW_SP × (SA_MIX_CNT/SA_MIX_STD)      │
│    SA_FLOW_MODE=1: bám công thức với SA_MIX_CNT                   │
│        q1 = TANK_VOL × SA_MIX_CNT × vận_tốc × 60 / mission_dist  │
└────────────────────────────────────────────────────────────────────┘
    ↓
Bơm điều chỉnh PWM theo PID
    ↓
Nếu phát hiện hút không khí (lưu lượng đột ngột > 1.7 L/min trong 5s)
    → Cảnh báo CRITICAL "THÙNG HẾT VI SINH" (bơm vẫn tiếp tục)
    ↓
Dữ liệu lưu lượng + trạng thái bơm gửi lên GCS liên tục
```

---

## 6. Tất cả Case và Output

### Case 1 — Nấc thấp: Lái tay (PASSTHROUGH)

- **Điều kiện:** Nút RC ở nấc thấp nhất
- **Hành vi:** Tín hiệu joystick thứ 2 truyền thẳng đến bơm, hệ thống không can thiệp
- **Output GCS:** Mode=0, lưu lượng đọc liên tục nhưng không điều khiển

### Case 2 — Nấc giữa, FLOW_MODE=0: bám setpoint cố định

- **Điều kiện:** Nút RC ở nấc giữa + SA_FLOW_MODE=0
- **Hành vi:** Bơm tự tăng/giảm để giữ lưu lượng vi sinh đúng SA_FLOW_SP
- **Output GCS:** Mode=1, lưu lượng thực và mục tiêu ổn định

### Case 3 — Nấc giữa, FLOW_MODE=1: phân phối vi sinh đều theo tuyến

- **Điều kiện:** RC nấc giữa + SA_FLOW_MODE=1 + đã upload mission + xe đang chạy
- **Hành vi:** Tự tính lưu lượng theo công thức `q1 = TANK_VOL × SA_MIX_STD × speed × 60 / mission_dist`. Lưu lượng tăng khi xe đi nhanh, giảm khi xe đi chậm — đảm bảo luôn dùng hết đúng `TANK_VOL × SA_MIX_STD` lít mỗi lần chạy mission.
- **Khi ARM:** GCS thông báo q1 ước tính, thời gian hoàn thành, lượng vi sinh sẽ dùng
- **Output GCS:** Mode=1, q1 thực tế, dist_max (khoảng tối đa theo tốc độ hiện tại), vi/run

### Case 4 — Nấc giữa hoặc cao, FLOW_MODE=1: không đủ điều kiện bơm

- **Điều kiện:** FLOW_MODE=1 nhưng: chưa upload mission / xe đứng yên / q1 tính ra < 0.3 L/min (mission quá dài hoặc xe quá chậm). (Từ 2026-08-19: **không còn** trần trên q1 — mission quá ngắn/xe quá nhanh không còn bị chặn, bơm vẫn chạy dù q1 lớn.)
- **Hành vi:** Bơm DỪNG + cảnh báo GCS giải thích lý do cụ thể
- **Output GCS:** Cảnh báo kèm gợi ý sửa: "rút ngắn mission", "chờ xe chạy"

### Case 5 — Nấc cao, FLOW_MODE=0: setpoint Chống nghẹt

- **Điều kiện:** RC nấc cao + SA_FLOW_MODE=0
- **Hành vi:** Setpoint vi sinh tăng theo tỉ lệ SA_MIX_CNT/SA_MIX_STD; van vi sinh giữ nguyên vị trí như nấc giữa, chỉ chỉnh van hồ
- **Output GCS:** Mode=2, setpoint cao hơn nấc giữa

### Case 6 — Nấc cao, FLOW_MODE=1: phân phối vi sinh đều với MIX_CNT

- **Điều kiện:** RC nấc cao + SA_FLOW_MODE=1 + xe đang chạy
- **Hành vi:** Như Case 3 nhưng dùng SA_MIX_CNT → vi_per_run = TANK_VOL × MIX_CNT (lớn hơn nấc giữa)
- **Output GCS:** vi/run cao hơn nấc giữa; dist_max ngắn hơn (cùng tốc độ)

### Case 7 — Cảnh báo khi ARM (FLOW_MODE=1)

- **Điều kiện:** Người lái ấn ARM khi spray_mode 1 hoặc 2 + FLOW_MODE=1
- **Hành vi:** Hệ thống in một lần duy nhất thông tin dự báo cho lần chạy: q1 ước tính theo tốc độ hiện tại, thời gian dự kiến, lượng vi sinh sẽ dùng. Nếu có vấn đề (chưa có mission, q1 ngoài dải), in cảnh báo thay
- **Disarm → ARM lại:** Thông báo được in lại từ đầu

### Case 8 — Phát hiện hết thùng vi sinh (cả 2 FLOW_MODE)

- **Điều kiện:** Spray_mode 1 hoặc 2 + đang ARM + lưu lượng đột ngột > 1.7 L/min liên tục 5 giây
- **Hành vi:** Bơm hút không khí khi thùng cạn → bánh xe cảm biến quay nhanh bất thường → lưu lượng đọc tăng vọt. Sau 5s liên tục → in cảnh báo CRITICAL 1 lần duy nhất. **Bơm KHÔNG dừng** (người lái tự quyết định)
- **Output GCS:** `SA: TANK EMPTY - flow X.XL/min > 1.7 for 5s`
- **Reset:** Disarm → ARM lại → detector hoạt động bình thường trở lại

### Case 9 — Chế độ giả lập (SA_SIM=1)

- **Điều kiện:** Bật chế độ giả lập
- **Hành vi:** Cảm biến được thay bằng dữ liệu mô phỏng sin, không cần phần cứng thật
- **Output GCS:** Dữ liệu dao động; console log có tiền tố `[SIM]`

---

## 7. Những gì KHÔNG thay đổi

- Hành vi điều hướng ArduPilot (navigation, waypoint, auto mode)
- Module pH (Module 2) và Dosing Motor (Module 3)
- GCS hiển thị tất cả thông số khác của ArduRover

> **Đã loại bỏ so với bản thiết kế ban đầu:** `SA_APP_RATE` và `SA_BOOM_W` (slot 12–13) đã bị gỡ bỏ hoàn toàn khỏi param — không chỉ "không dùng". Slot 12 được tái sử dụng cho `SA_PH_CAP_M` (Module 2); slot 13 không còn tham số nào. Vai trò của hai param này được thay bằng `SA_MIX_STD` / `SA_MIX_CNT` (tỉ lệ vi sinh theo nấc RC).

---

## 8. Tài liệu liên quan

- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — thuật toán, code flow, wiring đầy đủ
- [MODULE1_FLOW_TEST_CASES.md](MODULE1_FLOW_TEST_CASES.md) — test cases với số đối ứng cụ thể
- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — layout SA_DATA tổng thể