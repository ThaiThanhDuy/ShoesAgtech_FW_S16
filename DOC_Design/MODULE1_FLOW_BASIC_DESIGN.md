# Module 1 — Flow Sensor & Spray Controller
## Basic Design Document

> **Đây là tài liệu TRƯỚC KHI code.**
> Mô tả YÊU CẦU và HÀNH VI — không liên quan đến code, tham số hay kỹ thuật bên trong.
> Người không biết lập trình cũng đọc được và hiểu hệ thống làm gì.

**Dự án:** `ardupilot-jbdcan_testing_S16`
**Ngày tạo:** 2026-05-01
**Người viết:** ThaiThanhDuy
**Trạng thái:** `[x] Draft   [ ] Review   [ ] Approved`

---

## 1. Loại thay đổi

```
[x] Tính năng mới hoàn toàn
[ ] Bổ sung vào hệ thống có sẵn
[ ] Sửa lỗi / thay đổi hành vi hiện tại
```

> Thêm mới toàn bộ hệ thống điều khiển phun nước tự động dựa trên cảm biến lưu lượng — không có chức năng này trong ArduPilot gốc.

---

## 2. Mục đích

> **Tính năng này giải quyết vấn đề gì? Ai cần nó? Dùng trong tình huống nào?**

Module 1 điều khiển bơm phun nước/hóa chất trên robot nông nghiệp.
Người lái chỉ cần **chọn chế độ bằng một nút RC** — hệ thống tự đọc lưu lượng thực tế và điều chỉnh bơm để đạt mục tiêu đặt trước.

Dùng khi: phun thuốc, phun nước, phun phân bón trên đồng/ao — cần phun đều và đúng lượng theo diện tích hoặc thể tích tank.

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

| Thiết bị | Vai trò | Kết nối vào hệ thống qua | Ghi chú |
|---|---|---|---|
| Cảm biến lưu lượng (YF-S402B) | Đo lưu lượng nước thực tế đang chạy qua ống | Chân GPIO của FC | Cần chọn đúng chân, cấu hình 1 lần lúc cài đặt |
| Bơm (servo PWM 360°) | Bơm nước/hóa chất | Kênh servo của FC | Phải cài đúng theo hướng dẫn wiring, không dùng chức năng servo chuẩn |
| Tay lái / Remote RC | Người lái chọn chế độ phun | Kênh RC riêng | Một kênh 3 nấc: nấc thấp/giữa/cao = 3 chế độ |

---

## 5. Flow hoạt động

```
Người lái gạt nút RC chọn chế độ phun + điều kiện van ruộng
    ↓
Cảm biến đếm xung liên tục → hệ thống tính lưu lượng thực (L/min)
    ↓
┌────────────────────────────────────────────────────────────────┐
│  Nấc thấp → Chế độ 0 (LÁI TAY)                               │
│    Tín hiệu joystick thứ 2 truyền thẳng ra bơm                │
│                                                                │
│  Nấc giữa → PID bám lưu lượng vi sinh (van Mặc định)         │
│    SA_FLOW_MODE=0: bám SA_FLOW_SP (người dùng calib tay)      │
│    SA_FLOW_MODE=1: bám công thức L/ha × SA_MIX_STD            │
│                    có kiểm tra khoảng cách tank còn đủ không   │
│                                                                │
│  Nấc cao → PID bám lưu lượng vi sinh (van Chống nghẹt)        │
│    SA_FLOW_MODE=0: bám SA_FLOW_SP × (SA_MIX_CNT/SA_MIX_STD)  │
│                    (van mở thêm → setpoint vi sinh tăng tỉ lệ) │
│    SA_FLOW_MODE=1: bám công thức L/ha × SA_MIX_CNT            │
│                    có kiểm tra khoảng cách tank còn đủ không   │
└────────────────────────────────────────────────────────────────┘
    ↓
Tín hiệu điều khiển gửi đến bơm
    ↓
Dữ liệu lưu lượng + trạng thái bơm gửi lên GCS liên tục
```

---

## 6. Tất cả Case và Output

### Case 1: Chế độ 0 — Lái tay (PASSTHROUGH)

- **Điều kiện:** Nút RC ở nấc thấp nhất
- **Hành vi hệ thống:** Tín hiệu joystick thứ 2 truyền thẳng đến bơm, hệ thống không can thiệp
- **Output người dùng thấy:** Bơm phản hồi trực tiếp với tay lái; GCS hiển thị Mode=0, lưu lượng thực đọc liên tục nhưng không điều khiển

### Case 2: Nấc giữa, SA_FLOW_MODE=0 — bám setpoint cố định (Mặc định van)

- **Điều kiện:** Nút RC ở nấc giữa + SA_FLOW_MODE=0
- **Hành vi hệ thống:** Bơm tự tăng/giảm để giữ lưu lượng vi sinh đúng bằng SA_FLOW_SP; van ruộng đang ở vị trí Mặc định (người dùng chỉnh tay khi lắp đặt)
- **Output người dùng thấy:** Lưu lượng ổn định về SA_FLOW_SP; GCS hiển thị Mode=1, lưu lượng thực và mục tiêu

### Case 3: Nấc giữa, SA_FLOW_MODE=1 — L/ha công thức + kiểm tra tank

- **Điều kiện:** Nút RC ở nấc giữa + SA_FLOW_MODE=1 + đã upload tuyến đường + xe đang chạy
- **Hành vi hệ thống:** Tự tính lưu lượng vi sinh theo `SA_MIX_STD × SA_APP_RATE × speed × SA_BOOM_W × 0.006`. Kiểm tra tank đủ cho cả mission trước khi bơm.
- **Output người dùng thấy:** Lưu lượng tăng/giảm theo tốc độ xe; GCS hiển thị r, dist_max và dist mỗi SA_LOG_FL_MS

### Case 4: Nấc giữa hoặc cao, FLOW_MODE=1 — không đủ điều kiện bơm

- **Điều kiện:** FLOW_MODE=1 nhưng: chưa upload mission / tank không đủ cho mission / xe đứng yên / lưu lượng ngoài dải 0.3–6 L/min
- **Hành vi hệ thống:** Bơm DỪNG hoàn toàn + cảnh báo GCS giải thích lý do cụ thể
- **Output người dùng thấy:** Bơm dừng; GCS hiển thị một trong các cảnh báo: "chua co mission", "tank chi du Xm", "Q visin X < 0.3", "Q visin X > 6.0"

### Case 5: Nấc cao, SA_FLOW_MODE=0 — bám setpoint Chống nghẹt

- **Điều kiện:** Nút RC ở nấc cao + SA_FLOW_MODE=0; van vi sinh mở thêm so với nấc giữa (van Chống nghẹt)
- **Hành vi hệ thống:** Setpoint vi sinh tăng theo tỉ lệ SA_MIX_CNT/SA_MIX_STD so với SA_FLOW_SP, giữ nguyên tổng lưu lượng ra boom
- **Output người dùng thấy:** GCS hiển thị Mode=2, setpoint cao hơn nấc giữa (VD: SP=2.0, MIX_CNT/MIX_STD=0.50/0.35=1.43 → target=2.86 L/min)

### Case 6: Nấc cao, SA_FLOW_MODE=1 — L/ha công thức + MIX_CNT

- **Điều kiện:** Nút RC ở nấc cao + SA_FLOW_MODE=1 + xe đang chạy
- **Hành vi hệ thống:** Dùng SA_MIX_CNT (cao hơn MIX_STD) → dist_max ngắn hơn (hết tank nhanh hơn vì nhiều vi sinh hơn) → cảnh báo tank sớm hơn nấc giữa
- **Output người dùng thấy:** Hoạt động như Case 3 nhưng dist_max nhỏ hơn; cảnh báo "tank chi du Xm" xuất hiện sớm hơn

### Case 7: Cảnh báo ước tính tank (nấc cao, FLOW_MODE=0)

- **Điều kiện:** Nấc cao + SA_FLOW_MODE=0 + SA_TANK_VOL>0 + xe đang chạy
- **Hành vi hệ thống:** Ước tính còn bơm được bao nhiêu mét đường nữa dựa trên setpoint vi sinh hiện tại + tốc độ xe, thông báo định kỳ
- **Output người dùng thấy:** Thông báo GCS mỗi 30 giây: "Tank còn ~Xm (YL @ ZL/min)"

### Case 8: Chế độ thử nghiệm (SA_SIM=1)

- **Điều kiện:** Bật chế độ giả lập trong cài đặt
- **Hành vi hệ thống:** Cảm biến được thay bằng dữ liệu mô phỏng sin, không cần phần cứng thật
- **Output người dùng thấy:** GCS nhận dữ liệu dao động bình thường; console log có tiền tố [SIM]

---

## 7. Những gì KHÔNG thay đổi

- Hành vi điều hướng ArduPilot (navigation, waypoint, auto mode)
- Các kênh RC không được cài cho module này
- Module pH (Module 2) và Dosing Motor (Module 3)
- Toàn bộ cài đặt servo các kênh khác không liên quan
- GCS hiển thị tất cả thông số khác của ArduRover

---

## 8. Tài liệu liên quan

- [MODULE1_FLOW_DETAIL_DESIGN.md](MODULE1_FLOW_DETAIL_DESIGN.md) — thuật toán, code flow, wiring đầy đủ
- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — layout SA_DATA tổng thể
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
