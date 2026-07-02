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
Người lái gạt nút RC chọn chế độ phun
    ↓
Cảm biến đếm xung liên tục → hệ thống tính lưu lượng thực (L/min)
    ↓
┌────────────────────────────────────────────────────────────────┐
│  Nấc thấp → Chế độ 0 (LÁI TAY)                               │
│    Tín hiệu joystick thứ 2 truyền thẳng ra bơm                │
│                                                                │
│  Nấc giữa → Chế độ 1 (TỰ ĐỘNG LƯU LƯỢNG)                    │
│    Bộ điều khiển tăng/giảm bơm để bám lưu lượng mục tiêu     │
│                                                                │
│  Nấc cao → Chế độ 2 (TỰ ĐỘNG DIỆN TÍCH)                      │
│    Hệ thống tự tính lưu lượng cần theo tốc độ + độ rộng boom │
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

### Case 2: Chế độ 1 — Tự động bám lưu lượng, setpoint cố định

- **Điều kiện:** Nút RC ở nấc giữa + đặt chế độ "setpoint cố định"
- **Hành vi hệ thống:** Bơm tự tăng/giảm để giữ lưu lượng đúng bằng giá trị cài đặt trước
- **Output người dùng thấy:** Lưu lượng ổn định về giá trị đặt; GCS hiển thị Mode=1, lưu lượng thực và mục tiêu

### Case 3: Chế độ 1 — Tự động phân bổ đều theo tank + tuyến đường

- **Điều kiện:** Nút RC ở nấc giữa + đặt chế độ "tank + mission" + đã upload tuyến đường + xe đang chạy
- **Hành vi hệ thống:** Tự tính lưu lượng cần để phân bổ đều toàn bộ lượng nước trong tank trên tuyến đường đã đặt
- **Output người dùng thấy:** Lưu lượng tăng/giảm theo tốc độ xe; GCS cập nhật lưu lượng đang áp dụng

### Case 4: Chế độ 1 (tank + mission) — Chưa có tuyến đường hoặc xe đứng yên

- **Điều kiện:** Đặt chế độ tank+mission nhưng chưa upload tuyến đường LÊN FC, hoặc xe không di chuyển
- **Hành vi hệ thống:** Bơm DỪNG hoàn toàn — không tự chuyển sang tốc độ cố định
- **Output người dùng thấy:** Bơm dừng; cảnh báo xuất hiện trên màn hình GCS giải thích lý do

### Case 5: Chế độ 2 — Tự động theo diện tích, xe đang chạy

- **Điều kiện:** Nút RC ở nấc cao nhất + xe đang di chuyển
- **Hành vi hệ thống:** Lưu lượng tự tăng khi xe chạy nhanh, giảm khi xe chạy chậm để phun đều trên mỗi mét vuông
- **Output người dùng thấy:** GCS hiển thị Mode=2; bơm hoạt động theo tốc độ xe

### Case 6: Chế độ 2 — Xe dừng

- **Điều kiện:** Mode 2, xe dừng lại
- **Hành vi hệ thống:** Bơm tự dừng ngay để tránh phun tập trung vào một điểm
- **Output người dùng thấy:** Bơm dừng ngay khi xe dừng; khởi động lại khi xe chạy

### Case 7: Cảnh báo sắp hết nước trong tank (mode 2)

- **Điều kiện:** Chế độ 2 + đã cài dung tích tank trong cài đặt
- **Hành vi hệ thống:** Ước tính còn bơm được bao nhiêu mét đường nữa, thông báo định kỳ
- **Output người dùng thấy:** Thông báo trên GCS mỗi 30 giây: "Tank còn ~Xm (YL @ ZL/min)"

### Case 8: Chế độ thử nghiệm (SA_SIM=1)

- **Điều kiện:** Bật chế độ giả lập trong cài đặt
- **Hành vi hệ thống:** Cảm biến được thay bằng dữ liệu mô phỏng, không cần phần cứng thật
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
