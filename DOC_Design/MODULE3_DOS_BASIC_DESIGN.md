# Module 3 — Dosing Motor (Vít Tải Thức Ăn Tôm)
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

> Thêm mới chức năng điều khiển động cơ vít tải để định lượng và phân bổ thức ăn tôm — không có trong ArduPilot gốc.

---

## 2. Mục đích

> **Tính năng này giải quyết vấn đề gì? Ai cần nó? Dùng trong tình huống nào?**

Module 3 điều khiển động cơ vít tải (servo 360°) để phân phối thức ăn tôm tự động khi robot di chuyển dọc theo ao.

Người vận hành chỉ cần:
1. Bật/tắt motor bằng **một nút trên remote**
2. Hệ thống tự tính tốc độ quay để phân bổ đúng lượng thức ăn đã đặt

Hỗ trợ 2 chế độ:
- **Tốc độ cố định**: Quay đều với tốc độ đặt sẵn
- **Tỉ lệ theo tuyến đường**: Tự điều chỉnh tốc độ theo tốc độ xe để phân bổ đều theo quãng đường

---

## 3. Phạm vi thay đổi

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| Điều khiển motor vít tải | Có | Toàn bộ logic bật/tắt và tính tốc độ motor |
| Giao tiếp GCS / MAVLink | Có | Gửi setpoint, tốc độ, trạng thái motor lên màn hình |
| Phần cứng / kết nối | Có | Thêm servo 360° và kênh servo điều khiển motor |
| Điều hướng (navigation) | Không | Không thay đổi |
| Module Flow, pH | Không | Không liên quan trực tiếp |

---

## 4. Phần cứng / Giao tiếp sử dụng

| Thiết bị | Vai trò | Kết nối vào hệ thống qua | Ghi chú |
|---|---|---|---|
| Động cơ vít tải (servo 360°) | Quay vít tải để đẩy thức ăn ra | Kênh servo của FC | Phải cài đúng 4 thông số servo (xem ràng buộc) |
| Nút bấm RC | Người lái bật/tắt motor | Kênh RC riêng | Nút bấm hoặc toggle switch; PWM > 1500 = bật |

> Nếu tín hiệu RC mất (drone mất kết nối remote) → motor tự dừng an toàn.

---

## 5. Flow hoạt động

```
Người lái nhấn nút RC để bật motor
    ↓
Hệ thống kiểm tra cấu hình servo có đúng không
    ↓ Đúng
Chọn chế độ tính tốc độ:
┌─────────────────────────────────────────────────────────┐
│ Chế độ 0 (tốc độ cố định):                             │
│   Tính tốc độ motor từ lượng thức ăn và tỉ lệ cài sẵn │
│                                                         │
│ Chế độ 1 (tỉ lệ tuyến đường):                         │
│   Tính tốc độ theo: lượng thức ăn + tốc độ xe         │
│                     + tổng quãng đường tuyến đường      │
│   Nếu xe chưa chạy hoặc chưa có tuyến → motor DỪNG    │
└─────────────────────────────────────────────────────────┘
    ↓
Tín hiệu PWM xuất ra motor theo chiều quay đã cài
    ↓
Trạng thái motor gửi lên GCS
```

---

## 6. Tất cả Case và Output

### Case 1: Cấu hình servo chưa đúng

- **Điều kiện:** Kênh servo chưa được cài 4 thông số bắt buộc theo hướng dẫn (giá trị min/trim/max và chức năng)
- **Hành vi hệ thống:** Motor KHÔNG chạy dù người lái đã bật nút RC; phát cảnh báo cụ thể điều kiện nào sai
- **Output người dùng thấy:** Cảnh báo liên tục trên GCS mỗi 5 giây: "SERVO<n> FUNCTION=X, cần đặt =0 (None)" (hoặc MIN/TRIM/MAX tương tự); motor không phản hồi

### Case 2: Nút RC tắt (hoặc mất tín hiệu RC)

- **Điều kiện:** Nút RC ở trạng thái tắt (PWM ≤ 1500) hoặc mất tín hiệu remote
- **Hành vi hệ thống:** Motor dừng ngay lập tức, giữ nguyên dừng
- **Output người dùng thấy:** Thông báo "Dosing motor OFF" trên GCS; motor ngừng quay; GCS hiển thị PWM=1500

### Case 3: Nút RC bật — Chế độ 0 (tốc độ cố định)

- **Điều kiện:** Nút RC bật + chế độ tốc độ cố định + setpoint > 0
- **Hành vi hệ thống:** Motor quay đều với tốc độ tính từ lượng thức ăn đặt sẵn và tỉ lệ chuyển đổi; theo đúng chiều quay đã cài
- **Output người dùng thấy:** Thông báo "Dosing motor ON"; motor quay; GCS hiển thị PWM thực tế đang xuất

### Case 4: Nút RC bật — Chế độ 1 (phân bổ theo tuyến đường) — đủ điều kiện

- **Điều kiện:** Nút RC bật + chế độ tỉ lệ tuyến đường + đã upload tuyến đường + xe đang chạy
- **Hành vi hệ thống:** Motor thay đổi tốc độ theo tốc độ xe để phân bổ đều lượng thức ăn đặt sẵn trên toàn tuyến đường
- **Output người dùng thấy:** Motor quay nhanh hơn khi xe chạy nhanh và ngược lại; GCS hiển thị PWM thay đổi

### Case 5: Nút RC bật — Chế độ 1 — Chưa có tuyến đường hoặc xe đứng yên

- **Điều kiện:** Chế độ tỉ lệ tuyến đường nhưng chưa upload tuyến đường LÊN FC, hoặc xe không di chuyển
- **Hành vi hệ thống:** Motor DỪNG — không fallback sang tốc độ cố định; phát cảnh báo
- **Output người dùng thấy:** Motor dừng; cảnh báo trên GCS: "chưa có mission" hoặc "tốc độ quá thấp"

### Case 6: Setpoint = 0 gam

- **Điều kiện:** Lượng thức ăn đặt bằng 0
- **Hành vi hệ thống:** Motor không quay (offset PWM = 0 → dừng tại 1500µs)
- **Output người dùng thấy:** Motor dừng; GCS hiển thị PWM=1500

---

## 7. Những gì KHÔNG thay đổi

- Điều khiển bơm phun (Module 1) và cảm biến pH (Module 2)
- Điều hướng ArduPilot (navigation, waypoint, auto mode)
- Toàn bộ kênh RC và servo không được cài cho module này
- Hành vi fail-safe khác của ArduPilot

---

## 8. Tài liệu liên quan

- [MODULE3_DOS_DETAIL_DESIGN.md](MODULE3_DOS_DETAIL_DESIGN.md) — thuật toán, code flow, wiring đầy đủ
- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — layout SA_DATA tổng thể
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
