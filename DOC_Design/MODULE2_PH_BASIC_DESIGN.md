# Module 2 — pH Sensor (Modbus RTU)
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

> Thêm mới chức năng đọc chất lượng nước ao từ cảm biến pH — không có chức năng này trong ArduPilot gốc.

---

## 2. Mục đích

> **Tính năng này giải quyết vấn đề gì? Ai cần nó? Dùng trong tình huống nào?**

Module 2 đọc và xử lý dữ liệu chất lượng nước từ cảm biến pH gắn trong ao:

- Đo **pH nước** để kiểm soát sức khỏe ao nuôi
- Đo **nhiệt độ nước** và **điện áp điện cực**
- Tính **độ kiềm (alkalinity)** dựa trên pH + dữ liệu test kit thực địa
- Tính **ΔpH ngày** (pH chiều − pH sáng) để đánh giá quang hợp / hoạt động sinh học trong ao

Dùng trong nuôi trồng thủy sản — người nuôi cần biết chất lượng nước realtime từ GCS/điện thoại mà không cần ra ao đo tay.

---

## 3. Phạm vi thay đổi

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| Đọc cảm biến chất lượng nước | Có | Toàn bộ giao tiếp với cảm biến pH qua dây RS485 |
| Giao tiếp GCS / MAVLink | Có | Gửi pH, nhiệt độ, kiềm, ΔpH lên màn hình |
| Phần cứng / kết nối | Có | Thêm cảm biến pH và module chuyển đổi tín hiệu |
| Điều khiển bơm / navigation | Không | Không liên quan |

---

## 4. Phần cứng / Giao tiếp sử dụng

| Thiết bị | Vai trò | Kết nối vào hệ thống qua | Ghi chú |
|---|---|---|---|
| Cảm biến pH (Nengshi ASPS3801D) | Đo pH, nhiệt độ, điện áp điện cực trong nước | Dây RS485 2 dây (A/B) | Ngâm trực tiếp trong ao |
| Module chuyển đổi RS485 → TTL | Chuyển tín hiệu RS485 của cảm biến sang tín hiệu đọc được bởi FC | Cắm vào cổng TELEM của FC (RX/TX) | Module nhỏ ngoài, không tích hợp trong FC |
| Cổng TELEM của FC | Nhận dữ liệu từ cảm biến | UART (cổng serial) | Số cổng cần cài đúng trong cài đặt |

> Module này không có phần cứng điều khiển đầu ra — chỉ đọc và gửi dữ liệu.

---

## 5. Flow hoạt động

```
Hệ thống tự động gửi yêu cầu đọc dữ liệu đến cảm biến mỗi 2 giây
    ↓
Cảm biến trả về thông tin: pH, điện áp điện cực, nhiệt độ
    ↓
Kiểm tra tính hợp lệ của dữ liệu nhận về (có đúng định dạng không?)
    ↓ Hợp lệ
Tính toán: pH trung bình → kiềm (dKH + mg/L)
    ↓
Phân loại thời gian đo (sáng / chiều) → tính ΔpH ngày
    ↓
Gửi tất cả dữ liệu lên GCS qua MAVLink
```

---

## 6. Tất cả Case và Output

### Case 1: Kết nối thành công, dữ liệu hợp lệ

- **Điều kiện:** Cảm biến đã kết nối đúng, dây RS485 tốt, FC nhận được dữ liệu
- **Hành vi hệ thống:** Cập nhật pH, nhiệt độ, kiềm mỗi 2 giây; tính trung bình và ΔpH theo slot sáng/chiều
- **Output người dùng thấy:** GCS hiển thị pH, nhiệt độ, kiềm cập nhật liên tục. Thông báo "pH sensor ready" khi khởi động

### Case 2: Dữ liệu lỗi (nhiễu trên đường truyền)

- **Điều kiện:** Cảm biến kết nối nhưng dây RS485 bị nhiễu, dữ liệu nhận về bị hỏng
- **Hành vi hệ thống:** Bỏ qua frame lỗi, không cập nhật dữ liệu; tiếp tục poll lần sau
- **Output người dùng thấy:** Cảnh báo "pH CRC fail" trên GCS; dữ liệu đứng yên không cập nhật

### Case 3: Mất kết nối cảm biến (không có phản hồi)

- **Điều kiện:** Cảm biến bị rút ra, đứt dây, hoặc hỏng — không gửi phản hồi trong thời gian cho phép
- **Hành vi hệ thống:** Xóa dữ liệu pH, kiềm về 0 trên SA_DATA; phát cảnh báo
- **Output người dùng thấy:** Cảnh báo "pH sensor mất kết nối (Xs)" trên GCS; toàn bộ dữ liệu pH bằng 0 trên dashboard

### Case 4: Chưa có dữ liệu lần nào (lần đầu boot)

- **Điều kiện:** Vừa khởi động, chưa nhận được frame nào từ cảm biến
- **Hành vi hệ thống:** Chờ đợi; sau một khoảng thời gian phát cảnh báo
- **Output người dùng thấy:** Cảnh báo "pH sensor chưa có dữ liệu" trên GCS; dữ liệu pH bằng 0

### Case 5: Đủ dữ liệu cả sáng lẫn chiều (slot FULL)

- **Điều kiện:** Hệ thống đã ghi nhận pH đo sáng (trước 12 giờ) VÀ pH đo chiều (sau 12 giờ) trong cùng một ngày
- **Hành vi hệ thống:** Tính ΔpH = pH_chiều − pH_sáng; kiềm tính theo công thức đầy đủ
- **Output người dùng thấy:** GCS hiển thị ΔpH và kiềm chính xác nhất; nhãn [FULL]

### Case 6: Chỉ có dữ liệu một buổi (MORN hoặc AFT)

- **Điều kiện:** Chỉ đo được một buổi (sáng hoặc chiều), chưa có buổi còn lại
- **Hành vi hệ thống:** Tính kiềm dựa trên một mốc pH duy nhất (kém chính xác hơn); ΔpH = 0
- **Output người dùng thấy:** GCS hiển thị kiềm ước tính; nhãn [MORN] hoặc [AFT]; cảnh báo cần chờ thêm

### Case 7: Dùng dữ liệu hôm qua (PREV)

- **Điều kiện:** Sang ngày mới, chưa có dữ liệu ngày hôm nay
- **Hành vi hệ thống:** Hiển thị kiềm từ hôm qua, đánh dấu là dữ liệu cũ
- **Output người dùng thấy:** GCS hiển thị kiềm hôm qua; nhãn [PREV] màu cam

### Case 8: Chưa có dữ liệu kiềm (NODATA)

- **Điều kiện:** Lần đầu khởi động, chưa từng đo được slot nào
- **Hành vi hệ thống:** Hiển thị kiềm ước tính thô từ pH tức thời; đánh dấu không chắc chắn
- **Output người dùng thấy:** GCS hiển thị kiềm sơ bộ; nhãn [NODATA] màu đỏ

### Case 9: Chưa có tín hiệu GPS (không phân được sáng/chiều)

- **Điều kiện:** FC chưa có GPS fix → không biết giờ địa phương
- **Hành vi hệ thống:** Đọc và hiển thị pH tức thời bình thường, nhưng không phân slot sáng/chiều
- **Output người dùng thấy:** Cảnh báo "Chưa có GPS time" trên GCS; pH hiển thị bình thường, ΔpH không có

### Case 10: Chế độ thử nghiệm (SA_SIM=1)

- **Điều kiện:** Bật chế độ giả lập
- **Hành vi hệ thống:** Không giao tiếp RS485, dùng dữ liệu pH giả lập hình sin
- **Output người dùng thấy:** pH và kiềm dao động ổn định; console log có tiền tố [SIM]

---

## 7. Những gì KHÔNG thay đổi

- Điều khiển bơm phun (Module 1) và dosing motor (Module 3)
- Điều hướng ArduPilot (navigation, waypoint)
- Toàn bộ phần cứng không liên quan đến cổng TELEM được chọn
- Cách hiển thị các thông số ArduRover khác trên GCS

---

## 8. Tài liệu liên quan

- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — giao thức Modbus, thuật toán kiềm, code flow đầy đủ
- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — layout SA_DATA tổng thể
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
