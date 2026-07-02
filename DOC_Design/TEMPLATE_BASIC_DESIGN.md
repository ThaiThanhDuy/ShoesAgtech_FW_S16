# [Tên tính năng / Module]
## Basic Design Document

> **Đây là tài liệu TRƯỚC KHI code.**
> Viết để mô tả YÊU CẦU và HÀNH VI — không liên quan đến code, tham số hay kỹ thuật bên trong.
> Người không biết lập trình cũng đọc được và hiểu hệ thống làm gì.

**Dự án:** `ardupilot-jbdcan_testing_S16`
**Ngày tạo:** [YYYY-MM-DD]
**Người viết:** [Tên]
**Trạng thái:** `[ ] Draft   [ ] Review   [ ] Approved`

---

## 1. Loại thay đổi

```
[ ] Tính năng mới hoàn toàn
[ ] Bổ sung vào hệ thống có sẵn
[ ] Sửa lỗi / thay đổi hành vi hiện tại
```

> Mô tả ngắn: [1–2 câu giải thích tại sao cần thay đổi này]

---

## 2. Mục đích

> **Tính năng này giải quyết vấn đề gì? Ai cần nó? Dùng trong tình huống nào?**

[Viết bằng ngôn ngữ của người vận hành, không phải ngôn ngữ lập trình]

---

## 3. Phạm vi thay đổi

> Những phần nào của hệ thống bị ảnh hưởng?
> Không cần liệt kê file code — chỉ cần mô tả phần chức năng.

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| [Điều khiển phun / pH / Dosing / ...] | Có / Không | [Thay đổi hành vi gì] |
| [Giao tiếp GCS / MAVLink] | Có / Không | [Dữ liệu mới / thay đổi] |
| [Phần cứng / kết nối] | Có / Không | [Thêm thiết bị / đổi chân] |

---

## 4. Phần cứng / Giao tiếp sử dụng

> Liệt kê thiết bị vật lý liên quan. Mô tả bằng ngôn ngữ thực tế, không cần kỹ thuật điện.

| Thiết bị | Vai trò | Kết nối vào hệ thống qua | Ghi chú |
|---|---|---|---|
| [Tên thiết bị] | [Làm gì trong tính năng này] | [Cổng / cáp / kênh nào] | [Lưu ý quan trọng] |

> Nếu không có phần cứng mới → ghi **"Không thay đổi phần cứng"**

---

## 5. Flow hoạt động

> Mô tả tuần tự những gì xảy ra từ khi bắt đầu đến kết thúc.
> Dùng ngôn ngữ đơn giản, sơ đồ mũi tên, hoặc danh sách có thứ tự.
> **Không đề cập đến code, tham số, hay địa chỉ kỹ thuật.**

```
[Điều kiện bắt đầu / người dùng làm gì]
    ↓
[Hệ thống phản ứng như thế nào]
    ↓
[Kết quả / trạng thái tiếp theo]
    ↓
[Kết thúc / điều kiện thoát]
```

---

## 6. Tất cả Case và Output

> Liệt kê **mọi tình huống** có thể xảy ra — cả trường hợp bình thường lẫn bất thường.
> Với mỗi case: mô tả điều kiện xảy ra → hệ thống làm gì → người dùng thấy/nghe/nhận được gì.

### Case 1: [Tên tình huống — ví dụ: Hoạt động bình thường]

- **Điều kiện:** [Khi nào case này xảy ra]
- **Hành vi hệ thống:** [Hệ thống làm gì]
- **Output người dùng thấy:** [Thiết bị chạy / dừng / đèn / thông báo trên màn hình GCS]

### Case 2: [Tên tình huống — ví dụ: Chưa đủ điều kiện]

- **Điều kiện:** [...]
- **Hành vi hệ thống:** [...]
- **Output người dùng thấy:** [...]

### Case 3: [Tên tình huống — ví dụ: Lỗi / mất kết nối]

- **Điều kiện:** [...]
- **Hành vi hệ thống:** [...]
- **Output người dùng thấy:** [Cảnh báo hiển thị trên GCS / thiết bị dừng an toàn / ...]

> *(Thêm case nếu cần — không giới hạn số lượng)*

---

## 7. Những gì KHÔNG thay đổi

> Liệt kê rõ những phần hệ thống **không bị ảnh hưởng** để tránh hiểu nhầm.

- [Tính năng X vẫn hoạt động như cũ]
- [Dữ liệu Y không thay đổi format]
- [Thiết bị Z không cần cấu hình lại]

---

## 8. Tài liệu liên quan

- [[Tên]_DETAIL_DESIGN.md]([Tên]_DETAIL_DESIGN.md) — mô tả kỹ thuật sau khi code xong
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
