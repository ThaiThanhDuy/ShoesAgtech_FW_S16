# [Tên tính năng / Module]

## Detail Design Document

> **Đây là tài liệu SAU KHI code + test + debug xong.**
> Ghi lại chính xác cách hệ thống hoạt động thực tế — mô tả kỹ thuật bằng lời,
> có bổ sung tham số, dữ liệu đầu ra, ràng buộc phần cứng đầy đủ.
> Đối chiếu với Basic Design: nếu có thay đổi so với thiết kế ban đầu → ghi chú lý do.

**Dự án:** `ardupilot-jbdcan_testing_S16`
**File nguồn:** `[đường dẫn file chính đã code]`
**Loại:** `[ ] Module mới   [ ] Bổ sung hệ thống   [ ] Sửa lỗi / thay đổi hành vi`

---

<!--
  HƯỚNG DẪN TAG:
  [ALL]   — luôn có mục này
  [HW]    — chỉ khi có phần cứng vật lý
  [DATA]  — chỉ khi gửi MAVLink hoặc ghi DataFlash
  [NEW]   — chỉ khi là module/library mới
  [SYS]   — chỉ khi sửa file hệ thống ArduPilot
  Mục không áp dụng → giữ heading, ghi "N/A"
-->

---

## 1. Chi tiết code — Function Flow [ALL]

> Mô tả các hàm đã viết theo dạng flow: hàm nào gọi hàm nào, truyền gì vào, xử lý gì, trả về gì.
> Mục đích: ai đọc cũng hiểu code chạy như thế nào mà không cần mở file.
> **Ghi theo thứ tự thực thi thực tế** (từ entry point → xuống sâu).

### 1.1 Sơ đồ luồng hàm (Call Flow)

```
[Entry point — ví dụ: update() @ 10Hz]
    │
    ├──► [hàm_A(param1, param2)]
    │         │ xử lý: ...
    │         └──► return [giá trị / void]
    │
    ├──► [hàm_B(param)]
    │         │ xử lý: ...
    │         ├──► [hàm_con_C()]
    │         │         └──► return [giá trị]
    │         └──► return [giá trị / void]
    │
    └──► [hàm_D()]
              └──► return [giá trị / void]
```

### 1.2 Mô tả từng hàm

> Với mỗi hàm: tên đầy đủ, file chứa, thông số vào, xử lý gì, output/return.

---

**`[TênHàm](param1: kiểu, param2: kiểu, ...)`**

- **File:** `[path/to/file.cpp : dòng N]`
- **Được gọi bởi:** `[hàm cha / scheduler / ISR]`
- **Đầu vào:**
    - `param1` — [ý nghĩa, đơn vị nếu có]
    - `param2` — [ý nghĩa, đơn vị nếu có]
- **Xử lý:**
    1. [Bước 1: làm gì]
    2. [Bước 2: làm gì]
    3. [Điều kiện rẽ nhánh nếu có]
- **Đầu ra / Return:**
    - `[kiểu trả về]` — [ý nghĩa, đơn vị]
    - Hoặc: `void` — [side effect: cập nhật biến nào, gửi gì]
- **Ghi chú:** [constraint đặc biệt, lý do thiết kế, bug đã fix]

---

**`[TênHàm2](param: kiểu)`**

- **File:** `[path/to/file.cpp : dòng N]`
- **Được gọi bởi:** `[...]`
- **Đầu vào:**
    - `param` — [ý nghĩa]
- **Xử lý:**
    1. [...]
- **Đầu ra / Return:**
    - `[...]`
- **Ghi chú:** [...]

---

> _(Thêm block hàm nếu cần — một block cho mỗi hàm quan trọng)_

---

## 2. Tham số cài đặt [ALL]

> Toàn bộ tham số cần cài đặt để tính năng này hoạt động.
> Bao gồm cả tham số nâng cao và tham số ít thay đổi.

| Tham số   | Slot | Kiểu                 | Mặc định | Min   | Max   | Mô tả đầy đủ                              |
| --------- | ---- | -------------------- | -------- | ----- | ----- | ----------------------------------------- |
| `SA_XXXX` | N    | Int8 / Float / Int16 | [val]    | [min] | [max] | [đơn vị, ý nghĩa, ảnh hưởng khi thay đổi] |

> **Param chỉ có hiệu lực sau reboot:** `[danh sách nếu có]`

---

## 3. Mô tả kỹ thuật [ALL]

> Giải thích CÁCH hệ thống thực hiện những gì đã mô tả trong Basic Design.
> Viết bằng lời, có thể kèm công thức hoặc pseudocode ngắn để làm rõ.
> **Không copy nguyên code — mô tả logic, không mô tả cú pháp.**

### 3.1 Khởi tạo

> Những gì xảy ra khi hệ thống boot / vào mode / arm.

- [Bước 1: ...]
- [Bước 2: ...]
- Thành công → [thông báo gì]
- Thất bại → [thông báo gì, hành vi fallback]

### 3.2 Luồng xử lý chính

> Thuật toán / giao thức / logic điều khiển chính.

**[Tên bước / giao thức]:**

```
[Pseudocode hoặc công thức hoặc frame bytes]
Đơn vị: [...]
Ví dụ:  input=... → output=...
```

### 3.3 Xử lý các trường hợp đặc biệt

- [Trường hợp A]: [làm gì]
- [Trường hợp B]: [làm gì]

---

## 4. Dữ liệu đầu ra chi tiết [DATA]

### 4.1 MAVLink SA_DATA

| Index     | Tên field    | Đơn vị | Điều kiện ghi    | Mô tả + edge case |
| --------- | ------------ | ------ | ---------------- | ----------------- |
| `data[N]` | `field_name` | [unit] | [Luôn / khi ...] | [mô tả đầy đủ]    |

### 4.2 DataFlash Log

> N/A nếu không ghi log nhị phân.

```
Message name:  "[MSGNAME]"
Format string: "[Qff / QffffffB / ...]"
Điều kiện ghi: [luôn / SA_XX_LOG=1 / ...]
```

| Field | Tên cột       | Kiểu         | Đơn vị | Mô tả                |
| ----- | ------------- | ------------ | ------ | -------------------- |
| 1     | `TimeUS`      | uint64_t (Q) | µs     | `AP_HAL::micros64()` |
| N     | `[FieldName]` | [type]       | [unit] | [mô tả]              |

### 4.3 Console Log (GCS Messages)

> N/A nếu không có.

```
Trigger: [mỗi N ms / mỗi lần event / ...]
Format:  [PREFIX] [nội dung với placeholder]
```

### 4.4 STATUSTEXT — Toàn bộ thông báo

| Nội dung thông báo                   | Mức                       | Điều kiện | Tần suất         |
| ------------------------------------ | ------------------------- | --------- | ---------------- |
| `[chuỗi đầy đủ với placeholder <x>]` | CRITICAL / WARNING / INFO | [khi nào] | [1 lần / mỗi Ns] |

---

## 5. Yêu cầu / Ràng buộc [ALL]

> Những điều kiện bắt buộc phải đúng để tính năng hoạt động. Ghi rõ hậu quả nếu sai.

```
[Điều kiện / Param]  =  [Giá trị]    → nếu sai: [hậu quả cụ thể]
```

**Ràng buộc phần cứng:** [board, pin, nguồn điện, kết nối]

**Ràng buộc vận hành:** [upload mission, GPS fix, arm, ... cần làm trước khi dùng]

---

## 6. Kết nối phần cứng [HW]

> N/A nếu là pure software.

```
[Tên thiết bị]:
  [Chân thiết bị]  →  [Chân / port FC]   ([ghi chú: điện áp, cực tính])

FC config:
  [PARAM_NAME] = [VALUE]
```

**Lưu ý board:** [Khác biệt CubeBlack / Pixhawk4 / ... nếu có]

---

## 7. So sánh với Basic Design [ALL]

> Ghi lại những điểm implementation thực tế khác so với Basic Design.
> Nếu không có thay đổi → ghi "Implement đúng theo Basic Design."

| Điểm khác biệt | Basic Design dự kiến | Thực tế đã làm | Lý do thay đổi                |
| -------------- | -------------------- | -------------- | ----------------------------- |
| [Tên điểm]     | [Dự kiến]            | [Thực tế]      | [Lý do kỹ thuật / constraint] |

---

## 8. Tài liệu liên quan [ALL]

- [[Tên]\_BASIC_DESIGN.md]([Tên]_BASIC_DESIGN.md) — yêu cầu và hành vi ban đầu
- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ SA_DATA
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
