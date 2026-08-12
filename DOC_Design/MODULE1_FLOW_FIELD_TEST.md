# Module Phun Vi Sinh — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích module:** Tự động phun vi sinh xử lý nước ao, có 3 chế độ: điều khiển tay, tự động theo mức cố định, tự động theo mức cao (chống nghẹt).
**Ngày viết:** 2026-08-10

> Trước khi đánh giá, đề nghị kỹ thuật viên cài đặt/hiệu chuẩn xong hệ thống theo đúng ao/mission dự kiến chạy. Người đánh giá chỉ cần quan sát hành vi thực tế của robot và đánh dấu Đạt/Không đạt — không cần đọc bất kỳ tham số hay màn hình kỹ thuật nào.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Module này điều khiển trực tiếp một cơ cấu vật lý (bơm/van vi sinh) — nếu có lỗi, hậu quả có thể là: tốn vi sinh do phun sai lúc, phun sai liều lượng ảnh hưởng chất lượng nước ao, hoặc nguy hiểm nếu bơm tự chạy ngoài ý muốn người vận hành.

---

## Danh sách tình huống đánh giá

### 1. Bơm không được tự chạy khi chưa có ai điều khiển ⚠️ (đánh giá đầu tiên, bắt buộc)

**Tình huống:** Cấp điện cho robot, nhưng **chưa bật tay cầm điều khiển**.

**Quan sát:** Nhìn trực tiếp vào bơm/van vi sinh trong khoảng 30 giây đầu sau khi cấp điện.

**Kết quả đạt:** Bơm đứng yên hoàn toàn, không chạy dù chỉ 1 chút.

**Rủi ro nếu không đạt:** Bơm tự phun vi sinh vô tội vạ khi không ai giám sát — lãng phí vi sinh, có thể gây nguy hiểm nếu đứng gần vòi phun.

☐ Đạt ☐ Không đạt

---

### 2. Điều khiển tay (chế độ thủ công)

**Tình huống:** Bật tay cầm, gạt cần chọn chế độ về vị trí "thủ công".

**Quan sát:** Chỉnh cần ga bơm lên/xuống/về giữa.

**Kết quả đạt:** Bơm chạy nhanh/chậm đúng theo tay chỉnh; khi trả cần ga về giữa, bơm dừng hẳn (không chạy lửng).

☐ Đạt ☐ Không đạt

---

### 3. Tự động — mức tiêu chuẩn

**Tình huống:** Gạt cần chọn chế độ sang "tự động — mức tiêu chuẩn".

**Quan sát:** Chờ khoảng 10 giây, sau đó theo dõi lưu lượng phun ổn định trong 1-2 phút.

**Kết quả đạt:** Lưu lượng phun ổn định đúng mức kỹ thuật viên đã cài đặt trước (dao động rất nhẹ, không tăng/giảm liên tục).

☐ Đạt ☐ Không đạt

---

### 4. Tự động — mức cao (chống nghẹt)

**Tình huống:** Gạt cần chọn chế độ sang "tự động — mức cao".

**Quan sát:** Tương tự tình huống 3, đồng thời để robot chạy hết 1 tuyến đường đã định.

**Kết quả đạt:** Lưu lượng ổn định ở mức cao hơn tình huống 3; nếu robot sắp hết vi sinh trong thùng, màn hình điều khiển có cảnh báo báo trước — chứ không dừng đột ngột không rõ lý do.

☐ Đạt ☐ Không đạt

---

### 5. Đầu phun bị bịt tạm (giả lập nghẹt)

**Tình huống:** Trong lúc đang phun ở chế độ tự động, bịt tạm đầu vòi phun vài giây rồi thả ra.

**Quan sát:** Theo dõi robot có phản ứng bất thường không (đứng hình, mất điều khiển, tự tắt máy...).

**Kết quả đạt:** Robot tự điều chỉnh bù lại, không bị "đơ"/treo máy, không cần khởi động lại.

☐ Đạt ☐ Không đạt

---

### 6. Mất tín hiệu tay cầm giữa lúc đang phun

**Tình huống:** Đang phun ở chế độ thủ công, tắt tay cầm điều khiển đột ngột.

**Quan sát:** Theo dõi bơm ngay sau khi mất tín hiệu.

**Kết quả đạt:** Bơm dừng ngay lập tức, không tiếp tục phun theo lệnh cuối cùng trước khi mất tín hiệu.

☐ Đạt ☐ Không đạt

---

### 7. Vận hành liên tục thời gian dài

**Tình huống:** Cho robot chạy liên tục ít nhất 30 phút (càng dài càng tốt, ví dụ nửa buổi thực tế), có ghi dữ liệu liên tục trong lúc chạy.

**Quan sát:** Theo dõi robot xuyên suốt, để ý có bị đứng máy/mất phản hồi giữa chừng không.

**Kết quả đạt:** Không có lần nào robot bị treo/mất phản hồi/tự dừng bất thường trong suốt thời gian chạy.

☐ Đạt ☐ Không đạt

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Bơm không tự chạy khi chưa điều khiển | ☐ | ☐ | | |
| 2 | Điều khiển tay | ☐ | ☐ | | |
| 3 | Tự động mức tiêu chuẩn | ☐ | ☐ | | |
| 4 | Tự động mức cao + cảnh báo hết vi sinh | ☐ | ☐ | | |
| 5 | Đầu phun bị bịt tạm | ☐ | ☐ | | |
| 6 | Mất tín hiệu tay cầm | ☐ | ☐ | | |
| 7 | Vận hành liên tục thời gian dài | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
