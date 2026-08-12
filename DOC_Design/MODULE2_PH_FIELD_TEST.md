# Module Giám Sát Chất Lượng Nước — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích module:** Đo pH, nhiệt độ và ước lượng độ kiềm của nước ao, theo dõi riêng cho từng ao.
**Ngày viết:** 2026-08-10

> Trước khi đánh giá, đề nghị kỹ thuật viên đã lắp đặt, hiệu chuẩn đầu dò và cài đặt khung giờ đo sáng/chiều. Người đánh giá quan sát kết quả đo thực tế, đối chiếu bằng dụng cụ đo tay (nếu có) — không cần đọc màn hình kỹ thuật.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Dữ liệu chất lượng nước sai lệch hoặc gán nhầm ao có thể dẫn đến quyết định xử lý ao sai (bón vôi nhầm ao, bỏ sót ao cần xử lý) — ảnh hưởng trực tiếp đến sức khỏe vật nuôi.

---

## Danh sách tình huống đánh giá

### 1. Phát hiện đúng khi mất/có kết nối đầu dò

**Tình huống:** Rút đầu dò đo nước ra khỏi robot, sau đó cắm lại.

**Quan sát:** Theo dõi thông báo trên màn hình điều khiển.

**Kết quả đạt:** Khi rút ra, hệ thống báo đúng "mất tín hiệu cảm biến"; khi cắm lại, báo trở lại bình thường trong vài giây.

☐ Đạt ☐ Không đạt

---

### 2. Đo giá trị hợp lý khi nhúng vào nước thật

**Tình huống:** Nhúng đầu dò vào nước ao thật.

**Quan sát:** Đối chiếu kết quả đo (pH, nhiệt độ) với dụng cụ đo tay (nếu có sẵn tại hiện trường).

**Kết quả đạt:** Giá trị đo được hợp lý (không phải số âm/vô lý), gần đúng với dụng cụ đo tay.

☐ Đạt ☐ Không đạt

---

### 3. Cảnh báo khi có thể đang đo nhầm ao

**Tình huống:** Chọn đo cho 1 ao đã có dữ liệu trước đó, nhưng đứng đo ở vị trí cách khá xa vị trí ao đó từng đo.

**Quan sát:** Theo dõi thông báo trên màn hình.

**Kết quả đạt:** Hệ thống cảnh báo "có thể chọn nhầm ao/vị trí lệch" — nhắc người vận hành kiểm tra lại đã chọn đúng ao chưa.

**Lưu ý:** hệ thống KHÔNG tự nhận diện ao qua vị trí GPS — người vận hành luôn phải tự chọn đúng ao trước khi đo.

☐ Đạt ☐ Không đạt

---

### 4. Tính ra chỉ số kiềm sau khi đo đủ sáng + chiều

**Tình huống:** Đo đủ cả buổi sáng và buổi chiều trong cùng 1 ngày cho cùng 1 ao (theo đúng khung giờ đã cài).

**Quan sát:** Xem kết quả tổng hợp của ao đó sau buổi chiều.

**Kết quả đạt:** Hệ thống tự tính ra được chỉ số kiềm ước lượng cho ao đó; đối chiếu bằng bộ test kiềm hóa học (nếu có) cho thấy sai lệch chấp nhận được.

☐ Đạt ☐ Không đạt

---

### 5. Dữ liệu từng ao không lẫn nhau, không mất khi tắt nguồn

**Tình huống:** Đo qua nhiều ao khác nhau trong ngày, sau đó tắt nguồn robot rồi bật lại.

**Quan sát:** Kiểm tra lại dữ liệu từng ao sau khi bật lại nguồn.

**Kết quả đạt:** Dữ liệu của từng ao vẫn đúng, riêng biệt, không bị lẫn qua ao khác, không bị mất.

☐ Đạt ☐ Không đạt

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Phát hiện đúng mất/có kết nối đầu dò | ☐ | ☐ | | |
| 2 | Đo giá trị hợp lý trong nước thật | ☐ | ☐ | | |
| 3 | Cảnh báo khi có thể đo nhầm ao | ☐ | ☐ | | |
| 4 | Tính ra chỉ số kiềm sau sáng + chiều | ☐ | ☐ | | |
| 5 | Dữ liệu từng ao không lẫn, không mất | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
