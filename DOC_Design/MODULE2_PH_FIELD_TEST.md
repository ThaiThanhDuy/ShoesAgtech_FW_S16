# Module Giám Sát Chất Lượng Nước — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích module:** Đo pH, nhiệt độ và ước lượng độ kiềm của nước ao, theo dõi riêng cho từng ao.
**Ngày viết:** 2026-08-10
**Cập nhật:** 2026-08-15 — bổ sung chi tiết Đạt/Không đạt, thêm tình huống thiếu GPS và lỗi lưu thẻ nhớ hay gặp ngoài hiện trường.

> Trước khi đánh giá, đề nghị kỹ thuật viên đã lắp đặt, hiệu chuẩn đầu dò và cài đặt khung giờ đo sáng/chiều. Người đánh giá quan sát kết quả đo thực tế, đối chiếu bằng dụng cụ đo tay (nếu có) — không cần đọc màn hình kỹ thuật.
>
> **Lưu ý quan trọng:** Tính năng phân loại mẫu đo theo buổi sáng/chiều (và từ đó tính độ kiềm) **cần có tín hiệu GPS còn hiệu lực** (không chỉ cần định vị đúng — còn cần lấy được giờ chuẩn từ vệ tinh). Nếu đánh giá ở nơi tín hiệu GPS yếu (dưới tán cây, gần nhà xưởng...), hãy kiểm tra đèn/biểu tượng GPS trên màn hình trước khi bắt đầu.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Dữ liệu chất lượng nước sai lệch hoặc gán nhầm ao có thể dẫn đến quyết định xử lý ao sai (bón vôi nhầm ao, bỏ sót ao cần xử lý) — ảnh hưởng trực tiếp đến sức khỏe vật nuôi. Mất dữ liệu đo (do lỗi thẻ nhớ) đồng nghĩa mất cả buổi đo, phải làm lại từ đầu.

---

## Danh sách tình huống đánh giá

### 1. Phát hiện đúng khi mất/có kết nối đầu dò

**Tình huống:** Rút đầu dò đo nước ra khỏi robot, sau đó cắm lại.

**Quan sát:** Theo dõi thông báo trên màn hình điều khiển.

**Kết quả ĐẠT:** Khi rút ra, hệ thống báo đúng "mất tín hiệu cảm biến"; khi cắm lại, báo trở lại bình thường trong vài giây.

**Kết quả KHÔNG ĐẠT:** Không có cảnh báo nào khi rút đầu dò (hệ thống vẫn hiện số liệu như bình thường dù không có cảm biến), hoặc sau khi cắm lại không tự phục hồi mà phải khởi động lại thiết bị.

☐ Đạt ☐ Không đạt

---

### 2. Đo giá trị hợp lý khi nhúng vào nước thật

**Tình huống:** Nhúng đầu dò vào nước ao thật.

**Quan sát:** Đối chiếu kết quả đo (pH, nhiệt độ) với dụng cụ đo tay (nếu có sẵn tại hiện trường).

**Kết quả ĐẠT:** Giá trị đo được hợp lý (không phải số âm/vô lý), gần đúng với dụng cụ đo tay.

**Kết quả KHÔNG ĐẠT:** Giá trị đo ra số âm, số 0 cố định, số nhảy loạn liên tục, hoặc lệch nhiều (quá 0.5 đơn vị pH) so với dụng cụ đo tay.

☐ Đạt ☐ Không đạt

---

### 3. Cảnh báo khi có thể đang đo nhầm ao

**Tình huống:** Chọn đo cho 1 ao đã có dữ liệu trước đó, nhưng đứng đo ở vị trí cách khá xa vị trí ao đó từng đo.

**Quan sát:** Theo dõi thông báo trên màn hình.

**Kết quả ĐẠT:** Hệ thống cảnh báo "vị trí GPS lệch xa so với ao đã lưu — kiểm tra lại vị trí ao" — nhắc người vận hành kiểm tra lại đã chọn đúng ao chưa.

**Kết quả KHÔNG ĐẠT:** Hệ thống im lặng ghi nhận dữ liệu vào ao đã chọn mà không cảnh báo gì, dù vị trí đo lệch rất xa so với lần đo trước của ao đó.

**Lưu ý:** hệ thống KHÔNG tự nhận diện ao qua vị trí GPS — người vận hành luôn phải tự chọn đúng ao trước khi đo; cảnh báo trên chỉ là gợi ý kiểm tra lại, không tự động sửa.

☐ Đạt ☐ Không đạt

---

### 4. Không tính được kiềm khi chưa có GPS ⚠️ (hay gặp ở nơi sóng yếu)

**Tình huống:** Ở vị trí tín hiệu GPS yếu hoặc chưa bắt được vệ tinh (ví dụ vừa khởi động robot, hoặc đứng dưới mái che), tiến hành đo pH.

**Quan sát:** Đọc dòng chữ trên màn hình điều khiển.

**Kết quả ĐẠT:** Màn hình hiện rõ thông báo dạng **"[WM] No GPS - alkalinity waiting for GPS/time"**; hệ thống KHÔNG tính ra một con số độ kiềm sai lệch từ dữ liệu chưa đủ điều kiện — chỉ tính lại khi có GPS đầy đủ.

**Kết quả KHÔNG ĐẠT:** Hệ thống vẫn tính ra và hiển thị một chỉ số độ kiềm dù chưa có GPS (nguy cơ dữ liệu sai bị hiểu nhầm là hợp lệ), hoặc không có cảnh báo gì giải thích vì sao không tính được.

☐ Đạt ☐ Không đạt

---

### 5. Tính ra chỉ số kiềm sau khi đo đủ sáng + chiều

**Tình huống:** Ở nơi có GPS ổn định, đo đủ cả buổi sáng và buổi chiều trong cùng 1 ngày cho cùng 1 ao (theo đúng khung giờ đã cài).

**Quan sát:** Xem kết quả tổng hợp của ao đó sau buổi chiều.

**Kết quả ĐẠT:** Hệ thống tự tính ra được chỉ số kiềm ước lượng cho ao đó; đối chiếu bằng bộ test kiềm hóa học (nếu có) cho thấy sai lệch chấp nhận được.

**Kết quả KHÔNG ĐẠT:** Không có chỉ số kiềm nào được tính ra dù đã đo đủ cả 2 buổi đúng khung giờ, hoặc chỉ số tính ra sai lệch rất lớn (vô lý) so với bộ test hóa học.

☐ Đạt ☐ Không đạt

---

### 6. Dữ liệu từng ao không lẫn nhau, không mất khi tắt nguồn

**Tình huống:** Đo qua nhiều ao khác nhau trong ngày, sau đó tắt nguồn robot rồi bật lại.

**Quan sát:** Kiểm tra lại dữ liệu từng ao sau khi bật lại nguồn.

**Kết quả ĐẠT:** Dữ liệu của từng ao vẫn đúng, riêng biệt, không bị lẫn qua ao khác, không bị mất; màn hình hiện dòng xác nhận đã nạp lại dữ liệu ao từ thẻ nhớ lúc khởi động.

**Kết quả KHÔNG ĐẠT:** Dữ liệu ao bị lẫn giữa các ao, bị mất hoàn toàn, hoặc không có xác nhận nào cho thấy dữ liệu đã được nạp lại sau khi bật nguồn.

☐ Đạt ☐ Không đạt

---

### 7. Cảnh báo khi thẻ nhớ đầy/lỗi ⚠️ (rủi ro mất dữ liệu ngoài hiện trường)

**Tình huống:** Nếu có thể giả lập được (nhờ kỹ thuật viên dùng thẻ nhớ gần đầy hoặc thẻ lỗi để test), thực hiện đo và lưu dữ liệu ao khi thẻ nhớ không còn ghi được.

**Quan sát:** Theo dõi màn hình điều khiển ngay sau khi hệ thống cố lưu dữ liệu.

**Kết quả ĐẠT:** Màn hình hiện cảnh báo rõ ràng dạng **"SA: could not save pond data to SD (card full/error?)"** — người vận hành biết ngay để thay thẻ, không mất dữ liệu một cách âm thầm.

**Kết quả KHÔNG ĐẠT:** Không có cảnh báo nào khi lưu thất bại — người vận hành chỉ phát hiện mất dữ liệu sau khi về văn phòng kiểm tra, không còn cách nào đo lại buổi đó.

☐ Đạt ☐ Không đạt ☐ Không kiểm tra được (thiếu thẻ nhớ lỗi để test)

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Phát hiện đúng mất/có kết nối đầu dò | ☐ | ☐ | | |
| 2 | Đo giá trị hợp lý trong nước thật | ☐ | ☐ | | |
| 3 | Cảnh báo khi có thể đo nhầm ao | ☐ | ☐ | | |
| 4 | Không tính kiềm sai khi chưa có GPS | ☐ | ☐ | | |
| 5 | Tính ra chỉ số kiềm sau sáng + chiều | ☐ | ☐ | | |
| 6 | Dữ liệu từng ao không lẫn, không mất | ☐ | ☐ | | |
| 7 | Cảnh báo khi thẻ nhớ đầy/lỗi | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
