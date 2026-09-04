# Module Phun Vi Sinh — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích module:** Tự động phun vi sinh xử lý nước ao, có 3 chế độ: điều khiển tay, tự động theo mức cố định, tự động theo mức cao (chống nghẹt).
**Ngày viết:** 2026-08-10
**Cập nhật:** 2026-08-15 — bổ sung chi tiết Đạt/Không đạt cho từng mục, thêm các tình huống thực tế khi dùng chế độ tự động theo công thức mission. | 2026-08-19 — gỡ bỏ giới hạn trần q1 (2.0 L/phút). | 2026-08-20 — thêm lại dải q1 [0.9, 1.2] theo đúng khả năng thật của bơm (đo thực tế), cảnh báo chỉ 1 lần/ARM; cập nhật lại tình huống 6.

> Trước khi đánh giá, đề nghị kỹ thuật viên cài đặt/hiệu chuẩn xong hệ thống theo đúng ao/mission dự kiến chạy, và **cho biết rõ đang dùng mức setpoint nào**:
> - **Cố định** (kỹ thuật viên đặt sẵn 1 con số lưu lượng) — dùng cho tình huống 3, 4.
> - **Tự động theo công thức mission** (hệ thống tự tính lưu lượng dựa trên dung tích thùng + độ dài tuyến đường + tốc độ xe) — dùng cho tình huống 5, 6.
>
> Nếu đánh giá tình huống 5/6, kỹ thuật viên cần chuẩn bị **sẵn 2 loại mission**: một tuyến đủ dài (để bơm chạy được), một tuyến rất ngắn (để giả lập trường hợp không đủ điều kiện bơm). Người đánh giá chỉ cần quan sát hành vi thực tế của robot và đánh dấu Đạt/Không đạt — không cần đọc bất kỳ tham số hay màn hình kỹ thuật nào, chỉ cần đọc đúng câu chữ cảnh báo hiện trên màn hình khi được yêu cầu.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Module này điều khiển trực tiếp một cơ cấu vật lý (bơm/van vi sinh) — nếu có lỗi, hậu quả có thể là: tốn vi sinh do phun sai lúc, phun sai liều lượng ảnh hưởng chất lượng nước ao, bơm dừng lặng lẽ mà không ai biết (mất cả buổi xử lý ao mà không hay), hoặc nguy hiểm nếu bơm tự chạy ngoài ý muốn người vận hành.

---

## Danh sách tình huống đánh giá

### 1. Bơm không được tự chạy khi chưa có ai điều khiển ⚠️ (đánh giá đầu tiên, bắt buộc)

**Tình huống:** Cấp điện cho robot, nhưng **chưa bật tay cầm điều khiển**.

**Quan sát:** Nhìn trực tiếp vào bơm/van vi sinh trong khoảng 30 giây đầu sau khi cấp điện.

**Kết quả ĐẠT:** Bơm đứng yên hoàn toàn, không chạy dù chỉ 1 chút.

**Kết quả KHÔNG ĐẠT:** Bơm chạy, rung, hoặc nhỏ giọt liên tục dù chỉ một chút — dù chưa bật tay cầm.

**Rủi ro nếu không đạt:** Bơm tự phun vi sinh vô tội vạ khi không ai giám sát — lãng phí vi sinh, có thể gây nguy hiểm nếu đứng gần vòi phun.

☐ Đạt ☐ Không đạt

---

### 2. Điều khiển tay (chế độ thủ công)

**Tình huống:** Bật tay cầm, gạt cần chọn chế độ về vị trí "thủ công".

**Quan sát:** Chỉnh cần ga bơm lên/xuống/về giữa.

**Kết quả ĐẠT:** Bơm chạy nhanh/chậm đúng theo tay chỉnh; khi trả cần ga về giữa, bơm dừng hẳn (không chạy lửng).

**Kết quả KHÔNG ĐẠT:** Bơm không phản hồi theo tay ga, phản hồi trễ/giật cục rõ rệt, hoặc vẫn chạy nhẹ khi cần ga đã về đúng vị trí giữa.

☐ Đạt ☐ Không đạt

---

### 3. Tự động — mức tiêu chuẩn (setpoint cố định)

**Chuẩn bị:** Xác nhận với kỹ thuật viên đang dùng **mức setpoint cố định** (không phải công thức mission).

**Tình huống:** Gạt cần chọn chế độ sang "tự động — mức tiêu chuẩn".

**Quan sát:** Chờ khoảng 10 giây, sau đó theo dõi lưu lượng phun ổn định trong 1-2 phút.

**Kết quả ĐẠT:** Lưu lượng phun ổn định đúng mức kỹ thuật viên đã cài đặt trước (dao động rất nhẹ, không tăng/giảm liên tục).

**Kết quả KHÔNG ĐẠT:** Lưu lượng nhảy lên xuống liên tục và rõ rệt, lệch nhiều so với mức đã cài, hoặc bằng 0 dù đang ở đúng chế độ này và đã bật tay cầm.

☐ Đạt ☐ Không đạt

---

### 4. Tự động — mức cao (chống nghẹt, setpoint cố định)

**Tình huống:** Gạt cần chọn chế độ sang "tự động — mức cao". Để robot chạy hết 1 tuyến đường đã định.

**Quan sát:** Tương tự tình huống 3, đồng thời theo dõi robot suốt tuyến.

**Kết quả ĐẠT:** Lưu lượng ổn định ở mức cao hơn tình huống 3, giữ suốt tuyến.

**Kết quả KHÔNG ĐẠT:** Lưu lượng không ổn định, hoặc bơm dừng đột ngột giữa chừng mà màn hình không hiện bất kỳ dòng cảnh báo nào giải thích lý do.

☐ Đạt ☐ Không đạt

---

### 5. Tự động theo công thức mission — chưa upload mission ⚠️ (lỗi hay gặp ngoài hiện trường)

**Tình huống:** Kỹ thuật viên chuyển sang chế độ setpoint tự động theo công thức mission, nhưng **chưa upload mission lên robot** (hoặc mission bị xoá). Vào chế độ tự động — mức tiêu chuẩn hoặc mức cao.

**Quan sát:** Nhìn bơm và đọc dòng chữ xuất hiện trên màn hình điều khiển.

**Kết quả ĐẠT:** Bơm đứng yên (không chạy), đồng thời màn hình hiện rõ dòng cảnh báo dạng **"SA FM1: no mission - pump stopped"** trong vòng vài giây — người vận hành biết ngay lý do bơm không chạy, không phải đoán mò.

**Kết quả KHÔNG ĐẠT:** Bơm đứng yên nhưng màn hình không hiện cảnh báo gì (người vận hành không biết lý do), hoặc bơm vẫn chạy dù chưa có mission.

☐ Đạt ☐ Không đạt

---

### 6. Tự động theo công thức mission — mission quá ngắn/quá dài so với khả năng bơm thật ⚠️ (cập nhật 2026-08-20)

> Đo thực tế cho thấy bơm hiện tại chỉ đạt lưu lượng thật **0.9–1.2 L/phút**
> trên toàn dải PWM MIN→MAX — ngoài dải này bơm vật lý không thể đạt được
> con số tính toán, nên hệ thống chủ động dừng bơm thay vì chạy sai. Cảnh
> báo chỉ hiện **1 lần mỗi phiên ARM** (không lặp lại liên tục).

**Tình huống:** Dùng setpoint tự động theo công thức mission, upload một **mission rất ngắn** (vài chục mét) trong khi thùng vi sinh và tốc độ xe đang cài đặt cho tuyến dài hơn nhiều (khiến lưu lượng tính ra vượt quá 1.2 L/phút). Cho xe chạy AUTO theo mission ngắn này.

**Quan sát:** Nhìn bơm và đọc dòng chữ xuất hiện trên màn hình điều khiển khi xe bắt đầu di chuyển.

**Kết quả ĐẠT:** Bơm đứng yên (không phun), màn hình hiện **đúng 1 lần** dòng cảnh báo dạng **"SA FM1: q1=...L/min > 1.2 (pump range) - lengthen mission or reduce speed"** — người vận hành biết ngay lý do và cách xử lý (kéo dài mission hoặc giảm tốc độ), không phải đoán mò.

**Kết quả KHÔNG ĐẠT:** Bơm vẫn phun vi sinh bất chấp mission quá ngắn (nguy cơ phun quá liều, vượt khả năng thật của bơm), HOẶC bơm đứng yên nhưng không có cảnh báo nào, HOẶC cảnh báo lặp lại liên tục thay vì chỉ 1 lần/phiên ARM.

**Lưu ý cho người đánh giá:** Đây là giới hạn PHẦN CỨNG thật (bơm không thể vượt quá 1.2 L/phút dù PID cố gắng thế nào), không phải lỗi hệ thống — mục đích test là xác nhận cảnh báo xuất hiện đúng và chỉ đúng 1 lần, giúp kỹ thuật viên biết cần chỉnh lại mission/tốc độ thay vì để bơm chạy sai liều lượng trong im lặng.

☐ Đạt ☐ Không đạt

---

### 7. Phát hiện hết vi sinh trong thùng khi đang bơm tự động

**Tình huống:** Đang phun ở chế độ tự động (mức tiêu chuẩn hoặc mức cao), để thùng vi sinh cạn hẳn trong lúc bơm vẫn đang chạy (hoặc giả lập bằng cách tháo đường ống hút ra khỏi thùng).

**Quan sát:** Theo dõi màn hình điều khiển sau khi thùng cạn khoảng 5-7 giây.

**Kết quả ĐẠT:** Hệ thống phát hiện và hiện thông báo dạng **"SA: TANK EMPTY ..."** (mức INFO) trong vòng ~5 giây sau khi thùng cạn (tăng từ 3s lên 5s theo yêu cầu 2026-08-19) — không để bơm chạy khan kéo dài mà không ai hay biết.

**Kết quả KHÔNG ĐẠT:** Bơm tiếp tục chạy khan nhiều phút mà không có cảnh báo nào, hoặc cảnh báo xuất hiện quá trễ (sau hơn 30 giây).

☐ Đạt ☐ Không đạt

---

### 8. Đầu phun bị bịt tạm (giả lập nghẹt)

**Tình huống:** Trong lúc đang phun ở chế độ tự động, bịt tạm đầu vòi phun vài giây rồi thả ra.

**Quan sát:** Theo dõi robot có phản ứng bất thường không (đứng hình, mất điều khiển, tự tắt máy...).

**Kết quả ĐẠT:** Robot tự điều chỉnh bù lại, không bị "đơ"/treo máy, không cần khởi động lại.

**Kết quả KHÔNG ĐẠT:** Robot bị treo/đơ máy cần khởi động lại, hoặc sau khi thả tay bịt ra bơm chạy vọt lên bất thường (bù quá tay) thay vì trở lại mức bình thường.

☐ Đạt ☐ Không đạt

---

### 9. Mất tín hiệu tay cầm giữa lúc đang phun

**Tình huống:** Đang phun ở chế độ thủ công, tắt tay cầm điều khiển đột ngột.

**Quan sát:** Theo dõi bơm ngay sau khi mất tín hiệu.

**Kết quả ĐẠT:** Bơm dừng ngay lập tức, không tiếp tục phun theo lệnh cuối cùng trước khi mất tín hiệu.

**Kết quả KHÔNG ĐẠT:** Bơm tiếp tục chạy thêm (dù chỉ vài giây) theo mức ga cuối cùng trước khi mất tín hiệu.

☐ Đạt ☐ Không đạt

---

### 10. Vận hành liên tục thời gian dài

**Tình huống:** Cho robot chạy liên tục ít nhất 30 phút (càng dài càng tốt, ví dụ nửa buổi thực tế), có ghi dữ liệu liên tục trong lúc chạy.

**Quan sát:** Theo dõi robot xuyên suốt, để ý có bị đứng máy/mất phản hồi giữa chừng không.

**Kết quả ĐẠT:** Không có lần nào robot bị treo/mất phản hồi/tự dừng bất thường trong suốt thời gian chạy.

**Kết quả KHÔNG ĐẠT:** Có ít nhất 1 lần robot bị treo, mất phản hồi, hoặc tự dừng/tự khởi động lại không rõ nguyên nhân trong thời gian chạy.

☐ Đạt ☐ Không đạt

---

### 11. ⚠️ Lưu ý cần xác nhận thêm với bộ phận kỹ thuật

**Tình huống:** Khi dùng setpoint tự động theo công thức mission (tình huống 5/6), nếu **xe đứng yên** (chưa di chuyển hoặc dừng giữa tuyến), bơm cũng về 0 — nhưng hiện tại hệ thống **không hiện cảnh báo nào** cho trường hợp riêng này (khác với tình huống 5/6 luôn có cảnh báo rõ ràng).

**Việc cần làm:** Không phải lỗi cần đánh giá Đạt/Không đạt ở đây — chỉ cần xác nhận với người phụ trách kỹ thuật xem có cần bổ sung cảnh báo riêng cho trường hợp "xe đứng yên" hay không, để người vận hành không nhầm lẫn với lỗi mission/cấu hình.

☐ Đã trao đổi với kỹ thuật, không cần bổ sung
☐ Đã trao đổi với kỹ thuật, cần bổ sung (ghi vào mục theo dõi riêng)

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Bơm không tự chạy khi chưa điều khiển | ☐ | ☐ | | |
| 2 | Điều khiển tay | ☐ | ☐ | | |
| 3 | Tự động mức tiêu chuẩn (setpoint cố định) | ☐ | ☐ | | |
| 4 | Tự động mức cao (setpoint cố định) | ☐ | ☐ | | |
| 5 | Công thức mission — chưa có mission | ☐ | ☐ | | |
| 6 | Công thức mission — mission ngoài dải bơm (0.9-1.2), cảnh báo 1 lần | ☐ | ☐ | | |
| 7 | Phát hiện hết vi sinh trong thùng | ☐ | ☐ | | |
| 8 | Đầu phun bị bịt tạm | ☐ | ☐ | | |
| 9 | Mất tín hiệu tay cầm | ☐ | ☐ | | |
| 10 | Vận hành liên tục thời gian dài | ☐ | ☐ | | |
| 11 | Xác nhận cảnh báo xe đứng yên (xem trên) | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
