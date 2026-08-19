# Tự Động Chọn Kiểu Lái Theo Tốc Độ — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích tính năng:** Khi robot chạy tự động (mission) ở nhiều tốc độ khác nhau trong cùng 1 chuyến, hệ thống tự đổi cách điều khiển ga/lái cho phù hợp với từng khoảng tốc độ (chậm/bình thường/nhanh), thay vì dùng cố định 1 kiểu lái cho mọi tốc độ.
**Ngày viết:** 2026-08-10
**Cập nhật:** 2026-08-15 — bổ sung chi tiết Đạt/Không đạt, thêm tình huống cấu hình sai ngưỡng và làm rõ cách tạo đoạn tốc độ chậm/nhanh đúng cách.

> Trước khi đánh giá, đề nghị kỹ thuật viên đã bật tính năng và cài đặt xong ngưỡng tốc độ chậm/nhanh phù hợp với robot. Người đánh giá chỉ cần quan sát độ ổn định khi xe chạy và đọc các dòng thông báo xuất hiện trên màn hình điều khiển (không cần hiểu ý nghĩa kỹ thuật, chỉ cần xác nhận CÓ xuất hiện đúng lúc, và đọc đúng câu chữ khi được yêu cầu).
>
> **Lưu ý quan trọng khi chuẩn bị mission:** Hệ thống đổi kiểu lái dựa theo **tốc độ đã đặt cho từng đoạn đường** (do kỹ thuật viên cấu hình sẵn trong mission hoặc đổi tay qua màn hình điều khiển), **không phải** tốc độ tức thời quan sát bằng mắt (ví dụ xe tự chậm lại khi vào cua không tính là "đổi tốc độ"). Vì vậy để tạo đúng đoạn "chậm"/"nhanh" cho các tình huống dưới đây, kỹ thuật viên cần **chủ động đặt tốc độ khác nhau cho từng đoạn** trong mission (hoặc đổi tốc độ qua màn hình giữa chừng), không chỉ dựa vào địa hình.
>
> Hệ thống có độ trễ xác nhận trước khi đổi kiểu lái (mặc định khoảng 1 giây) để tránh đổi qua lại liên tục — người đánh giá cần đợi vài giây sau khi tốc độ đổi mới đánh giá kết quả, không đánh giá ngay lập tức.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Đây là tính năng ảnh hưởng trực tiếp đến cách xe chạy tự động (ga, lái). Nếu có lỗi, xe có thể chạy giật cục, lắc lư, chệch hướng, mang theo "thói quen lái" sai sang chế độ khác sau khi hết chuyến tự động, hoặc — nếu cấu hình ngưỡng sai — hoạt động không như kỹ thuật viên mong đợi mà không ai biết — ảnh hưởng an toàn vận hành.

---

## Danh sách tình huống đánh giá

### 1. Chạy ổn định ở tốc độ bình thường

**Tình huống:** Bật chuyến chạy tự động ở tốc độ đã cấu hình là "bình thường".

**Quan sát:** Nhìn xe chạy trong vài phút đầu.

**Kết quả ĐẠT:** Xe chạy êm, không giật, không lắc; màn hình điều khiển hiện đúng 1 dòng thông báo xác nhận ngay khi vừa bật (cắm điện điều khiển + bắt đầu chạy tự động).

**Kết quả KHÔNG ĐẠT:** Xe giật/lắc bất thường ngay ở tốc độ bình thường, hoặc không có dòng thông báo nào xuất hiện khi vừa bắt đầu chạy tự động.

☐ Đạt ☐ Không đạt

---

### 2. Chạy ổn định ở đoạn tốc độ CHẬM

**Tình huống:** Mission có đoạn được kỹ thuật viên **chủ động đặt tốc độ** chậm hơn ngưỡng dưới đã cấu hình (xem lưu ý ở đầu tài liệu — không dựa vào xe tự chậm do địa hình).

**Quan sát:** Nhìn xe khi vào đúng đoạn chạy chậm, đợi vài giây sau khi vào đoạn này rồi mới đánh giá.

**Kết quả ĐẠT:** Xe vẫn chạy ổn định, không mất lái/lắc lư khi chuyển từ bình thường sang chậm; màn hình có dòng thông báo xác nhận đã đổi đúng lúc chuyển đoạn (trong vòng khoảng 1-2 giây).

**Kết quả KHÔNG ĐẠT:** Xe mất lái/lắc lư rõ rệt khi vào đoạn chậm, hoặc không có dòng thông báo xác nhận nào xuất hiện dù đã đợi đủ lâu.

☐ Đạt ☐ Không đạt

---

### 3. Chạy ổn định ở đoạn tốc độ NHANH

**Tình huống:** Mission có đoạn được kỹ thuật viên **chủ động đặt tốc độ** nhanh hơn ngưỡng trên đã cấu hình.

**Quan sát:** Nhìn xe khi vào đúng đoạn chạy nhanh, đợi vài giây rồi mới đánh giá.

**Kết quả ĐẠT:** Xe vẫn giữ đúng hướng đi, không rung lắc/chệch hướng khi chạy nhanh hơn; màn hình có dòng thông báo xác nhận.

**Kết quả KHÔNG ĐẠT:** Xe chệch hướng/rung lắc rõ rệt khi vào đoạn nhanh, hoặc không có dòng thông báo xác nhận.

☐ Đạt ☐ Không đạt

---

### 4. Không bị giật cục khi tốc độ dao động gần ranh giới

**Tình huống:** Yêu cầu kỹ thuật viên đổi tốc độ đặt liên tục qua lại quanh đúng ranh giới chậm/bình thường (đổi nhanh, nhiều lần liên tiếp, mỗi lần cách nhau dưới 1 giây).

**Quan sát:** Theo dõi cảm giác lái/ga của xe trong lúc này.

**Kết quả ĐẠT:** Xe không bị đổi cách lái "giật cục" liên tục theo từng lần đổi tốc độ nhỏ — chỉ đổi khi tốc độ thực sự giữ ổn định ở mức mới đủ lâu (khoảng 1 giây trở lên).

**Kết quả KHÔNG ĐẠT:** Xe đổi cách lái ngay lập tức theo từng lần dao động nhỏ, cảm giác lái/ga thay đổi liên tục dồn dập.

☐ Đạt ☐ Không đạt

---

### 5. Trở lại bình thường khi thoát chế độ tự động (quan trọng)

**Tình huống:** Đang chạy tự động ở đoạn tốc độ nhanh hoặc chậm, sau đó chuyển xe sang chế độ khác (ví dụ điều khiển tay, hoặc quay về).

**Quan sát:** Theo dõi cách xe lái/chạy NGAY SAU KHI vừa đổi khỏi chế độ tự động.

**Kết quả ĐẠT:** Xe vận hành đúng như bình thường ở chế độ mới — không còn "dấu vết" của kiểu lái nhanh/chậm từ lúc chạy tự động trước đó (không giật, không lắc lạ so với mọi khi).

**Kết quả KHÔNG ĐẠT:** Xe ở chế độ mới vẫn lái/chạy khác thường (quá nhạy hoặc quá lì so với mọi khi) — dấu hiệu kiểu lái nhanh/chậm còn sót lại từ lúc chạy tự động.

**Rủi ro nếu không đạt:** Xe mang nhầm "thói quen lái" của tốc độ khác sang chế độ hiện tại, có thể gây mất an toàn nếu người vận hành không biết.

☐ Đạt ☐ Không đạt

---

### 6. Cấu hình ngưỡng sai (ngưỡng nhanh ≤ ngưỡng chậm) ⚠️ (lỗi cấu hình hay gặp)

**Tình huống:** Yêu cầu kỹ thuật viên **chủ động đặt sai** ngưỡng cấu hình — đặt ngưỡng tốc độ "nhanh" bằng hoặc thấp hơn ngưỡng "chậm" (một lỗi nhập liệu dễ xảy ra ngoài hiện trường). Sau đó chạy tự động bình thường.

**Quan sát:** Đọc dòng chữ xuất hiện trên màn hình điều khiển trong vài giây đầu, quan sát xe có chạy bất thường không.

**Kết quả ĐẠT:** Màn hình hiện rõ cảnh báo dạng **"AUTO_SPD_MAX <= AUTO_SPD_MIN - cấu hình không hợp lệ, giữ PID BÌNH THƯỜNG"**; xe vẫn chạy bằng kiểu lái bình thường (không bị kẹt ở kiểu lái sai/không lái được) cho đến khi kỹ thuật viên sửa lại cấu hình.

**Kết quả KHÔNG ĐẠT:** Xe chạy bất thường (mất lái, không phản hồi ga) mà không có cảnh báo nào giải thích lý do, hoặc không có cảnh báo gì dù cấu hình đang sai.

☐ Đạt ☐ Không đạt

---

## Bảng tổng hợp

| #   | Tình huống                                       | Đạt | Không đạt | Người đánh giá | Ngày |
| --- | ------------------------------------------------ | --- | --------- | -------------- | ---- |
| 1   | Chạy ổn định tốc độ bình thường                  | ☐   | ☐         |                |      |
| 2   | Chạy ổn định tốc độ chậm                         | ☐   | ☐         |                |      |
| 3   | Chạy ổn định tốc độ nhanh                        | ☐   | ☐         |                |      |
| 4   | Không giật cục khi tốc độ dao động               | ☐   | ☐         |                |      |
| 5   | Trở lại bình thường khi thoát tự động            | ☐   | ☐         |                |      |
| 6   | Cấu hình ngưỡng sai — có cảnh báo, không kẹt lái | ☐   | ☐         |                |      |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
