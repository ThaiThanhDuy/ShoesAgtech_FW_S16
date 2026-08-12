# Tự Động Chọn Kiểu Lái Theo Tốc Độ — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích tính năng:** Khi robot chạy tự động (mission) ở nhiều tốc độ khác nhau trong cùng 1 chuyến, hệ thống tự đổi cách điều khiển ga/lái cho phù hợp với từng khoảng tốc độ (chậm/bình thường/nhanh), thay vì dùng cố định 1 kiểu lái cho mọi tốc độ.
**Ngày viết:** 2026-08-10

> Trước khi đánh giá, đề nghị kỹ thuật viên đã bật tính năng và cài đặt xong ngưỡng tốc độ chậm/nhanh phù hợp với robot. Người đánh giá chỉ cần quan sát độ ổn định khi xe chạy và đọc các dòng thông báo xuất hiện trên màn hình điều khiển (không cần hiểu ý nghĩa kỹ thuật, chỉ cần xác nhận CÓ xuất hiện đúng lúc).

---

## Vì sao mỗi tình huống dưới đây quan trọng

Đây là tính năng ảnh hưởng trực tiếp đến cách xe chạy tự động (ga, lái). Nếu có lỗi, xe có thể chạy giật cục, lắc lư, chệch hướng, hoặc mang theo "thói quen lái" sai sang chế độ khác sau khi hết chuyến tự động — ảnh hưởng an toàn vận hành.

---

## Danh sách tình huống đánh giá

### 1. Chạy ổn định ở tốc độ bình thường

**Tình huống:** Bật chuyến chạy tự động ở tốc độ đã cấu hình là "bình thường".

**Quan sát:** Nhìn xe chạy trong vài phút đầu.

**Kết quả đạt:** Xe chạy êm, không giật, không lắc; màn hình điều khiển hiện đúng 1 dòng thông báo xác nhận ngay khi vừa bật (cắm điện điều khiển + bắt đầu chạy tự động).

☐ Đạt ☐ Không đạt

---

### 2. Chạy ổn định ở đoạn tốc độ CHẬM

**Tình huống:** Mission có đoạn tốc độ chậm hơn mức bình thường.

**Quan sát:** Nhìn xe khi vào đúng đoạn chạy chậm.

**Kết quả đạt:** Xe vẫn chạy ổn định, không mất lái/lắc lư khi chuyển từ bình thường sang chậm; màn hình có dòng thông báo xác nhận đã đổi đúng lúc chuyển đoạn.

☐ Đạt ☐ Không đạt

---

### 3. Chạy ổn định ở đoạn tốc độ NHANH

**Tình huống:** Mission có đoạn tốc độ nhanh hơn mức bình thường.

**Quan sát:** Nhìn xe khi vào đúng đoạn chạy nhanh.

**Kết quả đạt:** Xe vẫn giữ đúng hướng đi, không rung lắc/chệch hướng khi chạy nhanh hơn; màn hình có dòng thông báo xác nhận.

☐ Đạt ☐ Không đạt

---

### 4. Không bị giật cục khi tốc độ dao động gần ranh giới

**Tình huống:** Yêu cầu kỹ thuật viên đổi tốc độ liên tục qua lại quanh đúng ranh giới chậm/bình thường (đổi nhanh, nhiều lần liên tiếp).

**Quan sát:** Theo dõi cảm giác lái/ga của xe trong lúc này.

**Kết quả đạt:** Xe không bị đổi cách lái "giật cục" liên tục theo từng lần đổi tốc độ nhỏ — chỉ đổi khi tốc độ thực sự giữ ổn định ở mức mới đủ lâu.

☐ Đạt ☐ Không đạt

---

### 5. Trở lại bình thường khi thoát chế độ tự động ⚠️ (quan trọng)

**Tình huống:** Đang chạy tự động ở đoạn tốc độ nhanh hoặc chậm, sau đó chuyển xe sang chế độ khác (ví dụ điều khiển tay, hoặc quay về).

**Quan sát:** Theo dõi cách xe lái/chạy NGAY SAU KHI vừa đổi khỏi chế độ tự động.

**Kết quả đạt:** Xe vận hành đúng như bình thường ở chế độ mới — không còn "dấu vết" của kiểu lái nhanh/chậm từ lúc chạy tự động trước đó (không giật, không lắc lạ so với mọi khi).

**Rủi ro nếu không đạt:** Xe mang nhầm "thói quen lái" của tốc độ khác sang chế độ hiện tại, có thể gây mất an toàn nếu người vận hành không biết.

☐ Đạt ☐ Không đạt

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Chạy ổn định tốc độ bình thường | ☐ | ☐ | | |
| 2 | Chạy ổn định tốc độ chậm | ☐ | ☐ | | |
| 3 | Chạy ổn định tốc độ nhanh | ☐ | ☐ | | |
| 4 | Không giật cục khi tốc độ dao động | ☐ | ☐ | | |
| 5 | Trở lại bình thường khi thoát tự động | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
