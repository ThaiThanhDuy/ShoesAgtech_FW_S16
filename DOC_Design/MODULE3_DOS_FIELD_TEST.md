# Module Cho Ăn — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích module:** Tự động cấp thức ăn cho từng ao theo đúng khẩu phần đã cài đặt.
**Ngày viết:** 2026-08-10
**Cập nhật:** 2026-08-15 — bổ sung chi tiết Đạt/Không đạt, thêm tình huống chế độ rải khẩu phần theo mission (giống lỗi đã gặp ở Module Phun Vi Sinh).

> Trước khi đánh giá, đề nghị kỹ thuật viên đã lắp đặt, hiệu chuẩn và cài đặt khẩu phần/loại thức ăn cho từng ao. Người đánh giá quan sát hành vi thực tế của động cơ cho ăn và đối chiếu bằng cân thực tế — không cần đọc màn hình kỹ thuật.
>
> **Lưu ý:** Module này có 2 chế độ tính lượng cho ăn tương tự Module Phun Vi Sinh: **tốc độ cố định** (tình huống 2) và **rải đều theo mission** (tình huống 3, 4, 8, 9). Với chế độ rải đều theo mission, kỹ thuật viên cần cho biết đang dùng chế độ nào và **chuẩn bị sẵn mission** trước khi đánh giá các tình huống liên quan.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Cho ăn sai lượng (quá nhiều/quá ít) ảnh hưởng trực tiếp đến vật nuôi và chi phí thức ăn. Nếu hệ thống cấp nhầm khẩu phần giữa các ao, chạy không kiểm soát, hoặc dừng lặng lẽ mà không cảnh báo, thiệt hại có thể xảy ra ngay trong lần vận hành đầu hoặc kéo dài nhiều ngày mà không ai phát hiện.

---

## Danh sách tình huống đánh giá

### 1. Khóa an toàn khi lắp đặt sai

**Tình huống:** Giả lập lắp đặt sai (kỹ thuật viên chủ động chỉnh sai 1 cấu hình động cơ).

**Quan sát:** Bật hệ thống cho ăn.

**Kết quả ĐẠT:** Động cơ đứng yên, KHÔNG chạy khi cấu hình chưa đúng; màn hình điều khiển báo lỗi rõ ràng để kỹ thuật viên biết chỗ cần sửa.

**Kết quả KHÔNG ĐẠT:** Động cơ vẫn chạy dù cấu hình sai, hoặc đứng yên nhưng không có cảnh báo nào chỉ ra lỗi ở đâu.

☐ Đạt ☐ Không đạt

---

### 2. Cho ăn theo tốc độ cố định

**Tình huống:** Bật chế độ cho ăn ở mức cố định trong 1 phút.

**Quan sát:** Cân thực tế lượng thức ăn ra sau đúng 1 phút.

**Kết quả ĐẠT:** Lượng thức ăn cân được gần đúng với khẩu phần/phút đã cài đặt trước (sai số nhỏ, chấp nhận được).

**Kết quả KHÔNG ĐẠT:** Lượng thức ăn cân được lệch nhiều (quá 15-20%) so với khẩu phần đã cài, hoặc động cơ không chạy dù đã bật đúng chế độ.

☐ Đạt ☐ Không đạt

---

### 3. Cho ăn tự động rải đều theo lộ trình — mission hợp lệ

**Tình huống:** Bật chế độ cho ăn tự động rải đều (theo mission), dùng một mission đủ dài và tốc độ phù hợp đã kỹ thuật viên xác nhận trước là hợp lệ, cho robot chạy hết tuyến.

**Quan sát:** Cân tổng lượng thức ăn ra sau khi chạy hết tuyến.

**Kết quả ĐẠT:** Tổng lượng thức ăn cân được gần đúng tổng khẩu phần đã đặt cho cả tuyến.

**Kết quả KHÔNG ĐẠT:** Tổng lượng thức ăn lệch nhiều so với khẩu phần đã đặt, hoặc động cơ không chạy dù mission và tốc độ đã được xác nhận hợp lệ trước đó.

☐ Đạt ☐ Không đạt

---

### 4. Tự dừng khi robot không di chuyển

**Tình huống:** Ở chế độ rải đều theo lộ trình (tình huống 3), cho robot dừng hẳn hoặc chưa có lộ trình chạy.

**Quan sát:** Theo dõi động cơ cho ăn lúc robot đứng yên.

**Kết quả ĐẠT:** Động cơ tự dừng, không tiếp tục cấp thức ăn vô ích khi robot không di chuyển.

**Kết quả KHÔNG ĐẠT:** Động cơ vẫn tiếp tục chạy dù robot đang đứng yên hoàn toàn.

☐ Đạt ☐ Không đạt

---

### 5. Đổi ao — khẩu phần tự đổi đúng, không lẫn ao

**Tình huống:** Đổi từ ao A sang ao B (mỗi ao đã cài khẩu phần/loại thức ăn khác nhau từ trước).

**Quan sát:** Kiểm tra lượng/loại thức ăn thực tế cấp ra khi đang ở ao B.

**Kết quả ĐẠT:** Đúng khẩu phần/loại thức ăn của ao B, không bị lẫn với ao A. Đổi qua lại nhiều lần vẫn đúng.

**Kết quả KHÔNG ĐẠT:** Ao B nhận nhầm khẩu phần/loại thức ăn của ao A (hoặc ngược lại) ít nhất 1 lần trong các lần đổi qua lại.

☐ Đạt ☐ Không đạt

---

### 6. Nhớ đúng khẩu phần sau khi tắt/bật lại nguồn

**Tình huống:** Sau khi đã cài khẩu phần cho vài ao khác nhau, tắt nguồn robot rồi bật lại.

**Quan sát:** Kiểm tra lại từng ao đã cài trước đó.

**Kết quả ĐẠT:** Khẩu phần/loại thức ăn của từng ao vẫn đúng như trước khi tắt nguồn, không bị mất/reset về mặc định.

**Kết quả KHÔNG ĐẠT:** Có ít nhất 1 ao bị mất khẩu phần đã cài, hoặc bị reset về giá trị mặc định sau khi bật lại nguồn.

☐ Đạt ☐ Không đạt

---

### 7. Cập nhật khẩu phần mới ngay tại hiện trường qua điện thoại/máy tính bảng

**Tình huống:** Khi kết nối robot với ứng dụng điều khiển trên điện thoại, nhập lượng thức ăn mới cho lần cho ăn này.

**Quan sát:** Kiểm tra lượng thức ăn thực tế cấp ra sau khi xác nhận.

**Kết quả ĐẠT:** Hệ thống cấp đúng theo lượng mới vừa nhập, không dùng nhầm giá trị cũ.

**Kết quả KHÔNG ĐẠT:** Hệ thống vẫn cấp theo lượng cũ dù đã xác nhận nhập lượng mới trên điện thoại.

☐ Đạt ☐ Không đạt

---

### 8. Rải đều theo mission — chưa upload mission ⚠️ (lỗi hay gặp ngoài hiện trường)

**Tình huống:** Dùng chế độ rải đều theo mission, nhưng **chưa upload mission lên robot** (hoặc mission bị xoá). Bật hệ thống cho ăn.

**Quan sát:** Nhìn động cơ và đọc dòng chữ xuất hiện trên màn hình điều khiển.

**Kết quả ĐẠT:** Động cơ đứng yên (không cấp thức ăn), màn hình hiện rõ dòng cảnh báo dạng **"SA DOS2: no mission (dist=...m) - motor stopped"** trong vòng vài giây — người vận hành biết ngay lý do, không phải đoán mò.

**Kết quả KHÔNG ĐẠT:** Động cơ đứng yên nhưng không có cảnh báo nào (không rõ lý do), hoặc động cơ vẫn chạy dù chưa có mission.

☐ Đạt ☐ Không đạt

---

### 9. Rải đều theo mission — xe đứng yên hoặc tốc độ quá thấp

**Tình huống:** Dùng chế độ rải đều theo mission, cho robot dừng hẳn hoặc di chuyển rất chậm (gần như đứng yên) giữa tuyến.

**Quan sát:** Theo dõi động cơ và màn hình điều khiển.

**Kết quả ĐẠT:** Động cơ dừng cấp thức ăn, màn hình hiện cảnh báo dạng **"SA DOS2: speed too low (...m/s) - motor stopped"** — không cấp thức ăn dồn vào một chỗ khi xe không di chuyển.

**Kết quả KHÔNG ĐẠT:** Động cơ vẫn cấp thức ăn dù xe gần như đứng yên (dồn thức ăn vào một điểm), hoặc dừng cấp nhưng không có cảnh báo giải thích lý do.

☐ Đạt ☐ Không đạt

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Khóa an toàn khi lắp đặt sai | ☐ | ☐ | | |
| 2 | Cho ăn tốc độ cố định | ☐ | ☐ | | |
| 3 | Cho ăn rải đều theo lộ trình (mission hợp lệ) | ☐ | ☐ | | |
| 4 | Tự dừng khi robot không di chuyển | ☐ | ☐ | | |
| 5 | Đổi ao — không lẫn khẩu phần | ☐ | ☐ | | |
| 6 | Nhớ đúng khẩu phần sau tắt/bật lại | ☐ | ☐ | | |
| 7 | Cập nhật khẩu phần mới qua app | ☐ | ☐ | | |
| 8 | Rải đều theo mission — chưa có mission | ☐ | ☐ | | |
| 9 | Rải đều theo mission — xe đứng yên/quá chậm | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
