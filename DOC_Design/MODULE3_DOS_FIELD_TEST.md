# Module Cho Ăn — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích module:** Tự động cấp thức ăn cho từng ao theo đúng khẩu phần đã cài đặt.
**Ngày viết:** 2026-08-10

> Trước khi đánh giá, đề nghị kỹ thuật viên đã lắp đặt, hiệu chuẩn và cài đặt khẩu phần/loại thức ăn cho từng ao. Người đánh giá quan sát hành vi thực tế của động cơ cho ăn và đối chiếu bằng cân thực tế — không cần đọc màn hình kỹ thuật.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Cho ăn sai lượng (quá nhiều/quá ít) ảnh hưởng trực tiếp đến vật nuôi và chi phí thức ăn. Nếu hệ thống cấp nhầm khẩu phần giữa các ao hoặc chạy không kiểm soát, thiệt hại có thể xảy ra ngay trong lần vận hành đầu.

---

## Danh sách tình huống đánh giá

### 1. Khóa an toàn khi lắp đặt sai

**Tình huống:** Giả lập lắp đặt sai (kỹ thuật viên chủ động chỉnh sai 1 cấu hình động cơ).

**Quan sát:** Bật hệ thống cho ăn.

**Kết quả đạt:** Động cơ đứng yên, KHÔNG chạy khi cấu hình chưa đúng; màn hình điều khiển báo lỗi rõ ràng để kỹ thuật viên biết chỗ cần sửa.

☐ Đạt ☐ Không đạt

---

### 2. Cho ăn theo tốc độ cố định

**Tình huống:** Bật chế độ cho ăn ở mức cố định trong 1 phút.

**Quan sát:** Cân thực tế lượng thức ăn ra sau đúng 1 phút.

**Kết quả đạt:** Lượng thức ăn cân được gần đúng với khẩu phần/phút đã cài đặt trước (sai số nhỏ, chấp nhận được).

☐ Đạt ☐ Không đạt

---

### 3. Cho ăn tự động rải đều theo lộ trình

**Tình huống:** Bật chế độ cho ăn tự động rải đều trong lúc robot chạy hết 1 tuyến đường.

**Quan sát:** Cân tổng lượng thức ăn ra sau khi chạy hết tuyến.

**Kết quả đạt:** Tổng lượng thức ăn cân được gần đúng tổng khẩu phần đã đặt cho cả tuyến.

☐ Đạt ☐ Không đạt

---

### 4. Tự dừng khi robot không di chuyển

**Tình huống:** Ở chế độ rải đều theo lộ trình (tình huống 3), cho robot dừng hẳn hoặc chưa có lộ trình chạy.

**Quan sát:** Theo dõi động cơ cho ăn lúc robot đứng yên.

**Kết quả đạt:** Động cơ tự dừng, không tiếp tục cấp thức ăn vô ích khi robot không di chuyển.

☐ Đạt ☐ Không đạt

---

### 5. Đổi ao — khẩu phần tự đổi đúng, không lẫn ao

**Tình huống:** Đổi từ ao A sang ao B (mỗi ao đã cài khẩu phần/loại thức ăn khác nhau từ trước).

**Quan sát:** Kiểm tra lượng/loại thức ăn thực tế cấp ra khi đang ở ao B.

**Kết quả đạt:** Đúng khẩu phần/loại thức ăn của ao B, không bị lẫn với ao A. Đổi qua lại nhiều lần vẫn đúng.

☐ Đạt ☐ Không đạt

---

### 6. Nhớ đúng khẩu phần sau khi tắt/bật lại nguồn

**Tình huống:** Sau khi đã cài khẩu phần cho vài ao khác nhau, tắt nguồn robot rồi bật lại.

**Quan sát:** Kiểm tra lại từng ao đã cài trước đó.

**Kết quả đạt:** Khẩu phần/loại thức ăn của từng ao vẫn đúng như trước khi tắt nguồn, không bị mất/reset về mặc định.

☐ Đạt ☐ Không đạt

---

### 7. Cập nhật khẩu phần mới ngay tại hiện trường qua điện thoại/máy tính bảng

**Tình huống:** Khi kết nối robot với ứng dụng điều khiển trên điện thoại, nhập lượng thức ăn mới cho lần cho ăn này.

**Quan sát:** Kiểm tra lượng thức ăn thực tế cấp ra sau khi xác nhận.

**Kết quả đạt:** Hệ thống cấp đúng theo lượng mới vừa nhập, không dùng nhầm giá trị cũ.

☐ Đạt ☐ Không đạt

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Khóa an toàn khi lắp đặt sai | ☐ | ☐ | | |
| 2 | Cho ăn tốc độ cố định | ☐ | ☐ | | |
| 3 | Cho ăn rải đều theo lộ trình | ☐ | ☐ | | |
| 4 | Tự dừng khi robot không di chuyển | ☐ | ☐ | | |
| 5 | Đổi ao — không lẫn khẩu phần | ☐ | ☐ | | |
| 6 | Nhớ đúng khẩu phần sau tắt/bật lại | ☐ | ☐ | | |
| 7 | Cập nhật khẩu phần mới qua app | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
