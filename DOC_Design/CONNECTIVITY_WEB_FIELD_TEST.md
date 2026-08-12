# Kết Nối, Gửi Dữ Liệu Và Hiển Thị Web — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích:** Dữ liệu từ robot được truyền qua ứng dụng điều khiển (điện thoại/máy tính bảng), rồi gửi lên trang web để xem/quản lý từ xa.
**Ngày viết:** 2026-08-10

> Trước khi đánh giá, đề nghị kỹ thuật viên đã kết nối xong ứng dụng điều khiển với máy chủ trung tâm, có tài khoản đăng nhập sẵn. Người đánh giá quan sát dữ liệu hiển thị trên app và trang web — không cần hiểu cách dữ liệu truyền đi kỹ thuật ra sao.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Nếu dữ liệu không lên được app/web, người quản lý sẽ không giám sát được hoạt động ao nuôi từ xa — mất đi lợi ích chính của hệ thống, dù robot vẫn hoạt động tốt tại hiện trường.

---

## Danh sách tình huống đánh giá

### 1. Kết nối và hiển thị trên ứng dụng điều khiển

**Tình huống:** Kết nối robot với ứng dụng trên điện thoại/máy tính bảng.

**Quan sát:** Nhìn màn hình ứng dụng.

**Kết quả đạt:** Ứng dụng hiển thị đầy đủ thông tin đang hoạt động: tình trạng bơm, tình trạng cho ăn, tình trạng cảm biến nước.

☐ Đạt ☐ Không đạt

---

### 2. Kiểm tra kết nối tới máy chủ trung tâm

**Tình huống:** Vào mục "Kết nối" trên ứng dụng, bấm kiểm tra kết nối máy chủ.

**Quan sát:** Kết quả hiển thị trên màn hình.

**Kết quả đạt:** Báo đúng thành công/thất bại; nếu nhập sai địa chỉ máy chủ, ứng dụng KHÔNG lưu lại địa chỉ sai đó (giữ nguyên địa chỉ cũ đang hoạt động).

☐ Đạt ☐ Không đạt

---

### 3. Dữ liệu lên đúng trang web sau khi vận hành

**Tình huống:** Vận hành robot đủ lâu (khoảng vài phút, đo nước + cho ăn), sau đó đăng nhập trang web giám sát.

**Quan sát:** Xem dữ liệu chất lượng nước, dữ liệu hoạt động robot, dữ liệu cho ăn trên trang web.

**Kết quả đạt:** Dữ liệu trên web khớp đúng với những gì vừa vận hành thực tế (đúng ao, đúng thời gian, đúng giá trị gần như trên ứng dụng điện thoại).

☐ Đạt ☐ Không đạt

---

### 4. Vẫn hoạt động bình thường khi mất mạng

**Tình huống:** Tắt mạng/wifi của điện thoại hoặc tắt máy chủ trong lúc đang vận hành, sau đó bật mạng lại.

**Quan sát:** Theo dõi ứng dụng trong lúc mất mạng và sau khi có mạng lại.

**Kết quả đạt:** Ứng dụng không bị treo/đứng trong lúc mất mạng, vẫn dùng bình thường; sau khi có mạng lại, dữ liệu bị dồn lại tự động gửi bù lên trang web.

☐ Đạt ☐ Không đạt

---

### 5. Phân quyền tài khoản đúng

**Tình huống:** Đăng nhập bằng tài khoản của trạm/thiết bị mình, sau đó thử đăng nhập bằng tài khoản quản trị (nếu có).

**Quan sát:** So sánh dữ liệu thấy được giữa 2 loại tài khoản.

**Kết quả đạt:** Tài khoản trạm chỉ thấy đúng dữ liệu của trạm mình; tài khoản quản trị thấy được dữ liệu của tất cả các trạm.

☐ Đạt ☐ Không đạt

---

### 6. ⚠️ Lưu ý cần xác nhận thêm với bộ phận kỹ thuật

**Tình huống:** Có một phần dữ liệu chi tiết (độ kiềm/vị trí GPS theo từng ao, gửi qua một kênh dữ liệu riêng từ robot) hiện **chưa được hiển thị** lên ứng dụng/trang web.

**Việc cần làm:** Không phải lỗi cần đánh giá Đạt/Không đạt ở đây — chỉ cần xác nhận với người phụ trách kỹ thuật xem phần dữ liệu này có cần thiết cho việc giám sát vận hành hay không, để quyết định có bổ sung hiển thị hay không.

☐ Đã trao đổi với kỹ thuật, không cần bổ sung
☐ Đã trao đổi với kỹ thuật, cần bổ sung (ghi vào mục theo dõi riêng)

---

## Bảng tổng hợp

| # | Tình huống | Đạt | Không đạt | Người đánh giá | Ngày |
|---|---|---|---|---|---|
| 1 | Kết nối và hiển thị trên ứng dụng | ☐ | ☐ | | |
| 2 | Kiểm tra kết nối máy chủ | ☐ | ☐ | | |
| 3 | Dữ liệu lên đúng trang web | ☐ | ☐ | | |
| 4 | Hoạt động bình thường khi mất mạng | ☐ | ☐ | | |
| 5 | Phân quyền tài khoản đúng | ☐ | ☐ | | |
| 6 | Xác nhận dữ liệu chưa hiển thị (xem trên) | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
