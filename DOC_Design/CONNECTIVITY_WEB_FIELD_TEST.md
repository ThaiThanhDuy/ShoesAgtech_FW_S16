# Kết Nối, Gửi Dữ Liệu Và Hiển Thị Web — Đánh giá vận hành thực tế

**Dùng cho:** Người đánh giá vận hành / QA hiện trường (không cần biết kỹ thuật)
**Mục đích:** Dữ liệu từ robot được truyền qua ứng dụng điều khiển (điện thoại/máy tính bảng), rồi gửi lên trang web để xem/quản lý từ xa.
**Ngày viết:** 2026-08-10
**Cập nhật:** 2026-08-15 — bổ sung chi tiết Đạt/Không đạt, tách rõ test mạng giả lập (văn phòng) và test mạng thật ngoài hiện trường, thêm tình huống nhiều trạm cùng lúc.

> Trước khi đánh giá, đề nghị kỹ thuật viên đã kết nối xong ứng dụng điều khiển với máy chủ trung tâm, có tài khoản đăng nhập sẵn. Người đánh giá quan sát dữ liệu hiển thị trên app và trang web — không cần hiểu cách dữ liệu truyền đi kỹ thuật ra sao.
>
> **Lưu ý quan trọng:** Tình huống 4 dưới đây có **2 phiên bản** — một phiên bản giả lập tại văn phòng (tắt/bật wifi thủ công) và một phiên bản test **ngay tại ao/hiện trường thực tế** (nơi sóng 3G/4G yếu, chập chờn, khác hẳn wifi ổn định trong nhà). Chỉ đánh giá "Đạt" ở tình huống 4 nếu đã test được cả 2 phiên bản — kết quả giả lập trong văn phòng tốt không đảm bảo hoạt động tốt ngoài ao có sóng yếu.

---

## Vì sao mỗi tình huống dưới đây quan trọng

Nếu dữ liệu không lên được app/web, người quản lý sẽ không giám sát được hoạt động ao nuôi từ xa — mất đi lợi ích chính của hệ thống, dù robot vẫn hoạt động tốt tại hiện trường. Vì phần lớn ao nuôi ở xa trung tâm, sóng di động yếu/chập chờn là điều kiện thực tế thường xuyên, không phải trường hợp hiếm.

---

## Danh sách tình huống đánh giá

### 1. Kết nối và hiển thị trên ứng dụng điều khiển

**Tình huống:** Kết nối robot với ứng dụng trên điện thoại/máy tính bảng.

**Quan sát:** Nhìn màn hình ứng dụng.

**Kết quả ĐẠT:** Ứng dụng hiển thị đầy đủ thông tin đang hoạt động: tình trạng bơm, tình trạng cho ăn, tình trạng cảm biến nước.

**Kết quả KHÔNG ĐẠT:** Thiếu ít nhất 1 trong 3 thông tin trên, hoặc dữ liệu hiển thị không cập nhật (đứng yên) dù robot đang hoạt động.

☐ Đạt ☐ Không đạt

---

### 2. Kiểm tra kết nối tới máy chủ trung tâm

**Tình huống:** Vào mục "Kết nối" trên ứng dụng, bấm kiểm tra kết nối máy chủ. Thử cả 2 trường hợp: nhập đúng địa chỉ và nhập sai địa chỉ.

**Quan sát:** Kết quả hiển thị trên màn hình sau mỗi lần bấm kiểm tra.

**Kết quả ĐẠT:** Báo đúng thành công/thất bại tương ứng; nếu nhập sai địa chỉ máy chủ, ứng dụng KHÔNG lưu lại địa chỉ sai đó (giữ nguyên địa chỉ cũ đang hoạt động).

**Kết quả KHÔNG ĐẠT:** Báo sai kết quả (báo thành công dù địa chỉ sai, hoặc ngược lại), hoặc ứng dụng lưu đè địa chỉ sai vào cấu hình khiến mất kết nối tới máy chủ đúng.

☐ Đạt ☐ Không đạt

---

### 3. Dữ liệu lên đúng trang web sau khi vận hành

**Tình huống:** Vận hành robot đủ lâu (khoảng vài phút, đo nước + cho ăn), sau đó đăng nhập trang web giám sát.

**Quan sát:** Xem dữ liệu chất lượng nước, dữ liệu hoạt động robot, dữ liệu cho ăn trên trang web.

**Kết quả ĐẠT:** Dữ liệu trên web khớp đúng với những gì vừa vận hành thực tế (đúng ao, đúng thời gian, đúng giá trị gần như trên ứng dụng điện thoại).

**Kết quả KHÔNG ĐẠT:** Dữ liệu trên web thiếu, sai giá trị, sai ao, sai thời gian, hoặc không xuất hiện dù đã vận hành xong và chờ hợp lý (quá 5 phút).

☐ Đạt ☐ Không đạt

---

### 4a. Vẫn hoạt động bình thường khi mất mạng (giả lập tại văn phòng)

**Tình huống:** Tắt mạng/wifi của điện thoại hoặc tắt máy chủ trong lúc đang vận hành, sau đó bật mạng lại.

**Quan sát:** Theo dõi ứng dụng trong lúc mất mạng và sau khi có mạng lại.

**Kết quả ĐẠT:** Ứng dụng không bị treo/đứng trong lúc mất mạng, vẫn dùng bình thường; sau khi có mạng lại, dữ liệu bị dồn lại tự động gửi bù lên trang web.

**Kết quả KHÔNG ĐẠT:** Ứng dụng bị treo/đứng khi mất mạng cần thoát vào lại, hoặc dữ liệu trong lúc mất mạng bị mất hẳn (không gửi bù được sau khi có mạng lại).

☐ Đạt ☐ Không đạt

---

### 4b. Vẫn hoạt động khi sóng yếu/chập chờn ngoài hiện trường thực tế ⚠️ (bắt buộc test tại ao, không thay thế bằng 4a)

**Tình huống:** Ngay tại vị trí ao thực tế (không phải văn phòng/gần wifi), vận hành robot ở nơi sóng 3G/4G yếu hoặc chập chờn (kiểm tra vạch sóng điện thoại yếu hoặc đi vào vùng khuất sóng nếu có).

**Quan sát:** Theo dõi ứng dụng và độ trễ dữ liệu trong suốt thời gian vận hành tại đó.

**Kết quả ĐẠT:** Ứng dụng vẫn dùng được để điều khiển tại chỗ dù sóng yếu (không phụ thuộc hoàn toàn vào máy chủ để vận hành cơ bản); dữ liệu gửi lên web có thể trễ nhưng cuối cùng vẫn lên đủ, không bị mất hẳn.

**Kết quả KHÔNG ĐẠT:** Ứng dụng không điều khiển được robot khi sóng yếu (dù robot vẫn ở gần, kết nối trực tiếp), hoặc dữ liệu bị mất hẳn không lên web sau khi hết vận hành dù đã đợi hợp lý.

☐ Đạt ☐ Không đạt ☐ Không kiểm tra được (khu vực test có sóng ổn định, chưa gặp điều kiện yếu)

---

### 5. Phân quyền tài khoản đúng

**Tình huống:** Đăng nhập bằng tài khoản của trạm/thiết bị mình, sau đó thử đăng nhập bằng tài khoản quản trị (nếu có).

**Quan sát:** So sánh dữ liệu thấy được giữa 2 loại tài khoản.

**Kết quả ĐẠT:** Tài khoản trạm chỉ thấy đúng dữ liệu của trạm mình; tài khoản quản trị thấy được dữ liệu của tất cả các trạm.

**Kết quả KHÔNG ĐẠT:** Tài khoản trạm nhìn thấy được dữ liệu của trạm khác (rò rỉ dữ liệu), hoặc tài khoản quản trị thiếu dữ liệu của một số trạm.

☐ Đạt ☐ Không đạt

---

### 6. Nhiều trạm/robot cùng gửi dữ liệu đồng thời ⚠️ (điều kiện thực tế khi triển khai nhiều ao)

**Tình huống:** Nếu tại hiện trường đang triển khai từ 2 robot/trạm trở lên, cho các trạm cùng vận hành và gửi dữ liệu lên cùng một tài khoản/máy chủ trong cùng khung giờ.

**Quan sát:** Kiểm tra trên trang web dữ liệu của từng trạm.

**Kết quả ĐẠT:** Dữ liệu của mỗi trạm hiển thị riêng biệt, đúng trạm, không bị trộn lẫn hoặc ghi đè lên nhau dù gửi cùng lúc.

**Kết quả KHÔNG ĐẠT:** Dữ liệu của trạm này bị ghi đè/trộn lẫn với trạm khác, hoặc chỉ 1 trong các trạm cập nhật được còn lại bị treo/mất dữ liệu.

☐ Đạt ☐ Không đạt ☐ Không kiểm tra được (hiện trường chỉ có 1 trạm)

---

### 7. ⚠️ Lưu ý cần xác nhận thêm với bộ phận kỹ thuật

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
| 4a | Hoạt động khi mất mạng (giả lập văn phòng) | ☐ | ☐ | | |
| 4b | Hoạt động khi sóng yếu (thực địa tại ao) | ☐ | ☐ | | |
| 5 | Phân quyền tài khoản đúng | ☐ | ☐ | | |
| 6 | Nhiều trạm gửi dữ liệu đồng thời | ☐ | ☐ | | |
| 7 | Xác nhận dữ liệu chưa hiển thị (xem trên) | ☐ | ☐ | | |

**Kết luận chung:** ☐ Đạt yêu cầu vận hành thực tế ☐ Chưa đạt, cần kỹ thuật viên kiểm tra lại

**Ghi chú/mô tả sự cố (nếu có):**

---
