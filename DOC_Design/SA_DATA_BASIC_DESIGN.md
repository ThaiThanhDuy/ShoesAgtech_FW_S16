# SA_DATA — MAVLink DEBUG_FLOAT_ARRAY
## Basic Design Document

> **Đây là tài liệu TRƯỚC KHI code.**
> Mô tả YÊU CẦU và HÀNH VI — không liên quan đến code, tham số hay kỹ thuật bên trong.
> Người không biết lập trình cũng đọc được và hiểu hệ thống làm gì.

**Dự án:** `ardupilot-jbdcan_testing_S16`
**Ngày tạo:** 2026-05-01 | **Cập nhật lần cuối:** 2026-07-16
**Người viết:** ThaiThanhDuy
**Trạng thái:** `[x] Draft   [ ] Review   [ ] Approved`

---

## 1. Loại thay đổi

```
[ ] Tính năng mới hoàn toàn
[x] Bổ sung vào hệ thống có sẵn
[ ] Sửa lỗi / thay đổi hành vi hiện tại
```

> Bổ sung một gói dữ liệu MAVLink tổng hợp vào hệ thống truyền telemetry đã có của ArduRover. File hệ thống `GCS_MAVLink_Rover.cpp` được chỉnh sửa để thêm hàm gửi gói này.

---

## 2. Mục đích

> **Tính năng này giải quyết vấn đề gì? Ai cần nó? Dùng trong tình huống nào?**

SA_DATA gom **tất cả dữ liệu của 3 module** (lưu lượng phun, pH nước, dosing motor) vào **một gói dữ liệu duy nhất** gửi liên tục lên GCS qua MAVLink.

Lý do gom chung:
- Phần mềm GCS (Mission Planner, QGroundControl, app tự viết) chỉ cần lắng nghe **một loại message** thay vì nhiều loại
- Giảm độ phức tạp của việc hiển thị và ghi log phía nhận
- Dễ dàng đẩy lên server / cloud theo dạng array dữ liệu

---

## 3. Phạm vi thay đổi

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| Giao tiếp GCS / MAVLink | Có | Thêm gói SA_DATA vào stream EXTRA3 |
| File hệ thống ArduPilot | Có | `GCS_MAVLink_Rover.cpp` được thêm hàm gửi |
| Logic điều khiển | Không | SA_DATA chỉ đọc dữ liệu, không điều khiển gì |
| Phần cứng | Không | Không thêm phần cứng mới |

---

## 4. Phần cứng / Giao tiếp sử dụng

> Không có phần cứng mới — SA_DATA truyền hoàn toàn qua kênh telemetry (radio, USB, UDP) đã có sẵn của ArduPilot.

| Kênh | Vai trò |
|---|---|
| Telemetry radio / USB / UDP | Truyền gói SA_DATA lên GCS |
| GCS (Mission Planner / QGC / app tự viết) | Nhận và hiển thị dữ liệu |

---

## 5. Flow hoạt động

```
Mỗi chu kỳ stream dữ liệu EXTRA3 (cài đặt trong ArduPilot):
    ↓
Kiểm tra: Hệ thống ShoesAgtech có đang bật không?
    ↓ Có
Thu thập dữ liệu từ 3 module (pH/dosing lấy theo AO ĐANG ĐO — SA_POND_IDX):
    - Module 1: lưu lượng thực tế, lưu lượng mục tiêu, PWM bơm, chế độ phun
    - Module 2: pH, nhiệt độ, kiềm, ΔpH, số hiệu ao, pH sáng/chiều, ngày đo
      (chỉ khi cảm biến pH đang bật và có dữ liệu)
    - Module 3: setpoint thức ăn, tỉ lệ quy đổi, loại thức ăn, PWM motor
    ↓
Đóng gói vào một mảng 58 số thực → gửi GCS (message DEBUG_FLOAT_ARRAY, tên "SA_DATA")
    ↓
Riêng khi một ao vừa đo đủ dữ liệu sáng+chiều trong ngày (tính xong kiềm):
Gửi thêm 1 gói riêng "SA_PHK" báo ngay kết quả kiềm của ao đó cho GCS/app
```

---

## 6. Tất cả Case và Output

### Case 1: Hoạt động bình thường — đủ dữ liệu

- **Điều kiện:** Hệ thống ShoesAgtech bật, telemetry EXTRA3 được bật trên GCS, cảm biến pH đang có dữ liệu
- **Hành vi hệ thống:** Gửi gói SA_DATA đầy đủ 3 module theo chu kỳ đặt sẵn
- **Output người dùng thấy:** GCS cập nhật liên tục: lưu lượng, pH, kiềm, trạng thái motor

### Case 2: Cảm biến pH tắt hoặc mất kết nối

- **Điều kiện:** SA_PH_EN=0 hoặc cảm biến pH mất kết nối > thời gian cho phép
- **Hành vi hệ thống:** Vẫn gửi gói SA_DATA, nhưng phần dữ liệu pH điền 0.0 (zero-padded)
- **Output người dùng thấy:** pH, kiềm, nhiệt độ hiển thị 0 trên GCS; phần lưu lượng và dosing vẫn hiển thị bình thường

### Case 3: Toàn bộ hệ thống ShoesAgtech tắt (SA_ENABLE=0)

- **Điều kiện:** SA_ENABLE=0
- **Hành vi hệ thống:** Không gửi gói SA_DATA nào
- **Output người dùng thấy:** GCS không nhận được DEBUG_FLOAT_ARRAY nào với tên "SA_DATA"

### Case 4: Stream EXTRA3 chưa bật trên GCS

- **Điều kiện:** Firmware gửi bình thường nhưng GCS chưa yêu cầu stream EXTRA3 (MAV1_EXTRA3=0)
- **Hành vi hệ thống:** Firmware sẵn sàng nhưng không có ai đăng ký nhận
- **Output người dùng thấy:** Không hiển thị dữ liệu SA_DATA dù firmware đang chạy → cần bật EXTRA3 ≥ 1 Hz trong cài đặt GCS

### Case 5: Phân biệt "pH tắt" với "pH mất kết nối"

- **Điều kiện:** Cả hai trường hợp đều trả về 0.0 ở vị trí pH
- **Hành vi hệ thống:** GCS không thể phân biệt từ SA_DATA — cần đọc tham số SA_PH_EN để xác nhận
- **Output người dùng thấy:** Phát hiện bằng: pH=0 VÀ điện áp điện cực=0 VÀ nhiệt độ=0 → hiển thị cảnh báo "pH không có dữ liệu"

### Case 6: Một ao vừa đo đủ dữ liệu trong ngày — gói SA_PHK

- **Điều kiện:** Đang bật pH; ao đang đo (SA_POND_IDX) vừa có đủ cả pH sáng và chiều trong ngày, hệ thống vừa tính xong kiềm
- **Hành vi hệ thống:** Ngoài gói SA_DATA gửi liên tục, hệ thống gửi thêm **một gói riêng "SA_PHK"** ngay tại thời điểm đó — mang đủ pH sáng/chiều, ΔpH, kiềm, tọa độ và số hiệu ao vừa đo xong
- **Output người dùng thấy:** GCS/app nhận được thông báo kiềm mới ngay lập tức, không cần chờ tải lại log từ thẻ SD

---

## 7. Những gì KHÔNG thay đổi

- Logic điều khiển của cả 3 module — SA_DATA chỉ đọc, không ghi
- Các stream MAVLink khác của ArduPilot (ATTITUDE, GPS, v.v.)
- Hành vi điều hướng và toàn bộ chức năng ArduRover cốt lõi
- Giới hạn cứng: tối đa 58 số thực mỗi gói — nếu cần thêm field phải dùng gói mới

---

## 8. Tài liệu liên quan

- [SA_DATA_DETAIL_DESIGN.md](SA_DATA_DETAIL_DESIGN.md) — layout đầy đủ từng field, code flow, pymavlink, QGC config
- [MODULE1_FLOW_BASIC_DESIGN.md](MODULE1_FLOW_BASIC_DESIGN.md) — nguồn gốc dữ liệu lưu lượng
- [MODULE2_PH_BASIC_DESIGN.md](MODULE2_PH_BASIC_DESIGN.md) — nguồn gốc dữ liệu pH
- [MODULE3_DOS_BASIC_DESIGN.md](MODULE3_DOS_BASIC_DESIGN.md) — nguồn gốc dữ liệu dosing
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống
