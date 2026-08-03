# Module 2 — pH Sensor (Modbus RTU)
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
[x] Tính năng mới hoàn toàn
[ ] Bổ sung vào hệ thống có sẵn
[ ] Sửa lỗi / thay đổi hành vi hiện tại
```

> Thêm mới chức năng đọc chất lượng nước ao từ cảm biến pH — không có chức năng này trong ArduPilot gốc.

---

## 2. Mục đích

Module 2 đọc và xử lý dữ liệu chất lượng nước từ cảm biến pH gắn trong ao:

- Đo **pH nước** để kiểm soát sức khỏe ao nuôi
- Đo **nhiệt độ nước** và **điện áp điện cực (mV)**
- Tính **độ kiềm (alkalinity)** dựa trên pH sáng+chiều cùng ao
- Tính **ΔpH ngày** (pH chiều − pH sáng) để đánh giá quang hợp / hoạt động sinh học
- **Quản lý tối đa 100 ao** — mỗi ao lưu riêng pH sáng/chiều/kiềm/ngày, người vận hành chọn ao đang đo bằng một tham số (SA_POND_IDX)
- **Lưu trữ dữ liệu ao vào thẻ SD**, còn nguyên qua reboot/mất điện — không cần đo lại từ đầu khi khởi động lại
- **Ghi vào SD card** để phân tích theo thời gian và vị trí (từng ao)

Dùng trong nuôi trồng thủy sản — người nuôi cần biết chất lượng nước realtime từ GCS/điện thoại mà không cần ra ao đo tay, và có thể luân phiên đo nhiều ao trong ngày mà không mất dữ liệu ao trước.

---

## 3. Phạm vi thay đổi

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| Đọc cảm biến chất lượng nước | Có | Toàn bộ giao tiếp với cảm biến pH qua dây RS485 |
| Giao tiếp GCS / MAVLink | Có | Gửi pH, nhiệt độ, mV, kiềm của ao đang active lên màn hình liên tục |
| Ghi SD card | Có | 2 loại bản ghi realtime/kiềm ngày (PHWD, PHAK) + 1 file trạng thái toàn bộ ao (SA_PONDS.bin) |
| Quản lý nhiều ao | Có | Tối đa 100 ao, chọn bằng SA_POND_IDX; mỗi ao lưu riêng pH sáng/chiều/kiềm/ngày, tồn tại qua reboot |
| Định vị GPS | Có | Mỗi mẫu pH được gắn tọa độ GPS; GPS dùng để cảnh báo lệch vị trí so với tâm ao, không dùng để tự động chọn ao |
| Phần cứng / kết nối | Có | Thêm cảm biến pH và module chuyển đổi tín hiệu |
| Điều khiển bơm / navigation | Không | Không liên quan |

---

## 4. Phần cứng / Giao tiếp sử dụng

| Thiết bị | Vai trò | Kết nối vào hệ thống qua | Ghi chú |
|---|---|---|---|
| Cảm biến pH (Nengshi ASPS3801D) | Đo pH, nhiệt độ, điện áp điện cực | Dây RS485 2 dây (A/B) | Ngâm trực tiếp trong ao |
| Module chuyển đổi RS485 → TTL | Chuyển tín hiệu RS485 sang tín hiệu FC đọc được | Cắm vào cổng TELEM của FC (RX/TX) | Module nhỏ ngoài, không tích hợp trong FC |
| Cổng TELEM của FC | Nhận dữ liệu từ cảm biến | UART (cổng serial) | Số cổng cài đúng trong cài đặt |
| GPS (module sẵn có của ArduPilot) | Cung cấp thời gian và tọa độ | Đã có sẵn trong hệ thống | Bắt buộc để phân slot và xác định ao |
| Thẻ SD card (trong FC) | Lưu lịch sử đo pH và kiềm theo thời gian | Khe cắm SD trên FC | Phân tích offline sau mỗi ngày vận hành |

> Module này không có phần cứng điều khiển đầu ra — chỉ đọc và ghi dữ liệu.

---

## 5. Flow hoạt động

```
Hệ thống tự động gửi yêu cầu đọc dữ liệu đến cảm biến mỗi 2 giây
    ↓
Cảm biến trả về: pH, điện áp điện cực (mV), nhiệt độ
    ↓
Kiểm tra tính hợp lệ (CRC) của dữ liệu nhận về
    ↓ Hợp lệ
Tính pH trung bình (MA, 10 mẫu) → gửi GCS qua MAVLink (ao đang active)
    │
    ├──► [Ghi vào SD card — PHWD]
    │         pH, pH-MA, nhiệt độ, mV, tọa độ GPS
    │         Mỗi 2 giây liên tục (không phân biệt ao)
    │
    └──► [Chọn ao đang đo — SA_POND_IDX, 1-100, người vận hành đặt tay]
              ↓
         Ao vừa đổi (khác lần trước)? → thông báo "Chuyển sang ao #n"
              → Nếu đã có ≥ 6 mẫu GPS trước đó: so tọa độ hiện tại với tâm ao
                đã lưu — lệch quá SA_PH_POND_D mét → cảnh báo (chỉ để tham
                khảo, KHÔNG chặn đo)
              ↓
         Đang ARM? (đứng yên/disarm không tính mẫu)
              ↓ Có
         Trong cửa sổ sáng (SA_PH_MS..SA_PH_ME) hoặc chiều (SA_PH_AS..SA_PH_AE)?
              ↓ Có, và cách mẫu trước ≥ SA_PH_CAP_S giây
         Ghi đè giá trị pH mới nhất vào slot (sáng hoặc chiều) của ao đó,
         đếm số mẫu — đủ SA_PH_CAP_SAM mẫu → báo "hoàn thành slot"
              ↓
         Ao đã có ĐỦ CẢ sáng và chiều trong ngày hôm nay?
              → Tính ΔpH và kiềm chính xác cho ao đó (1 lần/ngày/ao)
              → Ghi vào SD card — PHAK
              → Gửi SA_PHK qua MAVLink cho GCS
    ↓
Toàn bộ dữ liệu 100 ao được lưu định kỳ xuống thẻ SD (SA_PONDS.bin)
để không mất khi tắt nguồn/reboot
```

---

## 6. Tất cả Case và Output

### Case 1: Kết nối thành công, dữ liệu hợp lệ

- **Điều kiện:** Cảm biến đã kết nối đúng, dây RS485 tốt, FC nhận được dữ liệu
- **Hành vi hệ thống:** Cập nhật pH, nhiệt độ, mV mỗi 2 giây; ghi PHWD vào SD card liên tục
- **Output người dùng thấy:** GCS hiển thị pH, nhiệt độ, mV cập nhật liên tục. SD card có file log PHWD với tọa độ GPS từng điểm đo

### Case 2: Dữ liệu lỗi (nhiễu trên đường truyền)

- **Điều kiện:** Cảm biến kết nối nhưng dây RS485 bị nhiễu, dữ liệu nhận về bị hỏng
- **Hành vi hệ thống:** Bỏ qua frame lỗi, không ghi vào SD card; tiếp tục poll lần sau
- **Output người dùng thấy:** Cảnh báo "pH CRC fail" trên GCS; SD card không có record lỗi

### Case 3: Mất kết nối cảm biến

- **Điều kiện:** Cảm biến bị rút ra, đứt dây, hoặc hỏng — không gửi phản hồi
- **Hành vi hệ thống:** Xóa dữ liệu về 0 trên SA_DATA; phát cảnh báo; ngừng ghi SD card
- **Output người dùng thấy:** Cảnh báo "pH sensor mất kết nối (Xs)"; dữ liệu pH bằng 0; SD card ngừng nhận record mới

### Case 4: Chưa có dữ liệu lần nào (lần đầu boot)

- **Điều kiện:** Vừa khởi động, chưa nhận frame nào từ cảm biến
- **Hành vi hệ thống:** Chờ đợi; phát cảnh báo sau vài giây; không ghi SD card
- **Output người dùng thấy:** Cảnh báo "pH sensor chưa có dữ liệu"; SD card trống (chưa có PHWD)

### Case 5: Đủ dữ liệu sáng+chiều cho ao đang chọn (FULL)

- **Điều kiện:** Ao đang chọn (SA_POND_IDX) đã ghi nhận đủ pH sáng VÀ pH chiều trong cùng ngày (đang ARM khi đo)
- **Hành vi hệ thống:** Tính ΔpH và kiềm chính xác cho ao đó; ghi 1 record PHAK vào SD card với tọa độ ao (vị trí mẫu sáng cuối); gửi kèm qua MAVLink (SA_PHK) cho GCS/app
- **Output người dùng thấy:** GCS hiển thị kiềm chính xác nhất; nhãn [FULL]; SD card có 1 dòng PHAK cho ao đó; chỉ tính 1 lần/ngày/ao

### Case 6: Đổi ao đang đo (SA_POND_IDX) — cảnh báo lệch vị trí GPS

- **Điều kiện:** Người vận hành đổi `SA_POND_IDX` sang một ao khác (hoặc vừa boot xong)
- **Hành vi hệ thống:** Thông báo đã chuyển ao. Nếu ao đó đã có đủ lịch sử GPS (>5 mẫu), so tọa độ hiện tại với tâm ao đã lưu — lệch quá `SA_PH_POND_D` mét thì cảnh báo "cần kiểm tra lại vị trí ao", nhưng **KHÔNG chặn** việc đo/tính kiềm (đây là thông tin tham khảo, không phải điều kiện bắt buộc)
- **Output người dùng thấy:** `[SA] Chuyen sang ao #n`; sau đó `[SA] Ao#n GPS OK (Xm)` hoặc `[SA] Ao#n GPS lech Xm - can check lai vi tri ao`

### Case 7: Chỉ có dữ liệu một buổi (MORN hoặc AFT)

- **Điều kiện:** Ao đang chọn chỉ đo được một buổi, chưa có buổi còn lại trong ngày
- **Hành vi hệ thống:** Không tính kiềm; chờ buổi còn lại; không ghi PHAK. Khi vừa đủ số mẫu `SA_PH_CAP_SAM` của một buổi, hệ thống báo hoàn thành buổi đó một lần
- **Output người dùng thấy:** GCS không hiển thị kiềm ngày; nhãn [MORN] hoặc [AFT]; thông báo "Ao#n pH sang/chieu: X (N mau)"

### Case 8: Dùng dữ liệu hôm qua (PREV)

- **Điều kiện:** Sang ngày mới, ao đang chọn chưa có dữ liệu hôm nay nhưng đã có kiềm tính từ hôm trước
- **Hành vi hệ thống:** Hiển thị kiềm hôm qua; không ghi PHAK hôm nay cho đến khi đủ sáng+chiều
- **Output người dùng thấy:** GCS hiển thị kiềm hôm qua; nhãn [PREV]; SD card không có PHAK ngày hôm nay

### Case 9: Chưa từng có dữ liệu kiềm (NODATA)

- **Điều kiện:** Ao đang chọn chưa từng đo được slot nào (ao mới hoặc chưa từng đo)
- **Hành vi hệ thống:** Kiềm = 0; không ghi PHAK
- **Output người dùng thấy:** GCS hiển thị kiềm = 0; nhãn [NODATA]; không có PHAK trong log

### Case 10: Chưa có tín hiệu GPS hoặc chưa có giờ

- **Điều kiện:** FC chưa có GPS fix 3D hoặc chưa lấy được giờ UTC → không biết giờ địa phương và tọa độ
- **Hành vi hệ thống:** Vẫn đọc và ghi PHWD (Lat/Lng = 0 nếu chưa fix); KHÔNG chọn ao, KHÔNG phân slot sáng/chiều, KHÔNG tính kiềm cho đến khi có GPS
- **Output người dùng thấy:** Cảnh báo "Chua GPS - kiem doi GPS/gio" (tối đa mỗi 60s, chỉ khi SA_PH_LOG=1); PHWD có lat/lng = 0

### Case 11: Ghi realtime vào SD card (PHWD)

- **Điều kiện:** SA_PH_EN=1 và pH sensor có dữ liệu hợp lệ
- **Hành vi hệ thống:** Mỗi 2 giây ghi 1 record PHWD gồm pH, pH-MA, nhiệt độ, mV và tọa độ GPS tại thời điểm đó (không phân biệt ao)
- **Output người dùng thấy:** File .bin trên SD card; Mission Planner tab PHWD hiển thị đường pH theo thời gian có thể overlay lên bản đồ GPS

### Case 12: Ghi kiềm ngày vào SD card (PHAK) + gửi GCS (SA_PHK)

- **Điều kiện:** Ao đang chọn vừa đủ cả sáng và chiều trong ngày (FULL) — lần đầu tiên trong ngày cho ao đó
- **Hành vi hệ thống:** Ghi 1 record PHAK duy nhất với: pH sáng, pH chiều, ΔpH, kiềm dKH, kiềm mg/L, tọa độ ao (GPS mẫu sáng cuối), số thứ tự ao; đồng thời gửi 1 bản tin MAVLink riêng (SA_PHK) mang cùng dữ liệu cho GCS/app hiển thị ngay, không cần chờ tải log
- **Output người dùng thấy:** Mission Planner tab PHAK: 1 dòng mỗi ao mỗi ngày; lọc theo Lat/Lng hoặc PondIdx để xem kiềm từng ao

### Case 13: Khôi phục dữ liệu ao sau khi mất điện / reboot

- **Điều kiện:** FC vừa khởi động lại, đã từng có dữ liệu ao được lưu trước đó
- **Hành vi hệ thống:** Đọc lại toàn bộ dữ liệu 100 ao từ thẻ SD (pH sáng/chiều, kiềm, ngày, setpoint/loại thức ăn riêng của từng ao) ngay khi boot; dữ liệu mới tiếp tục được lưu định kỳ (mỗi 5s nếu có thay đổi)
- **Output người dùng thấy:** Thông báo "Load N ao tu SD card"; kiềm/PREV của các ao vẫn hiển thị đúng như trước khi mất điện

### Case 14: Chế độ thử nghiệm (SA_SIM=1)

- **Điều kiện:** Bật chế độ giả lập
- **Hành vi hệ thống:** Không giao tiếp RS485; dùng dữ liệu pH giả lập hình sin; ghi PHWD bình thường với GPS thực; vẫn chạy đủ pipeline chọn ao/phân slot/tính kiềm như dữ liệu thật nên PHAK/SA_PHK vẫn được tạo ra để kiểm tra trong SITL
- **Output người dùng thấy:** pH và nhiệt độ dao động ổn định; console log có tiền tố [SIM]; SD card có PHWD và PHAK với dữ liệu giả lập

---

## 7. Những gì KHÔNG thay đổi

- Điều khiển bơm phun (Module 1) và dosing motor (Module 3)
- Điều hướng ArduPilot (navigation, waypoint, auto mode)
- Toàn bộ phần cứng không liên quan đến cổng TELEM được chọn
- Cách hiển thị các thông số ArduRover khác trên GCS

---

## 8. Tài liệu liên quan

- [MODULE2_PH_DETAIL_DESIGN.md](MODULE2_PH_DETAIL_DESIGN.md) — giao thức Modbus, thuật toán kiềm, cơ chế SD card đầy đủ
- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — layout SA_DATA tổng thể
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống