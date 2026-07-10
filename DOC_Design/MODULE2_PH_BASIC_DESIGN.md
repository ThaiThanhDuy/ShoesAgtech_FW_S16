# Module 2 — pH Sensor (Modbus RTU)
## Basic Design Document

> **Đây là tài liệu TRƯỚC KHI code.**
> Mô tả YÊU CẦU và HÀNH VI — không liên quan đến code, tham số hay kỹ thuật bên trong.
> Người không biết lập trình cũng đọc được và hiểu hệ thống làm gì.

**Dự án:** `ardupilot-jbdcan_testing_S16`
**Ngày tạo:** 2026-05-01 | **Cập nhật lần cuối:** 2026-07-10
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
- **Ghi vào SD card** để phân tích theo thời gian và vị trí (từng ao)

Dùng trong nuôi trồng thủy sản — người nuôi cần biết chất lượng nước realtime từ GCS/điện thoại mà không cần ra ao đo tay.

---

## 3. Phạm vi thay đổi

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| Đọc cảm biến chất lượng nước | Có | Toàn bộ giao tiếp với cảm biến pH qua dây RS485 |
| Giao tiếp GCS / MAVLink | Có | Gửi pH, nhiệt độ, mV lên màn hình liên tục |
| Ghi SD card | Có | 3 loại bản ghi riêng biệt: realtime (PHWD), kiềm ngày (PHAK), điểm mẫu (PHSP) |
| Định vị GPS | Có | Mỗi mẫu pH được gắn tọa độ GPS; kiềm chỉ tính khi sáng+chiều cùng ao |
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
Tính pH trung bình (MA) → gửi GCS qua MAVLink
    │
    ├──► [Ghi vào SD card — PHWD]
    │         pH, pH-MA, nhiệt độ, mV, tọa độ GPS
    │         Mỗi 2 giây liên tục
    │
    ├──► [Phân loại thời gian đo]
    │         Trong cửa sổ sáng (SA_PH_MS..SA_PH_ME)?  → ghi slot sáng + GPS
    │         Trong cửa sổ chiều (SA_PH_AS..SA_PH_AE)? → ghi slot chiều + GPS
    │         Ngoài cả hai cửa sổ? → bỏ qua
    │         ↓
    │         Cả hai slot đủ VÀ cùng ao (≤ 300m)?
    │         → Tính ΔpH và kiềm chính xác
    │         → Ghi vào SD card — PHAK (1 lần/ngày)
    │
    └──► [Ghi điểm mẫu theo khoảng cách — PHSP]
              Mỗi SA_PH_SAMP_D mét (hoặc khi đến WP mới)
              → Ghi: WP.sub, pH, nhiệt độ, mV, GPS
              Ví dụ: SA_PH_SAMP_D=8m, WP1→WP2=32m → thêm 3 điểm (1.1, 1.2, 1.3)
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

### Case 5: Đủ dữ liệu sáng+chiều — cùng ao (FULL)

- **Điều kiện:** Đã ghi nhận pH sáng VÀ pH chiều trong cùng ngày, VÀ cả hai điểm đo cách nhau ≤ 300m (cùng ao)
- **Hành vi hệ thống:** Tính ΔpH và kiềm chính xác; ghi 1 record PHAK vào SD card với tọa độ ao (vị trí sáng)
- **Output người dùng thấy:** GCS hiển thị kiềm chính xác nhất; nhãn [FULL]; SD card có 1 dòng PHAK cho ao đó

### Case 6: Đủ dữ liệu sáng+chiều — khác ao

- **Điều kiện:** Đã ghi nhận pH sáng và chiều nhưng hai điểm đo cách nhau > 300m (khác ao)
- **Hành vi hệ thống:** KHÔNG tính kiềm — cảnh báo và chờ thêm dữ liệu cùng ao; không ghi PHAK
- **Output người dùng thấy:** Cảnh báo "Kiem: sang/chieu khac ao (XXXm) - bo qua"; GCS không hiển thị kiềm ngày hôm đó

### Case 7: Chỉ có dữ liệu một buổi (MORN hoặc AFT)

- **Điều kiện:** Chỉ đo được một buổi, chưa có buổi còn lại
- **Hành vi hệ thống:** Không tính kiềm; chờ buổi còn lại; không ghi PHAK
- **Output người dùng thấy:** GCS không hiển thị kiềm ngày; nhãn [MORN] hoặc [AFT]; cảnh báo cần chờ thêm

### Case 8: Dùng dữ liệu hôm qua (PREV)

- **Điều kiện:** Sang ngày mới, chưa có dữ liệu hôm nay
- **Hành vi hệ thống:** Hiển thị kiềm hôm qua; không ghi PHAK hôm nay cho đến khi đủ sáng+chiều
- **Output người dùng thấy:** GCS hiển thị kiềm hôm qua; nhãn [PREV]; SD card không có PHAK ngày hôm nay

### Case 9: Chưa từng có dữ liệu kiềm (NODATA)

- **Điều kiện:** Lần đầu khởi động, chưa từng đo được slot nào
- **Hành vi hệ thống:** Kiềm = 0; không tính ước tính realtime; không ghi PHAK
- **Output người dùng thấy:** GCS hiển thị kiềm = 0; nhãn [NODATA]; không có PHAK trong log

### Case 10: Chưa có tín hiệu GPS

- **Điều kiện:** FC chưa có GPS fix → không biết giờ và tọa độ
- **Hành vi hệ thống:** Đọc và ghi PHWD với Lat=0, Lng=0; không phân slot sáng/chiều; không ghi điểm mẫu PHSP
- **Output người dùng thấy:** Cảnh báo "Chưa có GPS time"; PHWD có lat/lng = 0

### Case 11: Ghi realtime vào SD card (PHWD)

- **Điều kiện:** SA_PH_EN=1 và pH sensor có dữ liệu hợp lệ
- **Hành vi hệ thống:** Mỗi 2 giây ghi 1 record PHWD gồm pH, pH-MA, nhiệt độ, mV và tọa độ GPS tại thời điểm đó
- **Output người dùng thấy:** File .bin trên SD card; Mission Planner tab PHWD hiển thị đường pH theo thời gian có thể overlay lên bản đồ GPS

### Case 12: Ghi kiềm ngày vào SD card (PHAK)

- **Điều kiện:** Slot FULL + cùng ao (≤ 300m) — lần đầu tiên trong ngày
- **Hành vi hệ thống:** Ghi 1 record PHAK duy nhất với: pH sáng, pH chiều, ΔpH, kiềm dKH, kiềm mg/L, tọa độ ao (GPS buổi sáng)
- **Output người dùng thấy:** Mission Planner tab PHAK: 1 dòng mỗi ao mỗi ngày; lọc theo Lat/Lng để xem kiềm từng ao

### Case 13: Ghi điểm mẫu theo khoảng cách (PHSP)

- **Điều kiện:** SA_PH_SAMP_D > 0 và GPS fix và pH có dữ liệu
- **Hành vi hệ thống:** Mỗi SA_PH_SAMP_D mét ghi 1 record PHSP với WP.sub (ví dụ 1.0, 1.1, 1.2, 2.0), pH-MA, nhiệt độ, mV, GPS. Khi đến WP mới → ghi "WP.0" và reset sub-counter
- **Output người dùng thấy:** Mission Planner tab PHSP: chuỗi điểm đo dọc theo tuyến đường — đúng vị trí trong ao nào, khoảng nào

### Case 14: Chế độ thử nghiệm (SA_SIM=1)

- **Điều kiện:** Bật chế độ giả lập
- **Hành vi hệ thống:** Không giao tiếp RS485; dùng dữ liệu pH giả lập hình sin; ghi PHWD và PHSP bình thường với GPS thực; PHAK không ghi (không phân slot trong simulation)
- **Output người dùng thấy:** pH và nhiệt độ dao động ổn định; console log có tiền tố [SIM]; SD card có PHWD với dữ liệu giả lập

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