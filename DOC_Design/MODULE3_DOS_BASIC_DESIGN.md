# Module 3 — Dosing Motor (Vít Tải Thức Ăn Tôm)
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

> Thêm mới chức năng điều khiển động cơ vít tải để định lượng và phân bổ thức ăn tôm — không có trong ArduPilot gốc.

---

## 2. Mục đích

Module 3 điều khiển động cơ vít tải (servo 360°) để phân phối thức ăn tôm tự động khi robot di chuyển dọc theo ao.

Người vận hành chỉ cần:
1. Bật/tắt motor bằng **một nút trên remote**
2. Hệ thống tự tính tốc độ quay để phân bổ đúng lượng thức ăn đã đặt

Hỗ trợ 2 chế độ:
- **Tốc độ cố định (DOS_MODE=0)**: Quay đều với tốc độ đặt sẵn, không phụ thuộc tốc độ xe hay tuyến đường
- **Tỉ lệ theo tuyến đường (DOS_MODE=1)**: Tự điều chỉnh tốc độ theo tốc độ xe để phân bổ đều toàn tuyến

**Tính năng khối lượng riêng:** Hệ thống phân tách thông số cơ học vít tải (thể tích tống ra) với đặc tính hạt thức ăn (khối lượng riêng g/mL). Mỗi loại thức ăn có thể cài riêng, không cần calibrate lại vít tải khi đổi loại hạt. Cả 2 chế độ tốc độ (cố định lẫn theo tuyến đường) đều dùng chung bộ thông số theo loại thức ăn này.

**Tính năng theo ao:** Lượng thức ăn (setpoint) và loại thức ăn đang dùng được lưu **riêng cho từng ao** (chung cơ chế chọn ao với Module 2 — pH). Đổi ao đang đo sẽ tự nạp lại đúng lượng/loại thức ăn đã cài cho ao đó; sửa setpoint hoặc loại thức ăn sẽ lưu lại cho ao đang chọn, còn nguyên qua reboot.

---

## 3. Phạm vi thay đổi

| Phần hệ thống | Bị ảnh hưởng? | Mô tả thay đổi |
|---|---|---|
| Điều khiển motor vít tải | Có | Toàn bộ logic bật/tắt và tính tốc độ motor |
| Giao tiếp GCS / MAVLink | Có | Gửi setpoint, tốc độ, trạng thái motor lên màn hình |
| Phần cứng / kết nối | Có | Thêm servo 360° và kênh servo điều khiển motor |
| Điều hướng (navigation) | Không | Không thay đổi |
| Module Flow, pH | Không | Không liên quan trực tiếp |

---

## 4. Phần cứng / Giao tiếp sử dụng

| Thiết bị | Vai trò | Kết nối vào hệ thống qua | Ghi chú |
|---|---|---|---|
| Động cơ vít tải (servo 360°) | Quay vít tải để đẩy thức ăn ra | Kênh servo của FC | Phải cài đúng 4 thông số servo (xem ràng buộc) |
| Nút bấm RC | Người lái bật/tắt motor | Kênh RC riêng | Nút bấm hoặc toggle switch; PWM > 1500 = bật |

> Nếu tín hiệu RC mất (drone mất kết nối remote) → motor tự dừng an toàn.

---

## 5. Flow hoạt động

```
Người lái nhấn nút RC để bật motor
    ↓
Hệ thống kiểm tra cấu hình servo có đúng không
    ↓ Đúng
Chọn loại thức ăn hiện tại của ao đang đo (SA_DOS_FOOD = 1..7, riêng theo ao)
    → Lấy thể tích vít tải (SA_DOS_Fx, mL/50us) — dùng chung cho cả 2 chế độ
    → Lấy khối lượng riêng hạt (SA_DOS_Dx, g/mL)
    ↓
Chọn chế độ tính tốc độ:
┌─────────────────────────────────────────────────────────────────────────┐
│ Chế độ 0 (tốc độ cố định):                                             │
│   offset(µs) = SA_DOS_SP(g) × 50 / (SA_DOS_Fx(mL/50us) × Dx(g/mL))   │
│   Tốc độ motor không thay đổi theo vận tốc xe                           │
│                                                                         │
│ Chế độ 1 (tỉ lệ tuyến đường):                                         │
│   dos_gpm = SA_DOS_SP(g) × speed × 60 / mission_dist                   │
│   offset(µs) = dos_gpm × 50 / (SA_DOS_Fx(mL/50us) × Dx(g/mL))        │
│   Xe nhanh → motor quay nhanh; xe chậm → motor quay chậm               │
│   Nếu xe chưa chạy hoặc chưa có tuyến → motor DỪNG                    │
└─────────────────────────────────────────────────────────────────────────┘
    ↓
Tín hiệu PWM xuất ra motor theo chiều quay đã cài (SA_DOS_REV)
    ↓
Trạng thái motor gửi lên GCS
```

---

## 6. Tất cả Case và Output

### Case 1: Cấu hình servo chưa đúng

- **Điều kiện:** Kênh servo chưa được cài 4 thông số bắt buộc (FUNCTION/MIN/TRIM/MAX)
- **Hành vi hệ thống:** Motor KHÔNG chạy dù người lái đã bật nút RC; phát cảnh báo cụ thể điều kiện nào sai
- **Output người dùng thấy:** Cảnh báo liên tục trên GCS mỗi 5 giây; motor không phản hồi

### Case 2: Nút RC tắt (hoặc mất tín hiệu RC)

- **Điều kiện:** Nút RC ở trạng thái tắt (PWM ≤ 1500) hoặc mất tín hiệu remote
- **Hành vi hệ thống:** Motor dừng ngay lập tức, giữ nguyên dừng
- **Output người dùng thấy:** Thông báo "Dosing motor OFF"; motor ngừng quay; GCS hiển thị PWM=1500

### Case 3: Nút RC bật — Chế độ 0 (tốc độ cố định)

- **Điều kiện:** Nút RC bật + DOS_MODE=0 + setpoint > 0
- **Hành vi hệ thống:** Tính offset từ `SA_DOS_SP ÷ (SA_DOS_Fx × SA_DOS_Dx)` (x = loại thức ăn đang chọn) — motor quay đều. Khi đổi loại thức ăn (SA_DOS_FOOD), hệ thống tự dùng thể tích + khối lượng riêng tương ứng (SA_DOS_F1..F7 / SA_DOS_D1..D7) mà không cần calibrate lại
- **Output người dùng thấy:** "Dosing motor ON"; GCS hiển thị PWM, setpoint, density của loại hạt đang dùng

### Case 4: Nút RC bật — Chế độ 1 (phân bổ theo tuyến đường) — đủ điều kiện

- **Điều kiện:** DOS_MODE=1 + đã upload tuyến đường + xe đã đạt ≥`SA_DOS_SPD_PCT`% tốc độ ĐẶT cho mission (mặc định 50%, xem Case 5)
- **Hành vi hệ thống:** Motor thay đổi tốc độ theo tốc độ xe để phân bổ đều `SA_DOS_SP` gam trên toàn tuyến đường. Xe nhanh → quay nhanh hơn, xe chậm → quay chậm hơn
- **Output người dùng thấy:** Motor quay nhanh/chậm theo xe; GCS hiển thị PWM thay đổi

### Case 5: Nút RC bật — Chế độ 1 — Chưa có tuyến đường hoặc xe chưa đủ tốc độ

- **Điều kiện:** DOS_MODE=1 nhưng chưa upload tuyến đường lên FC, hoặc xe đứng yên/mới tăng tốc chưa đạt **`SA_DOS_SPD_PCT`% tốc độ ĐẶT cho mission** (mặc định 50%, `SA_DOS_SPD_PCT=0` tắt kiểm tra này; ngưỡng tối thiểu tuyệt đối 0.05 m/s luôn áp dụng — cập nhật 2026-08-19, trước đây chỉ cần xe nhích bánh là đủ)
- **Hành vi hệ thống:** Motor DỪNG — không fallback sang tốc độ cố định; phát cảnh báo. Mục đích: tránh rải dồn thức ăn vào đoạn xe còn đang tăng tốc lúc bắt đầu chạy hoặc vừa qua khúc cua
- **Output người dùng thấy:** Motor dừng; cảnh báo "no mission" hoặc "speed too low"

### Case 6: Setpoint = 0 gam

- **Điều kiện:** `SA_DOS_SP = 0`
- **Hành vi hệ thống:** Motor không quay (offset = 0 → dừng tại 1500µs)
- **Output người dùng thấy:** Motor dừng; GCS hiển thị PWM=1500

### Case 7: Đổi loại thức ăn (SA_DOS_FOOD)

- **Điều kiện:** Người dùng thay SA_DOS_FOOD từ 1 → 2 (hoặc bất kỳ)
- **Hành vi hệ thống:** Hệ thống tự dùng SA_DOS_D2 (khối lượng riêng loại 2) và SA_DOS_F2 (thể tích loại 2 — dùng cho cả 2 chế độ tốc độ). Không cần reboot, áp dụng ngay chu kỳ kế tiếp (100ms). Giá trị mới được lưu lại riêng cho ao đang chọn
- **Output người dùng thấy:** GCS log hiển thị `F<n>` và `D:<x>g/mL` cập nhật theo loại hạt mới

### Case 8: Đổi ao đang đo (SA_POND_IDX dùng chung với Module 2)

- **Điều kiện:** Người vận hành đổi ao đang chọn
- **Hành vi hệ thống:** Hệ thống tự nạp lại lượng thức ăn (SA_DOS_SP) và loại thức ăn (SA_DOS_FOOD) đã cài riêng cho ao mới, ghi đè lên giá trị đang hiển thị. Nếu sau đó người dùng sửa setpoint/loại thức ăn, giá trị mới được lưu lại cho ao đang chọn (không ảnh hưởng ao khác)
- **Output người dùng thấy:** SA_DOS_SP và SA_DOS_FOOD trên GCS tự đổi theo giá trị đã lưu của ao mới; dữ liệu này còn nguyên qua reboot (lưu trên thẻ SD cùng dữ liệu ao của Module 2)

---

## 7. Những gì KHÔNG thay đổi

- Điều khiển bơm phun (Module 1) và cảm biến pH (Module 2)
- Điều hướng ArduPilot (navigation, waypoint, auto mode)
- Toàn bộ kênh RC và servo không được cài cho module này
- Hành vi fail-safe khác của ArduPilot

---

## 8. Tài liệu liên quan

- [MODULE3_DOS_DETAIL_DESIGN.md](MODULE3_DOS_DETAIL_DESIGN.md) — thuật toán, code flow, wiring đầy đủ
- [SA_DATA_BASIC_DESIGN.md](SA_DATA_BASIC_DESIGN.md) — layout SA_DATA tổng thể
- [AP_SHOESAGTECH_REFERENCE.md](AP_SHOESAGTECH_REFERENCE.md) — tổng hợp toàn hệ thống