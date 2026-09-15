# Module Bơm — Hướng dẫn vận hành bằng van cơ (điều khiển thủ công)

**Ngày viết:** 2026-09-15
**Áp dụng cho:** Cấu hình bơm chạy **cố định PWM max**, lưu lượng thực tế điều tiết bằng **van cơ** vặn tay, thay cho PID tự động bám setpoint.
**Không cần sửa firmware** — toàn bộ quy trình dưới đây dùng đúng các tham số/chế độ đã có sẵn (nấc 1 truyền thẳng tay), không thêm code mới.

---

## 1. Nguyên lý

Trước đây (tự động): PID tự đổi tốc độ bơm (PWM biến thiên) để bám đúng lưu lượng mục tiêu đã tính.

Giờ (thủ công): Bơm **luôn chạy ở PWM max cố định** — không PID, không biến thiên tốc độ nữa. Lưu lượng thực tế được điều tiết hoàn toàn bằng **van cơ** vặn tay, chặn dòng lại nhiều hay ít. Hệ thống vẫn tự **tính** lưu lượng mục tiêu (theo công thức mission hoặc setpoint cố định) để anh có con số tham chiếu, nhưng con số đó **không còn điều khiển bơm** — chỉ dùng để so sánh khi vặn van.

**Lợi ích phụ:** bỏ được giới hạn sàn 0.8 L/min từng ghi nhận trước đây (khi PID cố giảm tốc bơm để hạ lưu lượng, dưới một ngưỡng tốc độ motor không quay ổn định nữa) — vì giờ hạ lưu lượng bằng cách chặn van cơ khí, không phải giảm tốc motor, nên có thể xuống thấp hơn 0.8 L/min nếu cần, miễn van đủ tuyến tính ở vị trí đóng gần hết.

**Đánh đổi:** vì không còn PID bù trừ, van bị lệch/rung/mòn theo thời gian sẽ **không được tự động sửa** — cần kiểm tra định kỳ (xem mục 4, bước 5).

---

## 2. Bảng thông số liên quan

| Tham số | Vai trò trong quy trình này |
|---|---|
| `SA_RC_CHAN` | Cần chọn nấc — dùng để CHUYỂN QUA nấc 1 khi chạy thủ công. Nấc 1 = PWM < 1300. |
| `SA_RC_PUMP` | Kênh RC dùng ở nấc 1 (truyền thẳng) — đẩy/vặn kênh này lên MAX để bơm chạy full tốc. |
| `SA_PUMP_CHAN` | Kênh servo đầu ra bơm — tra `SERVOx_MIN/TRIM/MAX` của kênh này (x = số kênh) để biết PWM max thật là bao nhiêu. |
| `SA_FLOW_MODE` | Vẫn cần đặt đúng (0 = setpoint cố định `SA_FLOW_SP`, 1 = công thức mission) — vì đây là nguồn ra con số **tham chiếu** ở bước 1 của quy trình, dù không còn điều khiển PWM. |
| `SA_TANK_VOL`, `SA_FLOW_MIX_STD`, `SA_FLOW_MIX_CNT` | Chỉ cần nếu dùng `SA_FLOW_MODE=1` — quyết định con số mục tiêu tính theo mission. |
| `SA_FLOW_SP` | Chỉ cần nếu dùng `SA_FLOW_MODE=0` — con số mục tiêu cố định trực tiếp. |
| `SA_FLOW_LOG`, `SA_FLOW_LOG_MS` | Bật (`=1`) để xem log `[FLOW] FM<x> N<nấc> Q:<số>` — **CHỈ hiện con số MỤC TIÊU**, xem mục 3 lưu ý quan trọng. |
| `SA_CAL_FAC`, `SA_EMA_AL` | Cảm biến lưu lượng thật — không đổi gì trong quy trình này, cứ để nguyên đã hiệu chuẩn từ trước. |

---

## 3. ⚠️ Lưu ý quan trọng — đọc số MỤC TIÊU và số THẬT ở 2 nơi khác nhau

- **Số mục tiêu** (`flow_target`) → đọc qua dòng log console `[FLOW] FM<x> N<nấc> Q:<số>` (cần `SA_FLOW_LOG=1`).
- **Số lưu lượng THẬT** (đo từ cảm biến) → đọc qua màn hình app (ô "Lưu lượng"/"Trung bình") hoặc Mission Planner → MAVLink Inspector → `SA_DATA` → `data[0]`/`data[1]`. **KHÔNG có trong log console.**

**Cái bẫy cần tránh:** khi đã gạt về **nấc 1** để chạy thủ công, dòng log `Q:` sẽ **luôn hiện 0.00** — đây là điều **bình thường**, không phải lỗi (nấc 1 là truyền thẳng tay, không có "mục tiêu" nào để tính). Số 0.00 đó **không phải** là lưu lượng thật đang chảy — phải xem app/MAVLink Inspector mới thấy số thật.

Vì vậy quy trình bên dưới yêu cầu **đọc số mục tiêu TRƯỚC khi gạt về nấc 1** — nếu gạt về nấc 1 rồi mới đi tìm số mục tiêu thì sẽ không thấy nó ở đâu nữa (log đã về 0.00).

---

## 4. Quy trình vận hành / test từng bước

### Bước 1 — Lấy số mục tiêu tham chiếu (còn ở nấc 2 hoặc 3)

1. Cấu hình đúng `SA_FLOW_MODE` (0 hoặc 1) và các tham số liên quan theo ý muốn (xem bảng mục 2).
2. Bật `SA_FLOW_LOG=1`.
3. Gạt cần chọn nấc về **nấc 2** (mức tiêu chuẩn) hoặc **nấc 3** (mức cao) — tùy mức muốn hiệu chỉnh.
4. Nếu dùng `SA_FLOW_MODE=1`: cho xe chạy đúng mission + tốc độ dự kiến dùng thật ngoài hiện trường (số mục tiêu phụ thuộc tốc độ xe + quãng đường mission).
5. Đọc dòng log `[FLOW] FM<x> N<nấc> Q:<số>` — **ghi lại số này**, đây là mục tiêu cần vặn van tới.

### Bước 2 — Chuyển qua thủ công, đẩy PWM max

1. Gạt cần chọn nấc về **nấc 1**.
2. Đẩy/vặn kênh `SA_RC_PUMP` (tay cầm/công tắc tương ứng) lên hết cỡ.
3. Xác nhận PWM ra bơm đã đúng MAX: xem `data[3]` (`pump_pwm`) trong `SA_DATA`, so với `SERVOx_MAX` đã cấu hình cho kênh `SA_PUMP_CHAN`. Nếu chưa khớp — kiểm tra lại calib tay cầm/kênh RC.

### Bước 3 — Vặn van cơ theo số thật

1. Trong lúc bơm đang chạy max, mở màn hình app (ô "Lưu lượng") hoặc MAVLink Inspector (`data[0]`).
2. Vặn van cơ từ từ: **đóng bớt** để giảm lưu lượng, **mở thêm** để tăng.
3. Dừng lại khi số lưu lượng thật rơi vào khoảng chấp nhận quanh số mục tiêu đã ghi ở Bước 1 (xem bảng dung sai mục 5).

### Bước 4 — Đánh dấu vị trí van

Đánh dấu (sơn/khắc vạch/dán nhãn) vị trí van vừa chỉnh ứng với từng mức nấc/tỉ lệ mix cụ thể đang dùng — để các lần sau chỉ cần vặn thẳng tới vạch đó, không phải lặp lại từ đầu Bước 1-3.

### Bước 5 — Kiểm tra định kỳ (bắt buộc, vì không còn PID bù)

Vì van cơ không tự bù hao mòn/rung lắc/tắc nghẽn dần theo thời gian, cần **lặp lại Bước 3** theo định kỳ (khuyến nghị: đầu mỗi ca làm việc, hoặc sau mỗi lần vệ sinh/bảo trì bơm) để xác nhận vị trí van đã đánh dấu vẫn cho đúng lưu lượng mong muốn.

---

## 5. Bảng dung sai tham khảo (±20%)

| Mục tiêu (L/phút) | Cận dưới (−20%) | Cận trên (+20%) |
|---|---|---|
| 0.8 | 0.64 | 0.96 |
| 0.9 | 0.72 | 1.08 |
| 1.0 | 0.80 | 1.20 |
| 1.1 | 0.88 | 1.32 |
| 1.2 | 0.96 | 1.44 |
| 1.3 | 1.04 | 1.56 |

> **Ví dụ anh nêu (1.0 → 0.85–1.25) không hoàn toàn khớp ±20% đối xứng** (±20% của 1.0 đúng ra là 0.80–1.20) — có thể anh đang lấy khoảng dung sai theo kinh nghiệm thực tế của van, không phải tính đúng 20%. Nếu vậy, cứ dùng trực tiếp khoảng anh đã thấy phù hợp ngoài hiện trường; bảng trên chỉ là công thức tổng quát `mục_tiêu × 0.8` đến `mục_tiêu × 1.2` để tính nhanh cho các mức khác.

**Công thức tổng quát:** `cận_dưới = mục_tiêu × 0.8`, `cận_trên = mục_tiêu × 1.2` (đổi 0.8/1.2 nếu muốn dung sai khác 20%).

---

## 6. Rủi ro cần lưu ý khi chuyển sang cách vận hành này

- **Mất cảnh báo "ngoài dải bơm"/"chưa có mission"/"tốc độ quá thấp":** các cảnh báo này (`SA FM1: no mission...`, hậu tố `- out range`...) đều gắn với logic tự động (PID + gate mission/tốc độ) ở nấc 2/3 — khi đã gạt về **nấc 1**, các gate này **không còn áp dụng**, bơm chạy PWM max liên tục bất kể có mission hay xe đứng yên hay không. Người vận hành phải tự chủ động theo dõi, không còn được firmware nhắc.
- **Tank-empty detection** (`SA: TANK EMPTY...`) vẫn hoạt động ở mọi nấc vì chỉ dựa vào số đo lưu lượng thật, không liên quan PID — vẫn còn cảnh báo được nếu bơm hút khí.
- **Ramp-up từ từ lúc mới bật** (tăng dần 0.3 L/phút/giây) chỉ áp dụng cho nấc 2/3 — ở nấc 1, PWM nhảy thẳng lên max ngay khi đẩy tay ga, không có ramp. Cần lưu ý nếu hệ thống ống/van nhạy với thay đổi áp suất đột ngột.
