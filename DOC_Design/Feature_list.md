# Feature List — AP_ShoesAgtech Extension

Danh sách tất cả file có liên quan đến tính năng mở rộng so với hệ thống ArduPilot mặc định.

---

## Thư viện mới

| Tên file | Đường dẫn | Loại | Chức năng chính |
|---|---|---|---|
| `AP_ShoesAgtech.h` | `libraries/AP_ShoesAgtech/` | Viết mới | Khai báo class, struct `PondEntry` (100 ao), tham số SA_*, getter cho flow/pH/dosing/pond, khai báo `consume_alk_log_pending()`, `consume_gcs_alk_pending()`, `_io_update()` |
| `AP_ShoesAgtech.cpp` | `libraries/AP_ShoesAgtech/` | Viết mới | Module 1 (flow sensor YF-S402B + pump PID), Module 2 (pH Modbus RTU + daily slot sáng/chiều + tính kiềm ΔpH + quản lý 100 ao), Module 3 (dosing motor servo 360°), load/save pond state vào SD (`_io_update` chạy IO thread) |
| `AP_BattMonitor_JBDCAN.h` | `libraries/AP_BattMonitor/` | Viết mới | Khai báo backend `AP_BattMonitor_JBDCAN` kế thừa `AP_BattMonitor_Backend`; đọc điện áp/dòng/SoC từ JBD BMS qua CAN bus |
| `AP_BattMonitor_JBDCAN.cpp` | `libraries/AP_BattMonitor/` | Viết mới | Nhận CAN frame từ JBD BMS (request ID 0x100), parse điện áp/dòng/SoC, ghi vào BattMonitor state; callback `handle_frame_callback()` + rate-limited 200ms |

---

## Rover firmware — mở rộng

| Tên file | Đường dẫn | Loại | Chỉnh sửa gì |
|---|---|---|---|
| `Parameters.h` | `Rover/` | Mở rộng | Thêm `AP_ShoesAgtech custom_nav` vào `ParametersG2`; thêm params Pitch Safety: `man_pitch_en`, `man_pitch_scale`, `man_pitch_delay`, `auto_pitch_en`, `auto_pitch_scale`, `auto_pitch_delay` |
| `Parameters.cpp` | `Rover/` | Mở rộng | Đăng ký `AP_SUBGROUPINFO` cho `custom_nav` prefix `SA_` (slot 58); đăng ký 6 param Pitch Safety (`MAN_PITCH_EN`, `MAN_PITCH_SCL`, `MAN_PITCH_DLY`, `AUTO_PITCH_EN`, `AUTO_PITCH_SCL`, `AUTO_PITCH_DLY`) |
| `Rover.h` | `Rover/` | Mở rộng | Thêm `#include AP_ShoesAgtech`, alias `custom_nav`, khai báo 3 hàm log: `Log_Write_Flow_Realtime()`, `Log_Write_Ph_Realtime()`, `Log_Write_Ph_Alkalinity()`, khai báo scheduler task `update_custom_flow()` |
| `Rover.cpp` | `Rover/` | Mở rộng | Đăng ký scheduler task `update_custom_flow` 10Hz (priority 22); implement `update_custom_flow()`: gọi `g2.custom_nav.update()` + 3 hàm log; gọi `g2.custom_nav.init()` trong `init_ardupilot()` |
| `defines.h` | `Rover/` | Mở rộng | Thêm 3 log message ID: `LOG_FLOW_DATA_MSG` (FLWD), `LOG_PH_DATA_MSG` (PHWD), `LOG_PH_ALK_MSG` (PHAK) |
| `Log.cpp` | `Rover/` | Mở rộng | Thêm 3 hàm ghi SD: `Log_Write_Flow_Realtime()` (FLWD), `Log_Write_Ph_Realtime()` (PHWD — pH/temp/mV/GPS), `Log_Write_Ph_Alkalinity()` (PHAK — kiềm ngày, dùng `consume_alk_log_pending()`); đăng ký struct trong `LOG_COMMON_STRUCTURES` |
| `GCS_MAVLink_Rover.cpp` | `Rover/` | Mở rộng | Thêm `mavlink_sa_data_send()`: gửi `SA_DATA` (DEBUG_FLOAT_ARRAY array_id=0, 19 field flow/pH/dosing) và `SA_PHK` (array_id=1, 8 field pH/kiềm/GPS ao) qua MAVLink; dùng `consume_gcs_alk_pending()` cho SA_PHK |
| `mode.h` | `Rover/` | Mở rộng | Thêm state Pitch Safety (`_pitch_safe_start_ms`); thêm khai báo AUTO_TUNE: `_autotune_poll()`, `_autotune_start()`, `_autotune_finish()`, `_autotune_sample_speed()` và các biến thống kê XTE/speed |
| `mode_auto.cpp` | `Rover/` | Mở rộng | Tích hợp Pitch Safety (AUTO) qua `AUTO_PITCH_EN`; thêm AUTO_TUNE — thu thập chỉ số ổn định 1 chu kỳ AUTO (XTE mean/max/oscilation, speed error) và in báo cáo khi thoát AUTO |
| `mode_manual.cpp` | `Rover/` | Mở rộng | Tích hợp Pitch Safety (MANUAL) qua `MAN_PITCH_EN`: phát hiện pitch nguy hiểm qua gyro, giảm throttle theo `MAN_PITCH_SCL`, trì hoãn phục hồi theo `MAN_PITCH_DLY` |

---

## Ghi chú

- `alk_pending` (SD) và `gcs_alk_pending` (MAVLink) là 2 flag độc lập trong `PondEntry` — được set đồng thời khi tính đủ ΔpH, consumed riêng biệt.
- `_io_update()` được đăng ký qua `hal.scheduler->register_io_process()` để đảm bảo `AP::FS()` chạy đúng IO thread (tương thích cả SITL lẫn hardware).
- `SA_PONDS_VER = 2` — bump version khi thêm field `gcs_alk_pending` vào struct.
- Pitch Safety dùng chung state `_pitch_safe_start_ms` giữa Manual và Auto (khai báo trong `mode.h`).
- JBDCAN backend đăng ký callback CAN qua `_iface->register_callback()`, không poll trong `read()`.
