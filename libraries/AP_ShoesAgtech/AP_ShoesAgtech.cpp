#include "AP_ShoesAgtech.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

volatile uint32_t AP_ShoesAgtech::_pulse_count = 0;

const AP_Param::GroupInfo AP_ShoesAgtech::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_ShoesAgtech, _enable_flag, 1),

    // @Param: CAL_FAC
    // @DisplayName: Flow sensor calibration (pulses/Litre) — YF-S402B
    // @Description: Số xung trên mỗi lít của cảm biến YF-S402B. Dãy hoạt động:
    // 0.3–6 L/min.
    // @User: Standard
    AP_GROUPINFO("CAL_FAC", 2, AP_ShoesAgtech, _cal_factor, 3874.5f),

    // @Param: EMA_AL
    // @DisplayName: Flow EMA smoothing alpha (0.01-1.0)
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("EMA_AL", 3, AP_ShoesAgtech, _ema_alpha, 0.1f),

    // @Param: FLOW_LOG
    // @DisplayName: Flow sensor console log enable
    // @Description: Print spray mode, flow target/actual/avg and pump PWM every
    // 1s.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("FLOW_LOG", 4, AP_ShoesAgtech, _flow_log_enable, 0),

    // @Param: RC_CHAN
    // @DisplayName: RC channel for spray mode switch (1-indexed)
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_CHAN", 5, AP_ShoesAgtech, _rc_chan, 6),

    // @Param: RC_PUMP
    // @DisplayName: RC channel for manual pump control in mode 0 (1-indexed)
    // @Description: Set SERVOx_FUNCTION=0(None) on the pump channel. In mode 0
    //   this library reads SA_RC_PUMP and writes it to SA_PUMP_CHAN directly,
    //   giving identical behaviour to RC passthrough without blocking modes
    //   1/2.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RC_PUMP", 6, AP_ShoesAgtech, _rc_pump, 9),

    // @Param: PUMP_CHAN
    // @DisplayName: Servo output channel for pump (1-indexed)
    // @Description: Must have SERVOx_FUNCTION=0 (None). This library controls
    //   it in all modes: mode 0 passes RC_PUMP through, modes 1/2 use PID.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("PUMP_CHAN", 7, AP_ShoesAgtech, _pump_chan, 8),

    // @Param: FLOW_SP
    // @DisplayName: Flow setpoint L/min (mode 1)
    // @Range: 0 200
    // @User: Standard
    AP_GROUPINFO("FLOW_SP", 8, AP_ShoesAgtech, _flow_setpoint, 5.0f),

    // @Param: PID_P
    // @DisplayName: PID P gain (us per L/min error)
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("PID_P", 9, AP_ShoesAgtech, _pid_p, 80.0f),

    // @Param: PID_I
    // @DisplayName: PID I gain (us per L/min/s)
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("PID_I", 10, AP_ShoesAgtech, _pid_i, 20.0f),

    // @Param: PID_LPF
    // @DisplayName: PID output LPF alpha (0.01=smooth, 1.0=raw)
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("PID_LPF", 11, AP_ShoesAgtech, _pid_lpf, 0.3f),

    // @Param: APP_RATE
    // @DisplayName: Application rate L/ha (mode 2)
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("APP_RATE", 12, AP_ShoesAgtech, _app_rate, 100.0f),

    // @Param: BOOM_W
    // @DisplayName: Boom width in metres (mode 2)
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("BOOM_W", 13, AP_ShoesAgtech, _boom_width, 1.0f),

    // [AP_ShoesAgtech] pH sensor parameters — Nengshi ASPS3801D-0.5M (slots
    // 14-19)

    // @Param: PH_EN
    // @DisplayName: Enable pH sensor
    // @Description: Enable Nengshi ASPS3801D-0.5M pH sensor via Modbus RTU
    // (RS485-TTL)
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_EN", 14, AP_ShoesAgtech, _ph_en, 0),

    // @Param: PH_PORT
    // @DisplayName: UART port for pH sensor (matches SERIALx number)
    // @Description: Set to match the SERIALx port connected to the RS485-TTL
    // module.
    //   Set SERIALx_BAUD=9 (9600) and SERIALx_PROTOCOL=0 (None) on that port.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("PH_PORT", 15, AP_ShoesAgtech, _ph_port, 2),

    // @Param: PH_TOFF
    // @DisplayName: Temperature offset (°C)
    // @Description: Added to the raw sensor temperature reading after /10
    // decode.
    // @Range: -10 10
    // @User: Standard
    AP_GROUPINFO("PH_TOFF", 16, AP_ShoesAgtech, _ph_toff, -3.5f),

    // @Param: PH_OFF
    // @DisplayName: pH calibration offset
    // @Description: Added to decoded pH value. Use buffer solution to
    // determine.
    // @Range: -2.0 2.0
    // @User: Standard
    AP_GROUPINFO("PH_OFF", 17, AP_ShoesAgtech, _ph_off, 0.0f),

    // @Param: PH_KH
    // @DisplayName: Base alkalinity dKH
    // @Description: Reference alkalinity measured by test kit (dKH). Used as
    // base
    //   for the alkalinity estimation algorithm. Update when you test the pond.
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("PH_KH", 18, AP_ShoesAgtech, _ph_kh, 4.0f),

    // @Param: PH_EMA
    // @DisplayName: pH EMA smoothing alpha (0.01-1.0)
    // @Description: Exponential Moving Average coefficient. Lower = smoother
    // but slower.
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("PH_EMA", 19, AP_ShoesAgtech, _ph_ema_alpha, 0.15f),

    // @Param: PH_LOG
    // @DisplayName: pH sensor console log enable (independent of SA_LOG_EN)
    // @Description: SA_LOG_EN controls flow/spray console log. SA_PH_LOG
    // controls
    //   pH sensor console log separately. Both can be enabled at the same time.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("PH_LOG", 20, AP_ShoesAgtech, _ph_log_enable, 0),

    // @Param: PH_TZ
    // @DisplayName: Timezone offset (hours, UTC+N)
    // @Description: Local time = UTC + PH_TZ. Vietnam is UTC+7 (default). Dung
    //   de phan loai doc pH vao slot sang [SA_PH_MS..SA_PH_ME] hoac slot chieu
    //   [SA_PH_AS..SA_PH_AE] cho tinh kiem nuoc hang ngay.
    // @Range: -12 14
    // @User: Standard
    AP_GROUPINFO("PH_TZ", 21, AP_ShoesAgtech, _ph_tz, 7),

    // @Param: LOG_FL_MS
    // @DisplayName: Flow console log interval (ms)
    // @Description: Khoảng thời gian giữa hai lần in dữ liệu lưu lượng ra
    // console khi SA_FLOW_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("LOG_FL_MS", 22, AP_ShoesAgtech, _flow_log_ms, 1000),

    // @Param: PH_LOG_MS
    // @DisplayName: pH console log interval (ms)
    // @Description: Khoảng thời gian giữa hai lần in dữ liệu pH/nhiệt độ/kiềm
    // ra console khi SA_PH_LOG=1.
    //   Không nên đặt nhỏ hơn chu kỳ Modbus (2000ms) vì sensor chỉ trả dữ liệu
    //   mỗi 2s.
    // @Range: 500 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("PH_LOG_MS", 23, AP_ShoesAgtech, _ph_log_ms, 2000),

    // @Param: PH_TIMEOUT
    // @DisplayName: pH disconnection timeout (s)
    // @Description: Số giây không nhận được frame pH hợp lệ thì coi là
    //   "mat ket noi" - phát cảnh báo qua STATUSTEXT và reset các giá trị
    //   pH/nhiệt độ/kiềm trong gói SA_DATA (DEBUG_FLOAT_ARRAY) về 0.
    // @Range: 1 300
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("PH_TIMEOUT", 24, AP_ShoesAgtech, _ph_timeout, 2),

    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] -------- Dosing motor (vít tải thức ăn tôm): servo xoay
    // liên tục 360° --------
    // @Param: DOS_CHAN
    // @DisplayName: Servo output channel for dosing motor (1-indexed)
    // @Description: Phải đặt SERVOx_FUNCTION=0 (None). Thư viện xuất PWM trực
    //   tiếp ra kênh này để điều khiển động cơ servo 360 độ định lượng.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_CHAN", 25, AP_ShoesAgtech, _dos_chan, 10),

    // @Param: DOS_RC
    // @DisplayName: RC channel to toggle dosing motor on/off (1-indexed)
    // @Description: PWM > 1500 -> bật động cơ (quay theo SA_DOS_SP/SA_DOS_RATE
    //   quy đổi), PWM <= 1500 -> tắt (xuất 1500, dừng). VD: nút B trên tay
    //   Skydroid T10 thường gán ở kênh 8.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_RC", 26, AP_ShoesAgtech, _dos_rc, 8),

    // @Param: DOS_RATE
    // @DisplayName: Dosing volume rate - DOS_MODE=0 (mL per 50us PWM offset)
    // @Description: The tich (mL) vit tai tong ra ung voi 50us PWM lech khoi
    //   diem dung (1500). Day la thong so co hoc cua vit tai, khong phu thuoc
    //   loai hat. VD: 100 -> 50us lech = 100mL tong ra.
    //   Cong thuc (DOS_MODE=0):
    //     offset(us) = SA_DOS_SP(g) * 50 / (SA_DOS_RATE(mL/50us) x SA_DOS_Dx(g/mL))
    //   Backward compat: dat SA_DOS_D1..D7=1.0 -> offset = DOS_SP*50/DOS_RATE.
    // @Units: mL
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_RATE", 27, AP_ShoesAgtech, _dos_rate, 100.0f),

    // @Param: DOS_SP
    // @DisplayName: Dosing setpoint (lượng thức ăn muốn cấp, gam)
    // @Description: Người dùng nhập trực tiếp lượng thức ăn mong muốn (gam).
    //   Firmware tự quy đổi ra độ lệch PWM theo tỉ lệ SA_DOS_RATE rồi cộng/trừ
    //   vào điểm dừng 1500 tuỳ chiều quay SA_DOS_REV. VD nhập 1000 (1kg) với
    //   SA_DOS_RATE=100 -> offset = 1000*50/100 = 500us.
    // @Units: g
    // @User: Standard
    AP_GROUPINFO("DOS_SP", 28, AP_ShoesAgtech, _dos_sp, 0.0f),

    // @Param: DOS_REV
    // @DisplayName: Dosing motor direction
    // @Description: Servo 360 độ, 1500=dừng. 0 = chiều thuận: PWM chạy trong
    //   dải 800-1500 (800=tốc độ cao nhất, giảm dần độ lệch về 1500). 1 =
    //   chiều ngược: PWM chạy trong dải 1500-2200 (xuất xung tăng dần từ 1500
    //   lên 2200, motor tự đảo chiều theo mức xung này).
    // @Values: 0:Thuan (800-1500), 1:Nguoc (1500-2200)
    // @User: Standard
    AP_GROUPINFO("DOS_REV", 29, AP_ShoesAgtech, _dos_rev, 0),

    // @Param: DOS_LOG
    // @DisplayName: Dosing motor console log enable
    // @Description: In ra console trạng thái ON/OFF, setpoint (SA_DOS_SP) và
    //   PWM đang xuất ra của dosing motor mỗi SA_DOS_LOG_MS mili-giây.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DOS_LOG", 30, AP_ShoesAgtech, _dos_log_enable, 0),

    // @Param: DOS_LOG_MS
    // @DisplayName: Dosing motor console log interval (ms)
    // @Description: Khoảng thời gian giữa hai lần in log dosing motor ra
    //   console khi SA_DOS_LOG=1.
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("DOS_LOG_MS", 31, AP_ShoesAgtech, _dos_log_ms, 1000),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] Simulation mode (slot 32)
    // @Param: SIM
    // @DisplayName: Simulation mode
    // @Description: Khi bật (1), bỏ qua cảm biến thật và inject dữ liệu giả lập
    //   có biến thiên hình sin để test hiển thị GCS và logic mode 1/2. Khi tắt
    //   (0)
    //   quay về đọc sensor thật.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SIM", 32, AP_ShoesAgtech, _simulation, 0),

    // [AP_ShoesAgtech] Flow sensor GPIO pin (slot 33)
    // @Param: FLOW_PIN
    // @DisplayName: Flow sensor GPIO pin number
    // @Description: Số chân GPIO kết nối tín hiệu cảm biến lưu lượng YF-S402B.
    //   Mặc định = 55 (Pixhawk/CubeOrange AUX GPIO). Thay đổi theo phần cứng.
    // @Range: 1 200
    // @User: Standard
    AP_GROUPINFO("FLOW_PIN", 33, AP_ShoesAgtech, _flow_pin, 55),

    // [AP_ShoesAgtech] Tank volume + flow mode (slots 34-35)
    // @Param: TANK_VOL
    // @DisplayName: Tank volume (Litres)
    // @Description: Dung tích tank nước/hóa chất (lít). Dùng để:
    //   (1) tính flow_target trong mode 1 khi SA_FLOW_MODE=1
    //   (2) hiển thị cảnh báo khoảng cách còn bơm được trong mode 2.
    //   Đặt = 0 để tắt cả hai chức năng.
    // @Units: L
    // @Range: 0 2000
    // @User: Standard
    // [AP_ShoesAgtech] Dosing motor mode (slot 36)
    // @Param: DOS_MODE
    // @DisplayName: Dosing motor speed mode
    // @Description: Cách tính tốc độ động cơ định lượng khi RC bật:
    //   0 = tốc độ cố định từ SA_DOS_SP/SA_DOS_RATE (hành vi cũ).
    //   1 = tốc độ tỉ lệ theo vận tốc + tổng quãng đường mission:
    //       offset = (SA_DOS_SP × speed × 60 / mission_dist) × 50 /
    //       SA_DOS_RATE.
    //       Phân bổ SA_DOS_SP gam đều trên toàn tuyến đường.
    //       Khi không có mission hoặc speed < 0.05 m/s → dừng + cảnh báo.
    // @Values: 0:Fixed,1:MissionProportional
    // @User: Standard
    AP_GROUPINFO("DOS_MODE", 36, AP_ShoesAgtech, _dos_mode, 0),
    // [/AP_ShoesAgtech]

    AP_GROUPINFO("TANK_VOL", 34, AP_ShoesAgtech, _tank_vol, 0.0f),

    // @Param: FLOW_MODE
    // @DisplayName: Mode 1 setpoint source
    // @Description: Cách tính flow_target trong mode 1 (FLOW PID):
    //   0 = trực tiếp từ SA_FLOW_SP (L/min) — như cũ.
    //   1 = tự tính từ SA_TANK_VOL + tổng quãng đường mission:
    //       flow = (tank_vol × speed × 60) / mission_dist.
    //       Khi không có mission hoặc SA_TANK_VOL=0, fallback về SA_FLOW_SP.
    // @Values: 0:DirectSetpoint,1:TankMissionFormula
    // @User: Standard
    AP_GROUPINFO("FLOW_MODE", 35, AP_ShoesAgtech, _flow_mode, 0),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] Vi sinh mixing ratio by RC field condition (slots 37-38)
    // @Param: MIX_STD
    // @DisplayName: Vi sinh ratio — nac giua (Mac dinh van)
    // @Description: Ti le vi sinh trong tong luong phun khi chon nac giua RC.
    //   Dung trong FLOW_MODE=1: q1_target = MIX_STD * APP_RATE * speed * BOOM *
    //   0.006.
    //   Dung trong FLOW_MODE=0: setpoint chinh la SA_FLOW_SP (khong can ratio).
    //   dist_max = TANK_VOL * 10000 / (MIX_STD * APP_RATE * BOOM).
    // @Range: 0.01 1.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("MIX_STD", 37, AP_ShoesAgtech, _mix_std, 0.35f),

    // @Param: MIX_CNT
    // @DisplayName: Vi sinh ratio — nac cao (Chong nghet van)
    // @Description: Ti le vi sinh trong tong luong phun khi chon nac cao RC
    //   (van vi sinh mo nhieu hon, chong nghet). FLOW_MODE=1: dung MIX_CNT thay
    //   MIX_STD trong cong thuc. FLOW_MODE=0: flow_target = SA_FLOW_SP *
    //   (MIX_CNT / MIX_STD) de giu tong luong ra boom giong nac giua.
    // @Range: 0.01 1.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("MIX_CNT", 38, AP_ShoesAgtech, _mix_cnt, 0.50f),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] Override speed for FLOW_MODE=1 calibration (slot 39)
    // @Param: FLOW_VEL
    // @DisplayName: Override speed for FLOW_MODE=1 (m/s)
    // @Description: 0 = dung van toc that tu GPS/AHRS. > 0 = ep van toc bang
    // gia
    //   tri nay (m/s) de tinh q1_target va dist_max — dung calib FLOW_MODE=1
    //   khi
    //   xe dung yen. Khong anh huong khi SA_SIM=1 (SA_SIM uu tien hon
    //   FLOW_VEL).
    // @Range: 0 10
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("FLOW_VEL", 39, AP_ShoesAgtech, _flow_vel, 0.0f),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] Dosing food type selector + per-type rate (slots 40-47)
    // @Param: DOS_FOOD
    // @DisplayName: Dosing food type selector (1-7)
    // @Description: Chon loai thuc an dang dung. He thong se dung SA_DOS_Fx
    // tuong
    //   ung de tinh toc do motor. Moi loai thuc an co the co ti le quy doi khac
    //   nhau
    //   do do nhot, khoi luong rieng khac nhau.
    // @Range: 1 7
    // @User: Standard
    AP_GROUPINFO("DOS_FOOD", 40, AP_ShoesAgtech, _dos_food, 1),
    // @Param: DOS_F1
    // @DisplayName: Dosing volume rate food type 1 - DOS_MODE=1 (mL per 50us offset)
    // @Description: The tich (mL) vit tai tong ra ung voi 50us lech, loai thuc an 1.
    //   Dung trong DOS_MODE=1 cung voi SA_DOS_D1 de tinh offset PWM.
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F1", 41, AP_ShoesAgtech, _dos_fr[0], 100.0f),
    // @Param: DOS_F2
    // @DisplayName: Dosing volume rate food type 2 - DOS_MODE=1 (mL per 50us offset)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F2", 42, AP_ShoesAgtech, _dos_fr[1], 100.0f),
    // @Param: DOS_F3
    // @DisplayName: Dosing volume rate food type 3 - DOS_MODE=1 (mL per 50us offset)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F3", 43, AP_ShoesAgtech, _dos_fr[2], 100.0f),
    // @Param: DOS_F4
    // @DisplayName: Dosing volume rate food type 4 - DOS_MODE=1 (mL per 50us offset)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F4", 44, AP_ShoesAgtech, _dos_fr[3], 100.0f),
    // @Param: DOS_F5
    // @DisplayName: Dosing volume rate food type 5 - DOS_MODE=1 (mL per 50us offset)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F5", 45, AP_ShoesAgtech, _dos_fr[4], 100.0f),
    // @Param: DOS_F6
    // @DisplayName: Dosing volume rate food type 6 - DOS_MODE=1 (mL per 50us offset)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F6", 46, AP_ShoesAgtech, _dos_fr[5], 100.0f),
    // @Param: DOS_F7
    // @DisplayName: Dosing volume rate food type 7 - DOS_MODE=1 (mL per 50us offset)
    // @Range: 0.1 10000
    // @User: Standard
    AP_GROUPINFO("DOS_F7", 47, AP_ShoesAgtech, _dos_fr[6], 100.0f),

    // [AP_ShoesAgtech] Bulk density per food type (slots 48-54)
    // offset(us) = SP(g) * 50 / (vol_rate(mL/50us) x density(g/mL))
    // Mac dinh 1.0 g/mL -> tuong duong cong thuc cu khi DOS_RATE = g/50us.
    // Do thuc te: do day 1L hat, can => chia cho 1000 = g/mL (khoi luong rieng xop).
    // @Param: DOS_D1
    // @DisplayName: Bulk density food type 1 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D1", 48, AP_ShoesAgtech, _dos_dr[0], 1.0f),
    // @Param: DOS_D2
    // @DisplayName: Bulk density food type 2 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D2", 49, AP_ShoesAgtech, _dos_dr[1], 1.0f),
    // @Param: DOS_D3
    // @DisplayName: Bulk density food type 3 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D3", 50, AP_ShoesAgtech, _dos_dr[2], 1.0f),
    // @Param: DOS_D4
    // @DisplayName: Bulk density food type 4 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D4", 51, AP_ShoesAgtech, _dos_dr[3], 1.0f),
    // @Param: DOS_D5
    // @DisplayName: Bulk density food type 5 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D5", 52, AP_ShoesAgtech, _dos_dr[4], 1.0f),
    // @Param: DOS_D6
    // @DisplayName: Bulk density food type 6 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D6", 53, AP_ShoesAgtech, _dos_dr[5], 1.0f),
    // @Param: DOS_D7
    // @DisplayName: Bulk density food type 7 (g/mL)
    // @Range: 0.1 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DOS_D7", 54, AP_ShoesAgtech, _dos_dr[6], 1.0f),
    // [/AP_ShoesAgtech]

    // [AP_ShoesAgtech] pH daily slot time windows (slots 55-58)
    // Gio theo dinh dang 24h (0-24). Moc ket thuc = 24 nghia la het gio trong ngay.
    // Readings ngoai ca hai cua so bi bo qua (khong cap nhat slot nao).
    // @Param: PH_MS
    // @DisplayName: pH morning slot start time (fractional hours, 0.0-23.99)
    // @Description: Gio bat dau cua so sang. Dung so thap phan: 5.5 = 5h30, 6.0 = 6h00.
    //   Doc pH trong khoang [PH_MS, PH_ME] -> cap nhat morning slot.
    // @Range: 0 23.99
    // @User: Standard
    AP_GROUPINFO("PH_MS", 55, AP_ShoesAgtech, _ph_ms, 5.0f),
    // @Param: PH_ME
    // @DisplayName: pH morning slot end time (fractional hours, 0.0-24.0, inclusive)
    // @Description: Gio ket thuc cua so sang. Dung so thap phan: 11.5 = 11h30, 11.0 = 11h00.
    //   Gia tri tai thoi diem nay van duoc ghi nhan (<=). Dat =24.0 de chay den het ngay.
    // @Range: 0 24
    // @User: Standard
    AP_GROUPINFO("PH_ME", 56, AP_ShoesAgtech, _ph_me, 11.0f),
    // @Param: PH_AS
    // @DisplayName: pH afternoon slot start time (fractional hours, 0.0-23.99)
    // @Description: Gio bat dau cua so chieu. Dung so thap phan: 13.5 = 13h30, 12.0 = 12h00.
    // @Range: 0 23.99
    // @User: Standard
    AP_GROUPINFO("PH_AS", 57, AP_ShoesAgtech, _ph_as, 12.0f),
    // @Param: PH_AE
    // @DisplayName: pH afternoon slot end time (fractional hours, 0.0-24.0, inclusive)
    // @Description: Gio ket thuc cua so chieu. Dung so thap phan: 16.5 = 16h30, 16.0 = 16h00.
    //   Gia tri tai thoi diem nay van duoc ghi nhan (<=). Dat =24.0 de chay den het ngay.
    // @Range: 0 24
    // @User: Standard
    AP_GROUPINFO("PH_AE", 58, AP_ShoesAgtech, _ph_ae, 16.0f),

    // @Param: PH_POND_D
    // @DisplayName: pH same-pond distance threshold (m)
    // @Description: Khoang cach toi da (m) giua diem do sang va chieu de coi la
    //   cung ao. Neu > nguong nay: bao canh bao khac ao, khong tinh kiem.
    //   Tang len neu ao lon hoac robot di nhieu vong. Max 5000m.
    // @Range: 10 5000
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("PH_POND_D", 60, AP_ShoesAgtech, _ph_pond_dist, 300.0f),
    // @Param: POND_IDX
    // @DisplayName: Pond index (manual selection)
    // @Description: Index ao dang do, nhap thu cong (1-100). GPS chi dung de validate
    //   vi tri: neu lech qua SA_PH_POND_D thi in log. Dat = 0 de tat pH.
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("POND_IDX", 59, AP_ShoesAgtech, _pond_select, 1),
    // @Param: PH_CAP_S
    // @DisplayName: pH capture interval (seconds)
    // @Description: Khoang thoi gian giua hai lan lay mau pH trong slot sang/chieu (giay).
    //   Mau duoc lay moi SA_PH_CAP_S giay, tich luy toi SA_PH_CAP_SAM mau roi tinh trung binh.
    //   Vi du: 20 -> lay mau moi 20 giay.
    // @Range: 1 3600
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("PH_CAP_S", 61, AP_ShoesAgtech, _ph_cap_s, 20),
    // @Param: PH_CAP_SAM
    // @DisplayName: pH capture sample count
    // @Description: So mau pH tich luy trong moi slot sang/chieu de tinh gia tri trung binh.
    //   Sau khi du so mau, slot bi khoa lai (khong lay them). Max 100 mau.
    //   Vi du: 20 -> lay 20 mau roi tinh trung binh lam gia tri pH buoi sang/chieu.
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("PH_CAP_SAM", 62, AP_ShoesAgtech, _ph_cap_sam, 20),
    // [/AP_ShoesAgtech]

    AP_GROUPEND};

AP_ShoesAgtech::AP_ShoesAgtech()
    : _last_timestamp_ms(0), _last_pulse_snapshot(0), _last_log_ms(0),
      _flow_rate_filtered(0.0f), _flow_rate_avg(0.0f), _is_initialized(false),
      _buffer_index(0), _buffer_sum(0.0f), _samples_count(0), _spray_mode(0),
      _pump_pwm(0), _flow_target(0.0f), _sim_speed(0.0f),
      // [AP_ShoesAgtech] mission distance cache + tank monitor
      _mission_dist_m(0.0f), _mission_ncmds(0), _tank_warn_ms(0),
      _arm_dist_warned(false), _was_armed(false), _tank_empty_detected(false),
      _tank_empty_ms(0),
      // [/AP_ShoesAgtech]
      _pid_integral(0.0f), _pid_output_lpf(0.0f), _pid_last_ms(0),
      _last_pump_chan(-1), _last_pump_func_val(-1), _pump_config_ok(false),
      _last_warn_ms(0),
      // [AP_ShoesAgtech] dosing motor initial state
      _dos_pwm(1500), _dos_config_ok(false), _dos_warn_ms(0),
      _dos_was_ok(false), _dos_was_on(false), _dos_last_log_ms(0),
      // [/AP_ShoesAgtech]
      // [AP_ShoesAgtech] pH sensor initial state
      _ph_uart(nullptr), _ph_update_ms(0), _ph_req_sent_ms(0),
      _ph_req_pending(false), _ph_last_good_ms(0), _ph_nodata_warn_ms(0),
      _ph_last_log_ms(0), _ph_value(0.0f), _ph_value_ema(-1.0f),
      _ph_value_ma(0.0f), _ph_mv(0), _ph_temp(25.0f),
      _ph_buf_idx(0), _ph_buf_count(0), _ph_buf_sum(0.0f),
      // per-pond manual selection tracking
      _pond_count(0),
      _ph_morn_val(0.0f), _ph_morn_lat(0), _ph_morn_lng(0),
      _ph_aft_val(0.0f), _delta_ph(0.0f),
      _alk_dkh(0.0f), _alk_mgl(0.0f),
      _alk_slot_status(4), _alk_pond_idx(0), _active_pond_idx(0), _slot_warn_ms(0),
      _ponds_save_ms(0), _pond_first_detect_done(false), _ponds_dirty(false),
      _ponds_loaded(false)
// [/AP_ShoesAgtech]
{
  memset(_sample_buffer, 0, sizeof(_sample_buffer));
  // [AP_ShoesAgtech]
  memset(_ph_buf, 0, sizeof(_ph_buf));
  memset(_ponds, 0, sizeof(_ponds));
  // [/AP_ShoesAgtech]
  AP_Param::setup_object_defaults(this, var_info);
}

void AP_ShoesAgtech::init(void) {
  if (!is_enabled()) {
    return;
  }

  uint8_t flow_pin = (uint8_t)constrain_int16(_flow_pin.get(), 1, 200);
  hal.gpio->pinMode(flow_pin, HAL_GPIO_INPUT);

  if (!hal.gpio->attach_interrupt(flow_pin, irq_handler,
                                  AP_HAL::GPIO::INTERRUPT_RISING)) {
    gcs().send_text(MAV_SEVERITY_CRITICAL, "ShoesAgtech: IRQ attach failed");
  } else {
    gcs().send_text(MAV_SEVERITY_INFO, "ShoesAgtech: Flow sensor ready");
  }

  _flow_rate_avg = 0.0f;
  _buffer_index = 0;
  _buffer_sum = 0.0f;
  _samples_count = 0;
  _pid_integral = 0.0f;
  _pid_output_lpf = 0.0f;
  _pid_last_ms = 0;
  memset(_sample_buffer, 0, sizeof(_sample_buffer));

  // [AP_ShoesAgtech] init pH sensor if enabled
  _ph_init();
  // [/AP_ShoesAgtech]
}

void AP_ShoesAgtech::irq_handler(void) { _pulse_count++; }

// =============================================================
// MAIN UPDATE — called at 10Hz from scheduler
// Flow sensor: YF-S402B, dãy hoạt động 0.3–6 L/min, GPIO pin 55
// =============================================================
void AP_ShoesAgtech::update(void) {
  if (!is_enabled()) {
    _flow_rate_filtered = 0.0f;
    _flow_rate_avg = 0.0f;
    return;
  }

  // [AP_ShoesAgtech] load pond state từ SD card lần đầu update (filesystem đã sẵn sàng)
  if (!_ponds_loaded) {
    _ponds_loaded = true;
    _pond_load();
  }
  // [/AP_ShoesAgtech]

  _check_pump_config();

  // [AP_ShoesAgtech] simulation or real sensor path
  if (_simulation.get() > 0) {
    _run_simulation();
  } else {
    // poll pH sensor over Modbus RTU (real hardware only)
    _ph_update();
  }
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] dosing motor — RC on/off + rate-to-PWM conversion
  _update_dosing_motor();
  // [/AP_ShoesAgtech]

  // [AP_ShoesAgtech] persist pond state to SD card when dirty (rate-limited 5s)
  if (_ponds_dirty) {
    uint32_t _now_ms = AP_HAL::millis();
    if (_now_ms - _ponds_save_ms >= 5000U) {
      _pond_save();
      _ponds_save_ms = _now_ms;
      _ponds_dirty   = false;
    }
  }
  // [/AP_ShoesAgtech]

  uint32_t now = AP_HAL::millis();
  uint32_t delta_t_ms = now - _last_timestamp_ms;

  if (!_is_initialized) {
    _last_timestamp_ms = now;
    _last_pulse_snapshot = _pulse_count;
    _pid_last_ms = now;
    _is_initialized = true;
    return;
  }

  // ---- 1. FLOW RATE CALCULATION (every 100ms) ----
  if (delta_t_ms >= 100) {
    _last_timestamp_ms = now;

    if (_simulation.get() > 0) {
      // Simulation: _flow_rate_filtered/_flow_rate_avg already set by
      // _run_simulation() Still need to advance the moving-average buffer with
      // the simulated value
      _buffer_sum -= _sample_buffer[_buffer_index];
      _sample_buffer[_buffer_index] = _flow_rate_filtered;
      _buffer_sum += _flow_rate_filtered;
      _buffer_index = (_buffer_index + 1) % WINDOW_SIZE;
      if (_samples_count < WINDOW_SIZE) {
        _samples_count++;
      }
      if (_samples_count > 0) {
        _flow_rate_avg = _buffer_sum / _samples_count;
      }
    } else {
      uint32_t snap = _pulse_count;
      uint32_t pulses = (snap >= _last_pulse_snapshot)
                            ? (snap - _last_pulse_snapshot)
                            : (UINT32_MAX - _last_pulse_snapshot) + snap + 1;
      _last_pulse_snapshot = snap;

      float dt = delta_t_ms * 0.001f;
      float cal = (_cal_factor.get() > 0.0f) ? _cal_factor.get() : 3874.5f;
      float raw = (dt > 0.0f) ? ((float)pulses / cal) * (60.0f / dt) : 0.0f;

      float alpha = constrain_float(_ema_alpha.get(), 0.01f, 1.0f);
      _flow_rate_filtered = _flow_rate_filtered * (1.0f - alpha) + raw * alpha;

      _buffer_sum -= _sample_buffer[_buffer_index];
      _sample_buffer[_buffer_index] = _flow_rate_filtered;
      _buffer_sum += _flow_rate_filtered;
      _buffer_index = (_buffer_index + 1) % WINDOW_SIZE;
      if (_samples_count < WINDOW_SIZE) {
        _samples_count++;
      }
      if (_samples_count > 0) {
        _flow_rate_avg = _buffer_sum / _samples_count;
      }
    }
  }

  // ---- 2. SPRAY CONTROL ----
  float dt_pid = (now - _pid_last_ms) * 0.001f;
  if (dt_pid <= 0.0f || dt_pid > 1.0f) {
    dt_pid = 0.1f;
  }
  _pid_last_ms = now;

  _update_spray_mode();

  bool now_armed = hal.util->get_soft_armed();

  // ARM edge: in trang thai FM1 mot lan khi arm, bat ke SA_FLOW_LOG
  if (now_armed && !_was_armed) {
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f &&
        (_spray_mode == 1 || _spray_mode == 2)) {
      float r = (_spray_mode == 2) ? _mix_cnt.get() : _mix_std.get();
      _print_fm1_arm_status(r);
    }
  }

  // Khi disarm: reset warning + cache mission + tank-empty detector
  if (!now_armed) {
    _arm_dist_warned = false;
    _mission_ncmds = 0;
    _mission_dist_m = 0.0f;
    _tank_empty_detected = false;
    _tank_empty_ms = 0;
  }
  _was_armed = now_armed;

  switch (_spray_mode) {

  case 0: {
    // ---- MODE 0: SOFTWARE PASSTHROUGH ----
    // Read SA_RC_PUMP channel and write directly to SA_PUMP_CHAN.
    // Requires SERVOx_FUNCTION = 0 (None) on the pump channel so
    // the hardware passthrough does not fight our writes in mode 1/2.
    uint8_t rc_pump_idx = (uint8_t)constrain_int16(_rc_pump.get() - 1, 0, 15);
    uint16_t rc_pwm = RC_Channels::get_radio_in(rc_pump_idx);
    if (rc_pwm < 800 || rc_pwm > 2200) {
      rc_pwm = 1500;
    }
    _flow_target = 0.0f;
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    _pump_pwm = rc_pwm;
    _write_pump_pwm(_pump_pwm);
    break;
  }

  case 1: {
    // ---- MODE 1: FLOW PID (nac giua — MIX_STD / Mac dinh van) ----
    // Yeu cau: vehicle phai duoc ARM truoc khi bom hoat dong
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch1 =
          SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
      if (ch1 != nullptr) {
        _write_pump_pwm(ch1->get_output_min());
      }
      break;
    }
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f) {
      // FLOW_MODE=1: cong thuc L/ha × vi sinh ratio (MIX_STD)
      _flow_target = _compute_visin_target(_mix_std.get());
      if (_flow_target < 0.01f) {
        // dieu kien khong dat (mission/dist/speed/q1): force min ngay, bo qua
        // PID
        SRV_Channel *ch1 =
            SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
        if (ch1 != nullptr) {
          _write_pump_pwm(ch1->get_output_min());
        }
        break;
      }
    } else {
      // FLOW_MODE=0: setpoint truc tiep tu SA_FLOW_SP
      _flow_target = _flow_setpoint.get();
    }
    _pump_pwm = _run_flow_pid(_flow_target, dt_pid);
    _write_pump_pwm(_pump_pwm);
    break;
  }

  case 2: {
    // ---- MODE 2: FLOW PID (nac cao — MIX_CNT / Chong nghet van) ----
    // Yeu cau: vehicle phai duoc ARM truoc khi bom hoat dong
    if (!hal.util->get_soft_armed()) {
      _flow_target = 0.0f;
      _pid_integral = 0.0f;
      _pid_output_lpf = 0.0f;
      SRV_Channel *ch2 =
          SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
      if (ch2 != nullptr) {
        _write_pump_pwm(ch2->get_output_min());
      }
      break;
    }
    if (_flow_mode.get() == 1 && _tank_vol.get() > 0.0f) {
      // FLOW_MODE=1: cong thuc L/ha × vi sinh ratio (MIX_CNT)
      _flow_target = _compute_visin_target(_mix_cnt.get());
      if (_flow_target < 0.01f) {
        // dieu kien khong dat: force min ngay, bo qua PID
        SRV_Channel *ch2 =
            SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
        if (ch2 != nullptr) {
          _write_pump_pwm(ch2->get_output_min());
        }
        break;
      }
    } else {
      // FLOW_MODE=0: setpoint × ti le MIX_CNT/MIX_STD (giu tong luong ra boom)
      float ratio =
          (_mix_std.get() > 0.01f) ? (_mix_cnt.get() / _mix_std.get()) : 1.0f;
      _flow_target =
          constrain_float(_flow_setpoint.get() * ratio, 0.0f, 200.0f);
      // Tank monitor: uoc tinh khoang cach con bom duoc (moi 30s)
      if (_tank_vol.get() > 0.0f && _flow_target > 0.01f) {
        float speed_ms2 = _get_spray_speed();
        if (speed_ms2 > 0.01f && now - _tank_warn_ms >= 30000U) {
          _tank_warn_ms = now;
          float dist_m = (_tank_vol.get() / _flow_target) * speed_ms2 * 60.0f;
          gcs().send_text(
              MAV_SEVERITY_INFO, "SA: Tank đủ ~%.0fm (%.1fL @%.1fL/min)",
              (double)dist_m, (double)_tank_vol.get(), (double)_flow_target);
        }
      }
    }
    _pump_pwm = _run_flow_pid(_flow_target, dt_pid);
    _write_pump_pwm(_pump_pwm);
    break;
  }

  default:
    break;
  }

  // ---- 3. PHAT HIEN THUNG HET VI SINH (ca FLOW_MODE=0 va 1) ----
  // Khi thung can, bom hut khi → banh xe cam bien quay nhanh bat thuong
  // → flow_rate_filtered dot ngot > 1.7 L/min du bom dang chay binh thuong.
  // Chi chay khi spray_mode 1 hoac 2 (bom dang duoc dieu khien PID), va ARM.
  // Nguong: 1.7 L/min lien tuc trong 3000ms → ket luan het thung, in 1 lan,
  // dung bom.
  if ((_spray_mode == 1 || _spray_mode == 2) && hal.util->get_soft_armed()) {
    if (!_tank_empty_detected) {
      if (_flow_rate_filtered > 1.7f) {
        if (_tank_empty_ms == 0) {
          _tank_empty_ms = now;
        } else if (now - _tank_empty_ms >= 3000U) {
          _tank_empty_detected = true;
          gcs().send_text(
              MAV_SEVERITY_CRITICAL,
              "SA: THÙNG HẾT VI SINH - flow %.1fL/ph > 1.7 trong 3s",
              (double)_flow_rate_filtered);
        }
      } else {
        _tank_empty_ms = 0;
      }
    }
  }

  // ---- 5. FLOW CONSOLE LOG (SA_LOG_EN) ----
  if (_flow_log_enable.get() > 0 &&
      now - _last_log_ms >= (uint32_t)_flow_log_ms.get()) {
    _last_log_ms = now;
    const char *flow_pfx = (_simulation.get() > 0) ? "[SIM][FLOW]" : "[FLOW]";
    gcs().send_text(MAV_SEVERITY_INFO,
                    "%s M%u Tgt:%.1f Act:%.1f Avg:%.1f PWM:%u", flow_pfx,
                    (unsigned)_spray_mode, (double)_flow_target,
                    (double)_flow_rate_filtered, (double)_flow_rate_avg,
                    (unsigned)_pump_pwm);
    // Khi nac giua/cao + FLOW_MODE=1: in them thong tin cong thuc moi
    // q1 = TANK_VOL * r * speed * 60 / mission_dist
    // vi/run = TANK_VOL * r  (hang so)
    // dmax = TANK_VOL * r * speed * 60 / 0.3  (thong tin, phu thuoc speed)
    if ((_spray_mode == 1 || _spray_mode == 2) && _flow_mode.get() == 1 &&
        _tank_vol.get() > 0.0f) {
      float r = (_spray_mode == 2) ? _mix_cnt.get() : _mix_std.get();
      float mdist = _get_mission_dist();
      float spd = _get_spray_speed();
      float vi_per_run = _tank_vol.get() * r;
      float q1_now = (mdist > 1.0f && spd > 0.1f)
                         ? (_tank_vol.get() * r * spd * 60.0f / mdist)
                         : 0.0f;
      float dmax =
          (spd > 0.1f) ? (_tank_vol.get() * r * spd * 60.0f / 0.3f) : 0.0f;
      gcs().send_text(MAV_SEVERITY_INFO,
                      "%s FM1 r:%.2f q1:%.2fL/ph miss:%.0fm dmax:%.0fm "
                      "spd:%.2fm/s vi/run:%.1fL",
                      flow_pfx, (double)r, (double)q1_now, (double)mdist,
                      (double)dmax, (double)spd, (double)vi_per_run);
    }
  }
}

// =============================================================
// =============================================================
// PUMP CONFIG CHECK
// Runs every update(). Prints servo MIN/TRIM/MAX on first boot
// or when PUMP_CHAN changes. Warns every 5s if function != 0.
// =============================================================
void AP_ShoesAgtech::_check_pump_config(void) {
  int8_t chan = _pump_chan.get();
  int32_t func_val = (int32_t)SRV_Channels::channel_function(
      (uint8_t)constrain_int16(chan - 1, 0, 15));

  bool changed = (chan != _last_pump_chan) || (func_val != _last_pump_func_val);

  if (!changed && _pump_config_ok) {
    return; // already OK and nothing changed
  }

  if (changed) {
    _last_pump_chan = chan;
    _last_pump_func_val = func_val;
  }

  if (func_val != (int32_t)SRV_Channel::k_none) {
    // Wrong function — print warning every 5s
    uint32_t now = AP_HAL::millis();
    if (changed || now - _last_warn_ms >= 5000) {
      _last_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA: SERVO%d_FUNCTION=%d must be 0(None)!", (int)chan,
                      (int)func_val);
    }
    _pump_config_ok = false;
  } else {
    // Correct function — print servo config
    SRV_Channel *ch = SRV_Channels::srv_channel((uint8_t)(chan - 1));
    if (ch != nullptr) {
      gcs().send_text(MAV_SEVERITY_INFO, "SA: SERVO%d OK Min:%u Trim:%u Max:%u",
                      (int)chan, (unsigned)ch->get_output_min(),
                      (unsigned)ch->get_trim(), (unsigned)ch->get_output_max());
    }
    _pump_config_ok = true;
  }
}

// RC → SPRAY MODE
//   Nấc 1 (PWM < 1300) : mode 0 — passthrough, không can thiệp
//   Nấc 2 (1300-1700)  : mode 1 — FLOW PID, tỉ lệ SA_MIX_STD (Mặc định van)
//   Nấc 3 (PWM > 1700) : mode 2 — FLOW PID, tỉ lệ SA_MIX_CNT (Chống nghẹt van)
// =============================================================
void AP_ShoesAgtech::_update_spray_mode(void) {
  uint8_t idx = (uint8_t)constrain_int16(_rc_chan.get() - 1, 0, 15);
  uint16_t pwm_in = RC_Channels::get_radio_in(idx);

  if (pwm_in == 0) {
    return; // no signal — keep current mode
  }

  if (pwm_in < 1300) {
    _spray_mode = 0;
  } else if (pwm_in < 1700) {
    _spray_mode = 1;
  } else {
    _spray_mode = 2;
  }
}

// =============================================================
// PI CONTROLLER + OUTPUT LPF
// Base PWM = servo TRIM (from SERVOx_TRIM parameter)
// Range    = servo MIN/MAX (from SERVOx_MIN/MAX parameters)
// =============================================================
uint16_t AP_ShoesAgtech::_run_flow_pid(float target_lmin, float dt) {
  SRV_Channel *ch = SRV_Channels::srv_channel((uint8_t)(_pump_chan.get() - 1));
  uint16_t pwm_min = (ch != nullptr) ? ch->get_output_min() : 1000;
  uint16_t pwm_max = (ch != nullptr) ? ch->get_output_max() : 2000;
  uint16_t pwm_trim = (ch != nullptr) ? ch->get_trim() : 1500;

  float error = target_lmin - _flow_rate_filtered;
  float i_gain = _pid_i.get();

  // Integral with anti-windup clamped to ±half PWM range
  _pid_integral += error * dt;
  if (i_gain > 0.0f) {
    float ilimit = (pwm_max - pwm_min) * 0.5f / i_gain;
    _pid_integral = constrain_float(_pid_integral, -ilimit, ilimit);
  }

  // PI raw output (in microseconds relative to trim)
  float pid_raw = _pid_p.get() * error + i_gain * _pid_integral;

  // Low-pass filter to smooth PWM commands
  float alpha = constrain_float(_pid_lpf.get(), 0.01f, 1.0f);
  _pid_output_lpf = _pid_output_lpf * (1.0f - alpha) + pid_raw * alpha;

  // Final PWM = trim + PID, clamped to servo min/max
  return (uint16_t)constrain_float((float)pwm_trim + _pid_output_lpf,
                                   (float)pwm_min, (float)pwm_max);
}

// =============================================================
// WRITE PUMP CHANNEL
// Calls set_output_pwm_chan() which sets have_pwm_mask so that
// calc_pwm() will NOT overwrite our value in subsequent loops.
// Requires SERVOx_FUNCTION = 0 (None) on the pump channel so
// output_ch() passthrough does not overwrite our output_pwm.
// =============================================================
void AP_ShoesAgtech::_write_pump_pwm(uint16_t pwm) {
  uint8_t chan_idx = (uint8_t)constrain_int16(_pump_chan.get() - 1, 0, 15);
  SRV_Channels::set_output_pwm_chan(chan_idx, pwm);
}

// =============================================================
// [AP_ShoesAgtech] DOSING MOTOR — vít tải thức ăn tôm, servo xoay liên tục 360°
//
// RC SA_DOS_RC bật/tắt: PWM > 1500 -> bật, PWM <= 1500 (kể cả mất tín hiệu = 0)
// -> tắt, xuất 1500 (dừng).
//
// Quy đổi lượng thức ăn (SA_DOS_SP, gam) -> độ lệch PWM:
//   DOS_MODE=0 (toc do co dinh):
//     vol_rate = SA_DOS_RATE          (mL / 50us — co hoc vit tai)
//     density  = SA_DOS_Dx            (g/mL — khoi luong rieng hat, x = SA_DOS_FOOD)
//     offset   = SA_DOS_SP * 50 / (vol_rate * density)
//   DOS_MODE=1 (phan bo deu theo mission):
//     vol_rate = SA_DOS_Fx            (mL / 50us — theo loai hat x)
//     density  = SA_DOS_Dx            (g/mL)
//     dos_gpm  = SA_DOS_SP * speed * 60 / mission_dist
//     offset   = dos_gpm * 50 / (vol_rate * density)
//
// Backward compat: dat SA_DOS_D1..D7 = 1.0 (mac dinh) -> offset = SP*50/RATE
//
// Chiều quay theo SA_DOS_REV (servo 360°, 1500 = dừng):
//   0 = thuận: pwm = constrain(1500 - offset,  800, 1500)  (800 = nhanh nhất)
//   1 = ngược: pwm = constrain(1500 + offset, 1500, 2200)
// Yêu cầu SERVOx_FUNCTION = 0 (None) trên kênh SA_DOS_CHAN.
// =============================================================

// =============================================================
// DOSING CHANNEL CONFIG CHECK
// Servo SA_DOS_CHAN BẮT BUỘC phải thoả cả 4 điều kiện mới cho phép
// motor chạy (dù bấm nút SA_DOS_RC):
//   FUNCTION = 0 (None), MIN = 800, TRIM = 1500, MAX = 2200
// Sai điều kiện nào -> chỉ báo (các) điều kiện đó, lặp lại mỗi 5 giây.
// Khi vừa đạt đủ cả 4 -> báo "setup thành công" một lần.
// =============================================================
void AP_ShoesAgtech::_check_dosing_config(void) {
  uint8_t chan_idx = (uint8_t)constrain_int16(_dos_chan.get() - 1, 0, 15);
  SRV_Channel *ch = SRV_Channels::srv_channel(chan_idx);
  int32_t func_val = (int32_t)SRV_Channels::channel_function(chan_idx);
  int32_t chan = (int32_t)_dos_chan.get();

  bool have_chan = (ch != nullptr);
  bool func_ok = have_chan && (func_val == (int32_t)SRV_Channel::k_none);
  bool min_ok = have_chan && (ch->get_output_min() == 800);
  bool trim_ok = have_chan && (ch->get_trim() == 1500);
  bool max_ok = have_chan && (ch->get_output_max() == 2200);

  _dos_config_ok = func_ok && min_ok && trim_ok && max_ok;

  if (_dos_config_ok) {
    if (!_dos_was_ok) {
      gcs().send_text(MAV_SEVERITY_INFO,
                      "SA: SERVO%d setup thành công - dosing motor sẵn sàng",
                      (int)chan);
    }
    _dos_was_ok = true;
    return;
  }
  _dos_was_ok = false;

  uint32_t now = AP_HAL::millis();
  if (now - _dos_warn_ms < 5000) {
    return;
  }
  _dos_warn_ms = now;

  if (!have_chan) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d không tồn tại",
                    (int)chan);
    return;
  }
  if (!func_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA: SERVO%d FUNCTION=%d, cần đặt =0 (None)", (int)chan,
                    (int)func_val);
  }
  if (!min_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MIN=%u, cần đặt =800",
                    (int)chan, (unsigned)ch->get_output_min());
  }
  if (!trim_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d TRIM=%u, cần đặt =1500",
                    (int)chan, (unsigned)ch->get_trim());
  }
  if (!max_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MAX=%u, cần đặt =2200",
                    (int)chan, (unsigned)ch->get_output_max());
  }
}

void AP_ShoesAgtech::_update_dosing_motor(void) {
  _check_dosing_config();
  if (!_dos_config_ok) {
    // Cấu hình servo sai — không cho chạy, kể cả khi bấm SA_DOS_RC
    _dos_pwm = 1500;
    return;
  }

  uint32_t now = AP_HAL::millis();
  uint8_t rc_idx = (uint8_t)constrain_int16(_dos_rc.get() - 1, 0, 15);
  uint16_t rc_pwm = RC_Channels::get_radio_in(rc_idx);
  bool motor_on = (rc_pwm > 1500);

  if (motor_on != _dos_was_on) {
    _dos_was_on = motor_on;
    gcs().send_text(MAV_SEVERITY_INFO, "SA: Dosing motor %s",
                    motor_on ? "ON" : "OFF");
  }

  if (motor_on) {
    float pwm_f = 1500.0f;

    uint8_t food_idx = (uint8_t)constrain_int16(_dos_food.get(), 1, 7) - 1;

    if (_dos_mode.get() == 0) {
      // ---- DOS_MODE 0: toc do co dinh ----
      // offset = SP(g) * 50 / (SA_DOS_RATE(mL/50us) x SA_DOS_Dx(g/mL))
      float vol_rate = _dos_rate.get();
      if (vol_rate < 0.1f) {
        vol_rate = 0.1f;
      }
      float density = _dos_dr[food_idx].get();
      if (density < 0.01f) {
        density = 0.01f;
      }
      float offset = _dos_sp.get() * 50.0f / (vol_rate * density);
      if (_dos_rev.get() == 0) {
        pwm_f = constrain_float(1500.0f - offset, 800.0f, 1500.0f);
      } else {
        pwm_f = constrain_float(1500.0f + offset, 1500.0f, 2200.0f);
      }
    } else {
      // ---- DOS_MODE 1: phan bo deu theo mission ----
      // dos_gpm = SP(g) * speed * 60 / mission_dist
      // offset  = dos_gpm * 50 / (SA_DOS_Fx(mL/50us) x SA_DOS_Dx(g/mL))
      float vol_rate1 = _dos_fr[food_idx].get();
      if (vol_rate1 < 0.1f) {
        vol_rate1 = 0.1f;
      }
      float density1 = _dos_dr[food_idx].get();
      if (density1 < 0.01f) {
        density1 = 0.01f;
      }
      float effective1 = vol_rate1 * density1;  // g/50us
      float mission_dist = _get_mission_dist();
      float speed_ms =
          (_simulation.get() > 0) ? _sim_speed : AP::ahrs().groundspeed();
      if (mission_dist > 1.0f && speed_ms >= 0.05f) {
        float dos_gpm = (_dos_sp.get() * speed_ms * 60.0f) / mission_dist;
        float offset = dos_gpm * 50.0f / effective1;
        if (_dos_rev.get() == 0) {
          pwm_f = constrain_float(1500.0f - offset, 800.0f, 1500.0f);
        } else {
          pwm_f = constrain_float(1500.0f + offset, 1500.0f, 2200.0f);
        }
      } else {
        pwm_f = 1500.0f;
        if (now - _dos_warn_ms >= 5000U) {
          _dos_warn_ms = now;
          if (mission_dist <= 1.0f) {
            gcs().send_text(
                MAV_SEVERITY_WARNING,
                "SA DOS1: chưa có mission (dist=%.1fm) - motor dừng",
                (double)mission_dist);
          } else {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "SA DOS1: tốc độ quá thấp (%.2fm/s) - motor dừng",
                            (double)speed_ms);
          }
        }
      }
    }
    _dos_pwm = (uint16_t)pwm_f;
  } else {
    _dos_pwm = 1500;
  }

  uint8_t chan_idx = (uint8_t)constrain_int16(_dos_chan.get() - 1, 0, 15);
  SRV_Channels::set_output_pwm_chan(chan_idx, _dos_pwm);

  // ---- DOSING CONSOLE LOG (SA_DOS_LOG) ----
  if (_dos_log_enable.get() > 0) {
    if (now - _dos_last_log_ms >= (uint32_t)_dos_log_ms.get()) {
      _dos_last_log_ms = now;
      uint8_t food_log = (uint8_t)constrain_int16(_dos_food.get(), 1, 7) - 1;
      gcs().send_text(MAV_SEVERITY_INFO,
                      "[DOS] M%d F%d SERVO%d %s SP:%.0fg D:%.2fg/mL PWM:%u",
                      (int)_dos_mode.get(), (int)_dos_food.get(),
                      (int)_dos_chan.get(), motor_on ? "ON" : "OFF",
                      (double)_dos_sp.get(),
                      (double)_dos_dr[food_log].get(),
                      (unsigned)_dos_pwm);
    }
  }
}
// [/AP_ShoesAgtech]

// =============================================================
// [AP_ShoesAgtech] pH SENSOR — Nengshi ASPS3801D-0.5M
//
// Protocol: Modbus RTU, 9600 8N1, Function Code 04
// Hardware: Sensor RS485 A/B → RS485-TTL module → TELEM port RX/TX
// Request:  [01][04][00 00][00 09][30 0C]  (9 registers from 0x0000)
// Response: [01][04][12][Reg0..Reg8 × 2B][CRC × 2B] = 23 bytes
//
// Register map (0-indexed in response buffer, data at buf[3]+):
//   0x0000 (buf[3..4])  pH × 100          unsigned
//   0x0002 (buf[7..8])  electrode mV       signed 16-bit
//   0x0008 (buf[19..20]) temperature × 10  signed 16-bit
// =============================================================

// CRC16/Modbus: polynomial 0xA001, init 0xFFFF, result LSB-first in frame
static uint16_t _ph_crc16(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001U) : (crc >> 1);
    }
  }
  return crc;
}

void AP_ShoesAgtech::_ph_init(void) {
  if (!ph_is_enabled()) {
    return;
  }

  uint8_t port_num = (uint8_t)constrain_int16(_ph_port.get(), 0, 4);
  _ph_uart = hal.serial(port_num);

  if (_ph_uart == nullptr) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: pH sensor SERIAL%d not found",
                    (int)port_num);
    return;
  }

  _ph_uart->begin(9600);
  memset(_ph_buf, 0, sizeof(_ph_buf));
  _ph_buf_idx = 0;
  _ph_buf_count = 0;
  _ph_buf_sum = 0.0f;
  _ph_value_ema = -1.0f; // sentinel: not yet initialized
  _ph_req_pending = false;
  _ph_update_ms = 0;
  _ph_last_good_ms = 0;
  _ph_nodata_warn_ms = 0;
  _ph_last_log_ms = 0;

  gcs().send_text(MAV_SEVERITY_INFO,
                  "SA: pH sensor on SERIAL%d (Modbus RTU 9600)", (int)port_num);
}

void AP_ShoesAgtech::_ph_update(void) {
  if (_ph_uart == nullptr) {
    return;
  }

  uint32_t now = AP_HAL::millis();

  // ---- Send new request every 2 seconds ----
  if (!_ph_req_pending) {
    if (now - _ph_update_ms < 2000) {
      return;
    }
    _ph_update_ms = now;

    // Flush stale RX bytes before sending
    uint16_t stale = _ph_uart->available();
    while (stale > 0) {
      _ph_uart->read();
      stale--;
    }

    // Modbus FC04: read 9 registers from 0x0000, CRC = 0x0C30 (LSB first: 30
    // 0C)
    static const uint8_t req[8] = {0x01, 0x04, 0x00, 0x00,
                                   0x00, 0x09, 0x30, 0x0C};
    _ph_uart->write(req, sizeof(req));
    _ph_req_pending = true;
    _ph_req_sent_ms = now;
    return;
  }

  // ---- Wait at least 150ms for sensor to respond ----
  if (now - _ph_req_sent_ms < 150) {
    return;
  }

  uint16_t avail = _ph_uart->available();

  // Timeout: if no full frame within 500ms, give up and warn if sensor absent
  if (avail < 23) {
    if (now - _ph_req_sent_ms > 500) {
      _ph_req_pending = false;
      const uint32_t timeout_ms = (uint32_t)MAX(_ph_timeout.get(), 1) * 1000U;
      bool no_data =
          (_ph_last_good_ms == 0) || (now - _ph_last_good_ms > timeout_ms);
      if (no_data && now - _ph_nodata_warn_ms >= 10000) {
        _ph_nodata_warn_ms = now;
        if (_ph_last_good_ms == 0) {
          gcs().send_text(MAV_SEVERITY_WARNING,
                          "SA: pH sensor chưa có dữ liệu - kiểm tra dây RS485");
        } else {
          gcs().send_text(
              MAV_SEVERITY_WARNING,
              "SA: pH sensor mất kết nối (%.0fs) - kiểm tra dây RS485",
              (double)((now - _ph_last_good_ms) / 1000U));
        }
      }
    }
    return;
  }

  _ph_req_pending = false;

  // ---- Read exactly 23 bytes ----
  uint8_t buf[23];
  for (uint8_t i = 0; i < 23; i++) {
    int16_t b = _ph_uart->read();
    buf[i] = (b >= 0) ? (uint8_t)b : 0;
  }

  // ---- Validate Modbus header and byte count ----
  if (buf[0] != 0x01 || buf[1] != 0x04 || buf[2] != 18) {
    return;
  }

  // ---- CRC verification (covers bytes 0..20, result compared to bytes 21-22)
  // ----
  uint16_t crc_calc = _ph_crc16(buf, 21);
  uint16_t crc_recv = (uint16_t)buf[21] | ((uint16_t)buf[22] << 8);
  if (crc_calc != crc_recv) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: pH CRC fail (noise on RS485?)");
    return;
  }

  // ---- Decode registers ----
  uint16_t raw_ph = ((uint16_t)buf[3] << 8) | buf[4];               // unsigned
  int16_t raw_mv = (int16_t)(((uint16_t)buf[7] << 8) | buf[8]);     // signed
  int16_t raw_temp = (int16_t)(((uint16_t)buf[19] << 8) | buf[20]); // signed

  float ph_cal = constrain_float(raw_ph / 100.0f + _ph_off.get(), 0.0f, 14.0f);
  _ph_mv = raw_mv;
  _ph_temp = raw_temp / 10.0f + _ph_toff.get();
  _ph_value = ph_cal;
  _ph_last_good_ms = now; // frame hợp lệ — reset bộ đếm mất kết nối

  // ---- Moving Average (PH_WINDOW = 10 samples) ----
  _ph_buf_sum -= _ph_buf[_ph_buf_idx];
  _ph_buf[_ph_buf_idx] = ph_cal;
  _ph_buf_sum += ph_cal;
  _ph_buf_idx = (_ph_buf_idx + 1) % PH_WINDOW;
  if (_ph_buf_count < PH_WINDOW) {
    _ph_buf_count++;
  }
  _ph_value_ma = _ph_buf_sum / _ph_buf_count;

  // ---- EMA (Exponential Moving Average) ----
  float alpha = constrain_float(_ph_ema_alpha.get(), 0.01f, 1.0f);
  if (_ph_value_ema < 0.0f) {
    _ph_value_ema = ph_cal; // first sample: seed EMA directly
  } else {
    _ph_value_ema = _ph_value_ema * (1.0f - alpha) + ph_cal * alpha;
  }

  // ---- Daily slot tracking + alkalinity from ΔpH ----
  _ph_update_daily_slots(ph_cal);

  // ---- PH CONSOLE LOG (SA_PH_LOG) — independent of SA_FLOW_LOG ----
  if (_ph_log_enable.get() > 0 &&
      now - _ph_last_log_ms >= (uint32_t)_ph_log_ms.get()) {
    _ph_last_log_ms = now;
    // Status tag for alkalinity data quality
    const char *slot_tag;
    switch (_alk_slot_status) {
    case 0:
      slot_tag = "FULL";
      break; // both morning + afternoon today
    case 1:
      slot_tag = "MORN";
      break; // morning only
    case 2:
      slot_tag = "AFT";
      break; // afternoon only
    case 3:
      slot_tag = "PREV";
      break; // yesterday's data
    default:
      slot_tag = "NODATA";
      break; // no data yet
    }
    const char *ph_pfx = (_simulation.get() > 0) ? "[SIM][WM]" : "[WM]";
    gcs().send_text(MAV_SEVERITY_INFO, "%s pH:%.2f MA:%.2f Tmp:%.1fC mV:%d [%s]",
                    ph_pfx, (double)_ph_value, (double)_ph_value_ma,
                    (double)_ph_temp, (int)_ph_mv, slot_tag);
    if (_alk_slot_status == 0) {
      gcs().send_text(MAV_SEVERITY_INFO, "%s Alk:%.2fdKH %.1fmg/L dPH:%+.2f",
                      ph_pfx, (double)_alk_dkh, (double)_alk_mgl,
                      (double)_delta_ph);
    }
    // In khung gio sang/chieu da quy doi sang HH:MM de xac nhan cai dat
    {
      float ms  = constrain_float(_ph_ms.get(),  0.0f, 23.99f);
      float me  = constrain_float(_ph_me.get(),  0.0f, 24.0f);
      float as_ = constrain_float(_ph_as.get(),  0.0f, 23.99f);
      float ae  = constrain_float(_ph_ae.get(),  0.0f, 24.0f);
      int ms_h = (int)ms,  ms_m = (int)((ms  - (int)ms)  * 60.0f + 0.5f);
      int me_h = (int)me,  me_m = (int)((me  - (int)me)  * 60.0f + 0.5f);
      int as_h = (int)as_, as_m = (int)((as_ - (int)as_) * 60.0f + 0.5f);
      int ae_h = (int)ae,  ae_m = (int)((ae  - (int)ae)  * 60.0f + 0.5f);
      gcs().send_text(MAV_SEVERITY_INFO,
                      "%s Sang:%d:%02d-%d:%02d Chieu:%d:%02d-%d:%02d",
                      ph_pfx, ms_h, ms_m, me_h, me_m, as_h, as_m, ae_h, ae_m);
    }
  }
}

// =============================================================
// [AP_ShoesAgtech] DAILY ΔpH SLOT TRACKING
//
// =============================================================
// consume_alk_log_pending — pop ONE pending pond PHAK write per call.
// Sets output mirror fields (_ph_morn_val, _alk_dkh, etc.) so Log.cpp
// can read getters and write the PHAK record.  Returns false when no
// more ponds are pending.  Called each update cycle by Rover.cpp →
// Log_Write_Ph_Alkalinity(), so multiple ponds flush across successive cycles.
// =============================================================
bool AP_ShoesAgtech::consume_alk_log_pending(void) {
  for (uint8_t i = 0; i < _pond_count; i++) {
    PondEntry &p = _ponds[i];
    if (!p.valid || !p.alk_pending) continue;
    p.alk_pending  = false;
    _alk_pond_idx  = i;
    _ph_morn_val   = p.ph_morn;
    _ph_aft_val    = p.ph_aft;
    _ph_morn_lat   = p.morn_lat;
    _ph_morn_lng   = p.morn_lng;
    _delta_ph      = p.delta_ph;
    _alk_dkh       = p.alk_dkh;
    _alk_mgl       = p.alk_mgl;
    return true;
  }
  return false;
}

// =============================================================
// _ph_update_daily_slots — GPS cluster approach (Hướng 2).
// Nhận diện ao bằng khoảng cách GPS (SA_PH_POND_D), tích lũy mẫu pH
// trung bình trong slot sáng/chiều, tính kiềm khi ao đủ cả hai slot.
// Dữ liệu ao không reset hàng ngày — chỉ xóa slot hôm nay khi quay lại
// ao đó vào ngày mới.  Ring buffer 16 ao.
// =============================================================
void AP_ShoesAgtech::_ph_update_daily_slots(float ph_cal) {
  uint32_t now = AP_HAL::millis();

  // ---- Require GPS time ----
  uint64_t utc_usec = 0;
  if (!AP::rtc().get_utc_usec(utc_usec)) {
    if (_ph_log_enable.get() > 0 && now - _slot_warn_ms >= 60000) {
      _slot_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO, "[WM] Chưa GPS - kiềm đợi GPS/giờ");
    }
    return;
  }

  // ---- Require GPS 3D fix ----
  int32_t cur_lat_i = 0, cur_lng_i = 0;
  const AP_GPS &gps_inst = AP::gps();
  if (gps_inst.status(0) < AP_GPS::GPS_OK_FIX_3D) {
    if (_ph_log_enable.get() > 0 && now - _slot_warn_ms >= 60000) {
      _slot_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_INFO, "[WM] Chưa GPS - kiềm đợi GPS/giờ");
    }
    return;
  }
  {
    const Location &loc = gps_inst.location(0);
    cur_lat_i = loc.lat;
    cur_lng_i = loc.lng;
  }

  int8_t tz = (int8_t)constrain_int16(_ph_tz.get(), -12, 14);
  uint32_t utc_sec   = (uint32_t)(utc_usec / 1000000ULL);
  uint32_t local_sec = utc_sec + (uint32_t)((int32_t)tz * 3600);
  uint32_t today     = local_sec / 86400U;
  float    local_h   = (float)(local_sec % 86400U) / 3600.0f;
  float    cur_lat_f = cur_lat_i * 1.0e-7f;
  float    cur_lng_f = cur_lng_i * 1.0e-7f;

  // ---- Slot time windows ----
  float ms  = constrain_float(_ph_ms.get(),  0.0f, 23.99f);
  float me  = constrain_float(_ph_me.get(),  0.0f, 24.0f);
  float as_ = constrain_float(_ph_as.get(),  0.0f, 23.99f);
  float ae  = constrain_float(_ph_ae.get(),  0.0f, 24.0f);
  const bool in_morn = (local_h >= ms  && local_h <= me);
  const bool in_aft  = (local_h >= as_ && local_h <= ae);

  // ---- Chọn ao theo SA_POND_IDX (nhập thủ công, 1-based) ----
  const int8_t sel = (int8_t)constrain_int16(_pond_select.get(), 1, (int16_t)MAX_PONDS);
  const uint8_t pond_idx = (uint8_t)(sel - 1);  // 0-based internal index

  // Khởi tạo slot nếu lần đầu dùng ao này
  const bool pond_is_new = !_ponds[pond_idx].valid;
  if (pond_is_new) {
    memset(&_ponds[pond_idx], 0, sizeof(PondEntry));
    _ponds[pond_idx].center_lat = cur_lat_f;
    _ponds[pond_idx].center_lng = cur_lng_f;
    _ponds[pond_idx].gps_count  = 1;
    _ponds[pond_idx].valid      = true;
    _ponds[pond_idx].last_day   = today;
    _ponds[pond_idx].dos_sp     = _dos_sp.get();
    if (pond_idx >= _pond_count) _pond_count = pond_idx + 1;
    _ponds_dirty = true;
  }

  PondEntry &pond = _ponds[pond_idx];
  const unsigned disp_idx = (unsigned)pond_idx + 1;  // hiển thị bắt đầu từ 1

  // ---- Thông báo khi ao thay đổi (kể cả lần đầu boot) — in 1 lần ----
  if (pond_idx != _active_pond_idx || !_pond_first_detect_done) {
    _pond_first_detect_done = true;
    gcs().send_text(MAV_SEVERITY_INFO,
                    "[SA] Chuyen sang ao #%u%s",
                    disp_idx, pond_is_new ? " (ao moi)" : "");
    // GPS validate — in 1 lần khi đổi ao, chỉ khi centroid đã ổn định
    if (pond.gps_count > 5) {
      const float DEG2M  = 111320.0f;
      const float coslat = cosf(cur_lat_f * DEG_TO_RAD);
      float dlat_m = (cur_lat_f - pond.center_lat) * DEG2M;
      float dlng_m = (cur_lng_f - pond.center_lng) * DEG2M * coslat;
      float dist_m = sqrtf(dlat_m * dlat_m + dlng_m * dlng_m);
      float pond_thr = constrain_float(_ph_pond_dist.get(), 10.0f, 5000.0f);
      if (dist_m <= pond_thr) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Ao#%u GPS OK (%.0fm)", disp_idx, (double)dist_m);
      } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Ao#%u GPS lech %.0fm - can check lai vi tri ao",
                        disp_idx, (double)dist_m);
      }
    }
  }
  _active_pond_idx = pond_idx;

  // ---- Cập nhật centroid GPS (rolling average, cap 1000 tránh mất độ chính xác float) ----
  if (pond.gps_count < 1000) pond.gps_count++;
  float w            = 1.0f / (float)pond.gps_count;
  pond.center_lat    = pond.center_lat * (1.0f - w) + cur_lat_f * w;
  pond.center_lng    = pond.center_lng * (1.0f - w) + cur_lng_f * w;

  // ---- Ngày mới cho ao này: xóa slot hôm nay, giữ alk từ ngày trước ----
  if (pond.last_day != today) {
    pond.ph_morn       = 0.0f;
    pond.ph_aft        = 0.0f;
    pond.morn_count    = 0;     pond.aft_count    = 0;
    pond.status        = 0;     pond.delta_ph     = 0.0f;
    pond.alk_pending   = false; pond.alk_computed  = false;
    pond.morn_reported = false; pond.aft_reported  = false;
    pond.morn_last_ms  = 0;     pond.aft_last_ms   = 0;
    pond.morn_lat      = 0;     pond.morn_lng      = 0;
    pond.last_day      = today;
    // alk_dkh / alk_mgl được giữ lại — dùng làm tham chiếu "hôm qua" cho ao này
  }

  // ---- Lấy mẫu pH — chỉ khi ARM, rate-limited SA_PH_CAP_S, lưu mẫu cuối cùng ----
  const bool is_armed = hal.util->get_soft_armed();
  if ((in_morn || in_aft) && is_armed) {
    uint32_t cap_ms  = (uint32_t)constrain_int16(_ph_cap_s.get(), 1, 3600) * 1000U;
    uint8_t  cap_min = (uint8_t)constrain_int16(_ph_cap_sam.get(), 1, 100);

    if (in_morn && now - pond.morn_last_ms >= cap_ms) {
      pond.ph_morn      = ph_cal;  // last-write-wins
      pond.morn_lat     = cur_lat_i;
      pond.morn_lng     = cur_lng_i;
      pond.morn_last_ms = now;
      pond.morn_count++;
      pond.status      |= 1;  // bit0 = có buổi sáng
      _ponds_dirty      = true;
      // Báo cáo lần đầu khi đủ ngưỡng mẫu tối thiểu
      if (pond.morn_count == cap_min && !pond.morn_reported) {
        pond.morn_reported = true;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Ao#%u pH sang: %.2f (%u mau)",
                        disp_idx, (double)pond.ph_morn, (unsigned)cap_min);
      }
    } else if (in_aft && now - pond.aft_last_ms >= cap_ms) {
      pond.ph_aft      = ph_cal;  // last-write-wins
      pond.aft_last_ms = now;
      pond.aft_count++;
      pond.status     |= 2;  // bit1 = có buổi chiều
      _ponds_dirty     = true;
      // Báo cáo lần đầu khi đủ ngưỡng mẫu tối thiểu
      if (pond.aft_count == cap_min && !pond.aft_reported) {
        pond.aft_reported = true;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "[SA] Ao#%u pH chieu: %.2f (%u mau)",
                        disp_idx, (double)pond.ph_aft, (unsigned)cap_min);
      }
    }
  }

  // ---- Tính kiềm khi ao đủ cả hai slot (chỉ tính một lần mỗi ngày) ----
  if (pond.status == 3 && !pond.alk_computed) {
    pond.delta_ph     = pond.ph_aft - pond.ph_morn;
    float kh_scaled   = _ph_kh.get() *
                        (1.0f + constrain_float(pond.delta_ph * 0.375f, -0.5f, 1.0f));
    pond.alk_dkh      = _ph_calc_alkalinity(pond.ph_morn, kh_scaled, _ph_temp);
    pond.alk_mgl      = pond.alk_dkh * 17.85f;
    pond.alk_computed = true;
    pond.alk_pending  = true;  // kích hoạt ghi PHAK vào SD
    _ponds_dirty      = true;
    _delta_ph = pond.delta_ph;
    _alk_dkh  = pond.alk_dkh;
    _alk_mgl  = pond.alk_mgl;
    // In tóm tắt đầy đủ: S / C / ΔpH / kiềm
    gcs().send_text(MAV_SEVERITY_INFO,
                    "[SA] Ao#%u S:%.2f C:%.2f dPH:%.2f",
                    disp_idx,
                    (double)pond.ph_morn, (double)pond.ph_aft,
                    (double)pond.delta_ph);
    gcs().send_text(MAV_SEVERITY_INFO,
                    "[SA] Ao#%u kiem:%.1fdKH/%.0fmgL",
                    disp_idx,
                    (double)pond.alk_dkh, (double)pond.alk_mgl);
  }

  // ---- Cập nhật trạng thái hiển thị từ ao đang active ----
  if (pond.status == 3) {
    _alk_slot_status = 0;           // FULL hôm nay
  } else if (pond.status == 1) {
    _alk_slot_status = 1;           // chỉ có sáng
  } else if (pond.status == 2) {
    _alk_slot_status = 2;           // chỉ có chiều
  } else if (pond.alk_dkh > 0.0f) {
    _alk_dkh = pond.alk_dkh;       // dùng dữ liệu ngày trước của ao này
    _alk_mgl = pond.alk_mgl;
    _alk_slot_status = 3;           // PREV
  } else {
    _alk_slot_status = 4;           // chưa có dữ liệu
  }

}

// =============================================================
// Alkalinity estimation algorithm
//
// Estimates relative alkalinity (KH) change based on deviation of
// the measured pH from a reference point (pH ≈ 8.0) and temperature.
//
// Physical basis:
//   - In pond water, CO2/HCO3-/CO3²- equilibrium couples pH and KH.
//   - pH > 8.3: HCO3- dominant, low free CO2 → higher effective KH.
//   - pH < 7.6: high free CO2 consumes KH → lower effective KH.
//   - Higher temperature → less dissolved CO2 → slight pH rise.
//
// This is a heuristic model, NOT a substitute for direct KH test kit
// measurement. Use to observe trends; update SA_PH_KH periodically.
// =============================================================
float AP_ShoesAgtech::_ph_calc_alkalinity(float ph, float base_kh_dkh,
                                          float temp_c) {
  ph = constrain_float(ph, 0.0f, 14.0f);
  temp_c = constrain_float(temp_c, -10.0f, 50.0f);

  // Temperature factor: each 1°C above 28°C reduces dissolved CO2 by ~0.8%
  float tf = constrain_float(1.0f - (temp_c - 28.0f) * 0.008f, 0.85f, 1.10f);

  // pH factor: linear segments around neutral reference pH 8.0
  float pf;
  if (ph >= 8.3f) {
    pf = 1.0f + (ph - 8.3f) * 0.32f;
  } else if (ph <= 7.6f) {
    pf = 1.0f - (7.6f - ph) * 0.45f;
  } else {
    pf = 1.0f + (ph - 8.0f) * 0.15f;
  }

  return base_kh_dkh * constrain_float(pf * tf, 0.55f, 1.75f);
}

// =============================================================
// POND STATE PERSISTENCE — /APM/SA_PONDS.bin
// Lưu toàn bộ _ponds[] vào SD card mỗi khi có thay đổi (rate-limited 5s).
// Đọc lại khi boot để khôi phục dữ liệu pH, kiềm, dos_sp của từng ao.
// Format: magic(4) + version(1) + count(1) + PondEntry[MAX_PONDS]
// =============================================================
#define SA_PONDS_FILE  "/APM/SA_PONDS.bin"
#define SA_PONDS_MAGIC 0x504F4E44UL  // 'POND'
#define SA_PONDS_VER   1

struct PondStateHdr {
  uint32_t magic;
  uint8_t  version;
  uint8_t  count;
};

void AP_ShoesAgtech::_pond_save(void) {
  int fd = AP::FS().open(SA_PONDS_FILE, O_WRONLY | O_CREAT | O_TRUNC);
  if (fd < 0) return;

  PondStateHdr hdr { SA_PONDS_MAGIC, SA_PONDS_VER, _pond_count };
  AP::FS().write(fd, &hdr, sizeof(hdr));
  AP::FS().write(fd, _ponds, sizeof(PondEntry) * MAX_PONDS);
  AP::FS().close(fd);
}

void AP_ShoesAgtech::_pond_load(void) {
  int fd = AP::FS().open(SA_PONDS_FILE, O_RDONLY);
  if (fd < 0) return;

  PondStateHdr hdr {};
  if (AP::FS().read(fd, &hdr, sizeof(hdr)) != (ssize_t)sizeof(hdr) ||
      hdr.magic != SA_PONDS_MAGIC || hdr.version != SA_PONDS_VER) {
    AP::FS().close(fd);
    return;
  }

  AP::FS().read(fd, _ponds, sizeof(PondEntry) * MAX_PONDS);
  AP::FS().close(fd);

  _pond_count = hdr.count;

  // Reset các field không hợp lệ sau reboot
  for (uint8_t i = 0; i < MAX_PONDS; i++) {
    if (!_ponds[i].valid) continue;
    _ponds[i].morn_last_ms = 0;  // timestamp ms không còn hợp lệ
    _ponds[i].aft_last_ms  = 0;
    _ponds[i].alk_pending  = false;  // PHAK đã được log trong session trước
  }

  gcs().send_text(MAV_SEVERITY_INFO,
                  "[SA] Load %u ao tu SD card", (unsigned)_pond_count);
}

// =============================================================
// SIMULATION — SA_SIM = 1
// Generates sinusoidal fake sensor data so modes 1/2 and GCS
// display can be verified without real hardware attached.
// Called from update() instead of _ph_update() when active.
// Flow values are written directly to _flow_rate_filtered and
// _sim_speed; the moving-average buffer is advanced by the
// caller (update()) as usual.
// =============================================================
void AP_ShoesAgtech::_run_simulation(void) {
  uint32_t now = AP_HAL::millis();
  float t = now * 0.001f; // seconds since boot

  // ---- Module 1: flow (2.5 ± 1.5 L/min, 20s period) ----
  _flow_rate_filtered = 2.5f + 1.5f * sinf(2.0f * M_PI * t / 20.0f);

  // ---- Module 1: simulated groundspeed (1.0 ± 0.8 m/s, 30s period) ----
  _sim_speed =
      constrain_float(1.0f + 0.8f * sinf(2.0f * M_PI * t / 30.0f), 0.1f, 2.0f);

  // ---- Module 2: pH sensor values ----
  // pH: 7.3 ± 0.4, 60s period
  float ph_sim = 7.3f + 0.4f * sinf(2.0f * M_PI * t / 60.0f);
  // Temperature: 28.0 ± 2.0°C, 120s period
  float temp_sim = 28.0f + 2.0f * sinf(2.0f * M_PI * t / 120.0f);
  // Electrode mV from Nernst: ~59.16 mV/pH unit relative to pH 7
  int16_t mv_sim = (int16_t)((7.0f - ph_sim) * 59.16f);

  _ph_value     = ph_sim;
  _ph_value_ema = ph_sim;
  _ph_value_ma  = ph_sim;
  _ph_mv        = mv_sim;
  _ph_temp      = temp_sim;

  // Keep ph_has_data() returning true
  _ph_last_good_ms = now;

  // Run full slot + alkalinity pipeline so PHAK SD logging works in SITL.
  // This overwrites _alk_dkh/mgl/delta_ph/_alk_slot_status with real computed values.
  _ph_update_daily_slots(ph_sim);
}

// =============================================================
// [AP_ShoesAgtech] SPRAY SPEED — uu tien: SA_SIM > SA_FLOW_VEL > AHRS
// SA_SIM=1       : dung _sim_speed (sin wave, dung cho test man hinh)
// SA_FLOW_VEL>0  : dung gia tri co dinh (calib FLOW_MODE=1 khi xe dung yen)
// Default (=0)   : dung van toc that tu AP::ahrs().groundspeed()
// =============================================================
float AP_ShoesAgtech::_get_spray_speed(void) {
  if (_simulation.get() > 0) {
    return _sim_speed;
  }
  float vel = _flow_vel.get();
  if (vel > 0.0f) {
    return vel;
  }
  return AP::ahrs().groundspeed();
}
// [/AP_ShoesAgtech]

// =============================================================
// _print_fm1_arm_status — in trang thai FLOW_MODE=1 khi ARM (bat ke FLOW_LOG).
// Goi mot lan moi ARM session khi spray_mode = 1 hoac 2.
// Cong thuc moi: q1 = TANK_VOL * r * speed * 60 / mission_dist
// vi_per_run = TANK_VOL * r  (hang so, khong phu thuoc speed/dist)
// dist_max (thong tin): khoang cach toi da de q1 >= 0.3 tai van toc hien tai
// =============================================================
void AP_ShoesAgtech::_print_fm1_arm_status(float r) {
  r = constrain_float(r, 0.01f, 1.0f);
  float dist = _get_mission_dist();

  // Check 1: co mission khong?
  if (dist <= 1.0f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: chưa có mission - bơm sẽ dừng");
    return;
  }

  // vi_per_run hang so: TANK_VOL * r (khong doi theo speed/dist)
  float vi_per_run = _tank_vol.get() * r;

  // Check 2: van toc
  float speed = _get_spray_speed();
  if (speed <= 0.1f) {
    gcs().send_text(MAV_SEVERITY_INFO,
                    "SA FM1 SẴN SÀNG: r=%.2f miss=%.0fm vi/run=%.1fL | vận "
                    "tốc=0 bơm chờ xe chạy",
                    (double)r, (double)dist, (double)vi_per_run);
    return;
  }

  // Tinh q1 va dist_max theo cong thuc moi
  float q1 = _tank_vol.get() * r * speed * 60.0f / dist;
  // dist_max: khoang cach toi da de q1 >= 0.3 tai van toc hien tai
  float dist_max = _tank_vol.get() * r * speed * 60.0f / 0.3f;

  if (q1 < 0.3f) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: q1=%.2fL/ph < 0.3 @%.1fm/s dist=%.0fm - rút ngắn "
                    "mission (dmax=%.0fm)",
                    (double)q1, (double)speed, (double)dist, (double)dist_max);
    return;
  }
  if (q1 > 2.0f) {
    // dist_min: khoang cach toi thieu de q1 <= 2.0
    float dist_min = _tank_vol.get() * r * speed * 60.0f / 2.0f;
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA FM1: q1=%.2fL/ph > 2.0 @%.1fm/s dist=%.0fm - kéo dài "
                    "mission (dmin=%.0fm)",
                    (double)q1, (double)speed, (double)dist, (double)dist_min);
    return;
  }

  uint32_t eta_s = (uint32_t)(dist / speed);
  uint32_t eta_min = eta_s / 60U;
  uint32_t eta_sec = eta_s % 60U;

  gcs().send_text(MAV_SEVERITY_INFO,
                  "SA FM1 OK: r=%.2f q1=%.2fL/ph miss=%.0fm dmax=%.0fm "
                  "~%um%02us vi/run=%.1fL",
                  (double)r, (double)q1, (double)dist, (double)dist_max,
                  (unsigned)eta_min, (unsigned)eta_sec, (double)vi_per_run);
}

// =============================================================
// [AP_ShoesAgtech] VISIN TARGET — FLOW_MODE=1 (nac giua/cao)
//
// Cong thuc moi: q1 = TANK_VOL * r * speed * 60 / mission_dist
//   → vi sinh duoc phan phoi deu tren toan tuyen, khong dung APP_RATE/BOOM_W.
//   → vi_per_run = TANK_VOL * r  (hang so, khong doi theo speed/dist)
// Kiem tra truoc khi chay PID:
//   1. Mission: dist <= 1m → warning + stop
//   2. Speed:   < 0.1 m/s → reset PID, stop (khong canh bao)
//   3. Range:   q1 < 0.3 → mission qua dai hoac speed qua cham → warning + stop
//              q1 > 2.0 → mission qua ngan hoac speed qua nhanh → warning +
//              stop
// Canh bao throttle chung qua _tank_warn_ms (5s min giua hai lan).
// =============================================================
float AP_ShoesAgtech::_compute_visin_target(float r) {
  uint32_t now = AP_HAL::millis();
  r = constrain_float(r, 0.01f, 1.0f);

  float speed_ms = _get_spray_speed();
  float dist = _get_mission_dist();

  // Kiem tra 1: mission da upload?
  if (dist <= 1.0f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA FM1: chưa có mission - bơm dừng");
    }
    return 0.0f;
  }

  // Kiem tra 2: toc do
  if (speed_ms < 0.1f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    return 0.0f;
  }

  // Tinh q1_target: phan phoi TANK_VOL*r deu tren mission_dist
  float q1 = _tank_vol.get() * r * speed_ms * 60.0f / dist;

  // Kiem tra 3: dai cam bien YF-S402B (0.3–2.0 L/min)
  if (q1 < 0.3f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(
          MAV_SEVERITY_WARNING,
          "SA FM1: q1=%.2fL/ph < 0.3 - rút ngắn mission hoặc tăng speed",
          (double)q1);
    }
    return 0.0f;
  }
  if (q1 > 2.0f) {
    _pid_integral = 0.0f;
    _pid_output_lpf = 0.0f;
    if (now - _tank_warn_ms >= 5000U) {
      _tank_warn_ms = now;
      gcs().send_text(
          MAV_SEVERITY_WARNING,
          "SA FM1: q1=%.2fL/ph > 2.0 - kéo dài mission hoặc giảm speed",
          (double)q1);
    }
    return 0.0f;
  }

  return constrain_float(q1, 0.0f, 200.0f);
}
// [/AP_ShoesAgtech]

// =============================================================
// MISSION DISTANCE — SA_FLOW_MODE = 1 (mode 1) + mode 2 monitor
// Iterates AP_Mission waypoints and sums leg distances.
// Result is cached until num_commands() changes (mission edited
// or re-uploaded). Returns 0 if no mission is loaded.
// =============================================================
float AP_ShoesAgtech::_get_mission_dist(void) {
  AP_Mission *mission = AP::mission();
  if (mission == nullptr) {
    return 0.0f;
  }

  uint16_t n = mission->num_commands();
  if (n < 2) {
    return 0.0f;
  }

  // Return cached value if mission hasn't changed
  if (n == _mission_ncmds && _mission_dist_m > 0.0f) {
    return _mission_dist_m;
  }

  float total = 0.0f;
  Location prev_loc;
  bool have_prev = false;

  for (uint16_t i = 0; i < n; i++) {
    AP_Mission::Mission_Command cmd;
    if (!mission->read_cmd_from_storage(i, cmd)) {
      continue;
    }
    // Only nav commands carry a meaningful location
    if (cmd.id != MAV_CMD_NAV_WAYPOINT && cmd.id != MAV_CMD_NAV_LOITER_UNLIM &&
        cmd.id != MAV_CMD_NAV_LOITER_TURNS &&
        cmd.id != MAV_CMD_NAV_LOITER_TIME &&
        cmd.id != MAV_CMD_NAV_SPLINE_WAYPOINT) {
      continue;
    }
    Location loc = cmd.content.location;
    if (loc.lat == 0 && loc.lng == 0) {
      continue;
    }
    if (have_prev) {
      total += prev_loc.get_distance(loc);
    }
    prev_loc = loc;
    have_prev = true;
  }

  _mission_dist_m = total;
  _mission_ncmds = n;
  return _mission_dist_m;
}
// [/AP_ShoesAgtech]
