#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_ShoesAgtech {
public:
  AP_ShoesAgtech();

  AP_ShoesAgtech(const AP_ShoesAgtech &) = delete;
  AP_ShoesAgtech &operator=(const AP_ShoesAgtech &) = delete;

  void init(void);
  void update(void);

  // Flow sensor getters
  float    get_flow_rate_lmin(void) const { return _flow_rate_filtered; }
  float    get_flow_rate_avg(void)  const { return _flow_rate_avg; }
  bool     is_enabled(void)         const { return _enable_flag.get() > 0; }

  // Spray control getters (for logging)
  uint8_t  get_spray_mode(void)     const { return _spray_mode; }
  uint16_t get_pump_pwm(void)       const { return _pump_pwm; }
  float    get_flow_target(void)    const { return _flow_target; }

  // [AP_ShoesAgtech] pH sensor getters — Nengshi ASPS3801D-0.5M via Modbus RTU
  float    get_ph(void)              const { return _ph_value_ma; }
  float    get_ph_raw(void)          const { return _ph_value; }
  float    get_ph_temp(void)         const { return _ph_temp; }
  float    get_ph_mv(void)           const { return (float)_ph_mv; }
  float    get_alk_dkh(void)         const { return _alk_dkh; }
  float    get_alk_mgl(void)         const { return _alk_mgl; }
  float    get_delta_ph(void)        const { return _delta_ph; }
  // slot_status: 0=FULL 1=MORN 2=AFT 3=PREV(yesterday) 4=NODATA
  uint8_t  get_alk_slot_status(void) const { return _alk_slot_status; }
  bool     alk_is_yesterday(void)    const { return _alk_slot_status == 3; }
  bool     ph_is_enabled(void)       const { return _ph_en.get() > 0; }
  // [/AP_ShoesAgtech]

  static const AP_Param::GroupInfo var_info[];
  static void irq_handler(void);

private:
  // ---- Parameters: flow sensor + spray controller (slots 1-13) ----
  AP_Int8  _enable_flag;    // SA_ENABLE
  AP_Float _cal_factor;     // SA_CAL_FAC   pulses/Litre
  AP_Float _ema_alpha;      // SA_EMA_AL    EMA smoothing
  AP_Int8  _flow_log_enable; // SA_FLOW_LOG  console print for flow sensor
  AP_Int8  _rc_chan;         // SA_RC_CHAN   RC channel for mode switch (1-indexed)
  AP_Int8  _rc_pump;        // SA_RC_PUMP  RC channel passed to pump in mode 0 (1-indexed)
  AP_Int8  _pump_chan;       // SA_PUMP_CHAN servo output channel (1-indexed, e.g. 8)
  AP_Float _flow_setpoint;  // SA_FLOW_SP   target L/min (mode 1)
  AP_Float _pid_p;          // SA_PID_P     P gain (us per L/min)
  AP_Float _pid_i;          // SA_PID_I     I gain (us per L/min/s)
  AP_Float _pid_lpf;        // SA_PID_LPF   output LPF alpha (0.01-1.0)
  AP_Float _app_rate;       // SA_APP_RATE  L/ha (mode 2)
  AP_Float _boom_width;     // SA_BOOM_W    boom width in metres (mode 2)

  // [AP_ShoesAgtech] Parameters: pH sensor — Nengshi ASPS3801D-0.5M (slots 14-21)
  AP_Int8  _ph_en;          // SA_PH_EN     enable pH sensor
  AP_Int8  _ph_port;        // SA_PH_PORT   UART port number (matches SERIALx)
  AP_Float _ph_toff;        // SA_PH_TOFF   temperature offset °C
  AP_Float _ph_off;         // SA_PH_OFF    pH calibration offset
  AP_Float _ph_kh;          // SA_PH_KH     base alkalinity dKH (from test kit)
  AP_Float _ph_ema_alpha;   // SA_PH_EMA    EMA smoothing alpha for pH
  AP_Int8  _ph_log_enable;  // SA_PH_LOG    console print for pH sensor (independent of SA_FLOW_LOG)
  AP_Int8  _ph_tz;          // SA_PH_TZ     UTC offset hours (Vietnam = 7)
  // [/AP_ShoesAgtech]

  // ---- Flow sensor state ----
  uint32_t _last_timestamp_ms;
  uint32_t _last_pulse_snapshot;
  uint32_t _last_log_ms;
  float    _flow_rate_filtered;
  float    _flow_rate_avg;
  bool     _is_initialized;

  static const uint8_t WINDOW_SIZE = 10;
  float    _sample_buffer[WINDOW_SIZE];
  uint8_t  _buffer_index;
  float    _buffer_sum;
  uint8_t  _samples_count;

  static volatile uint32_t _pulse_count;

  // ---- Spray control state ----
  uint8_t  _spray_mode;         // 0=PASSTHROUGH 1=FLOW_PID 2=AUTO_RATE
  uint16_t _pump_pwm;
  float    _flow_target;
  float    _pid_integral;
  float    _pid_output_lpf;
  uint32_t _pid_last_ms;

  // ---- Pump config check state ----
  int8_t   _last_pump_chan;      // last PUMP_CHAN value we checked
  int32_t  _last_pump_func_val;  // last servo function value we checked
  bool     _pump_config_ok;      // true when SERVOx_FUNCTION == 0 (None)
  uint32_t _last_warn_ms;        // last time we printed the warning

  // [AP_ShoesAgtech] pH sensor state — Nengshi ASPS3801D-0.5M Modbus RTU
  AP_HAL::UARTDriver *_ph_uart;      // UART driver for RS485→TTL module
  uint32_t  _ph_update_ms;           // last time a request was initiated
  uint32_t  _ph_req_sent_ms;         // timestamp of last Modbus TX
  bool      _ph_req_pending;         // waiting for response
  float     _ph_value;               // latest decoded pH (calibrated)
  float     _ph_value_ema;           // EMA-filtered pH (-1 = not yet init)
  float     _ph_value_ma;            // moving average pH
  int16_t   _ph_mv;                  // electrode voltage in mV (signed)
  float     _ph_temp;                // temperature in °C
  float     _alk_dkh;                // current best alkalinity dKH (today or yesterday)
  float     _alk_mgl;                // current best alkalinity mg/L CaCO3

  static const uint8_t PH_WINDOW = 10;
  float     _ph_buf[PH_WINDOW];      // circular buffer for pH MA
  uint8_t   _ph_buf_idx;
  uint8_t   _ph_buf_count;
  float     _ph_buf_sum;

  // ---- Daily slot tracking (ΔpH sáng–chiều) ----
  // Morning slot: 05:00–11:59 local time
  // Afternoon slot: 12:00–16:59 local time
  // Reset at 00:00, yesterday's alkalinity kept as fallback
  float     _ph_morn_val;            // morning slot pH
  float     _ph_aft_val;             // afternoon slot pH
  bool      _ph_morn_valid;          // morning slot captured today
  bool      _ph_aft_valid;           // afternoon slot captured today
  float     _delta_ph;               // ΔpH = afternoon − morning (0 if incomplete)
  float     _alk_today_dkh;          // today's calculated alkalinity
  float     _alk_today_mgl;
  float     _alk_prev_dkh;           // yesterday's alkalinity (kept at midnight)
  float     _alk_prev_mgl;

  // 0=both slots today  1=morning only  2=afternoon only
  // 3=yesterday's data  4=no data yet
  uint8_t   _alk_slot_status;

  uint32_t  _rtc_last_day;           // UTC day counter for midnight rollover
  uint32_t  _slot_warn_ms;           // last time slot warning was printed
  // [/AP_ShoesAgtech]

  // Private methods — flow/spray
  void     _update_spray_mode(void);
  void     _check_pump_config(void);
  uint16_t _run_flow_pid(float target_lmin, float dt);
  void     _write_pump_pwm(uint16_t pwm);

  // [AP_ShoesAgtech] Private methods — pH sensor
  void     _ph_init(void);
  void     _ph_update(void);
  void     _ph_update_daily_slots(float ph_cal);
  float    _ph_calc_alkalinity(float ph, float base_kh_dkh, float temp_c);
  // [/AP_ShoesAgtech]
};
