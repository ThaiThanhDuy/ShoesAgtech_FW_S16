#include "Rover.h"
// Update V1 - Giữ tốc độ đi thẳng và cua theo joystick
// void ModeAcro::update() {
//   // Static variables
//   static int8_t last_rail_enable = -1;
//   static float last_rail_speed = -1.0f;
//   static float last_rail_ramped_rate = -1.0f;
//   static float ramped_speed = 0.0f;

//   // === THEO DÕI TRẠNG THÁI ARMING ===
//   static bool last_armed = false;
//   bool currently_armed = rover.arming.is_armed(); // ← Sửa đúng ở đây
//   bool just_armed = (currently_armed && !last_armed);

//   // Phát hiện mode re-entry hoặc gap thời gian lớn
//   static uint32_t last_rail_update_ms = 0;
//   uint32_t now_ms = AP_HAL::millis();
//   bool new_rail_session =
//       (now_ms - last_rail_update_ms > 300); // tăng nhẹ lên 300ms
//   last_rail_update_ms = now_ms;

//   // Thu thập parameters
//   const int8_t current_rail_enable = (int8_t)g.rail_enable.get();
//   const float current_rail_speed = g.rail_speed.get();
//   const float current_rail_ramped_rate = g.rail_ramped_rate.get();

//   // --- STATE TRANSITION & INITIALIZATION ---
//   bool state_changed =
//       (current_rail_enable != last_rail_enable) ||
//       (!is_equal(current_rail_speed, last_rail_speed)) ||
//       (!is_equal(current_rail_ramped_rate, last_rail_ramped_rate));

//   if (state_changed) {
//     if (current_rail_enable == 1) {
//       if (last_rail_enable != 1) {
//         ramped_speed = 0.0f;
//       }
//       gcs().send_text(
//           MAV_SEVERITY_INFO, "Rail: ACTIVE | Target: %.2f m/s | Accel: %.3f
//           ", (double)current_rail_speed, (double)current_rail_ramped_rate);
//     } else {
//       if (last_rail_enable == 1) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Rail: DEACTIVATE");
//       }
//       ramped_speed = 0.0f;
//     }
//     last_rail_enable = current_rail_enable;
//     last_rail_speed = current_rail_speed;
//     last_rail_ramped_rate = current_rail_ramped_rate;
//   }

//   // === RESET: Xử lý khi vừa Arm lại hoặc mode re-entry ===
//   if (current_rail_enable == 1) {
//     if (just_armed || new_rail_session) {
//       float actual_speed = 0.0f;
//       if (attitude_control.get_forward_speed(actual_speed)) {
//         ramped_speed = actual_speed; // Reset về tốc độ thực tế
//       } else {
//         ramped_speed = 0.0f;
//       }
//     }
//   }

//   // Cập nhật last_armed
//   last_armed = currently_armed;
//   // --- LOGIC EXECUTION ---
//   if (current_rail_enable == 1) {
//     const float target_v = current_rail_speed;
//     const float accel_step = current_rail_ramped_rate * rover.G_Dt;

//     // Ramp logic
//     if (ramped_speed < target_v) {
//       ramped_speed += accel_step;
//       if (ramped_speed > target_v)
//         ramped_speed = target_v;
//     } else if (ramped_speed > target_v) {
//       ramped_speed -= accel_step;
//       if (ramped_speed < target_v)
//         ramped_speed = target_v;
//     }

//     float actual_speed;
//     if (attitude_control.get_forward_speed(actual_speed)) {
//       calc_throttle(ramped_speed, false);
//     } else {
//       g2.motors.set_throttle(g.rail_percent * 0.01f);
//     }

//     // --- STEERING LOGIC ---
//     float pilot_steering, pilot_throttle;
//     get_pilot_desired_steering_and_throttle(pilot_steering, pilot_throttle);
//     float steering_out = 0.0f;
//     float steer_deadzone =
//         constrain_float((float)g.rail_steer_dz, 0.0f, 500.0f);

//     // Có tác động vào cần ga steering
//     if (fabsf(pilot_steering) > steer_deadzone) {
//       const float target_turn_rate =
//           (pilot_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       steering_out = attitude_control.get_steering_out_rate(
//           target_turn_rate, g2.motors.limit.steer_left,
//           g2.motors.limit.steer_right, rover.G_Dt);
//     } else {
//       // Yêu cầu bộ điều khiển đưa vận tốc xoay về 0 (Active Braking)
//       steering_out = attitude_control.get_steering_out_rate(
//           0.0f, g2.motors.limit.steer_left, g2.motors.limit.steer_right,
//           rover.G_Dt);
//     }
//     // Giới hạn đầu ra trước khi set để bảo vệ ESC Hobbywing X6/X8
//     steering_out = constrain_float(steering_out, -1.0f, 1.0f);
//     // Hàm output duy nhất ở cuối logic
//     set_steering(steering_out * 4500.0f);

//   } else {
//     // --- DEFAULT ARDUPILOT ACRO ---
//     float speed, desired_steering;
//     if (!attitude_control.get_forward_speed(speed)) {
//       float desired_throttle;
//       get_pilot_desired_steering_and_throttle(desired_steering,
//                                               desired_throttle);
//       if (rover.is_balancebot())
//         rover.balancebot_pitch_control(desired_throttle);
//       g2.motors.set_throttle(desired_throttle);
//     } else {
//       float desired_speed;
//       get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
//       calc_throttle(desired_speed, true);
//     }

//     float steering_out;
//     if (!is_zero(desired_steering))
//       g2.sailboat.clear_tack();

//     if (g2.sailboat.tacking()) {
//       steering_out = attitude_control.get_steering_out_heading(
//           g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
//           g2.motors.limit.steer_left, g2.motors.limit.steer_right,
//           rover.G_Dt);
//     } else {
//       const float target_turn_rate =
//           (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       steering_out = attitude_control.get_steering_out_rate(
//           target_turn_rate, g2.motors.limit.steer_left,
//           g2.motors.limit.steer_right, rover.G_Dt);
//     }
//     set_steering(steering_out * 4500.0f);
//   }
// }

// UPDATE V2 - Hỗ trợ cua và cua tự động theo góc đã set
// void ModeAcro::update() {
//   // Static variables
//   static int8_t last_rail_enable = -1;
//   static float last_rail_speed = -1.0f;
//   static float last_rail_ramped_rate = -1.0f;
//   static float ramped_speed = 0.0f;
//   float actual_speed = 0.0f;
//   // === THEO DÕI TRẠNG THÁI ARMING ===
//   static bool last_armed = false;
//   bool currently_armed = rover.arming.is_armed(); // ← Sửa đúng ở đây
//   bool just_armed = (currently_armed && !last_armed);

//   // Phát hiện mode re-entry hoặc gap thời gian lớn
//   static uint32_t last_rail_update_ms = 0;
//   uint32_t now_ms = AP_HAL::millis();
//   bool new_rail_session =
//       (now_ms - last_rail_update_ms > 300); // tăng nhẹ lên 300ms
//   last_rail_update_ms = now_ms;

//   // Thu thập parameters
//   const int8_t current_rail_enable = (int8_t)g.rail_enable.get();
//   const float current_rail_speed = g.rail_speed.get();
//   const float current_rail_ramped_rate = g.rail_ramped_rate.get();

//   // --- STATE TRANSITION & INITIALIZATION ---
//   bool state_changed =
//       (current_rail_enable != last_rail_enable) ||
//       (!is_equal(current_rail_speed, last_rail_speed)) ||
//       (!is_equal(current_rail_ramped_rate, last_rail_ramped_rate));

//   if (state_changed) {
//     if (current_rail_enable == 1) {
//       if (last_rail_enable != 1) {
//         ramped_speed = 0.0f;
//       }
//       gcs().send_text(
//           MAV_SEVERITY_INFO, "Rail: ACTIVE | Target: %.2f m/s | Accel: %.3f
//           ", (double)current_rail_speed, (double)current_rail_ramped_rate);
//     } else {
//       if (last_rail_enable == 1) {
//         gcs().send_text(MAV_SEVERITY_INFO, "Rail: DEACTIVATE");
//       }
//       ramped_speed = 0.0f;
//     }
//     last_rail_enable = current_rail_enable;
//     last_rail_speed = current_rail_speed;
//     last_rail_ramped_rate = current_rail_ramped_rate;
//   }

//   // === RESET: Xử lý khi vừa Arm lại hoặc mode re-entry ===
//   if (current_rail_enable == 1) {
//     if (just_armed || new_rail_session) {
//       if (attitude_control.get_forward_speed(actual_speed)) {
//         ramped_speed = actual_speed; // Reset về tốc độ thực tế
//       } else {
//         ramped_speed = 0.0f;
//       }
//     }
//   }

//   // Cập nhật last_armed
//   last_armed = currently_armed;
//   // --- LOGIC EXECUTION ---
//   if (current_rail_enable == 1) {
//     // const float target_v = current_rail_speed;
//     // const float accel_step = current_rail_ramped_rate * rover.G_Dt;
//     // float actual_speed;

//     // // Ramp logic
//     // if (ramped_speed < target_v) {
//     //   ramped_speed += accel_step;
//     //   if (ramped_speed > target_v)
//     //     ramped_speed = target_v;
//     // } else if (ramped_speed > target_v) {
//     //   ramped_speed -= accel_step;
//     //   if (ramped_speed < target_v)
//     //     ramped_speed = target_v;
//     // }
//     if (attitude_control.get_forward_speed(actual_speed)) {
//       const float target_v = current_rail_speed;
//       const float accel_step = current_rail_ramped_rate * rover.G_Dt;

//       // GIỚI HẠN SAI SỐ: Không cho ramped_speed vượt quá actual_speed một
//       // khoảng Delta Ví dụ: chỉ cho phép dẫn trước tối đa 0.5 m/s
//       float max_lead_speed = actual_speed + 0.5f;
//       float min_lead_speed = actual_speed - 0.5f;

//       // Ramp logic có kiểm soát thực tế
//       if (ramped_speed < target_v) {
//         ramped_speed += accel_step;
//         if (ramped_speed > target_v)
//           ramped_speed = target_v;

//         // Cản ramped_speed lại nếu máy không theo kịp
//         if (ramped_speed > max_lead_speed)
//           ramped_speed = max_lead_speed;

//       } else if (ramped_speed > target_v) {
//         ramped_speed -= accel_step;
//         if (ramped_speed < target_v)
//           ramped_speed = target_v;

//         // Tương tự cho việc giảm tốc (Braking)
//         if (ramped_speed < min_lead_speed)
//           ramped_speed = min_lead_speed;
//       }
//     }
//     if (attitude_control.get_forward_speed(actual_speed)) {
//       calc_throttle(ramped_speed, false);
//     } else {
//       g2.motors.set_throttle(g.rail_percent * 0.01f);
//     }
//     float pilot_steering, pilot_throttle;
//     get_pilot_desired_steering_and_throttle(pilot_steering, pilot_throttle);

//     const float steer_dz = (float)g.rail_steer_dz.get();
//     const float dt = rover.G_Dt;
//     float target_turn_rate_rad = 0.0f;

//     // --- STAGE 2: STEERING PROCESS ---
//     if (g.rail_auto_steer.get() == 1) {
//       // CHẾ ĐỘ RAIL AUTO STEER (Flick-to-Steer)
//       if (pilot_steering > steer_dz) {
//         // Gạt phải -> Cua phải với tốc độ cố định
//         target_turn_rate_rad = radians(fabsf(g.rail_auto_turn_rate.get()));
//       } else if (pilot_steering < -steer_dz) {
//         // Gạt trái -> Cua trái với tốc độ cố định
//         target_turn_rate_rad = -radians(fabsf(g.rail_auto_turn_rate.get()));
//       } else {
//         // Thả cần về giữa -> Active Braking (Giữ thẳng tuyệt đối)
//         target_turn_rate_rad = 0.0f;
//       }
//     } else {
//       // CHẾ ĐỘ ACRO MẶC ĐỊNH (Tỷ lệ thuận theo Stick)
//       if (fabsf(pilot_steering) > steer_dz) {
//         target_turn_rate_rad =
//             (pilot_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       } else {
//         target_turn_rate_rad = 0.0f;
//       }
//     }

//     // Tính toán đầu ra thông qua bộ điều khiển Rate PID
//     float steering_out = attitude_control.get_steering_out_rate(
//         target_turn_rate_rad, g2.motors.limit.steer_left,
//         g2.motors.limit.steer_right, dt);

//     // Giới hạn đầu ra để bảo vệ ESC Hobbywing X6/X8
//     steering_out = constrain_float(steering_out, -1.0f, 1.0f);
//     set_steering(steering_out * 4500.0f);
//   } else {
//     // --- DEFAULT ARDUPILOT ACRO ---
//     float speed, desired_steering;
//     if (!attitude_control.get_forward_speed(speed)) {
//       float desired_throttle;
//       get_pilot_desired_steering_and_throttle(desired_steering,
//                                               desired_throttle);
//       if (rover.is_balancebot())
//         rover.balancebot_pitch_control(desired_throttle);
//       g2.motors.set_throttle(desired_throttle);
//     } else {
//       float desired_speed;
//       get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
//       calc_throttle(desired_speed, true);
//     }

//     float steering_out;
//     if (!is_zero(desired_steering))
//       g2.sailboat.clear_tack();

//     if (g2.sailboat.tacking()) {
//       steering_out = attitude_control.get_steering_out_heading(
//           g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
//           g2.motors.limit.steer_left, g2.motors.limit.steer_right,
//           rover.G_Dt);
//     } else {
//       const float target_turn_rate =
//           (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       steering_out = attitude_control.get_steering_out_rate(
//           target_turn_rate, g2.motors.limit.steer_left,
//           g2.motors.limit.steer_right, rover.G_Dt);
//     }
//     set_steering(steering_out * 4500.0f);
//   }
// }

// UPDATE 3 - safe góc pitch và giới hạn vọt lố speed setpoint (ramprate)
// void ModeAcro::update() {
//   // === 1. BIẾN STATIC VÀ KHỞI TẠO ===
//   static int8_t last_rail_enable = -1;
//   static float last_rail_speed = -1.0f;
//   static float last_rail_ramped_rate = -1.0f;
//   static float ramped_speed = 0.0f;
//   static bool last_armed = false;
//   static uint32_t last_rail_update_ms = 0;
//   static uint32_t last_log_ms = 0;
//   float actual_speed = 0.0f;
//   const uint32_t now_ms = AP_HAL::millis();
//   const float dt = rover.G_Dt;
//   float accel_step = 0.0f;
//   // Theo dõi trạng thái hệ thống
//   const bool currently_armed = rover.arming.is_armed();
//   const bool just_armed = (currently_armed && !last_armed);
//   const bool new_rail_session = (now_ms - last_rail_update_ms > 300);
//   last_rail_update_ms = now_ms;

//   // Biến cờ để kiểm soát hiển thị cảnh báo Pitch
//   static bool pitch_warning_sent = false;
//   // Thu thập parameters
//   const int8_t current_rail_enable = (int8_t)g.rail_enable.get();
//   const float current_rail_speed = g.rail_speed.get();
//   const float current_rail_ramped_rate = g.rail_ramped_rate.get();

//   // --- QUẢN LÝ TRẠNG THÁI CHUYỂN ĐỔI ---
//   bool state_changed =
//       (current_rail_enable != last_rail_enable) ||
//       (!is_equal(current_rail_speed, last_rail_speed)) ||
//       (!is_equal(current_rail_ramped_rate, last_rail_ramped_rate));

//   if (state_changed) {
//     if (current_rail_enable == 1) {
//       if (last_rail_enable != 1)
//         ramped_speed = 0.0f;
//       gcs().send_text(MAV_SEVERITY_INFO, "Rail: ACTIVE | Target: %.2f m/s",
//                       (double)current_rail_speed);
//     } else {
//       if (last_rail_enable == 1)
//         gcs().send_text(MAV_SEVERITY_INFO, "Rail: DEACTIVATE");
//       ramped_speed = 0.0f;
//     }
//     last_rail_enable = current_rail_enable;
//     last_rail_speed = current_rail_speed;
//     last_rail_ramped_rate = current_rail_ramped_rate;
//   }

//   // Lấy tốc độ thực tế từ EKF
//   const bool speed_available =
//   attitude_control.get_forward_speed(actual_speed);

//   // Reset logic khi mới Arm hoặc vào lại session
//   if (current_rail_enable == 1 && (just_armed || new_rail_session)) {
//     ramped_speed = speed_available ? actual_speed : 0.0f;
//   }
//   last_armed = currently_armed;

//   // === 2. LOGIC ĐIỀU KHIỂN CHÍNH (RAIL MODE) ===
//   if (current_rail_enable == 1) {
//     float target_v = current_rail_speed;

//     // --- STAGE 2.1: PITCH SAFETY (UP/DOWN) ---
//     if (g.rail_safe_pitch_en.get() == 1) {
//       const float current_pitch_deg = degrees(rover.ahrs.get_pitch());
//       const float limit_dn = -fabsf((float)g.rail_safe_pitch_down.get());
//       const float limit_up = fabsf((float)g.rail_safe_pitch_up.get());
//       const float percent_scale_safety = 0.70f; // 70%
//       if (current_pitch_deg < limit_dn || current_pitch_deg > limit_up) {
//         target_v = current_rail_speed * percent_scale_safety;

//         // Chỉ gửi tin nhắn nếu chưa gửi trước đó (One-time alert)
//         if (!pitch_warning_sent) {
//           gcs().send_text(MAV_SEVERITY_WARNING,
//                           "Rail: Pitch Safety ACTIVE (70%% Speed)");
//           pitch_warning_sent = true;
//         }
//       } else {
//         // Khi quay lại góc an toàn, reset cờ và thông báo (tùy chọn)
//         if (pitch_warning_sent) {
//           gcs().send_text(MAV_SEVERITY_INFO,
//                           "Rail: Pitch Safe - Resuming speed");
//           pitch_warning_sent = false;
//         }
//       }
//     }

//     // --- STAGE 2.2: ADAPTIVE RAMP LOGIC ---
//     if (speed_available) {
//       accel_step = current_rail_ramped_rate * dt;
//       // Giới hạn ramped_speed dẫn trước thực tế tối đa (m/s) để chống
//       I-Windup const float spd_lead = MAX(g.rail_speed_lead.get(), 0.1f);

//       const float max_lead = actual_speed + spd_lead;
//       const float min_lead = actual_speed - spd_lead;

//       if (ramped_speed < target_v) {
//         ramped_speed += accel_step;
//         if (ramped_speed > target_v)
//           ramped_speed = target_v;
//         if (ramped_speed > max_lead)
//           ramped_speed = max_lead; // Kìm hãm ramp nếu máy chưa bám kịp
//       } else if (ramped_speed > target_v) {
//         ramped_speed -= accel_step;
//         if (ramped_speed > target_v)
//           ramped_speed = target_v;
//         if (ramped_speed < min_lead)
//           ramped_speed = min_lead;
//       }
//       calc_throttle(ramped_speed, false);
//     } else {
//       // Trường hợp mất phản hồi tốc độ: Sử dụng giá trị phần trăm ga cố định
//       g2.motors.set_throttle(g.rail_percent * 0.01f);
//     }
//     // --- STAGE 2.3: STEERING PROCESS ---
//     float pilot_steering, pilot_throttle;
//     get_pilot_desired_steering_and_throttle(pilot_steering, pilot_throttle);
//     const float steer_dz = (float)g.rail_steer_dz.get();
//     float target_turn_rate_rad = 0.0f;

//     if (g.rail_auto_steer.get() == 1) {
//       // Thuật toán auto cua
//       if (pilot_steering > steer_dz) {
//         target_turn_rate_rad = radians(fabsf(g.rail_auto_turn_rate.get()));
//       } else if (pilot_steering < -steer_dz) {
//         target_turn_rate_rad = -radians(fabsf(g.rail_auto_turn_rate.get()));
//       }
//     } else {
//       if (fabsf(pilot_steering) > steer_dz) {
//         target_turn_rate_rad =
//             (pilot_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       }
//     }

//     float steering_out = attitude_control.get_steering_out_rate(
//         target_turn_rate_rad, g2.motors.limit.steer_left,
//         g2.motors.limit.steer_right, dt);

//     set_steering(constrain_float(steering_out, -1.0f, 1.0f) * 4500.0f);
//     if (g.rail_log_enable.get() == 1) {
//       if (now_ms - last_log_ms > 2000) {
//         // Log gộp các thông số quan trọng để tiết kiệm băng thông MAVLink
//         if (g.rail_log_enable.get() == 1) {
//           if (now_ms - last_log_ms > 2000) {
//             gcs().send_text(MAV_SEVERITY_INFO, "T:%.1f R:%.2f A:%.2f
//             STR:%.3f",
//                             (double)target_v, (double)ramped_speed,
//                             (double)actual_speed,
//                             (double)target_turn_rate_rad);
//             last_log_ms = now_ms;
//           }
//         }
//         last_log_ms = now_ms;
//       }
//     }
//   } else {
//     // === 3. DEFAULT ARDUPILOT ACRO MODE ===
//     float speed, desired_steering;
//     if (!attitude_control.get_forward_speed(speed)) {
//       float desired_throttle;
//       get_pilot_desired_steering_and_throttle(desired_steering,
//                                               desired_throttle);
//       if (rover.is_balancebot())
//         rover.balancebot_pitch_control(desired_throttle);
//       g2.motors.set_throttle(desired_throttle);
//     } else {
//       float desired_speed;
//       get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
//       calc_throttle(desired_speed, true);
//     }

//     float steering_out;
//     if (!is_zero(desired_steering))
//       g2.sailboat.clear_tack();

//     if (g2.sailboat.tacking()) {
//       steering_out = attitude_control.get_steering_out_heading(
//           g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
//           g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
//     } else {
//       const float target_turn_rate =
//           (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       steering_out = attitude_control.get_steering_out_rate(
//           target_turn_rate, g2.motors.limit.steer_left,
//           g2.motors.limit.steer_right, dt);
//     }
//     set_steering(steering_out * 4500.0f);
//   }
// }
// Update V4 - tối ưu hóa code và clean code
void ModeAcro::update() {
  // === 1. INITIALIZATION ===
  const uint32_t now_ms = AP_HAL::millis();
  const float dt = rover.G_Dt;
  const int8_t rail_en = static_cast<int8_t>(rover.g.rail_enable.get());

  const bool currently_armed = rover.arming.is_armed();
  const bool just_armed = (currently_armed && !_rail_last_armed);
  const bool new_session = (now_ms - _rail_last_update_ms > 300U);

  _rail_last_update_ms = now_ms;
  _rail_last_armed = currently_armed;

  // === 2. RAIL MODE LOGIC ===
  if (rail_en == 1) {
    // --- 2.1: State Change Handling ---
    const float target_v_param = rover.g.rail_speed.get();
    if (rail_en != _rail_last_enable ||
        !is_equal(target_v_param, _rail_last_speed)) {
      if (_rail_last_enable != 1) {
        _rail_ramped_speed = 0.0f;
      }
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:ACTIVE | Target: %.2f",
                      static_cast<double>(target_v_param));
      _rail_last_enable = rail_en;
      _rail_last_speed = target_v_param;
    }

    float actual_speed = 0.0f;
    const bool speed_available =
        attitude_control.get_forward_speed(actual_speed);

    // Reset Setpoint to actual speed when starting a new session or Arming
    if (just_armed || new_session) {
      _rail_ramped_speed = speed_available ? actual_speed : 0.0f;
      _rail_pitch_safe_start_ms = 0U; // Reset recovery timer
    }

    // --- 2.2: Pitch Safety Logic (Speed Saturation Limit) ---
    // --- 2.2: Pitch Safety Logic (Speed Saturation Limit) ---
    float target_v = target_v_param;

    if (rover.g.rail_safe_pitch_en.get() == 1) {
      const float pitch_deg = degrees(rover.ahrs.get_pitch());
      const float safe_pitch_down_limit = -fabsf(rover.g.safe_pitch_down.get());
      const float safe_pitch_up_limit = fabsf(rover.g.safe_pitch_up.get());

      const bool is_pitch_bad = (pitch_deg < safe_pitch_down_limit) ||
                                (pitch_deg > safe_pitch_up_limit);

      // ĐƯA KHỐI NÀY LÊN TRÊN: Khai báo tỷ lệ scale trước khi sử dụng
      const float pitch_scale_param = rover.g.rail_pitch_scale.get() * 0.01f;
      const float pitch_scale = constrain_float(pitch_scale_param, 0.0f, 1.0f);

      if (is_pitch_bad) {
        // Hợp lệ: pitch_scale đã được xác định ở phạm vi (scope) phía trên
        const float max_allowed_speed = target_v_param * pitch_scale;
        if (target_v > max_allowed_speed) {
          target_v = max_allowed_speed;
        }
        _rail_pitch_safe_start_ms = 0U; // Reset timer inside dangerous zone

        if (!_rail_pitch_warning_sent) {
          gcs().send_text(
              MAV_SEVERITY_CRITICAL,
              "[RAIL] PITCH DANGER: %.2f deg (LIMIT SPEED TO %.2f m/s)",
              static_cast<double>(pitch_deg),
              static_cast<double>(max_allowed_speed));
          _rail_pitch_warning_sent = true;
        }
      } else if (_rail_pitch_warning_sent) {
        if (_rail_pitch_safe_start_ms == 0U) {
          _rail_pitch_safe_start_ms = now_ms;
        }

        const uint32_t recovery_delay_ms =
            static_cast<uint32_t>(MAX(rover.g.rail_pitch_delay.get(), 0.0f));

        if (now_ms - _rail_pitch_safe_start_ms >= recovery_delay_ms) {
          gcs().send_text(MAV_SEVERITY_WARNING,
                          "[RAIL] Pitch Safe - Resuming normal speed");
          _rail_pitch_warning_sent = false;
          _rail_pitch_safe_start_ms = 0U;
        } else {
          // Hợp lệ: Tiếp tục sử dụng pitch_scale trong thời gian trễ phục hồi
          const float max_allowed_speed = target_v_param * pitch_scale;
          if (target_v > max_allowed_speed) {
            target_v = max_allowed_speed;
          }
        }
      }
    }

    // --- 2.3: Adaptive Ramp Logic ---
    if (speed_available) {
      const float accel_step = rover.g.rail_ramped_rate.get() * dt;
      const float spd_lead = MAX(rover.g.rail_speed_lead.get(), 0.1f);

      // Prevent I-Windup by constraining ramped speed around actual speed
      _rail_ramped_speed = constrain_float(
          _rail_ramped_speed, actual_speed - spd_lead, actual_speed + spd_lead);

      // Smooth profile generator to target_v
      if (_rail_ramped_speed < target_v) {
        _rail_ramped_speed = MIN(target_v, _rail_ramped_speed + accel_step);
      } else if (_rail_ramped_speed > target_v) {
        _rail_ramped_speed = MAX(target_v, _rail_ramped_speed - accel_step);
      }
      calc_throttle(_rail_ramped_speed, false);
    } else {
      // Failsafe: Standard raw duty cycle when EKF feedback is lost
      g2.motors.set_throttle(rover.g.rail_percent * 0.01f);
    }

    // --- 2.4: Steering Process ---
    float p_steer, p_throt;
    get_pilot_desired_steering_and_throttle(p_steer, p_throt);
    float target_turn_rad = 0.0f;
    const float steer_dz = static_cast<float>(rover.g.rail_steer_dz.get());

    if (rover.g.rail_auto_steer.get() == 1 && fabsf(p_steer) > steer_dz) {
      target_turn_rad = (p_steer > 0.0f)
                            ? radians(rover.g.rail_auto_turn_rate.get())
                            : -radians(rover.g.rail_auto_turn_rate.get());
    } else if (fabsf(p_steer) > steer_dz) {
      target_turn_rad = (p_steer / 4500.0f) * radians(g2.acro_turn_rate);
    }

    float steer_out = attitude_control.get_steering_out_rate(
        target_turn_rad, g2.motors.limit.steer_left,
        g2.motors.limit.steer_right, dt);
    set_steering(constrain_float(steer_out, -1.0f, 1.0f) * 4500.0f);

    // --- 2.5: Clean Telemetry Logging (2Hz) ---
    if (rover.g.rail_log_enable.get() == 1 &&
        (now_ms - _rail_last_log_ms > 2000U)) {
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] T:%.1f R:%.2f A:%.2f STR:%.3f",
                      static_cast<double>(target_v),
                      static_cast<double>(_rail_ramped_speed),
                      static_cast<double>(actual_speed),
                      static_cast<double>(target_turn_rad));
      _rail_last_log_ms = now_ms;
    }
  } else {
    // === 3. DEFAULT ARDUPILOT ACRO MODE (EXIT RAIL) ===
    if (_rail_last_enable == 1) {
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:DEACTIVATE");
      _rail_last_enable = 0;
      _rail_pitch_warning_sent = false;
      _rail_pitch_safe_start_ms = 0U;
    }

    float speed, desired_steering;
    if (!attitude_control.get_forward_speed(speed)) {
      float desired_throttle;
      get_pilot_desired_steering_and_throttle(desired_steering,
                                              desired_throttle);
      if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
      }
      g2.motors.set_throttle(desired_throttle);
    } else {
      float desired_speed;
      get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
      calc_throttle(desired_speed, true);
    }

    if (!is_zero(desired_steering)) {
      g2.sailboat.clear_tack();
    }

    float steering_out;
    if (g2.sailboat.tacking()) {
      steering_out = attitude_control.get_steering_out_heading(
          g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
          g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
    } else {
      const float target_turn_rate =
          (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
      steering_out = attitude_control.get_steering_out_rate(
          target_turn_rate, g2.motors.limit.steer_left,
          g2.motors.limit.steer_right, dt);
    }
    set_steering(steering_out * 4500.0f);
  }
}
bool ModeAcro::requires_velocity() const {
  return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from
// transmitter
void ModeAcro::handle_tack_request() { g2.sailboat.handle_tack_request_acro(); }