#include "Rover.h"

void ModeManual::_exit() {
  // clear lateral when exiting manual mode
  g2.motors.set_lateral(0);
}

void ModeManual::update() {
  float desired_steering, desired_throttle, desired_lateral;
  get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
  get_pilot_desired_lateral(desired_lateral);

  // --- Pitch Safety Warning Logic START ---
  const float pitch_deg = degrees(rover.ahrs.get_pitch());

  // Kiểm tra trạng thái góc Pitch dựa trên tham số cấu hình hệ thống
  // g.safe_pitch_down: Ngưỡng chúi mũi | g.safe_pitch_up: Ngưỡng ngóc mũi
  const bool is_pitch_bad = (pitch_deg < -fabsf(g.safe_pitch_down.get()) ||
                             pitch_deg > fabsf(g.safe_pitch_up.get()));

  if (is_pitch_bad) {
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_warn_ms = 0;
    desired_throttle = 0.0f;
    if (now_ms - last_warn_ms > 1000) { // Tần suất cảnh báo 1Hz
      gcs().send_text(MAV_SEVERITY_CRITICAL,
                      "PITCH DANGER: %.2f deg | THROTTLE LOCKED",
                      (double)pitch_deg);
      last_warn_ms = now_ms;
    }
  }
  // --- Pitch Safety Warning Logic END ---

  // apply manual steering expo
  desired_steering =
      4500.0f * input_expo(desired_steering / 4500.0f, g2.manual_steering_expo);

  // if vehicle is balance bot, calculate actual throttle required for balancing
  if (rover.is_balancebot()) {
    rover.balancebot_pitch_control(desired_throttle);
  }

  // walking robots support roll, pitch and walking_height
  float desired_roll, desired_pitch, desired_walking_height;
  get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
  get_pilot_desired_walking_height(desired_walking_height);
  g2.motors.set_roll(desired_roll);
  g2.motors.set_pitch(desired_pitch);
  g2.motors.set_walking_height(desired_walking_height);

  // set sailboat sails
  g2.sailboat.set_pilot_desired_mainsail();

  // copy RC scaled inputs to outputs
  g2.motors.set_throttle(desired_throttle);
  g2.motors.set_steering(desired_steering,
                         (g2.manual_options & ManualOptions::SPEED_SCALING));
  g2.motors.set_lateral(desired_lateral);
}