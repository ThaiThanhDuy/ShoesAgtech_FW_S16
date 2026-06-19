#include "Rover.h"

// Shoes_Agtech: khoi tao/giai phong trang thai Pitch Safety khi vao/thoat
// Manual
bool ModeManual::_enter() {
  _last_pitch_rate_rads = rover.ahrs.get_gyro().y;
  _filtered_pitch_accel_degs2 = 0.0f;
  _pitch_warning_sent = false;
  _pitch_safe_start_ms = 0U;
  return true;
}

void ModeManual::_exit() {
  // clear lateral when exiting manual mode
  g2.motors.set_lateral(0);
}

void ModeManual::update() {
  float desired_steering, desired_throttle, desired_lateral;
  get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
  get_pilot_desired_lateral(desired_lateral);

  // --- Shoes_Agtech: Pitch Safety (Manual) - bat/tat qua MAN_PITCH_EN ---
  if (rover.g.man_pitch_en.get() == 1) {
    const uint32_t now_ms = AP_HAL::millis();
    const float pitch_deg = degrees(rover.ahrs.get_pitch_rad());
    const float current_pitch_rate_rads = rover.ahrs.get_gyro().y;

    float raw_pitch_accel_degs2 = 0.0f;
    if (rover.G_Dt > 0.0001f) {
      raw_pitch_accel_degs2 =
          degrees(current_pitch_rate_rads - _last_pitch_rate_rads) / rover.G_Dt;
    }
    _last_pitch_rate_rads = current_pitch_rate_rads;

    const float lpf_alpha =
        constrain_float(rover.G_Dt / (0.04f + rover.G_Dt), 0.05f, 1.0f);
    _filtered_pitch_accel_degs2 =
        (lpf_alpha * raw_pitch_accel_degs2) +
        ((1.0f - lpf_alpha) * _filtered_pitch_accel_degs2);

    const float safe_pitch_down_limit = -fabsf(g.safe_pitch_down.get());
    const float safe_pitch_up_limit = fabsf(g.safe_pitch_up.get());
    const float safe_pitch_accel_limit = fabsf(rover.g.safe_pitch_accel.get());

    const bool is_angle_bad = (pitch_deg < safe_pitch_down_limit) ||
                              (pitch_deg > safe_pitch_up_limit);
    const bool is_inertia_bad =
        (fabsf(_filtered_pitch_accel_degs2) > safe_pitch_accel_limit);
    const bool is_pitch_bad = is_angle_bad || is_inertia_bad;

    const float pitch_scale =
        constrain_float(rover.g.man_pitch_scale.get() * 0.01f, 0.0f, 1.0f);

    if (is_pitch_bad) {
      desired_throttle *= pitch_scale;
      _pitch_safe_start_ms = 0U;
      if (!_pitch_warning_sent) {
        gcs().send_text(
            MAV_SEVERITY_CRITICAL,
            "[MAN] PITCH DANGER! Ang:%.1fdeg Acc:%.1fdeg/s2 -> x%.0f%%",
            static_cast<double>(pitch_deg),
            static_cast<double>(_filtered_pitch_accel_degs2),
            static_cast<double>(rover.g.man_pitch_scale.get()));
        _pitch_warning_sent = true;
      }
    } else if (_pitch_warning_sent) {
      if (_pitch_safe_start_ms == 0U) {
        _pitch_safe_start_ms = now_ms;
      }
      const uint32_t recovery_delay_ms =
          static_cast<uint32_t>(MAX(rover.g.man_pitch_delay.get(), 0));
      if (now_ms - _pitch_safe_start_ms >= recovery_delay_ms) {
        gcs().send_text(MAV_SEVERITY_WARNING, "[MAN] Pitch Safe - Resuming");
        _pitch_warning_sent = false;
        _pitch_safe_start_ms = 0U;
      } else {
        desired_throttle *= pitch_scale;
      }
    }
  } else {
    _last_pitch_rate_rads = 0.0f;
    _filtered_pitch_accel_degs2 = 0.0f;
    _pitch_warning_sent = false;
    _pitch_safe_start_ms = 0U;
  }
  // --- Pitch Safety END ---
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