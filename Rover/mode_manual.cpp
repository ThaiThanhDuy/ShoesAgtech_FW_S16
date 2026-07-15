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
  _apply_pitch_safety(desired_throttle, rover.g.man_pitch_en.get() == 1,
                      rover.g.man_pitch_scale.get(),
                      rover.g.man_pitch_delay.get(), "MAN");
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