/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   This is the ArduRover firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Randy Mackay, Grant Morphett

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew
   Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler,
   Jean-Louis Naudin, Grant Morphett

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani,
   James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi,
   Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat...

   Please contribute your ideas! See https://ardupilot.org/dev for details
*/

#include "Rover.h"

// Shoes_Agtech: AUTO_TIMER scheduled auto-run needs GPS/RTC local time
#include <AP_RTC/AP_RTC.h>

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, _max_time_micros, _priority)                 \
  SCHED_TASK_CLASS(Rover, &rover, func, rate_hz, _max_time_micros, _priority)

/*
  scheduler table - all regular tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,
    SCHED_TASK(read_radio, 50, 200, 3),
    SCHED_TASK(ahrs_update, 400, 400, 6),
#if AP_RANGEFINDER_ENABLED
    SCHED_TASK(read_rangefinders, 50, 200, 9),
#endif
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow, &rover.optflow, update, 200, 160, 11),
#endif
    SCHED_TASK(update_current_mode, 400, 200, 12),
    SCHED_TASK(set_servos, 400, 200, 15),
    SCHED_TASK_CLASS(AP_GPS, &rover.gps, update, 50, 300, 18),
    SCHED_TASK_CLASS(AP_Baro, &rover.barometer, update, 10, 200, 21),
    // [AP_ShoesAgtech] 10Hz task — priority 22 (between Baro=21 and Beacon=24)
    SCHED_TASK(update_custom_flow, 10, 80, 22),
    // [/AP_ShoesAgtech]
#if AP_BEACON_ENABLED
    SCHED_TASK_CLASS(AP_Beacon, &rover.g2.beacon, update, 50, 200, 24),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity, &rover.g2.proximity, update, 50, 200, 27),
#endif
    SCHED_TASK_CLASS(AP_WindVane, &rover.g2.windvane, update, 20, 100, 30),
    SCHED_TASK(update_wheel_encoder, 50, 200, 36),
    SCHED_TASK(update_compass, 10, 200, 39),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(update_logging1, 10, 200, 45),
    SCHED_TASK(update_logging2, 10, 200, 48),

#endif
    SCHED_TASK_CLASS(GCS, (GCS *)&rover._gcs, update_receive, 400, 500, 51),
    SCHED_TASK_CLASS(GCS, (GCS *)&rover._gcs, update_send, 400, 1000, 54),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels *)&rover.g2.rc_channels,
                     read_mode_switch, 7, 200, 57),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels *)&rover.g2.rc_channels,
                     read_aux_all, 10, 200, 60),
    SCHED_TASK_CLASS(AP_BattMonitor, &rover.battery, read, 10, 300, 63),

#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &rover.ServoRelayEvents,
                     update_events, 50, 200, 66),
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(update_precland, 400, 50, 70),
#endif
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &rover.camera_mount, update, 50, 200, 75),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera, &rover.camera, update, 50, 200, 78),
#endif
    SCHED_TASK(gcs_failsafe_check, 10, 200, 81),
#if AP_FENCE_ENABLED
    SCHED_TASK(fence_check, 10, 200, 84),
#endif
    SCHED_TASK(ekf_check, 10, 100, 87),
    SCHED_TASK_CLASS(ModeSmartRTL, &rover.mode_smartrtl, save_position, 3, 200,
                     90),
    SCHED_TASK(one_second_loop, 1, 1500, 96),
    // Shoes_Agtech: AUTO_TIMER scheduled auto-run, checked once a second
    SCHED_TASK(update_auto_timer, 1, 200, 97),
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer, &rover.g2.sprayer, update, 3, 90, 99),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger, &rover.logger, periodic_tasks, 50, 300, 108),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &rover.ins, periodic, 400, 200, 111),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &rover.scheduler, update_logging, 0.1, 200,
                     114),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button, &rover.button, update, 5, 200, 117),
#endif
    SCHED_TASK(crash_check, 10, 200, 123),
    SCHED_TASK(cruise_learn_update, 50, 200, 126),
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
    SCHED_TASK(afs_fs_check, 10, 200, 129),
#endif
};

void Rover::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count, uint32_t &log_bit) {
  tasks = &scheduler_tasks[0];
  task_count = ARRAY_SIZE(scheduler_tasks);
  log_bit = MASK_LOG_PM;
}

constexpr int8_t Rover::_failsafe_priorities[7];

Rover::Rover(void)
    : AP_Vehicle(), param_loader(var_info), modes(&g.mode1),
      control_mode(&mode_initializing) {}

#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
// set target location (for use by external control and scripting)
bool Rover::set_target_location(const Location &target_loc) {
  // exit if vehicle is not in Guided mode or Auto-Guided mode
  if (!control_mode->in_guided_mode()) {
    return false;
  }

  return mode_guided.set_desired_location(target_loc);
}
#endif // AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
// set target velocity (for use by scripting)
bool Rover::set_target_velocity_NED(const Vector3f &vel_ned) {
  // exit if vehicle is not in Guided mode or Auto-Guided mode
  if (!control_mode->in_guided_mode()) {
    return false;
  }

  // convert vector length into speed
  const float target_speed_m = safe_sqrt(sq(vel_ned.x) + sq(vel_ned.y));

  // convert vector direction to target yaw
  const float target_yaw_cd = degrees(atan2f(vel_ned.y, vel_ned.x)) * 100.0f;

  // send target heading and speed
  mode_guided.set_desired_heading_and_speed(target_yaw_cd, target_speed_m);

  return true;
}

// set steering and throttle (-1 to +1) (for use by scripting)
bool Rover::set_steering_and_throttle(float steering, float throttle) {
  // exit if vehicle is not in Guided mode or Auto-Guided mode
  if (!control_mode->in_guided_mode()) {
    return false;
  }

  // set steering and throttle
  mode_guided.set_steering_and_throttle(steering, throttle);
  return true;
}

// get steering and throttle (-1 to +1) (for use by scripting)
bool Rover::get_steering_and_throttle(float &steering, float &throttle) {
  steering = g2.motors.get_steering() / 4500.0;
  throttle = g2.motors.get_throttle() * 0.01;
  return true;
}

// set desired turn rate (degrees/sec) and speed (m/s). Used for scripting
bool Rover::set_desired_turn_rate_and_speed(float turn_rate, float speed) {
  // exit if vehicle is not in Guided mode or Auto-Guided mode
  if (!control_mode->in_guided_mode()) {
    return false;
  }

  // set turn rate and speed. Turn rate is expected in centidegrees/s and speed
  // in meters/s
  mode_guided.set_desired_turn_rate_and_speed(turn_rate * 100.0f, speed);
  return true;
}

// set desired nav speed (m/s). Used for scripting.
bool Rover::set_desired_speed(float speed) {
  return control_mode->set_desired_speed(speed);
}

// get control output (for use in scripting)
// returns true on success and control_value is set to a value in the range -1
// to +1
bool Rover::get_control_output(AP_Vehicle::ControlOutput control_output,
                               float &control_value) {
  switch (control_output) {
  case AP_Vehicle::ControlOutput::Roll:
    control_value = constrain_float(g2.motors.get_roll(), -1.0f, 1.0f);
    return true;
  case AP_Vehicle::ControlOutput::Pitch:
    control_value = constrain_float(g2.motors.get_pitch(), -1.0f, 1.0f);
    return true;
  case AP_Vehicle::ControlOutput::Walking_Height:
    control_value =
        constrain_float(g2.motors.get_walking_height(), -1.0f, 1.0f);
    return true;
  case AP_Vehicle::ControlOutput::Throttle:
    control_value =
        constrain_float(g2.motors.get_throttle() * 0.01f, -1.0f, 1.0f);
    return true;
  case AP_Vehicle::ControlOutput::Yaw:
    control_value =
        constrain_float(g2.motors.get_steering() / 4500.0f, -1.0f, 1.0f);
    return true;
  case AP_Vehicle::ControlOutput::Lateral:
    control_value =
        constrain_float(g2.motors.get_lateral() * 0.01f, -1.0f, 1.0f);
    return true;
  case AP_Vehicle::ControlOutput::MainSail:
    control_value =
        constrain_float(g2.motors.get_mainsail() * 0.01f, -1.0f, 1.0f);
    return true;
  case AP_Vehicle::ControlOutput::WingSail:
    control_value =
        constrain_float(g2.motors.get_wingsail() * 0.01f, -1.0f, 1.0f);
    return true;
  default:
    return false;
  }
  return false;
}

// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Rover::nav_scripting_enable(uint8_t mode) {
  return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Rover::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1,
                            float &arg2, int16_t &arg3, int16_t &arg4) {
  if (control_mode != &mode_auto) {
    return false;
  }

  return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// lua scripts use this to indicate when they have complete the command
void Rover::nav_script_time_done(uint16_t id) {
  if (control_mode != &mode_auto) {
    return;
  }

  return mode_auto.nav_script_time_done(id);
}
#endif // AP_SCRIPTING_ENABLED

// update AHRS system
void Rover::ahrs_update() {
  arming.update_soft_armed();

  // AHRS may use movement to calculate heading
  update_ahrs_flyforward();

  ahrs.update();

  // update position
  have_position = ahrs.get_location(current_loc);

  // set home from EKF if necessary and possible
  if (!ahrs.home_is_set()) {
    if (!set_home_to_current_location(false)) {
      // ignore this failure
    }
  }

  // if using the EKF get a speed update now (from accelerometers)
  Vector3f velocity;
  if (ahrs.get_velocity_NED(velocity)) {
    ground_speed = velocity.xy().length();
  } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
    ground_speed = ahrs.groundspeed();
  }

#if AP_FOLLOW_ENABLED
  g2.follow.update_estimates();
#endif

#if HAL_LOGGING_ENABLED
  if (should_log(MASK_LOG_ATTITUDE_FAST)) {
    Log_Write_Attitude();
    Log_Write_Sail();
  }

  if (should_log(MASK_LOG_IMU)) {
    AP::ins().Write_IMU();
  }

  if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
    ahrs.write_video_stabilisation();
  }
#endif
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void) {
  if (g.fs_gcs_enabled == FS_GCS_DISABLED) {
    // gcs failsafe disabled
    return;
  }

  const uint32_t gcs_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
  if (gcs_last_seen_ms == 0) {
    // we've never seen the GCS, so we never failsafe for not seeing it
    return;
  }

  // calc time since last gcs update
  // note: this only looks at the heartbeat from the device ids approved by
  // gcs().sysid_is_gcs()
  const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
  const uint32_t gcs_timeout_ms =
      uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

  const bool do_failsafe = last_gcs_update_ms >= gcs_timeout_ms ? true : false;

  failsafe_trigger(FAILSAFE_EVENT_GCS, "GCS", do_failsafe);
}

#if HAL_LOGGING_ENABLED
/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void) {
  if (should_log(MASK_LOG_ATTITUDE_MED) &&
      !should_log(MASK_LOG_ATTITUDE_FAST)) {
    Log_Write_Attitude();
    Log_Write_Sail();
  }

  if (should_log(MASK_LOG_THR)) {
    Log_Write_Throttle();
#if AP_BEACON_ENABLED
    g2.beacon.log();
#endif
  }

  if (should_log(MASK_LOG_NTUN)) {
    Log_Write_Nav_Tuning();
    if (g2.pos_control.is_active()) {
      g2.pos_control.write_log();
      logger.Write_PID(LOG_PIDN_MSG,
                       g2.pos_control.get_vel_pid().get_pid_info_x());
      logger.Write_PID(LOG_PIDE_MSG,
                       g2.pos_control.get_vel_pid().get_pid_info_y());
    }
  }

#if HAL_PROXIMITY_ENABLED
  if (should_log(MASK_LOG_RANGEFINDER)) {
    g2.proximity.log();
  }
#endif
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void) {
  if (should_log(MASK_LOG_STEERING)) {
    Log_Write_Steering();
  }

  if (should_log(MASK_LOG_RC)) {
    Log_Write_RC();
    g2.wheel_encoder.Log_Write();
  }

  if (should_log(MASK_LOG_IMU)) {
    AP::ins().Write_Vibration();
#if HAL_GYROFFT_ENABLED
    gyro_fft.write_log_messages();
#endif
  }
#if HAL_MOUNT_ENABLED
  if (should_log(MASK_LOG_CAMERA)) {
    camera_mount.write_log();
  }
#endif
}
#endif // HAL_LOGGING_ENABLED

#if AP_ROVER_AUTO_ARM_ONCE_ENABLED
void Rover::handle_auto_arm_once() {
  if (arming.is_armed()) {
    // never re-arm automatically if the user ever armed the vehicle
    auto_arm_once.done = true;
    return;
  }
  if (auto_arm_once.done) {
    return;
  }
  switch (arming.arming_required()) {
  case AP_Arming::Required::NO:
  case AP_Arming::Required::YES_MIN_PWM:
  case AP_Arming::Required::YES_ZERO_PWM:
    // in case the user changes the require parameter at runtime,
    // don't auto-arm:
    auto_arm_once.done = true;
    return;
  case AP_Arming::Required::YES_AUTO_ARM_MIN_PWM:
  case AP_Arming::Required::YES_AUTO_ARM_ZERO_PWM:
    break;
  }

  // don't try to arm if prearms are not passing:
  if (!arming.get_last_prearm_checks_result()) {
    return;
  }

  const uint32_t now_ms = AP_HAL::millis();
  // only attempt to auto arm once per 5 seconds:
  if (now_ms - auto_arm_once.last_arm_attempt_ms < 5000) {
    return;
  }
  auto_arm_once.last_arm_attempt_ms = now_ms;

  if (!arming.arm(AP_Arming::Method::AUTO_ARM_ONCE)) {
    return;
  }

  auto_arm_once.done = true;
}
#endif // AP_ROVER_AUTO_ARM_ONCE_ENABLED

/*
  once a second events
 */
void Rover::one_second_loop(void) {
  set_control_channels();

  // cope with changes to aux functions
  AP::srv().enable_aux_servos();

  // update notify flags
  AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
  AP_Notify::flags.pre_arm_gps_check = true;
  AP_Notify::flags.armed = arming.is_armed();
  AP_Notify::flags.flying = hal.util->get_soft_armed();

#if AP_ROVER_AUTO_ARM_ONCE_ENABLED
  handle_auto_arm_once();
#endif // AP_ROVER_AUTO_ARM_ONCE_ENABLED

  // attempt to update home position and baro calibration if not armed:
  if (!hal.util->get_soft_armed()) {
    update_home();
  }

  // need to set "likely flying" when armed to allow for compass
  // learning to run
  set_likely_flying(hal.util->get_soft_armed());

  // send latest param values to wp_nav
  g2.wp_nav.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());
  g2.pos_control.set_turn_params(g2.turn_radius,
                                 g2.motors.have_skid_steering());
  g2.wheel_rate_control.set_notch_sample_rate(
      AP::scheduler().get_filtered_loop_rate_hz());

#if AP_STATS_ENABLED
  // Update stats "flying" time
  AP::stats()->set_flying(g2.motors.active());
#endif
}

// [AP_ShoesAgtech] -------------------------------------------------------
void Rover::update_custom_flow(void) {
  g2.custom_nav.update();       // flow calc + spray control + pH poll + console log
#if HAL_LOGGING_ENABLED
  Log_Write_Flow_Realtime();    // write FLWD to SD card
  Log_Write_Ph_Realtime();      // write PHWD to SD card (skips if SA_PH_EN=0)
  Log_Write_Ph_Alkalinity();    // write PHAK once per day when morning+afternoon slots FULL
#endif
}
// [/AP_ShoesAgtech] -------------------------------------------------------

// Shoes_Agtech: -----------------------------------------------------------
// AUTO_TIMER — hen gio chay tu dong. Moi giay kiem tra gio dia phuong
// (AUTO_TIMER_TZ) so voi 3 moc AUTO_TIMER1/2/3. Khi toi/qua 1 moc (lan dau
// trong ngay) va DA ARM SAN (nguoi van hanh tu arm truoc, co giam sat, roi
// moi roi di) - BAT KE dang o mode nao: reset mission ve waypoint dau va
// chuyen sang mode AUTO — vi dang ARM nen xe SE BAT DAU CHAY MISSION NGAY,
// khong can ai co mat luc do. Neu dang KHONG arm luc do gio thi AN TOAN HON
// la bo qua, chi bao info, khong tu chuyen mode/mission. Khong con kiem tra
// mode hien tai truoc khi chuyen (bo theo yeu cau) - neu 1 moc khac dang
// chay mission ma toi gio 1 moc sau, mission se bi reset lai tu waypoint dau.
// Dinh dang moi AUTO_TIMER1/2/3: nhap THANG gio.phut (vd 15.30 = 15h30p,
// KHONG phai phan so gio nhu SA_PH_MS). 0 = tat slot. Phan phut (2 chu so
// sau dau cham) phai < 60, neu khong -> khong hop le. hh=24 & mm=0 (tuc
// "24.00") = quy uoc rieng cho nua dem (00:00), vi 0 da danh cho "tat".
enum class AutoTimerSlotState : uint8_t { DISABLED, INVALID, VALID };

static AutoTimerSlotState auto_timer_parse_slot(float t, uint8_t &hh,
                                                uint8_t &mm) {
  if (t <= 0.0f) {
    return AutoTimerSlotState::DISABLED;
  }
  const int hh_i = (int)t;
  const int mm_i = (int)((t - (float)hh_i) * 100.0f + 0.5f);
  if (mm_i >= 60 || hh_i > 24 || (hh_i == 24 && mm_i != 0)) {
    return AutoTimerSlotState::INVALID;
  }
  hh = (uint8_t)((hh_i == 24) ? 0 : hh_i); // "24.00" quy uoc = nua dem 00:00
  mm = (uint8_t)mm_i;
  return AutoTimerSlotState::VALID;
}

void Rover::update_auto_timer(void) {
  // Neu phien AUTO hien tai la do AUTO_TIMER kich hoat: theo doi mission.
  // Mission xong (MISSION_COMPLETE) -> tu dong chuyen ve MANUAL. Neu mode da
  // bi doi khoi AUTO vi ly do khac (nguoi dung/failsafe/RC...) -> ngung theo
  // doi, khong con la phien cua AUTO_TIMER nua. Chay TRUOC ca kiem tra
  // AUTO_TIMER master-enable, de van hoan tat dung cam ket ngay ca khi
  // AUTO_TIMER vua bi tat giua chung mission.
  if (_auto_timer_active) {
    if (control_mode != &mode_auto) {
      _auto_timer_active = false; // da roi AUTO vi ly do khac
    } else if (mode_auto.mission.state() == AP_Mission::MISSION_COMPLETE) {
      _auto_timer_active = false;
      set_mode(mode_manual, ModeReason::MISSION_END);
      gcs().send_text(MAV_SEVERITY_INFO,
                      "AUTO_TIMER: mission xong - tu dong ve mode MANUAL");
    }
  }

  if (g.auto_timer.get() <= 0) {
    return;
  }

  const float slots[3] = {g.auto_timer1.get(), g.auto_timer2.get(),
                          g.auto_timer3.get()};

  // Phat hien nguoi dung vua sua AUTO_TIMERx (vd sua lai gio sau khi bi bo
  // qua vi da qua gio). Neu gia tri moi chua toi gio thi se chay lai binh
  // thuong ngay (khong doi den ngay mai), va in 1 dong INFO bao cap nhat.
  for (uint8_t i = 0; i < 3; i++) {
    if (!_auto_timer_last_val_init[i]) {
      _auto_timer_last_val_init[i] = true;
      _auto_timer_last_val[i] = slots[i];
      continue; // lan doc dau tien - chi ghi nhan, khong bao "thay doi"
    }
    if (fabsf(slots[i] - _auto_timer_last_val[i]) <= 0.001f) {
      continue; // khong doi
    }
    _auto_timer_last_val[i] = slots[i];
    _auto_timer_triggered_day[i] = 0;   // cho danh gia lai voi gio moi
    _auto_timer_seen_before[i] = false; // danh gia "da qua gio" lai tu dau

    uint8_t hh_c = 0, mm_c = 0;
    switch (auto_timer_parse_slot(slots[i], hh_c, mm_c)) {
    case AutoTimerSlotState::DISABLED:
      gcs().send_text(MAV_SEVERITY_INFO, "AUTO_TIMER%u: da tat",
                      (unsigned)(i + 1));
      break;
    case AutoTimerSlotState::INVALID:
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "AUTO_TIMER%u: gia tri moi %.2f khong hop le",
                      (unsigned)(i + 1), (double)slots[i]);
      break;
    case AutoTimerSlotState::VALID:
      gcs().send_text(MAV_SEVERITY_INFO,
                      "AUTO_TIMER%u: cap nhat gio moi %02u:%02u",
                      (unsigned)(i + 1), (unsigned)hh_c, (unsigned)mm_c);
      break;
    }
  }

  if (!_auto_timer_boot_logged) {
    _auto_timer_boot_logged = true;
    // "--:--" = tat slot; "LOI" = gia tri nhap sai (phut >=60); nguoc lai
    // hien thi HH:MM that su. Buffer 32 byte du du de tranh canh bao
    // -Wformat-truncation (GCC uoc luong worst-case cho %u co the toi 10 chu
    // so du gia tri thuc luon nam trong uint8_t).
    char hm[3][32];
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t hh = 0, mm = 0;
      switch (auto_timer_parse_slot(slots[i], hh, mm)) {
      case AutoTimerSlotState::DISABLED:
        snprintf(hm[i], sizeof(hm[i]), "--:--");
        break;
      case AutoTimerSlotState::INVALID:
        snprintf(hm[i], sizeof(hm[i]), "LOI");
        break;
      case AutoTimerSlotState::VALID:
        snprintf(hm[i], sizeof(hm[i]), "%02u:%02u", (unsigned)hh,
                (unsigned)mm);
        break;
      }
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AUTO_TIMER bat: moc %s %s %s", hm[0],
                    hm[1], hm[2]);
  }

  uint64_t utc_usec;
  if (!AP::rtc().get_utc_usec(utc_usec)) {
    return; // chua co gio GPS/RTC
  }

  const int8_t tz = constrain_int16(g.auto_timer_tz.get(), -12, 14);
  const uint32_t utc_sec = (uint32_t)(utc_usec / 1000000ULL);
  const uint32_t local_sec = utc_sec + (uint32_t)((int32_t)tz * 3600);
  const uint32_t today = local_sec / 86400U;
  const float local_h = (float)(local_sec % 86400U) / 3600.0f;
  const uint32_t now_ms = AP_HAL::millis();

  // Sang ngay moi - reset "da tung thay chua toi gio" cho ca 3 moc.
  if (today != _auto_timer_seen_day) {
    _auto_timer_seen_day = today;
    _auto_timer_seen_before[0] = false;
    _auto_timer_seen_before[1] = false;
    _auto_timer_seen_before[2] = false;
  }

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t hh = 0, mm = 0;
    const AutoTimerSlotState state = auto_timer_parse_slot(slots[i], hh, mm);

    if (state == AutoTimerSlotState::DISABLED) {
      continue;
    }
    if (state == AutoTimerSlotState::INVALID) {
      // Canh bao gia tri sai, rate-limit 60s de khong spam log.
      if (now_ms - _auto_timer_invalid_warn_ms[i] >= 60000U) {
        _auto_timer_invalid_warn_ms[i] = now_ms;
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "AUTO_TIMER%u: gia tri %.2f khong hop le (phut phai "
                        "<60) - bo qua",
                        (unsigned)(i + 1), (double)slots[i]);
      }
      continue;
    }

    const float target_h = (float)hh + (float)mm / 60.0f;

    if (_auto_timer_triggered_day[i] == today) {
      continue; // moc nay da xu ly hom nay roi
    }
    if (local_h < target_h) {
      // Chua toi gio - nhung xac nhan la DA THEO DOI truoc gio hen hom nay,
      // de phan biet voi truong hop moi co GPS/RTC SAU KHI gio da troi qua.
      _auto_timer_seen_before[i] = true;
      continue;
    }

    // Tu day tro xuong: local_h >= target_h. Danh dau da xu ly moc nay hom
    // nay - bat ke ket qua ben duoi la gi, khong thu lai nua cho den ngay moi.
    _auto_timer_triggered_day[i] = today;

    if (!_auto_timer_seen_before[i]) {
      // Nguy hiem neu bo qua buoc nay: chua tung thay "chua toi gio" hom nay
      // truoc khi phat hien da qua gio - nghia la he thong (hoac GPS/RTC)
      // moi san sang SAU KHI gio hen da troi qua (vd boot tre trong ngay).
      // KHONG kich hoat chuyen mode "bu" cho gio da qua - chi bao 1 dong info.
      gcs().send_text(MAV_SEVERITY_INFO,
                      "AUTO_TIMER%u (%02u:%02u): da qua gio - bo qua hom nay",
                      (unsigned)(i + 1), (unsigned)hh, (unsigned)mm);
      continue;
    }

    // YEU CAU BAT BUOC: phai DA ARM san (nguoi van hanh tu arm truoc, co
    // giam sat, roi moi roi di) thi den gio moi duoc tu dong chuyen AUTO -
    // vi chuyen mode trong luc DANG ARM se lam xe CHAY MISSION NGAY LAP TUC.
    // Neu dang KHONG arm luc do -> AN TOAN HON la bo qua, khong tu chuyen
    // mode (tranh chuan bi xe khong nguoi giam sat).
    if (!hal.util->get_soft_armed()) {
      gcs().send_text(MAV_SEVERITY_INFO,
                      "AUTO_TIMER%u (%02u:%02u): chua ARM - bo qua tu dong "
                      "chuyen AUTO",
                      (unsigned)(i + 1), (unsigned)hh, (unsigned)mm);
      continue;
    }

    if (control_mode == &mode_auto) {
      // Da o mode AUTO san (do AUTO_TIMER khac dang chay, hoac nguoi dung tu
      // bat AUTO binh thuong) - bo qua, tranh xung dot/reset ngang mission
      // dang chay.
      gcs().send_text(MAV_SEVERITY_INFO,
                      "AUTO_TIMER%u (%02u:%02u): dang o mode AUTO - bo qua "
                      "tranh xung dot",
                      (unsigned)(i + 1), (unsigned)hh, (unsigned)mm);
      continue;
    }

    mode_auto.mission.reset(); // rewind ve waypoint dau
    set_mode(mode_auto, ModeReason::UNKNOWN);
    _auto_timer_active = true; // phien AUTO nay la do AUTO_TIMER kich hoat
    gcs().send_text(MAV_SEVERITY_INFO,
                    "AUTO_TIMER%u (%02u:%02u): da ARM - chuyen sang mode "
                    "AUTO, xe bat dau chay ngay",
                    (unsigned)(i + 1), (unsigned)hh, (unsigned)mm);
  }
}
// [/Shoes_Agtech] -----------------------------------------------------------

void Rover::update_current_mode(void) {
  // check for emergency stop
  if (SRV_Channels::get_emergency_stop()) {
    // relax controllers, motor stopping done at output level
    g2.attitude_control.relax_I();
  }

  control_mode->update();
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_distance_m(float &distance) const {
  // see GCS_MAVLINK_Rover::send_nav_controller_output()
  if (!rover.control_mode->is_autopilot_mode()) {
    return false;
  }
  distance = control_mode->get_distance_to_destination();
  return true;
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_bearing_deg(float &bearing) const {
  // see GCS_MAVLINK_Rover::send_nav_controller_output()
  if (!rover.control_mode->is_autopilot_mode()) {
    return false;
  }
  bearing = control_mode->wp_bearing();
  return true;
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_crosstrack_error_m(float &xtrack_error) const {
  // see GCS_MAVLINK_Rover::send_nav_controller_output()
  if (!rover.control_mode->is_autopilot_mode()) {
    return false;
  }
  xtrack_error = control_mode->crosstrack_error();
  return true;
}

Rover rover;
AP_Vehicle &vehicle = rover;

AP_HAL_MAIN_CALLBACKS(&rover);
