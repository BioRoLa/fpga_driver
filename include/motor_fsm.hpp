#ifndef __MOTOR_FSM_H
#define __MOTOR_FSM_H

#include <math.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <Eigen/Dense>

#include "leg_module.hpp"

#include "Motor.pb.h"
#include "Power.pb.h"


class MotorFSM
{
public:
  /* Constructor - pass modules vector by reference */
  MotorFSM(std::vector<LegModule>& module_list, bool& pb_state, double* pb_v);
  MotorFSM() = delete;
  
  // Public interface methods
  void runFsm(motor_msg::MotorStateStamped &motor_fb_msg, const motor_msg::MotorCmdStamped &motor_cmd_msg);
  bool switchMode(FunctionMode next_mode);
  
  // Getters
  FunctionMode getCurrentMode() const { return current_mode_; }
  bool isHallCalibrated() const { return hall_calibrated_; }
  int getCalibrationStatus() const { return hall_calibrate_status_; }
  
  // Setters for external dependencies
  void setTimeoutErrorFlags(bool* no_can_error, bool* no_switch_error) {
    NO_CAN_TIMEDOUT_ERROR_ = no_can_error;
    NO_SWITCH_TIMEDOUT_ERROR_ = no_switch_error;
  }
  
  // Configuration setters
  void setControlPeriod(double dt) { dt_ = dt; }
  void setMeasureOffset(int offset) { measure_offset_ = offset; }
  void setCalibrationVelocity(double vel) { cal_vel_ = vel; }
  void setCalibrationTolerance(double tol) { cal_tol_ = tol; }

private:
  // State
  FunctionMode current_mode_;
  std::vector<LegModule>& modules_list_;
  bool& pb_state_;

  // Calibration state
  bool hall_calibrated_;
  int hall_calibrate_status_;
  int measure_offset_ = 0;
  
  // Control parameters
  double dt_ = 0.001;     // second
  double cal_vel_ = 0.25; // rad/s
  double cal_tol_ = 0.05;
  double cal_dir_[4][2];
  double cal_command[4][2];

  // External flags (managed by Corgi)
  bool* NO_CAN_TIMEDOUT_ERROR_;
  bool* NO_SWITCH_TIMEDOUT_ERROR_;
  double* powerboard_voltage;
  
  // Private helper methods
  void publishMsg(motor_msg::MotorStateStamped &motor_fb_msg);
};
double theta_error(double start_theta, double goal_theta);


#endif
