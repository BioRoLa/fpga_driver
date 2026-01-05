#ifndef __MOTOR_FSM_H
#define __MOTOR_FSM_H

#include <math.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <Eigen/Dense>

#include "leg_module.hpp"
#include "Logger.h"

#include "Motor.pb.h"
#include "Power.pb.h"
#include "Config.pb.h"

// Forward declaration
class RobotFSM;

class MotorFSM
{
public:
  /* Constructor - pass modules vector by reference */
  MotorFSM(std::vector<LegModule>& module_list, std::vector<bool>& pb_state, double* pb_v);
  MotorFSM() = delete;
  
  // Public interface methods
  void runFsm(motor_msg::MotorStateStamped &motor_fb_msg, const motor_msg::MotorCmdStamped &motor_cmd_msg, config_msg::ConfigStamped &config_msg);
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
  
  void setRobotFSM(RobotFSM* robot_fsm) {
    robot_fsm_ = robot_fsm;
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
  std::vector<bool>& pb_state_;

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
  int last_process_seq = -1;

  // External flags (managed by Corgi)
  bool* NO_CAN_TIMEDOUT_ERROR_;
  bool* NO_SWITCH_TIMEDOUT_ERROR_;
  double* powerboard_voltage;
  RobotFSM* robot_fsm_;  // Pointer to RobotFSM for mode checking
  
  // Private helper methods
  void publishMsg(motor_msg::MotorStateStamped &motor_fb_msg);
  void handleConfigMessage(config_msg::ConfigStamped &config_data);
  void publishConfigMsg(config_msg::ConfigStamped &config_data);
  
  // Mode handlers
  void handleRestMode();
  void handleSetZeroMode();
  void handleHallCalibrateMode();
  void handleMotorMode(const motor_msg::MotorCmdStamped& motor_cmd_msg);
  void handleConfigMode(config_msg::ConfigStamped &config_msg);
};
double theta_error(double start_theta, double goal_theta);


#endif
