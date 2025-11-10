#ifndef __LEGMODULE_H
#define __LEGMODULE_H

#include <iostream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include "can_channel.hpp"

#include "msg.hpp"
#include "fpga_handler.hpp"

class LegModule
{
public:
  LegModule(const std::string& label, const YAML::Node& config,
                     NiFpga_Status& status, NiFpga_Session session);

  // ID of Module (LF, LH, RF, RH)
  std::string label_;
  bool enable_;

  // CAN Channel
  CANChannel* channel_;

  double linkR_bias;
  double linkL_bias;

  void sendCommands();
  void receiveFeedback();
  void setMode(Mode mode);
  bool hasTimeout() const;

  CANMotor* getMotor(size_t index);
  size_t getMotorCount() const;

  int CAN_timeout_us;

  ~LegModule();

private:
  NiFpga_Status& status_;
  NiFpga_Session session_;  
  YAML::Node config_;

  std::string CAN_port_;

  void load_config();
};

double deg2rad(double deg);
Eigen::Vector2d tb2phi(const Eigen::Vector2d &tb);
Eigen::Vector2d phi2tb(const Eigen::Vector2d &phi);

#endif