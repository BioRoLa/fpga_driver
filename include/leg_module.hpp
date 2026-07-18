#ifndef __LEGMODULE_H
#define __LEGMODULE_H

#include <iostream>
#include <vector>
#include <memory>
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
                     NiFpga_Status& status, NiFpga_Session& session);

  // ID of Module (LF, LH, RF, RH)
  std::string label_;
  bool enable_;

  // CAN Channel - non-owning. Valid for both the module that constructed the
  // channel (is_owner_ == true) and any module sharing it (is_owner_ == false).
  // Direct dereference (e.g. console.cpp's channel_->hasTxTimeout()) stays safe
  // for shared modules because this always points at a live CANChannel.
  CANChannel* channel_ = nullptr;

  // CAN-channel-sharing state. Populated by Corgi::load_config_()'s port-grouping
  // pass (main.cpp) after all LegModules are constructed. group_size_ == 1 means
  // this module has exclusive use of its channel (today's normal case for every
  // leg) and the round-robin gate in canLoop_() is a provable no-op.
  bool is_owner_ = true;
  size_t rr_slot_ = 0;
  size_t group_size_ = 1;

  double linkR_bias;
  double linkL_bias;
  double linkH_bias;

  void sendCommands();
  void receiveFeedback();
  void setMode(FunctionMode mode);
  void setMotorMode(size_t index, FunctionMode mode);
  void setConfigSubMode(ConfigSubMode sub_mode);
  void updateTimeoutDebounce(uint32_t loop_period_us);
  bool hasTimeout() const;

  // Reads from this module's own persistent motor storage (own_motors_), never
  // through channel_. Required so motor_fsm_.runFsm() (main loop, faster than the
  // CAN loop) can setCommand()/read feedback on this leg's motors every tick
  // regardless of whether it is currently this leg's turn on a shared channel.
  CANMotor* getMotor(size_t index);
  size_t getMotorCount() const;

  const std::string& getCanPort() const { return CAN_port_; }

  // Math utility functions
  static double deg2rad(double deg);
  static Eigen::Vector2d tb2phi(const Eigen::Vector2d &tb);
  static Eigen::Vector2d phi2tb(const Eigen::Vector2d &phi);

  int CAN_timeout_us;

private:
  NiFpga_Status& status_;
  NiFpga_Session session_;
  YAML::Node config_;

  std::string CAN_port_;

  // Owns the CANChannel only if this module is the one that constructs it
  // (is_owner_ == true). Sharers leave this null and borrow the owner's
  // channel_ pointer instead (set up in Corgi::load_config_()).
  std::unique_ptr<CANChannel> owned_channel_;

  // Persistent per-leg motor state. Lives here (not in CANChannel) so it survives
  // round-robin hand-offs unchanged - only CANChannel::motors_ (which of these
  // pointers currently occupies the shared hardware slots) changes per tick.
  std::vector<std::unique_ptr<CANMotor>> own_motors_;

  void load_config();
};

#endif
