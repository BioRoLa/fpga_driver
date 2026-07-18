#include <leg_module.hpp>

LegModule::LegModule(const std::string& label, const YAML::Node& config,
                     NiFpga_Status& status, NiFpga_Session& session)
    : label_(label)
    , config_(config)
    , status_(status)
    , session_(session)
    , enable_(false)
    , linkR_bias(0.0)
    , linkL_bias(0.0)
{
    load_config();
}

void LegModule::load_config()
{
    CAN_timeout_us = config_["CAN_Timeout_us"].as<int>();

    // load configuration from yaml file
    std::cout << "[ " << label_ << " Configuration ]" << std::endl;
    enable_ = config_[label_]["Enable"].as<int>();
    CAN_port_ = config_[label_]["CAN_PORT"].as<std::string>();

    std::cout << "CAN PORT: " << CAN_port_ << std::endl;
    std::cout << "Enabled: " << (enable_ ? "Yes" : "No") << std::endl;
    
    if (!enable_) {
        std::cout << "---------------------------" << std::endl;
        return;
    }

    // CAN Channel setup. Every module constructs its own channel here regardless of
    // whether it will end up sharing one (owner/sharer roles are only known after all
    // LegModules are built - see Corgi::load_config_()'s port-grouping pass in
    // main.cpp). If this module turns out to be a sharer, its channel_ pointer gets
    // reassigned to the group owner's channel afterwards; this module's own
    // owned_channel_ then simply goes unused (harmless - it targets the same FPGA
    // resource addresses as the owner's, but nothing calls through it again).
    owned_channel_ = std::make_unique<CANChannel>(status_, session_, CAN_port_);
    channel_ = owned_channel_.get();

    // Motor R setup
    Motor motor_r;
    motor_r.fw_version_ = config_[label_]["Motor_R"]["FW_Version"].as<int>();
    motor_r.CAN_ID_ = config_[label_]["Motor_R"]["CAN_ID"].as<int>();
    motor_r.kp_ = config_[label_]["Motor_R"]["KP"].as<double>();
    motor_r.ki_ = config_[label_]["Motor_R"]["KI"].as<double>();
    motor_r.kd_ = config_[label_]["Motor_R"]["KD"].as<double>();
    motor_r.kt_ = config_[label_]["Motor_R"]["KT"].as<double>();
    motor_r.torque_ff_ = config_[label_]["Motor_R"]["Torque_Feedfoward"].as<double>();
    linkR_bias = config_[label_]["Motor_R"]["Calibration_Bias"].as<double>();
    motor_r.calibration_bias = linkR_bias;

    // Motor R 由 LegModule 自己持有（持續存在，不受 round-robin 換手影響）
    own_motors_.push_back(std::make_unique<CANMotor>(motor_r.CAN_ID_, motor_r));

    std::cout << "Motor_R: " << std::endl;
    std::cout << "  FW_Version: " << motor_r.fw_version_ << std::endl;
    std::cout << "  CAN_ID: " << motor_r.CAN_ID_ << std::endl;
    std::cout << "  KP: " << motor_r.kp_ << std::endl;
    std::cout << "  KI: " << motor_r.ki_ << std::endl;
    std::cout << "  KD: " << motor_r.kd_ << std::endl;
    std::cout << "  KT: " << motor_r.kt_ << std::endl;
    std::cout << "  Torque_ff: " << motor_r.torque_ff_ << std::endl;
    std::cout << "  Bias: " << linkR_bias << std::endl;
    std::cout << "---------------------------" << std::endl;

    // Motor L setup
    Motor motor_l;
    motor_l.fw_version_ = config_[label_]["Motor_L"]["FW_Version"].as<int>();
    motor_l.CAN_ID_ = config_[label_]["Motor_L"]["CAN_ID"].as<int>();
    motor_l.kp_ = config_[label_]["Motor_L"]["KP"].as<double>();
    motor_l.ki_ = config_[label_]["Motor_L"]["KI"].as<double>();
    motor_l.kd_ = config_[label_]["Motor_L"]["KD"].as<double>();
    motor_l.kt_ = config_[label_]["Motor_L"]["KT"].as<double>();
    motor_l.torque_ff_ = config_[label_]["Motor_L"]["Torque_Feedfoward"].as<double>();
    linkL_bias = config_[label_]["Motor_L"]["Calibration_Bias"].as<double>();
    motor_l.calibration_bias = linkL_bias;

    // Motor L 由 LegModule 自己持有（持續存在，不受 round-robin 換手影響）
    own_motors_.push_back(std::make_unique<CANMotor>(motor_l.CAN_ID_, motor_l));

    std::cout << "Motor_L: " << std::endl;
    std::cout << "  FW_Version: " << motor_l.fw_version_ << std::endl;
    std::cout << "  CAN_ID: " << motor_l.CAN_ID_ << std::endl;
    std::cout << "  KP: " << motor_l.kp_ << std::endl;
    std::cout << "  KI: " << motor_l.ki_ << std::endl;
    std::cout << "  KD: " << motor_l.kd_ << std::endl;
    std::cout << "  KT: " << motor_l.kt_ << std::endl;
    std::cout << "  Torque_ff: " << motor_l.torque_ff_ << std::endl;
    std::cout << "  Bias: " << linkL_bias << std::endl;
    std::cout << "---------------------------" << std::endl;

    // Motor H setup
    Motor motor_h;
    motor_h.fw_version_ = config_[label_]["Motor_H"]["FW_Version"].as<int>();
    motor_h.CAN_ID_ = config_[label_]["Motor_H"]["CAN_ID"].as<int>();
    motor_h.kp_ = config_[label_]["Motor_H"]["KP"].as<double>();
    motor_h.ki_ = config_[label_]["Motor_H"]["KI"].as<double>();
    motor_h.kd_ = config_[label_]["Motor_H"]["KD"].as<double>();
    motor_h.kt_ = config_[label_]["Motor_H"]["KT"].as<double>();
    motor_h.torque_ff_ = config_[label_]["Motor_H"]["Torque_Feedfoward"].as<double>();
    linkH_bias = config_[label_]["Motor_H"]["Calibration_Bias"].as<double>();
    motor_h.calibration_bias = linkH_bias;

    // Motor H 由 LegModule 自己持有（持續存在，不受 round-robin 換手影響）
    own_motors_.push_back(std::make_unique<CANMotor>(motor_h.CAN_ID_, motor_h));

    std::cout << "Motor_L: " << std::endl;
    std::cout << "  FW_Version: " << motor_l.fw_version_ << std::endl;
    std::cout << "  CAN_ID: " << motor_l.CAN_ID_ << std::endl;
    std::cout << "  KP: " << motor_l.kp_ << std::endl;
    std::cout << "  KI: " << motor_l.ki_ << std::endl;
    std::cout << "  KD: " << motor_l.kd_ << std::endl;
    std::cout << "  KT: " << motor_l.kt_ << std::endl;
    std::cout << "  Torque_ff: " << motor_l.torque_ff_ << std::endl;
    std::cout << "  Bias: " << linkL_bias << std::endl;
    std::cout << "---------------------------" << std::endl;
    
    // Initial slot binding: attach this module's own 3 motors to its own channel.
    // If Corgi::load_config_()'s port-grouping pass later determines this module
    // shares its port with another, canLoop_()'s round-robin gate will re-run
    // attachMotorGroup() with the appropriate motor set on every hand-off; this
    // initial call just brings the channel up in a known-good state immediately.
    std::vector<CANMotor*> initial_group;
    for (auto& m : own_motors_) initial_group.push_back(m.get());
    channel_->attachMotorGroup(initial_group);

    // Setup channel (RX timeout only - see CANChannel::setup())
    channel_->setup(CAN_timeout_us);
}

namespace {
std::vector<CANMotor*> toPointers(std::vector<std::unique_ptr<CANMotor>>& motors)
{
    std::vector<CANMotor*> ptrs;
    ptrs.reserve(motors.size());
    for (auto& m : motors) ptrs.push_back(m.get());
    return ptrs;
}
}  // namespace

void LegModule::sendCommands()
{
    if (!enable_ || !channel_) return;

    // Always (re)bind this module's own (persistent) motors into the channel's
    // hardware slots before sending - pushes both the CAN ID and FC/mode
    // registers, since attachMotorGroup() reads FunctionMode straight off each
    // CANMotor. This is required even for group_size_ == 1 (exclusive channel):
    // setMode()/setMotorMode() only update own_motors_ state now (see below),
    // they no longer write hardware directly, so this is the only place that
    // FC register actually gets pushed. For group_size_ == 1 the channel is
    // already bound to these exact motors, so this is a cheap, idempotent
    // re-write, not a hand-off.
    channel_->attachMotorGroup(toPointers(own_motors_));

    channel_->sendCommands();

    if (group_size_ > 1) {
        // Block until this module's own transmit finishes before returning control
        // to the caller. Without this, a second module sharing the same physical
        // channel (e.g. calibration's start_hall_stage(), which calls sendCommands()
        // on every enabled LegModule back-to-back within a single mainLoop tick)
        // could re-attach and re-trigger transmit_ while this request is still
        // in flight.
        channel_->waitForTransmitComplete();
    }
}

void LegModule::receiveFeedback()
{
    if (!enable_ || !channel_) return;
    // No attach needed here: for group_size_ == 1 the channel is permanently
    // bound to this module's own motors. For group_size_ > 1, sendCommands()
    // (above) already attached + waited for completion immediately before this
    // call in every code path that pairs the two (canLoop_, and every direct
    // caller in motor_fsm.cpp - confirmed by inspection), so the channel is
    // still bound to this module's own motors when we read.
    channel_->receiveFeedback();
}

void LegModule::setMode(FunctionMode mode)
{
    // Update this module's own persistent motor state only. The FC/mode register
    // is not written to hardware here - it is pushed the next time this module's
    // motors are attached to a channel, inside sendCommands(). This avoids writing
    // FC registers for a shared channel while it may currently be bound to a
    // different module's motors.
    for (auto& m : own_motors_) m->setMode(mode);
}

void LegModule::setMotorMode(size_t index, FunctionMode mode)
{
    if (index < own_motors_.size()) {
        own_motors_[index]->setMode(mode);
    }
}

void LegModule::setConfigSubMode(ConfigSubMode sub_mode)
{
    for (auto& m : own_motors_) m->setConfigSubMode(sub_mode);
}

void LegModule::updateTimeoutDebounce(uint32_t loop_period_us)
{
    if (channel_) {
        channel_->updateTimeoutDebounce(loop_period_us);
    }
}

bool LegModule::hasTimeout() const
{
    return channel_ ? channel_->hasTimeout() : false;
}

CANMotor* LegModule::getMotor(size_t index)
{
    return index < own_motors_.size() ? own_motors_[index].get() : nullptr;
}

size_t LegModule::getMotorCount() const
{
    return own_motors_.size();
}

double LegModule::deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

Eigen::Vector2d LegModule::tb2phi(const Eigen::Vector2d &tb)
{
    Eigen::Vector2d phi;
    Eigen::Matrix2d t;
    Eigen::Vector2d b;
    t << -1, 1, 1, 1;
    b << deg2rad(17), -deg2rad(17);
    phi = t * tb + b;
    return phi;
}

Eigen::Vector2d LegModule::phi2tb(const Eigen::Vector2d &phi)
{
    Eigen::Vector2d tb;
    Eigen::Matrix2d t;
    Eigen::Vector2d b;
    t << -1, 1, 1, 1;
    b << deg2rad(17), -deg2rad(17);
    
    // 計算 tb = t.inverse() * (phi - b)
    tb = t.inverse() * (phi - b);

    return tb;
}
