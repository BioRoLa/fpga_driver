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

    // CAN Channel setup - using make_unique
    channel_ = std::make_unique<CANChannel>(status_, session_, CAN_port_);

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

    // 添加 Motor R 到 channel
    channel_->addMotor(motor_r.CAN_ID_, motor_r);

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

    // 添加 Motor L 到 channel
    channel_->addMotor(motor_l.CAN_ID_, motor_l);

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

    // Setup channel
    channel_->setup(CAN_timeout_us);
}

void LegModule::sendCommands()
{
    if (enable_ && channel_) {
        channel_->sendCommands();
    }
}

void LegModule::receiveFeedback()
{
    if (enable_ && channel_) {
        channel_->receiveFeedback();
    }
}

void LegModule::setMode(Mode mode)
{
    if (channel_) {
        channel_->setMode(mode);
    }
}

bool LegModule::hasTimeout() const
{
    return channel_ ? channel_->hasTimeout() : false;
}

CANMotor* LegModule::getMotor(size_t index)
{
    return channel_ ? channel_->getMotor(index) : nullptr;
}

size_t LegModule::getMotorCount() const
{
    return channel_ ? channel_->getMotorCount() : 0;
}

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

Eigen::Vector2d tb2phi(const Eigen::Vector2d &tb)
{
    Eigen::Vector2d phi;
    Eigen::Matrix2d t;
    Eigen::Vector2d b;
    t << -1, 1, 1, 1;
    b << deg2rad(17), -deg2rad(17);
    phi = t * tb + b;
    return phi;
}


Eigen::Vector2d phi2tb(const Eigen::Vector2d &phi)
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