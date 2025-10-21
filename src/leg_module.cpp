#include <leg_module.hpp>

LegModule::LegModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session)
{
    label_ = _label;
    config_ = _config;
    enable_ = false;

    load_config();

    CAN_tx_timedout_[0] = false;
    CAN_tx_timedout_[1] = false;
    CAN_rx_timedout_[0] = false;
    CAN_rx_timedout_[1] = false;

    CAN_mtr_timedout[0] = false;
    CAN_mtr_timedout[1] = false;

    CAN_module_timedout = false;

    txcmd_buffer_[0].position_ = 0;
    txcmd_buffer_[0].torque_ = 0;
    txcmd_buffer_[0].KP_ = motors_list_[0].kp_;
    txcmd_buffer_[0].KI_ = motors_list_[0].ki_;
    txcmd_buffer_[0].KD_ = motors_list_[0].kd_;
    txcmd_buffer_[0].KT_ = motors_list_[0].kt_;

    txcmd_buffer_[1].position_ = 0;
    txcmd_buffer_[1].torque_ = 0;
    txcmd_buffer_[1].KP_ = motors_list_[1].kp_;
    txcmd_buffer_[1].KI_ = motors_list_[1].ki_;
    txcmd_buffer_[1].KD_ = motors_list_[1].kd_;
    txcmd_buffer_[1].KT_ = motors_list_[1].kt_;

    rxcmd_buffer_[0].mode_ = Mode::REST;
    rxcmd_buffer_[0].mode_state_ = _REST_MODE;
    rxcmd_buffer_[0].position_ = 0;
    rxcmd_buffer_[0].torque_ = 0;
    rxcmd_buffer_[0].velocity_ = 0;
    rxcmd_buffer_[0].calibrate_finish_ = 0;
    rxcmd_buffer_[0].CAN_id_ = 0;
    rxcmd_buffer_[0].version_ = 0;

    rxcmd_buffer_[1].mode_ = Mode::REST;
    rxcmd_buffer_[1].mode_state_ = _REST_MODE;
    rxcmd_buffer_[1].position_ = 0;
    rxcmd_buffer_[1].torque_ = 0;
    rxcmd_buffer_[1].velocity_ = 0;
    rxcmd_buffer_[1].calibrate_finish_ = 0;
    rxcmd_buffer_[1].CAN_id_ = 0;
    rxcmd_buffer_[1].version_ = 0;

    io_ = ModuleIO(_status, _fpga_session, CAN_port_, &motors_list_);
    CAN_first_transmit_ = true;

    /* setup motors' CAN ID, port selection and timeout_us */
    io_.CAN_setup(CAN_timeout_us);
}

void LegModule::load_config()
{
    Motor motor_r;
    Motor motor_l;
    CAN_timeout_us = config_["CAN_Timeout_us"].as<int>();

    // load configuration from yaml file
    std::cout << "[ " << label_ << " Configuration ]" << std::endl;
    enable_ = config_[label_]["Enable"].as<int>();
    CAN_port_ = config_[label_]["CAN_PORT"].as<std::string>();

    // Motor F setup
    motor_r.fw_version_ = config_[label_]["Motor_R"]["FW_Version"].as<int>();
    motor_r.CAN_ID_ = config_[label_]["Motor_R"]["CAN_ID"].as<int>();
    motor_r.kp_ = config_[label_]["Motor_R"]["KP"].as<double>();
    motor_r.ki_ = config_[label_]["Motor_R"]["KI"].as<double>();
    motor_r.kd_ = config_[label_]["Motor_R"]["KD"].as<double>();
    motor_r.kt_ = config_[label_]["Motor_R"]["KT"].as<double>();
    motor_r.torque_ff_ = config_[label_]["Motor_R"]["Torque_Feedfoward"].as<double>();

    linkR_bias = config_[label_]["Motor_R"]["Calibration_Bias"].as<double>();
    motor_r.calibration_bias = 0;

    // Motor H setup
    motor_l.fw_version_ = config_[label_]["Motor_L"]["FW_Version"].as<int>();
    motor_l.CAN_ID_ = config_[label_]["Motor_L"]["CAN_ID"].as<int>();
    motor_l.kp_ = config_[label_]["Motor_L"]["KP"].as<double>();
    motor_l.ki_ = config_[label_]["Motor_L"]["KI"].as<double>();
    motor_l.kd_ = config_[label_]["Motor_L"]["KD"].as<double>();
    motor_l.kt_ = config_[label_]["Motor_L"]["KT"].as<double>();
    motor_l.torque_ff_ = config_[label_]["Motor_L"]["Torque_Feedfoward"].as<double>();

    linkL_bias = config_[label_]["Motor_L"]["Calibration_Bias"].as<double>();
    motor_l.calibration_bias = 0;

    motors_list_.push_back(motor_r);
    motors_list_.push_back(motor_l);

    std::cout << "CAN PORT: " << config_[label_]["CAN_PORT"].as<std::string>() << std::endl;

    std::cout << "Motor_R: " << std::endl;
    std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor_r.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  CAN_ID: " << std::setw(13) << motor_r.CAN_ID_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor_r.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor_r.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor_r.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motor_r.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor_r.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Bias: " << std::setw(13) << linkR_bias << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;

    std::cout << "Motor_L: " << std::endl;
    std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor_l.fw_version_ << std::endl;
    std::cout << std::setw(14) << "  CAN_ID: " << std::setw(13) << motor_l.CAN_ID_ << std::endl;
    std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor_l.kp_ << std::endl;
    std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor_l.ki_ << std::endl;
    std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor_l.kd_ << std::endl;
    std::cout << std::setw(14) << "  KT: " << std::setw(13) << motor_l.kt_ << std::endl;
    std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor_l.torque_ff_ << std::endl;
    std::cout << std::setw(14) << "  Bias: " << std::setw(13) << linkL_bias << std::endl;
    std::cout << std::setw(14) << "---------------------------" << std::endl;
}

void LegModule::CAN_timeoutCheck()
{
    CAN_rx_timedout_[0] = io_.read_rx_timeout_();
    CAN_tx_timedout_[0] = io_.read_tx_timeout_();

    CAN_mtr_timedout[0] = CAN_rx_timedout_[0] || CAN_tx_timedout_[0];
    CAN_mtr_timedout[1] = CAN_rx_timedout_[1] || CAN_tx_timedout_[1];

    CAN_module_timedout = CAN_mtr_timedout[0] || CAN_mtr_timedout[1];
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
// Eigen::Vector2d phi2tb(const Eigen::Vector2d &phi)
// {
//     Eigen::Vector2d tb;
//     /* Eigen::Matrix2d t;
//     Eigen::Vector2d b;
//     t << 1, -1, 1, 1;
//     b << deg2rad(17), 0;
//     tb = 1 / 2 * t * phi + b; */

//     std::complex<double> r_cpx(0, phi[0] + deg2rad(17));
//     std::complex<double> l_cpx(0, phi[1] - deg2rad(17));
//     std::complex<double> drl = std::exp(r_cpx) / std::exp(l_cpx);
//     double theta = std::arg(drl);
//     if (theta < 0)
//         theta += 2 * M_PI;
//     theta *= 0.5;
//     double beta = std::arg(std::exp(l_cpx)) + theta;
//     tb << theta, beta;
//     return tb;
// }
