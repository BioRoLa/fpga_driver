#include <motor_fsm.hpp>
#include "robot_fsm.hpp"
#include <atomic>

// External flag from main.cpp - indicates if a new motor command has been received
extern std::atomic<int> motor_message_updated;

MotorFSM::MotorFSM(std::vector<LegModule>& _modules, std::vector<bool>& _pb_state, double* pb_v)
    : modules_list_(_modules)
    , pb_state_(_pb_state)
    , current_mode_(FunctionMode::REST)
    , powerboard_voltage(pb_v)
    , hall_calibrated_(false)
    , hall_calibrate_status_(0)
    , robot_fsm_(nullptr)
{
    LOG_INFO << "Initialized in REST mode";
}

double theta_error(double start_theta, double goal_theta)
{
    double c1 = cos(start_theta);
    double s1 = sin(start_theta);
    double c2 = cos(goal_theta);
    double s2 = sin(goal_theta);

    double y_frame1 = 0;
    double theta_err = 0;
    y_frame1 = -s1 * c2 + c1 * s2;
    theta_err = asin(y_frame1);
    return theta_err;
}

void MotorFSM::runFsm(motor_msg::MotorStateStamped& motor_fb_msg, const motor_msg::MotorCmdStamped& motor_cmd_msg, config_msg::ConfigStamped& motor_config_reply, const config_msg::ConfigStamped& motor_config_request_data)
{
    // Publish feedback message at the beginning if not in HALL_CALIBRATE mode
    if (current_mode_ != FunctionMode::HALL_CALIBRATE)
    {
        publishMsg(motor_fb_msg);
    }
    
    // Dispatch to appropriate mode handler
    switch (current_mode_)
    {
        case FunctionMode::REST:
            handleRestMode();
            break;

        case FunctionMode::SET_ZERO:
            handleSetZeroMode();
            break;

        case FunctionMode::HALL_CALIBRATE:
            handleHallCalibrateMode();
            break;

        case FunctionMode::MOTOR:
            handleMotorMode(motor_cmd_msg);
            break;

        case FunctionMode::CONFIG:
            handleConfigMode(motor_config_reply, motor_config_request_data);
            break;
    }
}

void MotorFSM::handleRestMode()
{
    if (pb_state_.at(2) == true)
    {
        for (auto& mod : modules_list_)
        {
            if (mod.enable_)
            {
                for (size_t i = 0; i < mod.getMotorCount(); i++)
                {
                    CANMotor* motor = mod.getMotor(i);
                    if (motor) {
                        motor->setCommand(0, 0, 0, 0, 0);  // position, torque, kp, ki, kd
                    }
                }
            }
        }
    }
}

void MotorFSM::handleSetZeroMode()
{
    if (pb_state_.at(2) == true)
    {
        for (auto& mod : modules_list_)
        {
            if (mod.enable_)
            {
                for (size_t i = 0; i < mod.getMotorCount(); i++)
                {
                    CANMotor* motor = mod.getMotor(i);
                    if (motor) {
                        motor->setPositionBias(0);
                        motor->setCommand(0, 0, 0, 0, 0);
                    }
                }
            }
        }
    }
}

void MotorFSM::handleHallCalibrateMode()
{
    int module_enabled = 0;
    int total_motors = 0;
    
    // Set all motor command to zero initially
    for (auto& mod : modules_list_)
    {
        if (mod.enable_)
        {
            for (size_t j = 0; j < mod.getMotorCount(); j++)
            {
                CANMotor* motor = mod.getMotor(j);
                if (motor) {
                    motor->setCommand(0, 0, 0, 0, 0);
                    total_motors++;
                }
            }
            module_enabled++;
        }
    }

    switch (hall_calibrate_status_)
    {
        case -1:{
            switchMode(FunctionMode::REST);
        }
        break;
    
        case 0:{
            // check calibration finished
            int cal_cnt = 0;
            for (auto& mod : modules_list_)
            {
                if (mod.enable_) {
                    bool all_calibrated = true;
                    for (size_t j = 0; j < mod.getMotorCount(); j++)
                    {
                        CANMotor* motor = mod.getMotor(j);
                        if (motor) {
                            // Check hall_cal_state status
                            motor -> encodeRequestState();
                            if (motor->getHallCalibrateState() != 2) {
                                all_calibrated = false;
                                break;
                            }
                        }
                    }
                    
                    if (all_calibrated) cal_cnt++;
                }
            }
            
            if (cal_cnt == module_enabled && measure_offset_ == 0) hall_calibrate_status_++;
            else if (cal_cnt == module_enabled && measure_offset_ == 1) hall_calibrate_status_ = -1;
        }
        break;

        case 1:{
            // set initial calibration command
            int mod_index = 0;
            for (auto& mod : modules_list_)
            {
                if (mod.enable_)
                {
                    CANMotor* motorR = mod.getMotor(0);
                    CANMotor* motorL = mod.getMotor(1);
                    
                    if (motorR) {
                        motorR->setPositionBias(mod.linkR_bias);
                        cal_command[mod_index][0] = -mod.linkR_bias;
                        motorR->setCommand(cal_command[mod_index][0], 0, 0, 0, 0);
                        cal_dir_[mod_index][0] = 1;
                    }
                    
                    if (motorL) {
                        motorL->setPositionBias(mod.linkL_bias);
                        cal_command[mod_index][1] = -mod.linkL_bias;
                        motorL->setCommand(cal_command[mod_index][1], 0, 0, 0, 0);
                        cal_dir_[mod_index][1] = -1;
                    }
                    
                }
                mod_index++;
            }
            hall_calibrate_status_++;
        }
        break;

        case 2:{
            // move to calibration position
            int finish_cnt = 0;
            int mod_index = 0;
            
            for (auto& mod : modules_list_)
            {
                if (mod.enable_){
                    for (size_t j = 0; j < mod.getMotorCount() && j < 2; j++)
                    {
                        CANMotor* motor = mod.getMotor(j);
                        if (!motor) continue;
                        
                        double errj = theta_error(cal_command[mod_index][j], 0);

                        if (fabs(errj) < cal_tol_)
                        {
                            motor->setCommand(0, 0, 0, 0, 0);
                            finish_cnt++;
                        }
                        else
                        {
                            mod.setMode(FunctionMode::CONTROL);
                            cal_command[mod_index][j] += cal_dir_[mod_index][j] * cal_vel_ * dt_;
                            motor->setCommand(cal_command[mod_index][j], 0, 50, 0, 1.5);
                        }
                    }
                    
                }
                mod_index++;
            }
            if (finish_cnt == total_motors) hall_calibrate_status_++;
        }
        break;

        case 3:{
            hall_calibrated_ = true;
            hall_calibrate_status_ = 0;
            switchMode(FunctionMode::MOTOR);
        }
        break;
    }
}

void MotorFSM::handleMotorMode(const motor_msg::MotorCmdStamped& motor_cmd_msg)
{
    if (*NO_CAN_TIMEDOUT_ERROR_ && *NO_SWITCH_TIMEDOUT_ERROR_)
    {
        // Check if we should accept new commands from gRPC
        bool accept_grpc_commands = false;
        if (robot_fsm_ != nullptr)
        {
            RobotMode robot_mode = robot_fsm_->getCurrentMode();
            if (robot_mode == RobotMode::Standby)
            {
                if (motor_message_updated == 1)
                {
                    accept_grpc_commands = true;
                }
            }
        }
        
        int index = 0;
        for (auto& mod : modules_list_)
        {
            if (mod.enable_)
            {
                // Get theta-beta command based on module index
                Eigen::Vector2d tb_cmd;
                float torque_r, torque_l, kp_r, kp_l, ki_r, ki_l, kd_r, kd_l;
                
                // Only update from gRPC if in Standby or other active modes
                if (accept_grpc_commands)
                {
                
                    switch (index)
                    {
                        case 0: // module_a
                        {
                            double theta = motor_cmd_msg.module_a().theta();
                            theta = std::max(17.0*PI/180.0, std::min(160.0*PI/180.0, theta));
                            tb_cmd << theta, motor_cmd_msg.module_a().beta();
                            
                            torque_r = motor_cmd_msg.module_a().torque_r();
                            torque_l = motor_cmd_msg.module_a().torque_l();
                            kp_r = motor_cmd_msg.module_a().kp_r();
                            kp_l = motor_cmd_msg.module_a().kp_l();
                            ki_r = motor_cmd_msg.module_a().ki_r();
                            ki_l = motor_cmd_msg.module_a().ki_l();
                            kd_r = motor_cmd_msg.module_a().kd_r();
                            kd_l = motor_cmd_msg.module_a().kd_l();
                        }
                        break;
                    
                    case 1: // module_b
                    {
                        double theta = motor_cmd_msg.module_b().theta();
                        theta = std::max(17.0*PI/180.0, std::min(160.0*PI/180.0, theta));
                        tb_cmd << theta, motor_cmd_msg.module_b().beta();
                        
                        torque_r = motor_cmd_msg.module_b().torque_r();
                        torque_l = motor_cmd_msg.module_b().torque_l();
                        kp_r = motor_cmd_msg.module_b().kp_r();
                        kp_l = motor_cmd_msg.module_b().kp_l();
                        ki_r = motor_cmd_msg.module_b().ki_r();
                        ki_l = motor_cmd_msg.module_b().ki_l();
                        kd_r = motor_cmd_msg.module_b().kd_r();
                        kd_l = motor_cmd_msg.module_b().kd_l();
                    }
                    break;
                    
                    case 2: // module_c
                    {
                        double theta = motor_cmd_msg.module_c().theta();
                        theta = std::max(17.0*PI/180.0, std::min(160.0*PI/180.0, theta));
                        tb_cmd << theta, motor_cmd_msg.module_c().beta();
                        
                        torque_r = motor_cmd_msg.module_c().torque_r();
                        torque_l = motor_cmd_msg.module_c().torque_l();
                        kp_r = motor_cmd_msg.module_c().kp_r();
                        kp_l = motor_cmd_msg.module_c().kp_l();
                        ki_r = motor_cmd_msg.module_c().ki_r();
                        ki_l = motor_cmd_msg.module_c().ki_l();
                        kd_r = motor_cmd_msg.module_c().kd_r();
                        kd_l = motor_cmd_msg.module_c().kd_l();
                    }
                    break;
                    
                    case 3: // module_d
                    {
                        double theta = motor_cmd_msg.module_d().theta();
                        theta = std::max(17.0*PI/180.0, std::min(160.0*PI/180.0, theta));
                        tb_cmd << theta, motor_cmd_msg.module_d().beta();
                        
                        torque_r = motor_cmd_msg.module_d().torque_r();
                        torque_l = motor_cmd_msg.module_d().torque_l();
                        kp_r = motor_cmd_msg.module_d().kp_r();
                        kp_l = motor_cmd_msg.module_d().kp_l();
                        ki_r = motor_cmd_msg.module_d().ki_r();
                        ki_l = motor_cmd_msg.module_d().ki_l();
                        kd_r = motor_cmd_msg.module_d().kd_r();
                        kd_l = motor_cmd_msg.module_d().kd_l();
                    }
                    break;
                    
                        default:
                            index++;
                            continue;
                    }
                }  // end if (accept_grpc_commands)
                else
                {
                    // In IDLE mode: don't update from gRPC, motor will keep previous command
                    // CAN will automatically resend the last command stored in motors
                    index++;
                    continue;
                }
                
                // Convert theta-beta to phi coordinates
                Eigen::Vector2d phi_cmd = LegModule::tb2phi(tb_cmd);
                
                // Get motors and set commands
                CANMotor* motorR = mod.getMotor(0);
                CANMotor* motorL = mod.getMotor(1);
                
                if (motorR && motorL)
                {
                    // Convert torque using kt (torque constant)
                    float torque_r_motor = torque_r / motorR->getConfig().kt_;
                    float torque_l_motor = torque_l / motorL->getConfig().kt_;
                    
                    // Set commands (setCommand internally calls encodeControl)
                    motorR->setCommand(phi_cmd[0], torque_r_motor, kp_r, ki_r, kd_r);
                    motorL->setCommand(phi_cmd[1], torque_l_motor, kp_l, ki_l, kd_l);
                }

            }
            index++;
        }
    }
}

void MotorFSM::handleConfigMode(config_msg::ConfigStamped &motor_config_reply, const config_msg::ConfigStamped& motor_config_request_data)
{
    // Check if this is a new request
    if(motor_config_request_data.header().seq() != last_motor_config_seq)
    {
        // --- Logic: Process motor/config Request ---

        // Mark request as processed (Update Seq)
        last_motor_config_seq = motor_config_request_data.header().seq();
        motor_config_reply.mutable_header()->set_seq(last_motor_config_seq);

        int mod_index = (int)motor_config_request_data.module();
        int motor_index = (int)motor_config_request_data.motor();

        if (mod_index < 0 || mod_index >= modules_list_.size())
        {
            motor_config_reply.set_error_code(5); // Invalid module index
            return;
        }

        LegModule* mod = &modules_list_.at(mod_index);
        CANMotor* motor = mod->getMotor(motor_index);

        if(!motor) 
        {
            motor_config_reply.set_error_code(6); // Invalid motor index
            return;
        }

        uint8_t addr = (uint8_t)motor_config_request_data.address();

        if(motor_config_request_data.mode() == config_msg::ConfigMode::READ) 
        {
            motor->setConfigRead((ConfigType)motor_config_request_data.type(), addr);
        }
        else
        {
            if(motor_config_request_data.type() == config_msg::ConfigType::INT) 
            {
                motor->setConfigWriteInt(addr, motor_config_request_data.value_i());
            }
            else if(motor_config_request_data.type() == config_msg::ConfigType::FLOAT) 
            {
                motor->setConfigWriteFloat(addr, motor_config_request_data.value_f());
            }
        }
        mod->sendCommands();
        
        usleep(1000); // Wait for CAN response 

        mod->receiveFeedback();

        motor->decodeBasedOnMode();

        // --- Logic: Prepare motor/config Reply  ---

        motor_config_reply.set_module((config_msg::Module)mod_index);
        motor_config_reply.set_motor((config_msg::Motor)motor_index);

        const auto& motor_fb = motor->getConfigFeedback().config_fb;

        motor_config_reply.set_type((config_msg::ConfigType)motor_fb.type);
        motor_config_reply.set_error_code(motor_fb.state);
        motor_config_reply.set_address(motor_fb.target_addr);
        motor_config_reply.set_mode(motor_config_request_data.mode());

        if(motor_fb.type == ConfigType::INT) 
        {
            motor_config_reply.set_value_i(motor_fb.value.i);
            motor_config_reply.set_value_f(0); // Clear float field
        }
        else
        {
            motor_config_reply.set_value_f(motor_fb.value.f);
            motor_config_reply.set_value_i(0); // Clear int field
        }
    }
    else
    {
        motor_config_reply.set_error_code(7); // Invalid seq - duplicate request
        return;
    }
}

bool MotorFSM::switchMode(FunctionMode next_mode)
{
    int mode_switched_cnt = 0;
    int module_enabled = 0;
    bool success = false;
    FunctionMode next_mode_switch = next_mode;

    for (auto& mod : modules_list_)
    {
        if (mod.enable_) module_enabled++;
    }

    double time_elapsed = 0;
    while (1)
    {
        if (mode_switched_cnt == module_enabled)
        {
            current_mode_ = next_mode_switch;
            success = true;
            break;
        }
        else if (time_elapsed > 3)
        {
            /* Timeout */
            success = false;
            break;
        }
        else mode_switched_cnt = 0;

        for (auto& mod : modules_list_)
        {
            if (mod.enable_)
            {
                if (next_mode_switch == FunctionMode::CONFIG)
                {
                    mod.setMode(FunctionMode::REST);
                }
                else 
                {
                    mod.setMode(next_mode_switch);
                }
                
                // For SET_ZERO mode, prepare the motors before sending
                if (next_mode_switch == FunctionMode::SET_ZERO)
                {
                    for (size_t i = 0; i < mod.getMotorCount(); i++)
                    {
                        CANMotor* motor = mod.getMotor(i);
                        if (motor) {
                            motor->setPositionBias(0);
                            motor->setCommand(P_CMD_MAX, 0, 0, 0, 0);
                        }
                    }
                }
                
                mod.sendCommands();
                
                mod.receiveFeedback();
                
                bool all_motors_switched = true;
                for (size_t i = 0; i < mod.getMotorCount(); i++)
                {
                    CANMotor* motor = mod.getMotor(i);
                    if (!motor) {
                        all_motors_switched = false;
                        break;
                    }
                    
                    motor->decodeBasedOnMode();
                    
                    // SET_ZERO mode special processing
                    if (next_mode_switch == FunctionMode::SET_ZERO)
                    {
                        // Check raw position (motor should report near-zero after SET_ZERO)
                        float raw_pos = motor->getRawPosition();
                        if (fabs(raw_pos) > 0.01) {
                            all_motors_switched = false;
                            break;
                        }
                    }
                    else if (next_mode_switch == FunctionMode::CONFIG)
                    {
                        if (motor->getModeState() != (uint8_t)FunctionMode::REST) 
                        {
                            all_motors_switched = false;
                            break;
                        }
                    }
                    else
                    {
                        if (motor->getModeState() != (uint8_t)next_mode_switch) 
                        {
                            all_motors_switched = false;
                            break;
                        }
                    }
                }
                
                if (all_motors_switched) {
                    mode_switched_cnt++;
                }
            }
        }
        
        time_elapsed += 0.01;
        usleep(1e4);
    }

    for (auto& mod : modules_list_)
    {
        if (mod.enable_)
        {
            if (current_mode_ == FunctionMode::MOTOR) {
                mod.setMode(FunctionMode::CONTROL);
            }
            else if (current_mode_ == FunctionMode::CONFIG) {
                mod.setMode(FunctionMode::CONFIG);
                mod.setConfigSubMode(ConfigSubMode::CONFIG_OPERATION);
            }
            else {
                mod.setMode(FunctionMode::CONFIG);
                mod.setConfigSubMode(ConfigSubMode::REQUEST_STATE);
            }
        }
    }

    return success;
}

void MotorFSM::publishMsg(motor_msg::MotorStateStamped& motor_fb_msg)
{
    int index = 0;
    for (auto& mod : modules_list_)
    {
        if (mod.enable_)
        {
            CANMotor* motorR = mod.getMotor(0);
            CANMotor* motorL = mod.getMotor(1);
            
            if (!motorR || !motorL) {
                index++;
                continue;
            }
            
            float pos_r = motorR->getPosition();
            float pos_l = motorL->getPosition();
            float vel_r = motorR->getVelocity();
            float vel_l = motorL->getVelocity();
            float torque_r = motorR->getTorque() * motorR->getConfig().kt_;
            float torque_l = motorL->getTorque() * motorL->getConfig().kt_;
            
            Eigen::Vector2d phi_(pos_r, pos_l);
            Eigen::Vector2d tb_ = LegModule::phi2tb(phi_);
            
            switch (index)
            {
                case 0: // module_a
                {
                    motor_fb_msg.mutable_module_a()->set_velocity_r(vel_r); 
                    motor_fb_msg.mutable_module_a()->set_velocity_l(vel_l); 
                    motor_fb_msg.mutable_module_a()->set_theta(tb_[0]);
                    motor_fb_msg.mutable_module_a()->set_beta(tb_[1]);
                    motor_fb_msg.mutable_module_a()->set_torque_r(torque_r);
                    motor_fb_msg.mutable_module_a()->set_torque_l(torque_l);
                }
                break;

                case 1: // module_b
                {
                    motor_fb_msg.mutable_module_b()->set_velocity_r(vel_r); 
                    motor_fb_msg.mutable_module_b()->set_velocity_l(vel_l); 
                    motor_fb_msg.mutable_module_b()->set_theta(tb_[0]);
                    motor_fb_msg.mutable_module_b()->set_beta(tb_[1]);
                    motor_fb_msg.mutable_module_b()->set_torque_r(torque_r);
                    motor_fb_msg.mutable_module_b()->set_torque_l(torque_l);
                }
                break;

                case 2: // module_c
                {
                    motor_fb_msg.mutable_module_c()->set_velocity_r(vel_r); 
                    motor_fb_msg.mutable_module_c()->set_velocity_l(vel_l); 
                    motor_fb_msg.mutable_module_c()->set_theta(tb_[0]);
                    motor_fb_msg.mutable_module_c()->set_beta(tb_[1]);
                    motor_fb_msg.mutable_module_c()->set_torque_r(torque_r);
                    motor_fb_msg.mutable_module_c()->set_torque_l(torque_l);
                }
                break;

                case 3: // module_d
                {
                    motor_fb_msg.mutable_module_d()->set_velocity_r(vel_r); 
                    motor_fb_msg.mutable_module_d()->set_velocity_l(vel_l); 
                    motor_fb_msg.mutable_module_d()->set_theta(tb_[0]);
                    motor_fb_msg.mutable_module_d()->set_beta(tb_[1]);
                    motor_fb_msg.mutable_module_d()->set_torque_r(torque_r);
                    motor_fb_msg.mutable_module_d()->set_torque_l(torque_l);
                }
                break;
            }
        }
        else
        {
            switch (index)
            {
                case 0: // module_a
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_a()->set_velocity_r(0); // phi R
                    motor_fb_msg.mutable_module_a()->set_velocity_l(0); // phi L
                    motor_fb_msg.mutable_module_a()->set_theta(0);     // theta
                    motor_fb_msg.mutable_module_a()->set_beta(0);      // beta
                    motor_fb_msg.mutable_module_a()->set_torque_r(0); //torque R
                    motor_fb_msg.mutable_module_a()->set_torque_l(0); //torque L
                }
                break;

                case 1: // module_b
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_b()->set_velocity_r(0); // phi R
                    motor_fb_msg.mutable_module_b()->set_velocity_l(0); // phi L
                    motor_fb_msg.mutable_module_b()->set_theta(0);     // theta
                    motor_fb_msg.mutable_module_b()->set_beta(0);      // beta
                    motor_fb_msg.mutable_module_b()->set_torque_r(0); //torque R
                    motor_fb_msg.mutable_module_b()->set_torque_l(0); //torque L
                }
                break;

                case 2: // module_c
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_c()->set_velocity_r(0); // phi R
                    motor_fb_msg.mutable_module_c()->set_velocity_l(0); // phi L
                    motor_fb_msg.mutable_module_c()->set_theta(0);     // theta
                    motor_fb_msg.mutable_module_c()->set_beta(0);      // beta
                    motor_fb_msg.mutable_module_c()->set_torque_r(0); //torque R
                    motor_fb_msg.mutable_module_c()->set_torque_l(0); //torque L
                }
                break;

                case 3: // module_d
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_d()->set_velocity_r(0); // phi R
                    motor_fb_msg.mutable_module_d()->set_velocity_l(0); // phi L
                    motor_fb_msg.mutable_module_d()->set_theta(0);     // theta
                    motor_fb_msg.mutable_module_d()->set_beta(0);      // beta
                    motor_fb_msg.mutable_module_d()->set_torque_r(0); //torque R
                    motor_fb_msg.mutable_module_d()->set_torque_l(0); //torque L
                }
                break;
            }
        }
        index++;
    }

}

