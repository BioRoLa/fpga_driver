#include <fsm.hpp>

ModeFsm::ModeFsm(std::vector<LegModule>* _modules, std::vector<bool>* _pb_state, double* pb_v)
{
    workingMode_ = Mode::REST;
    prev_workingMode_ = Mode::REST;

    modules_list_ = _modules;
    pb_state_ = _pb_state;
    powerboard_voltage = pb_v;

    hall_calibrated = false;
    hall_calibrate_status = 0;
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


void ModeFsm::runFsm(motor_msg::MotorStateStamped& motor_fb_msg, const motor_msg::MotorCmdStamped& motor_cmd_msg)
{
    // position = P_CMD_MAX is to make sure the data received from CONFIG function code is the default one
    switch (workingMode_)
    {
        case Mode::REST: {
            if (pb_state_->at(2) == true)
            {
                publishMsg(motor_fb_msg);
                
                for (auto& mod : *modules_list_)
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
        break;

        case Mode::SET_ZERO: {
            if (pb_state_->at(2) == true)
            {
                publishMsg(motor_fb_msg);
                
                for (auto& mod : *modules_list_)
                {
                    if (mod.enable_)
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
                }
            }
        }
        break;

        case Mode::HALL_CALIBRATE: {
            int module_enabled = 0;
            int total_motors = 0;
            
            // Set all motor command to zero initially
            for (auto& mod : *modules_list_)
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

            switch (hall_calibrate_status)
            {
                case -1:{
                    switchMode(Mode::REST);
                }
                break;
            
                case 0:{
                    // check calibration finished
                    int cal_cnt = 0;
                    for (auto& mod : *modules_list_)
                    {
                        if (mod.enable_) {
                            mod.receiveFeedback();
                            
                            bool all_calibrated = true;
                            for (size_t j = 0; j < mod.getMotorCount(); j++)
                            {
                                CANMotor* motor = mod.getMotor(j);
                                if (motor) {
                                    motor->decodeFeedback();
                                    if (motor->getCalibrateFinish() != 2) {
                                        all_calibrated = false;
                                        break;
                                    }
                                }
                            }
                            
                            if (all_calibrated) cal_cnt++;
                        }
                    }
                    
                    if (cal_cnt == module_enabled && measure_offset == 0) hall_calibrate_status++;
                    else if (cal_cnt == module_enabled && measure_offset == 1) hall_calibrate_status = -1;
                }
                break;

                case 1:{
                    // set initial calibration command
                    int mod_index = 0;
                    for (auto& mod : *modules_list_)
                    {
                        if (mod.enable_)
                        {
                            // 假設每個 module 有 2 個馬達 (R 和 L)
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
                    hall_calibrate_status++;
                }
                break;

                case 2:{
                    // move to calibration position
                    int finish_cnt = 0;
                    int mod_index = 0;
                    
                    for (auto& mod : *modules_list_)
                    {
                        if (mod.enable_){
                            mod.receiveFeedback();
                            
                            for (size_t j = 0; j < mod.getMotorCount() && j < 2; j++)
                            {
                                CANMotor* motor = mod.getMotor(j);
                                if (!motor) continue;
                                
                                motor->decodeFeedback();
                                double errj = theta_error(cal_command[mod_index][j], 0);

                                if (fabs(errj) < cal_tol_)
                                {
                                    motor->setCommand(0, 0, 0, 0, 0);
                                    finish_cnt++;
                                }
                                else
                                {
                                    mod.setMode(Mode::MOTOR);
                                    cal_command[mod_index][j] += cal_dir_[mod_index][j] * cal_vel_ * dt_;
                                    motor->setCommand(cal_command[mod_index][j], 0, 50, 0, 1.5);
                                }
                            }
                            
                        }
                        mod_index++;
                    }
                    if (finish_cnt == total_motors) hall_calibrate_status++;
                }
                break;

                case 3:{
                    hall_calibrated = true;
                    hall_calibrate_status = 0;
                    switchMode(Mode::MOTOR);
                }
                break;
            }
        }
        break;

        case Mode::MOTOR: {
            /* Publish feedback data from Motors */
            publishMsg(motor_fb_msg);
            
            if (*NO_CAN_TIMEDOUT_ERROR_ && *NO_SWITCH_TIMEDOUT_ERROR_)
            {
                int index = 0;
                for (auto& mod : *modules_list_)
                {
                    if (mod.enable_)
                    {
                        // Get theta-beta command based on module index
                        Eigen::Vector2d tb_cmd;
                        float torque_r, torque_l, kp_r, kp_l, ki_r, ki_l, kd_r, kd_l;
                        
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
                        
                        // Convert theta-beta to phi coordinates
                        Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                        
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
        break;

        case Mode::CONFIG: {
            // for debug
        }
        break;
    }
}

bool ModeFsm::switchMode(Mode next_mode)
{
    int mode_switched_cnt = 0;
    int module_enabled = 0;
    bool success = false;
    Mode next_mode_switch = next_mode;

    for (auto& mod : *modules_list_)
    {
        if (mod.enable_) module_enabled++;
    }

    double time_elapsed = 0;
    while (1)
    {
        if (mode_switched_cnt == module_enabled)
        {
            prev_workingMode_ = workingMode_;
            workingMode_ = next_mode_switch;
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

        for (auto& mod : *modules_list_)
        {
            if (mod.enable_)
            {
                mod.setMode(next_mode_switch);
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
                    
                    motor->decodeFeedback();
                    
                    // SET_ZERO mode special processing
                    if (next_mode_switch == Mode::SET_ZERO)
                    {
                        float pos = motor->getPosition();
                        if (fabs(pos) > 0.01) {
                            all_motors_switched = false;
                            break;
                        }
                    }
                    else
                    {
                        if (motor->getModeState() != (uint8_t)next_mode_switch) {
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

    for (auto& mod : *modules_list_)
    {
        if (mod.enable_)
        {
            if (workingMode_ == Mode::MOTOR) {
                mod.setMode(Mode::CONTROL);
            }
            else {
                mod.setMode(Mode::CONFIG);
            }
        }
    }

    return success;
}

void ModeFsm::publishMsg(motor_msg::MotorStateStamped& motor_fb_msg)
{
    int index = 0;
    for (auto& mod : *modules_list_)
    {
        if (mod.enable_)
        {
            // 取得馬達指標
            CANMotor* motorR = mod.getMotor(0);
            CANMotor* motorL = mod.getMotor(1);
            
            // 檢查空指標
            if (!motorR || !motorL) {
                index++;
                continue;
            }
            
            // 取得馬達數據
            float pos_r = motorR->getPosition();
            float pos_l = motorL->getPosition();
            float vel_r = motorR->getVelocity();
            float vel_l = motorL->getVelocity();
            float torque_r = motorR->getTorque() * motorR->getConfig().kt_;
            float torque_l = motorL->getTorque() * motorL->getConfig().kt_;
            
            // 轉換 phi 到 theta-beta
            Eigen::Vector2d phi_(pos_r, pos_l);
            Eigen::Vector2d tb_ = phi2tb(phi_);
            
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
