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

void ModeFsm::runFsm(motor_msg::MotorStateStamped& motor_fb_msg, const motor_msg::MotorCmdStamped& motor_cmd_msg)
{
   switch (workingMode_)
    {
        case Mode::REST: {
            if (pb_state_->at(2) == true)
            {
                publishMsg(motor_fb_msg);
                for (auto& mod : *modules_list_)
                {
                    int index = 0;
                    if (mod.enable_)
                    {
                        mod.txdata_buffer_[0].position_ = 0;
                        mod.txdata_buffer_[0].torque_ = 0;
                        mod.txdata_buffer_[0].KP_ = 0;
                        mod.txdata_buffer_[0].KI_ = 0;
                        mod.txdata_buffer_[0].KD_ = 0;
                        mod.txdata_buffer_[1].position_ = 0;
                        mod.txdata_buffer_[1].torque_ = 0;
                        mod.txdata_buffer_[1].KP_ = 0;
                        mod.txdata_buffer_[1].KI_ = 0;
                        mod.txdata_buffer_[1].KD_ = 0;
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
                        mod.txdata_buffer_[0].position_ = 0;
                        mod.txdata_buffer_[0].torque_ = 0;
                        mod.txdata_buffer_[0].KP_ = 0;
                        mod.txdata_buffer_[0].KI_ = 0;
                        mod.txdata_buffer_[0].KD_ = 0;
                        mod.txdata_buffer_[1].position_ = 0;
                        mod.txdata_buffer_[1].torque_ = 0;
                        mod.txdata_buffer_[1].KP_ = 0;
                        mod.txdata_buffer_[1].KI_ = 0;
                        mod.txdata_buffer_[1].KD_ = 0;
                    }
                }
            }
        }
        break;

        case Mode::HALL_CALIBRATE: {
            int power_off = 0;
            for (int i = 0; i < 12; i++)
            {
                double v = *(powerboard_voltage + i);
                if (v < 45)
                {
                    power_off = 1;
                }
            }

            int module_enabled = 0;
            for (int i = 0; i < 4; i++)
            {
                if (modules_list_->at(i).enable_)
                {
                    modules_list_->at(i).txdata_buffer_[0].KP_ = 50;
                    modules_list_->at(i).txdata_buffer_[0].KI_ = 0;
                    modules_list_->at(i).txdata_buffer_[0].KD_ = 1.5;
                    modules_list_->at(i).txdata_buffer_[1].KP_ = 50;
                    modules_list_->at(i).txdata_buffer_[1].KI_ = 0;
                    modules_list_->at(i).txdata_buffer_[1].KD_ = 1.5;
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
                    int cal_cnt = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        if (modules_list_->at(i).enable_)
                        {
                            if (modules_list_->at(i).rxdata_buffer_[0].calibrate_finish_ == 2 &&
                                modules_list_->at(i).rxdata_buffer_[1].calibrate_finish_ == 2)
                                cal_cnt++;
                        }
                    }
                    if (cal_cnt == module_enabled && measure_offset == 0)
                        hall_calibrate_status++;
                    else if (cal_cnt == module_enabled && measure_offset == 1)
                        hall_calibrate_status = -1;
                }
                break;

                case 1:{
                    for (int i = 0; i < 4; i++)
                    {
                        if (modules_list_->at(i).enable_)
                        {
                            modules_list_->at(i).CAN_rx_timedout_[0] = false;
                            modules_list_->at(i).CAN_rx_timedout_[1] = false;
                            modules_list_->at(i).CAN_tx_timedout_[0] = false;
                            modules_list_->at(i).CAN_tx_timedout_[1] = false;

                            modules_list_->at(i).io_.motorR_bias = modules_list_->at(i).linkR_bias;
                            modules_list_->at(i).io_.motorL_bias = modules_list_->at(i).linkL_bias;

                            cal_command[i][0] =
                                modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias;
                            modules_list_->at(i).txdata_buffer_[0].position_ =
                                modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias;

                            if (theta_error(modules_list_->at(i).rxdata_buffer_[0].position_ - modules_list_->at(i).linkR_bias,
                                            0) > 0)
                                cal_dir_[i][0] = 1;
                            else
                                cal_dir_[i][0] = -1;

                            cal_command[i][1] =
                                modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias;
                            modules_list_->at(i).txdata_buffer_[1].position_ =
                                modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias;
                            if (theta_error(modules_list_->at(i).rxdata_buffer_[1].position_ - modules_list_->at(i).linkL_bias,
                                            0) > 0)
                                cal_dir_[i][1] = 1;
                            else
                                cal_dir_[i][1] = -1;
                        }
                    }
                    hall_calibrate_status++;
                }
                break;

                case 2:{
                    int finish_cnt = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            double errj = 0;
                            errj = theta_error(cal_command[i][j], 0);
                            modules_list_->at(i).txdata_buffer_[j].position_ = 0;

                            if (fabs(errj) < cal_tol_)
                            {
                                modules_list_->at(i).txdata_buffer_[j].position_ = 0;
                                finish_cnt++;
                            }
                            else
                            {
                                cal_command[i][j] += cal_dir_[i][j] * cal_vel_ * dt_;

                                modules_list_->at(i).txdata_buffer_[j].position_ = cal_command[i][j];
                                modules_list_->at(i).txdata_buffer_[j].torque_ = 0;
                                modules_list_->at(i).txdata_buffer_[j].KP_ = 50;
                                modules_list_->at(i).txdata_buffer_[j].KI_ = 0;
                                modules_list_->at(i).txdata_buffer_[j].KD_ = 1.5;
                            }
                        }
                    }
                    if (finish_cnt == 2 * module_enabled)
                        hall_calibrate_status++;
                }
                break;

                case 3:{
                    hall_calibrated = true;
                    switchMode(Mode::MOTOR);
                }
                break;
            }            
        }
        break;

        case Mode::MOTOR: {
            /* Pubish feedback data from Motors */
            publishMsg(motor_fb_msg);
            int index = 0;
            // 0 -> a
            // 1 -> b
            // 2 -> c
            // 3 -> d
            for (auto& mod : *modules_list_)
            {
                if (mod.enable_)
                {
                    /* Subscribe command from other nodes */
                    // initialize message
                    // update
                    if (*NO_CAN_TIMEDOUT_ERROR_ && *NO_SWITCH_TIMEDOUT_ERROR_ )
                    {
                        switch (index)
                        {
                            case 0:
                            {
                                Eigen::Vector2d tb_cmd;
                                tb_cmd<< motor_cmd_msg.module_a().theta(), motor_cmd_msg.module_a().beta();                            
                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_a().torque_r();
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_a().torque_l();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_a().kp();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_a().ki();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_a().kd();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_a().kp();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_a().ki();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_a().kd();
                            }
                            break;

                            case 1:
                            {
                                Eigen::Vector2d tb_cmd;
                                tb_cmd<< motor_cmd_msg.module_b().theta(), motor_cmd_msg.module_b().beta();                            
                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_b().torque_r();
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_b().torque_l();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_b().kp();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_b().ki();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_b().kd();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_b().kp();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_b().ki();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_b().kd();
                            }
                            break;

                            case 2:
                            {
                                Eigen::Vector2d tb_cmd;
                                tb_cmd<< motor_cmd_msg.module_c().theta(), motor_cmd_msg.module_c().beta();                            
                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_c().torque_r();
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_c().torque_l();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_c().kp();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_c().ki();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_c().kd();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_c().kp();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_c().ki();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_c().kd();
                            }
                            break;

                            case 3:
                            {
                                Eigen::Vector2d tb_cmd;
                                tb_cmd<< motor_cmd_msg.module_d().theta(), motor_cmd_msg.module_d().beta();                            
                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_d().torque_r();
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_d().torque_l();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_d().kp();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_d().ki();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_d().kd();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_d().kp();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_d().ki();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_d().kd();
                            }
                            break;
                        }
                    }
                }
                index++;
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

    for (int i = 0; i < 4; i++)
    {
        if (modules_list_->at(i).enable_)
        {
            module_enabled++;
        }
    }

    if (next_mode == Mode::HALL_CALIBRATE)
    {
        // Skip hall calibration if the modules have been calibrated
        if (hall_calibrated)
            next_mode_switch = workingMode_;
    }
    

    double time_elapsed = 0;
    while (1)
    {
        if (mode_switched_cnt == module_enabled)
        {
            success = true;
            break;
        }
        else if (time_elapsed > 3)
        {
            // time_elapsed > 1
            /* Timeout */
            success = false;
            break;
        }
        else
            mode_switched_cnt = 0;

        for (int i = 0; i < 4; i++)
        {
            if (modules_list_->at(i).enable_)
            {
                // std::cout<< modules_list_->at(i).rxdata_buffer_[0].CAN_id_<< std::endl;
                modules_list_->at(i).io_.CAN_set_mode(next_mode_switch);

                // modules_list_->at(i).io_.CAN_recieve_feedback(&modules_list_->at(i).rxdata_buffer_[0],
                //                                               &modules_list_->at(i).rxdata_buffer_[1]);
                important_message("here")
                if (modules_list_->at(i).io_.read_CAN_success_())
                {
                    mode_switched_cnt++;
                }
            }
        }
        
        time_elapsed += 0.01;
        usleep(1e4);
    }

    prev_workingMode_ = workingMode_;
    workingMode_ = next_mode_switch;

    return success;
}

void ModeFsm::publishMsg(motor_msg::MotorStateStamped& motor_fb_msg)
{
    int index = 0;
    for (auto& mod : *modules_list_)
    {
        if (mod.enable_)
        {
            switch (index)
            {
                case 0: // module_a
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_a()->set_velocity_r(mod.rxdata_buffer_[0].velocity_); 
                    motor_fb_msg.mutable_module_a()->set_velocity_l(mod.rxdata_buffer_[1].velocity_); 
                    Eigen::Vector2d phi_(mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
                    Eigen::Vector2d tb_ = phi2tb(phi_);
                    motor_fb_msg.mutable_module_a()->set_theta(tb_[0]);  // theta
                    motor_fb_msg.mutable_module_a()->set_beta(tb_[1]);   // beta
                    motor_fb_msg.mutable_module_a()->set_torque_r(mod.rxdata_buffer_[0].torque_); //torque R
                    motor_fb_msg.mutable_module_a()->set_torque_l(mod.rxdata_buffer_[1].torque_); //torque L
                }
                break;

                case 1: // module_b
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_b()->set_velocity_r(mod.rxdata_buffer_[0].velocity_); 
                    motor_fb_msg.mutable_module_b()->set_velocity_l(mod.rxdata_buffer_[1].velocity_); 
                    Eigen::Vector2d phi_(mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
                    Eigen::Vector2d tb_ = phi2tb(phi_);
                    motor_fb_msg.mutable_module_b()->set_theta(tb_[0]);  // theta
                    motor_fb_msg.mutable_module_b()->set_beta(tb_[1]);   // beta
                    motor_fb_msg.mutable_module_b()->set_torque_r(mod.rxdata_buffer_[0].torque_); //torque R
                    motor_fb_msg.mutable_module_b()->set_torque_l(mod.rxdata_buffer_[1].torque_); //torque L
                }
                break;

                case 2: // module_c
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_c()->set_velocity_r(mod.rxdata_buffer_[0].velocity_); 
                    motor_fb_msg.mutable_module_c()->set_velocity_l(mod.rxdata_buffer_[1].velocity_); 
                    Eigen::Vector2d phi_(mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
                    Eigen::Vector2d tb_ = phi2tb(phi_);
                    motor_fb_msg.mutable_module_c()->set_theta(tb_[0]);  // theta
                    motor_fb_msg.mutable_module_c()->set_beta(tb_[1]);   // beta
                    motor_fb_msg.mutable_module_c()->set_torque_r(mod.rxdata_buffer_[0].torque_); //torque R
                    motor_fb_msg.mutable_module_c()->set_torque_l(mod.rxdata_buffer_[1].torque_); //torque L
                }
                break;

                case 3: // module_d
                {
                    /* Publish feedback data from Motors */
                    motor_fb_msg.mutable_module_d()->set_velocity_r(mod.rxdata_buffer_[0].velocity_); 
                    motor_fb_msg.mutable_module_d()->set_velocity_l(mod.rxdata_buffer_[1].velocity_); 
                    Eigen::Vector2d phi_(mod.rxdata_buffer_[0].position_, mod.rxdata_buffer_[1].position_);
                    Eigen::Vector2d tb_ = phi2tb(phi_);
                    motor_fb_msg.mutable_module_d()->set_theta(tb_[0]);  // theta
                    motor_fb_msg.mutable_module_d()->set_beta(tb_[1]);   // beta
                    motor_fb_msg.mutable_module_d()->set_torque_r(mod.rxdata_buffer_[0].torque_); //torque R
                    motor_fb_msg.mutable_module_d()->set_torque_l(mod.rxdata_buffer_[1].torque_); //torque L
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
