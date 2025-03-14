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
                for (int i = 0; i < 4; i++)
                {
                    if (modules_list_->at(i).enable_)
                    {
                        modules_list_->at(i).io_.motorR_bias = 0;
                        modules_list_->at(i).io_.motorL_bias = 0;
                    }
                }

                publishMsg(motor_fb_msg);
                for (auto& mod : *modules_list_)
                {
                    if (mod.enable_)
                    {
                        mod.txdata_buffer_[0].position_ = P_CMD_MAX;
                        mod.txdata_buffer_[0].torque_ = 0;
                        mod.txdata_buffer_[0].KP_ = 0;
                        mod.txdata_buffer_[0].KI_ = 0;
                        mod.txdata_buffer_[0].KD_ = 0;
                        
                        mod.txdata_buffer_[1].position_ = P_CMD_MAX;
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
            int module_enabled = 0;
            for (int i = 0; i < 4; i++)
            {
                if (modules_list_->at(i).enable_)
                {
                    modules_list_->at(i).txdata_buffer_[0].position_ = 0;
                    modules_list_->at(i).txdata_buffer_[0].torque_ = 0;
                    modules_list_->at(i).txdata_buffer_[0].KP_ = 0;
                    modules_list_->at(i).txdata_buffer_[0].KI_ = 0;
                    modules_list_->at(i).txdata_buffer_[0].KD_ = 0;

                    modules_list_->at(i).txdata_buffer_[1].position_ = 0;
                    modules_list_->at(i).txdata_buffer_[1].torque_ = 0;
                    modules_list_->at(i).txdata_buffer_[1].KP_ = 0;
                    modules_list_->at(i).txdata_buffer_[1].KI_ = 0;
                    modules_list_->at(i).txdata_buffer_[1].KD_ = 0;
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
                        if (modules_list_->at(i).enable_ && modules_list_->at(i).rxdata_buffer_[0].calibrate_finish_ == 2 && modules_list_->at(i).rxdata_buffer_[1].calibrate_finish_ == 2) cal_cnt++;
                    }
                    if (cal_cnt == module_enabled && measure_offset == 0) hall_calibrate_status++;
                    else if (cal_cnt == module_enabled && measure_offset == 1) hall_calibrate_status = -1;
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

                            cal_command[i][0] = - modules_list_->at(i).linkR_bias;
                            modules_list_->at(i).txdata_buffer_[0].position_ = - modules_list_->at(i).linkR_bias;
                            cal_dir_[i][0] = 1;

                            cal_command[i][1] = - modules_list_->at(i).linkL_bias;
                            modules_list_->at(i).txdata_buffer_[1].position_ = - modules_list_->at(i).linkL_bias;
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
                        if (modules_list_->at(i).enable_){
                            for (int j = 0; j < 2; j++)
                            {
                                double errj = 0;
                                errj = theta_error(cal_command[i][j], 0);

                                if (fabs(errj) < cal_tol_)
                                {
                                    modules_list_->at(i).txdata_buffer_[j].position_ = 0;
                                    modules_list_->at(i).txdata_buffer_[j].torque_ = 0;
                                    modules_list_->at(i).txdata_buffer_[j].KP_ = 0;
                                    modules_list_->at(i).txdata_buffer_[j].KI_ = 0;
                                    modules_list_->at(i).txdata_buffer_[j].KD_ = 0;
                                    finish_cnt++;
                                }
                                else
                                {
                                    modules_list_->at(i).io_.write_CAN_id_fc_((int)Mode::CONTROL, (int)Mode::CONTROL);
                                    cal_command[i][j] += cal_dir_[i][j] * cal_vel_ * dt_;
                                    modules_list_->at(i).txdata_buffer_[j].position_ = cal_command[i][j];
                                    modules_list_->at(i).txdata_buffer_[j].torque_ = 0;
                                    modules_list_->at(i).txdata_buffer_[j].KP_ = 50;
                                    modules_list_->at(i).txdata_buffer_[j].KI_ = 0;
                                    modules_list_->at(i).txdata_buffer_[j].KD_ = 1.5;
                                }
                            }
                        }
                    }
                    if (finish_cnt == 2 * module_enabled) hall_calibrate_status++;
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
            /* Pubish feedback data from Motors */
            publishMsg(motor_fb_msg);
            int index = 0;
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
                                if(motor_cmd_msg.module_a().theta()<17*PI/180){
                                    tb_cmd<< 17*PI/180, motor_cmd_msg.module_a().beta();
                                }
                                else if(motor_cmd_msg.module_a().theta()>160*PI/180){
                                    tb_cmd<< 160*PI/180, motor_cmd_msg.module_a().beta();
                                }
                                else{
                                    tb_cmd<< motor_cmd_msg.module_a().theta(), motor_cmd_msg.module_a().beta();                            
                                }
                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);

                                // test data
                                // Eigen::Vector2d phi_cmd(1, 1);
                                // mod.txdata_buffer_[0].KP_ = 5;
                                // mod.txdata_buffer_[0].KI_ = 0;
                                // mod.txdata_buffer_[0].KD_ = 1;
                                // mod.txdata_buffer_[1].KP_ = 5;
                                // mod.txdata_buffer_[1].KI_ = 0;
                                // mod.txdata_buffer_[1].KD_ = 1;
                                
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_a().torque_r();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_a().kp_r();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_a().ki_r();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_a().kd_r();

                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_a().torque_l();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_a().kp_l();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_a().ki_l();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_a().kd_l();
                            }
                            break;

                            case 1:
                            {
                                Eigen::Vector2d tb_cmd;

                                if(motor_cmd_msg.module_a().theta()<17*PI/180){
                                    tb_cmd<< 17*PI/180, motor_cmd_msg.module_b().beta();
                                }
                                else if(motor_cmd_msg.module_a().theta()>160*PI/180){
                                    tb_cmd<< 160*PI/180, motor_cmd_msg.module_b().beta();
                                }
                                else{
                                    tb_cmd<< motor_cmd_msg.module_b().theta(), motor_cmd_msg.module_b().beta();                         
                                }

                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_b().torque_r();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_b().kp_r();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_b().ki_r();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_b().kd_r();

                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_b().torque_l();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_b().kp_l();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_b().ki_l();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_b().kd_l();
                            }
                            break;

                            case 2:
                            {
                                Eigen::Vector2d tb_cmd;
                                if(motor_cmd_msg.module_a().theta()<17*PI/180){
                                    tb_cmd<< 17*PI/180, motor_cmd_msg.module_c().beta();
                                }
                                else if(motor_cmd_msg.module_a().theta()>160*PI/180){
                                    tb_cmd<< 160*PI/180, motor_cmd_msg.module_c().beta();
                                }
                                else{
                                    tb_cmd<< motor_cmd_msg.module_c().theta(), motor_cmd_msg.module_c().beta();                         
                                }                           
                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_c().torque_r();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_c().kp_r();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_c().ki_r();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_c().kd_r();

                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_c().torque_l();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_c().kp_l();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_c().ki_l();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_c().kd_l();
                            }
                            break;

                            case 3:
                            {
                                Eigen::Vector2d tb_cmd;

                                if(motor_cmd_msg.module_a().theta()<17*PI/180){
                                    tb_cmd<< 17*PI/180, motor_cmd_msg.module_d().beta();
                                }
                                else if(motor_cmd_msg.module_a().theta()>160*PI/180){
                                    tb_cmd<< 160*PI/180, motor_cmd_msg.module_d().beta();
                                }
                                else{
                                    tb_cmd<< motor_cmd_msg.module_d().theta(), motor_cmd_msg.module_d().beta();                         
                                }
                         
                                Eigen::Vector2d phi_cmd = tb2phi(tb_cmd);
                                mod.txdata_buffer_[0].position_ = phi_cmd[0];
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_d().torque_r();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_d().kp_r();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_d().ki_r();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_d().kd_r();

                                mod.txdata_buffer_[1].position_ = phi_cmd[1];
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_d().torque_l();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_d().kp_l();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_d().ki_l();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_d().kd_l();
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
        if (modules_list_->at(i).enable_) module_enabled++;
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
        else  mode_switched_cnt = 0;

        for (int i = 0; i < 4; i++)
        {
            if (modules_list_->at(i).enable_)
            {
                modules_list_->at(i).io_.write_CAN_id_fc_((int)next_mode_switch, (int)next_mode_switch);
                modules_list_->at(i).io_.write_CAN_transmit_(1);
                modules_list_->at(i).io_.CAN_recieve_feedback(&modules_list_->at(i).rxdata_buffer_[0],
                                                              &modules_list_->at(i).rxdata_buffer_[1]);
                if ((next_mode_switch == Mode::SET_ZERO
                    && (int)modules_list_->at(i).rxdata_buffer_[0].position_ <= 0.01
                    && (int)modules_list_->at(i).rxdata_buffer_[0].position_ >= -0.01)
                    || 
                    ((int)modules_list_->at(i).rxdata_buffer_[0].mode_ == (int)next_mode_switch
                    && (int)modules_list_->at(i).rxdata_buffer_[1].mode_ == (int)next_mode_switch))
                {
                    mode_switched_cnt++;
                }
            }
        }
        
        time_elapsed += 0.01;
        usleep(1e4);
    }

    for (int i = 0; i < 4; i++)
    {
        if (modules_list_->at(i).enable_){
            if (workingMode_ == Mode::MOTOR) modules_list_->at(i).io_.write_CAN_id_fc_((int)Mode::CONTROL, (int)Mode::CONTROL);
            else modules_list_->at(i).io_.write_CAN_id_fc_((int)Mode::CONFIG, (int)Mode::CONFIG);
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
                    motor_fb_msg.mutable_module_a()->set_torque_r(mod.rxdata_buffer_[0].torque_*mod.txdata_buffer_[0].KT_); //torque R
                    motor_fb_msg.mutable_module_a()->set_torque_l(mod.rxdata_buffer_[1].torque_*mod.txdata_buffer_[1].KT_); //torque L
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
                    motor_fb_msg.mutable_module_b()->set_torque_r(mod.rxdata_buffer_[0].torque_*mod.txdata_buffer_[0].KT_); //torque R
                    motor_fb_msg.mutable_module_b()->set_torque_l(mod.rxdata_buffer_[1].torque_*mod.txdata_buffer_[1].KT_); //torque L
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
                    motor_fb_msg.mutable_module_c()->set_torque_r(mod.rxdata_buffer_[0].torque_*mod.txdata_buffer_[0].KT_); //torque R
                    motor_fb_msg.mutable_module_c()->set_torque_l(mod.rxdata_buffer_[1].torque_*mod.txdata_buffer_[1].KT_); //torque L
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
                    motor_fb_msg.mutable_module_d()->set_torque_r(mod.rxdata_buffer_[0].torque_*mod.txdata_buffer_[0].KT_); //torque R
                    motor_fb_msg.mutable_module_d()->set_torque_l(mod.rxdata_buffer_[1].torque_*mod.txdata_buffer_[1].KT_); //torque L
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
