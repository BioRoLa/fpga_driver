#include "fpga_server.hpp"

/* TCP node connection setup*/
volatile int motor_message_updated = 0;
volatile int fpga_message_updated = 0; //power
volatile int steer_message_updated = 0; //steering

std::ofstream term;
std::mutex mutex_;

motor_msg::MotorCmdStamped motor_cmd_data;
void motor_data_cb(motor_msg::MotorCmdStamped motor_msg)
{
    mutex_.lock();
    motor_message_updated = 1;
    motor_cmd_data = motor_msg;
    mutex_.unlock();
}

power_msg::PowerCmdStamped power_cmd_data;
void power_data_cb(power_msg::PowerCmdStamped power_msg)
{
    mutex_.lock();
    fpga_message_updated = 1;
    power_cmd_data = power_msg;
    mutex_.unlock();
}

steering_msg::SteeringCmdStamped steering_cmd_data;
void steer_data_cb(steering_msg::SteeringCmdStamped steer_msg)
{
    mutex_.lock();
    steer_message_updated = 1;
    steering_cmd_data = steer_msg;
    mutex_.unlock();
}

Corgi::Corgi()
{
   stop_ = false;
    /* default value of interrupt*/
    main_irq_period_us_ = 500;
    can_irq_period_us_ = 800;
    seq = 0;

    /* initialize powerboard state */
    digital_switch_ = false;
    signal_switch_ = false;
    power_switch_ = false;
    vicon_trigger_ = false;
    NO_CAN_TIMEDOUT_ERROR_ = true;
    NO_SWITCH_TIMEDOUT_ERROR_ = true;
    HALL_CALIBRATED_ = false;

    Steer_Cali = false;
    steering_cali_state = 0;
    steering_state_complete = 0;
    voltage = 4095;
    max_timeout_cnt_ = 100;
    steer_position = 0;
    hall_complete = true;
    r_hall = 0;
    l_hall = 0;
    zero_offset = 0;
    steer_current_angle = 0.00;
    steering_state = false;

    powerboard_state_.push_back(digital_switch_);
    powerboard_state_.push_back(signal_switch_);
    powerboard_state_.push_back(power_switch_);

    ModeFsm fsm(&modules_list_, &powerboard_state_, fpga_.powerboard_V_list_);
    fsm_ = fsm;
    fsm_.NO_CAN_TIMEDOUT_ERROR_ = &NO_CAN_TIMEDOUT_ERROR_;
    fsm_.NO_SWITCH_TIMEDOUT_ERROR_ = &NO_SWITCH_TIMEDOUT_ERROR_;

    load_config_();

    console_.init(&fpga_, &modules_list_, &powerboard_state_, &fsm_, &main_mtx_);

    fpga_.setIrqPeriod(main_irq_period_us_, can_irq_period_us_);
    fpga_.write_vicon_trigger(false);
}

void Corgi::load_config_()
{
    yaml_node_ = YAML::LoadFile(CONFIG_PATH);

    fsm_.dt_ = yaml_node_["MainLoop_period_us"].as<int>() * 0.000001;
    fsm_.measure_offset = yaml_node_["Measure_offset"].as<int>();
    fsm_.cal_vel_ = yaml_node_["Hall_calibration_vel"].as<double>();
    fsm_.cal_tol_ = yaml_node_["Hall_calibration_tol"].as<double>();

    if (yaml_node_["Scenario"].as<std::string>().compare("SingleModule") == 0)fsm_.scenario_ = Scenario::SINGLE_MODULE;
    else fsm_.scenario_ = Scenario::ROBOT;

    main_irq_period_us_ = yaml_node_["MainLoop_period_us"].as<int>();
    can_irq_period_us_ = yaml_node_["CANLoop_period_us"].as<int>();

    /* initialize leg modules */
    modules_num_ = yaml_node_["Number_of_modules"].as<int>();

    for (int i = 0; i < modules_num_; i++)
    {
        std::string label = yaml_node_["Modules_list"][i].as<std::string>();
        LegModule module(label, yaml_node_, fpga_.status_, fpga_.session_);
        modules_list_.push_back(module);
    }

    YAML::Node Factors_node_ = yaml_node_["Powerboard_Scaling_Factor"];
    int idx_ = 0;

    std::cout << "PowerBoard Scaling Factor" << std::endl;
    for (auto f : Factors_node_)
    {
        fpga_.powerboard_Ifactor[idx_] = f["Current_Factor"].as<double>();
        fpga_.powerboard_Vfactor[idx_] = f["Voltage_Factor"].as<double>();
        std::cout << "Index " << idx_ << " Current Factor: " << fpga_.powerboard_Ifactor[idx_]
                  << ", Voltage Factor: " << fpga_.powerboard_Vfactor[idx_] << std::endl;
        idx_++;
    }
}

void Corgi::interruptHandler(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                             core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                             core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                             core::Publisher<motor_msg::MotorStateStamped>& state_pub_,
                             core::Subscriber<steering_msg::SteeringCmdStamped>& steer_sub_,
                             core::Publisher<steering_msg::SteeringStateStamped>& steer_pub_)   

                                                       
{
    while (NiFpga_IsNotError(fpga_.status_) && !stop_ && !sys_stop)
    {
        uint32_t irqsAsserted;
        uint32_t irqTimeout = 10;  // ms
        NiFpga_Bool TimedOut = 0;

        // Wait on IRQ to ensure FPGA is ready
        NiFpga_MergeStatus(&fpga_.status_, NiFpga_WaitOnIrqs(fpga_.session_, fpga_.irqContext_, NiFpga_Irq_0 | NiFpga_Irq_1,
                                                             irqTimeout, &irqsAsserted, &TimedOut));

        if (NiFpga_IsError(fpga_.status_))
        {
            std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << fpga_.status_ << reset
                      << std::endl;
        }

        uint32_t irq0_cnt;
        uint32_t irq1_cnt;

        NiFpga_MergeStatus(&fpga_.status_,
                           NiFpga_ReadU32(fpga_.session_, NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_IRQ0_cnt, &irq0_cnt));

        NiFpga_MergeStatus(&fpga_.status_,
                           NiFpga_ReadU32(fpga_.session_, NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_IRQ0_cnt, &irq1_cnt));

        if (TimedOut)
        {
            std::cout << red << "IRQ timedout" << ", IRQ_0 cnt: " << irq0_cnt << ", IRQ_1 cnt: " << irq1_cnt << reset
                      << std::endl;
        }

        /* if an IRQ was asserted */
        if (NiFpga_IsNotError(fpga_.status_) && !TimedOut)
        {
            if (irqsAsserted & NiFpga_Irq_0)
            {
                mainLoop_(cmd_pb_sub_, state_pb_pub_, cmd_sub_, state_pub_, steer_sub_, steer_pub_);
                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
            if (irqsAsserted & NiFpga_Irq_1)
            {
                /* TODO: do something if IRQ1 */
                /* Handling CAN-BUS communication */
                canLoop_();

                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
        }
        usleep(10);
    }
}

void Corgi::mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                      core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                      core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                      core::Publisher<motor_msg::MotorStateStamped>& state_pub_,
                      core::Subscriber<steering_msg::SteeringCmdStamped>& steer_sub_,
                      core::Publisher<steering_msg::SteeringStateStamped>& steer_pub_)
{
    fpga_.write_powerboard_(&powerboard_state_);
    fpga_.read_powerboard_data_();

    core::spinOnce();
    mutex_.lock();
    power_msg::PowerStateStamped power_fb_msg;
    motor_msg::MotorStateStamped motor_fb_msg;
    steering_msg::SteeringStateStamped steer_fb_msg;

    fsm_.runFsm(motor_fb_msg, motor_cmd_data);
    motor_message_updated = 0;    
    HALL_CALIBRATED_ = fsm_.hall_calibrated;

    mutex_.unlock();

    // Communication with Node Architecture
    powerboardPack(power_fb_msg);

    // think where to put this
    steeringPack(steer_fb_msg);

    // Read Command
    mutex_.lock();
    if (power_cmd_data.clean() == true)
    {
        NO_CAN_TIMEDOUT_ERROR_ = true;
        NO_SWITCH_TIMEDOUT_ERROR_ = true;
        HALL_CALIBRATED_ = false;
        timeout_cnt_ = 0;
    }

    if (NO_SWITCH_TIMEDOUT_ERROR_)
    {
        if (fpga_message_updated)
        {
            powerboard_state_.at(0) = power_cmd_data.digital();
            powerboard_state_.at(1) = power_cmd_data.signal();
            powerboard_state_.at(2) = power_cmd_data.power();

            fpga_.write_vicon_trigger(power_cmd_data.trigger());
            vicon_trigger_ = power_cmd_data.trigger();

            if (powerboard_state_.at(1)  ==false)
            {
                steering_state=false;
                steering_state_complete = 0;
                Steer_Cali = false;
                steering_cali_state = 0;
            }
            if (power_cmd_data.steering_cali()==true && steering_cali_state<3 )
            {   
                voltage = 2000;
                steering_state=false;
                steering_state_complete = 0;
                Steer_Cali = false;
                fpga_.switch_steering(true);
                switch (steering_cali_state)
                {
                case 0:
                    // right turn
                    fpga_.switch_steer_dir(false);
                    fpga_.write_steer_vol(voltage);
                    hall_complete = fpga_.read_steer_hall();
                    steer_position = fpga_.read_steer_encoder();
                    if (hall_complete == true)
                    {
                        voltage = 0;
                        fpga_.write_steer_vol(voltage);
                        r_hall = steer_position;
                        steering_cali_state++;
                        voltage = 2000;
                        fpga_.switch_steer_dir(true);  
                        fpga_.write_steer_vol(voltage); 
                        usleep(1000*1000);
                    }
                    break;
                case 1:
                    // left turn
                    fpga_.switch_steer_dir(true);  
                    fpga_.write_steer_vol(voltage);                  
                    hall_complete = fpga_.read_steer_hall();
                    steer_position = fpga_.read_steer_encoder();
                    if (hall_complete == true)
                    { 
                        voltage = 0;
                        fpga_.write_steer_vol(voltage);
                        l_hall = steer_position;
                        steering_cali_state++;
                        voltage = 2000;
                        zero_offset = (r_hall+l_hall)/2-330;
                        r_hall = r_hall-zero_offset;
                        l_hall = l_hall-zero_offset;
                    }
                    break;
                case 2:
                    // turn back to set zero and initial place
                    fpga_.switch_steer_dir(false);
                    fpga_.write_steer_vol(voltage);
                    steer_position = fpga_.read_steer_encoder();
                    if (steer_position == zero_offset)
                    {
                        voltage = 0;
                        fpga_.write_steer_vol(voltage);
                        fpga_.switch_steering(false);
                        Steer_Cali = true;
                        steer_current_angle = 0.00;
                        steering_state_complete = 2;
                        steering_cali_state++;
                        steering_state = true;
                    }                    
                    break;
                }
            }
            if (power_cmd_data.steering_cali()==false && steering_cali_state==3 )
            { 
                steering_state_complete = 0;
                steering_cali_state++;
                Steer_Cali = true;
                steering_state = true;
            }


            if (power_cmd_data.robot_mode() == (int)Mode::MOTOR && fsm_.workingMode_ != Mode::MOTOR)fsm_.switchMode(Mode::MOTOR);
            else if (power_cmd_data.robot_mode() == (int)Mode::HALL_CALIBRATE && fsm_.workingMode_ != Mode::HALL_CALIBRATE && fsm_.workingMode_ != Mode::MOTOR)fsm_.switchMode(Mode::HALL_CALIBRATE);
            else if (power_cmd_data.robot_mode() == (int)Mode::SET_ZERO && fsm_.workingMode_ != Mode::SET_ZERO)fsm_.switchMode(Mode::SET_ZERO);
            else if (power_cmd_data.robot_mode() == (int)Mode::CONFIG && fsm_.workingMode_ != Mode::CONFIG)fsm_.switchMode(Mode::CONFIG);
            else if (power_cmd_data.robot_mode() == (int)Mode::REST && fsm_.workingMode_ != Mode::REST)fsm_.switchMode(Mode::REST);
            fpga_message_updated = 0;
        }
    
        
        if (steer_message_updated)
        {   
            
            steering_state_complete = 0;
            
            if (Steer_Cali = true && steering_cmd_data.voltage()!=0 )
            {            
                fpga_.switch_steering(true);  
                if (steering_cmd_data.angle()>r_hall || steering_cmd_data.angle()< l_hall){
                    steering_state_complete = 1;
                    voltage = 0;
                    fpga_.write_steer_vol(0);
                    fpga_.switch_steering(false);
                }

                else if ((float)steering_cmd_data.angle()!= (float)steer_current_angle && (steering_cmd_data.angle()-steer_current_angle)> 0.0 )
                {
                    voltage = steering_cmd_data.voltage();
                    fpga_.switch_steer_dir(false);
                    fpga_.write_steer_vol(voltage);
                    hall_complete = fpga_.read_steer_hall();
                    steer_position = fpga_.read_steer_encoder();	
                }
                else if ((float)steering_cmd_data.angle()!= (float)steer_current_angle && (steering_cmd_data.angle()-steer_current_angle)< 0.0 )
                {
                    voltage = steering_cmd_data.voltage();
                    fpga_.switch_steer_dir(true);
                    fpga_.write_steer_vol(voltage);
                    hall_complete = fpga_.read_steer_hall();
                    steer_position = fpga_.read_steer_encoder();	
                }
                else if ((float)steering_cmd_data.angle()== (float)steer_current_angle)
                {
                    voltage = 0;
                    fpga_.write_steer_vol(voltage);
                    hall_complete = fpga_.read_steer_hall();
                    steer_position = fpga_.read_steer_encoder();	
                    steering_state_complete = 2;
                    fpga_.switch_steering(false);
                }     
            }
            steer_message_updated = 0;
        }
    }
    motor_fb_msg.mutable_header()->set_seq(seq);
    mutex_.unlock();
    state_pub_.publish(motor_fb_msg);
    state_pb_pub_.publish(power_fb_msg);
    steer_pub_.publish(steer_fb_msg);
    seq++;
}

void Corgi::canLoop_()
{
    for (int i = 0; i < 4; i++)
    {
        if (modules_list_[i].enable_ && powerboard_state_.at(2) == true)
        {
            modules_list_[i].io_.CAN_recieve_feedback(&modules_list_[i].rxdata_buffer_[0], &modules_list_[i].rxdata_buffer_[1]);
            modules_list_[i].CAN_timeoutCheck();

            if (modules_list_[i].CAN_module_timedout)timeout_cnt_++;
            else timeout_cnt_ = 0;
            if (timeout_cnt_ < max_timeout_cnt_)
            {
                modules_list_[i].io_.CAN_send_command(modules_list_[i].txdata_buffer_[0], modules_list_[i].txdata_buffer_[1]);
                NO_CAN_TIMEDOUT_ERROR_ = true;
            }
            else NO_CAN_TIMEDOUT_ERROR_ = false;
        }
    }
}

void Corgi::powerboardPack(power_msg::PowerStateStamped&power_dashboard_reply)
{   
    
    mutex_.lock();
    gettimeofday(&t_stamp, NULL);
    power_dashboard_reply.mutable_header()->set_seq(seq);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);

    power_dashboard_reply.set_digital(powerboard_state_.at(0));
    power_dashboard_reply.set_signal(powerboard_state_.at(1));
    power_dashboard_reply.set_power(powerboard_state_.at(2));

    if (fsm_.hall_calibrated == true && NO_SWITCH_TIMEDOUT_ERROR_==true && NO_CAN_TIMEDOUT_ERROR_==true) power_dashboard_reply.set_clean(true);
    else power_dashboard_reply.set_clean(false);

    if (fsm_.workingMode_ == Mode::REST) power_dashboard_reply.set_robot_mode(power_msg::REST_MODE);
    else if (fsm_.workingMode_ == Mode::HALL_CALIBRATE) power_dashboard_reply.set_robot_mode(power_msg::HALL_CALIBRATE);
    else if (fsm_.workingMode_ == Mode::MOTOR) power_dashboard_reply.set_robot_mode(power_msg::MOTOR_MODE);
    else if (fsm_.workingMode_ == Mode::SET_ZERO) power_dashboard_reply.set_robot_mode(power_msg::SET_ZERO);

    power_dashboard_reply.set_v_0(fpga_.powerboard_V_list_[0]);
    power_dashboard_reply.set_i_0(fpga_.powerboard_I_list_[0]);

    power_dashboard_reply.set_v_1(fpga_.powerboard_V_list_[1]);
    power_dashboard_reply.set_i_1(fpga_.powerboard_I_list_[1]);

    power_dashboard_reply.set_v_2(fpga_.powerboard_V_list_[2]);
    power_dashboard_reply.set_i_2(fpga_.powerboard_I_list_[2]);

    power_dashboard_reply.set_v_3(fpga_.powerboard_V_list_[3]);
    power_dashboard_reply.set_i_3(fpga_.powerboard_I_list_[3]);

    power_dashboard_reply.set_v_4(fpga_.powerboard_V_list_[4]);
    power_dashboard_reply.set_i_4(fpga_.powerboard_I_list_[4]);

    power_dashboard_reply.set_v_5(fpga_.powerboard_V_list_[5]);
    power_dashboard_reply.set_i_5(fpga_.powerboard_I_list_[5]);

    power_dashboard_reply.set_v_6(fpga_.powerboard_V_list_[6]);
    power_dashboard_reply.set_i_6(fpga_.powerboard_I_list_[6]);

    power_dashboard_reply.set_v_7(fpga_.powerboard_V_list_[7]);
    power_dashboard_reply.set_i_7(fpga_.powerboard_I_list_[7]);

    power_dashboard_reply.set_v_8(fpga_.powerboard_V_list_[8]);
    power_dashboard_reply.set_i_8(fpga_.powerboard_I_list_[8]);

    power_dashboard_reply.set_v_9(fpga_.powerboard_V_list_[9]);
    power_dashboard_reply.set_i_9(fpga_.powerboard_I_list_[9]);

    power_dashboard_reply.set_v_10(fpga_.powerboard_V_list_[10]);
    power_dashboard_reply.set_i_10(fpga_.powerboard_I_list_[10]);

    power_dashboard_reply.set_v_11(fpga_.powerboard_V_list_[11]);
    power_dashboard_reply.set_i_11(fpga_.powerboard_I_list_[11]);

    mutex_.unlock();
}

void Corgi::steeringPack(steering_msg::SteeringStateStamped &steer_fb_msg)
{   
    mutex_.lock();
    gettimeofday(&t_stamp, NULL);
    steer_fb_msg.mutable_header()->set_seq(seq);
    steer_fb_msg.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
    steer_fb_msg.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);
    steer_position = fpga_.read_steer_encoder();
    steer_current_angle = (steer_position-zero_offset)*360/(512*4);
    hall_complete = fpga_.read_steer_hall();
    steer_fb_msg.set_current_angle((float)steer_current_angle);
    steer_fb_msg.set_current_state(steering_state);
    steer_fb_msg.set_cmd_finish(steering_state_complete);

    mutex_.unlock();
}

int main(int argc, char* argv[])
{
    signal(SIGINT, inthand);

    important_message("[FPGA Server] : Launched");

    if (argc == 3)
    {
        std::string s(argv[1]);
        if (s == "-t")
        {
            std::cout << "debug terminal output to " << argv[2] << std::endl;
            term = std::ofstream(argv[2], std::ios_base::out);
        }
    }
    Corgi corgi;

    core::NodeHandler nh;

    core::Publisher<power_msg::PowerStateStamped>& power_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");
    core::Subscriber<power_msg::PowerCmdStamped>& power_sub = nh.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_data_cb);

    core::Publisher<motor_msg::MotorStateStamped>& motor_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Subscriber<motor_msg::MotorCmdStamped>& motor_sub = nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_data_cb);

    core::Publisher<steering_msg::SteeringStateStamped>& steer_pub = nh.advertise<steering_msg::SteeringStateStamped>("steer/state");
    core::Subscriber<steering_msg::SteeringCmdStamped>& steer_sub = nh.subscribe<steering_msg::SteeringCmdStamped>("steer/command", 1000, steer_data_cb);

    
    corgi.interruptHandler(power_sub, power_pub, motor_sub, motor_pub, steer_sub, steer_pub);

    if (NiFpga_IsError(corgi.fpga_.status_)) std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << corgi.fpga_.status_ << reset << std::endl;
    else
    {
        endwin();
        important_message("\n[FPGA Server] : Exit Safely");
    }

    return 0;
}

/* CAPTURE SYS STOP SIGNAL TO KILL PROCESS*/
void inthand(int signum)
{
    sys_stop = 1;
}



