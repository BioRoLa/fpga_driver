#include "fpga_server.hpp"

/* TCP node connection setup*/
std::atomic<int> motor_message_updated{0};
std::atomic<int> fpga_message_updated{0}; //power

std::ofstream term;
std::mutex mutex_;

motor_msg::MotorCmdStamped motor_cmd_data;
void motor_data_cb(motor_msg::MotorCmdStamped motor_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    motor_message_updated = 1;
    motor_cmd_data = motor_msg;
}

// Robot gRPC message callbacks
std::atomic<int> robot_cmd_message_updated{0};
robot_msg::RobotCmdStamped robot_cmd_data;
void robot_cmd_data_cb(robot_msg::RobotCmdStamped robot_cmd_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    robot_cmd_message_updated = 1;
    robot_cmd_data = robot_cmd_msg;
}

// Robot Request Update callback
std::atomic<int> robot_request_update_received{0};
robot_msg::RobotRequestUpdate robot_request_update_data;
void robot_request_update_cb(robot_msg::RobotRequestUpdate robot_request_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    robot_request_update_received = 1;
    robot_request_update_data = robot_request_msg;
}

Corgi::Corgi()
    : motor_fsm_(modules_list_, powerboard_state_, fpga_.powerboard_V_list_)
    , robot_fsm_(motor_fsm_, modules_list_, powerboard_state_)
{
    /* default value of interrupt*/
    main_irq_period_us_ = 500;
    can_irq_period_us_ = 800;
    seq = 0;
    
    /* initialize robot mode - handled by robot_fsm_ initialization */
    robot_message_updated_ = 0;

    /* powerboard_state_ is already initialized to {false, false, false} in header */
    NO_CAN_TIMEDOUT_ERROR_ = true;
    NO_SWITCH_TIMEDOUT_ERROR_ = true;
    HALL_CALIBRATED_ = false;

    motor_fsm_.setTimeoutErrorFlags(&NO_CAN_TIMEDOUT_ERROR_, &NO_SWITCH_TIMEDOUT_ERROR_);
    motor_fsm_.setRobotFSM(&robot_fsm_);
    robot_fsm_.setErrorFlags(!NO_CAN_TIMEDOUT_ERROR_, !NO_SWITCH_TIMEDOUT_ERROR_);
    robot_fsm_.setLoopPeriod(main_irq_period_us_);

    load_config_();

    console_.init(&fpga_, &modules_list_, &powerboard_state_, &motor_fsm_, &robot_fsm_, &main_mtx_);

    fpga_.setIrqPeriod(main_irq_period_us_, can_irq_period_us_);
}

void Corgi::load_config_()
{
    yaml_node_ = YAML::LoadFile(CONFIG_PATH);

    // Configure motor FSM parameters
    motor_fsm_.setControlPeriod(yaml_node_["MainLoop_period_us"].as<int>() * 0.000001);
    motor_fsm_.setMeasureOffset(yaml_node_["Measure_offset"].as<int>());
    motor_fsm_.setCalibrationVelocity(yaml_node_["Hall_calibration_vel"].as<double>());
    motor_fsm_.setCalibrationTolerance(yaml_node_["Hall_calibration_tol"].as<double>());

    main_irq_period_us_ = yaml_node_["MainLoop_period_us"].as<int>();
    can_irq_period_us_ = yaml_node_["CANLoop_period_us"].as<int>();

    int modules_num_;
    /* initialize leg modules */
    modules_num_ = yaml_node_["Number_of_modules"].as<int>();

    for (int i = 0; i < modules_num_; i++)
    {
        std::string label = yaml_node_["Modules_list"][i].as<std::string>();
        // Use emplace_back to construct in-place, avoiding copy
        modules_list_.emplace_back(label, yaml_node_, fpga_.status_, fpga_.session_);
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

void Corgi::interruptHandler(core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                             core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                             core::Publisher<motor_msg::MotorStateStamped>& state_pub_,
                             core::Publisher<robot_msg::RobotStateStamped>& robot_state_pub_,
                             core::Subscriber<robot_msg::RobotCmdStamped>& robot_cmd_sub_,
                             core::Subscriber<robot_msg::RobotRequestUpdate>& robot_request_sub_,
                             core::Publisher<robot_msg::RobotRequestUpdate>& robot_request_pub_)   

                                                       
{
    while (NiFpga_IsNotError(fpga_.status_) && !sys_stop)
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
                mainLoop_(state_pb_pub_, cmd_sub_, state_pub_, robot_state_pub_, robot_cmd_sub_, robot_request_sub_, robot_request_pub_);
                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
            if (irqsAsserted & NiFpga_Irq_1)
            {
                /* Handling CAN-BUS communication */
                canLoop_();

                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
        }
        usleep(10);
    }
}

void Corgi::mainLoop_(core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                      core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                      core::Publisher<motor_msg::MotorStateStamped>& state_pub_,
                      core::Publisher<robot_msg::RobotStateStamped>& robot_state_pub_,
                      core::Subscriber<robot_msg::RobotCmdStamped>& robot_cmd_sub_,
                      core::Subscriber<robot_msg::RobotRequestUpdate>& robot_request_sub_,
                      core::Publisher<robot_msg::RobotRequestUpdate>& robot_request_pub_)
{
    fpga_.write_powerboard_(&powerboard_state_);
    fpga_.read_powerboard_data_();

    core::spinOnce();
    
    power_msg::PowerStateStamped power_fb_msg;
    motor_msg::MotorStateStamped motor_fb_msg;
    robot_msg::RobotStateStamped robot_fb_msg;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Handle Robot Command if mode_update is set to true
        if (robot_request_update_received == 1 && robot_request_update_data.mode_update()) {
            handleRobotCommand(robot_cmd_data);
            
            // Set mode_update to false and publish to avoid repeated actions
            robot_request_update_data.set_mode_update(false);
            robot_request_pub_.publish(robot_request_update_data);
            
            robot_request_update_received = 0;
        }
        
        // Update error flags and run Robot FSM
        robot_fsm_.setErrorFlags(!NO_CAN_TIMEDOUT_ERROR_, !NO_SWITCH_TIMEDOUT_ERROR_);
        robot_fsm_.runFsm();
        
        // Run Motor FSM
        motor_fsm_.runFsm(motor_fb_msg, motor_cmd_data);
        motor_message_updated = 0;    
        HALL_CALIBRATED_ = motor_fsm_.isHallCalibrated();
    }

    // Communication with Node Architecture
    powerboardPack(power_fb_msg);
    robotStatePack(robot_fb_msg);

    // Read Command
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motor_fb_msg.mutable_header()->set_seq(seq);
        robot_fb_msg.mutable_header()->set_seq(seq);
    }
    
    state_pub_.publish(motor_fb_msg);
    state_pb_pub_.publish(power_fb_msg);
    robot_state_pub_.publish(robot_fb_msg);
    seq++;
}

void Corgi::canLoop_()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (int i = 0; i < 4; i++)
    {
        if (modules_list_[i].enable_ && powerboard_state_.at(2) == true)
        {
            // Receive feedback from FPGA (stores to feedback_data_raw)
            modules_list_[i].receiveFeedback();
            
            for (size_t j = 0; j < modules_list_[i].getMotorCount(); j++)
            {
                CANMotor* motor = modules_list_[i].getMotor(j);
                if (motor) {
                    motor->decodeBasedOnMode();
                }
            }

            if (!modules_list_[i].hasTimeout())
            {
                modules_list_[i].sendCommands();
                NO_CAN_TIMEDOUT_ERROR_ = true;
            }
            else
            {
                NO_CAN_TIMEDOUT_ERROR_ = false;
            }
        }
    }
}

void Corgi::powerboardPack(power_msg::PowerStateStamped&power_dashboard_reply)
{   
    std::lock_guard<std::mutex> lock(mutex_);
    
    gettimeofday(&t_stamp, NULL);
    power_dashboard_reply.mutable_header()->set_seq(seq);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
    power_dashboard_reply.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);

    // Send individual switch states
    power_dashboard_reply.set_digital(powerboard_state_.at(0));
    power_dashboard_reply.set_signal(powerboard_state_.at(1));
    power_dashboard_reply.set_power(powerboard_state_.at(2));

    if (motor_fsm_.isHallCalibrated() == true && NO_SWITCH_TIMEDOUT_ERROR_==true && NO_CAN_TIMEDOUT_ERROR_==true) power_dashboard_reply.set_clean(true);
    else power_dashboard_reply.set_clean(false);

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
}

void Corgi::handleRobotCommand(const robot_msg::RobotCmdStamped& robot_cmd)
{
    robot_msg::ROBOTMODE proto_requested_mode = robot_cmd.request_robot_mode();
    RobotMode requested_mode = static_cast<RobotMode>(proto_requested_mode);
    RobotMode current_mode = robot_fsm_.getCurrentMode();
    
    // Only update mode if it's different from current mode
    if (requested_mode != current_mode) {
        std::cout << "[Robot Command] Mode switch requested from " << static_cast<int>(current_mode) 
                  << " to " << static_cast<int>(requested_mode) << std::endl;
        
        // Use RobotFSM to check if mode transition is available
        bool transition_available = robot_fsm_.requestModeTransition(requested_mode);
        
        if (transition_available) {
            std::cout << "[Robot Mode] Successfully requested transition to mode " << static_cast<int>(requested_mode) << std::endl;
        } else {
            std::cout << "[Robot Mode] Transition denied - invalid or not allowed at this time" << std::endl;
        }
    }
}

void Corgi::robotStatePack(robot_msg::RobotStateStamped& robot_state_reply)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    gettimeofday(&t_stamp, NULL);
    robot_state_reply.mutable_header()->set_seq(seq);
    robot_state_reply.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
    robot_state_reply.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);
    robot_state_reply.mutable_header()->set_frameid("robot_state");
    
    // Set current robot mode (get from robot_fsm_ and convert to proto enum)
    RobotMode current_mode = robot_fsm_.getCurrentMode();
    robot_state_reply.set_robot_mode(static_cast<robot_msg::ROBOTMODE>(current_mode));
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
    
    // Create Log publisher
    core::Publisher<log_msg::LogEntry>& log_pub = nh.advertise<log_msg::LogEntry>("/log", 100);
    
    // Initialize Logger with publish callback for remote logging
    core::Logger logger("fpga_driver", [&log_pub](const log_msg::LogEntry& entry) {
        log_pub.publish(entry);
    });
    logger.setMinLevel(core::LogLevel::DEBUG);  // Set minimum log level
    
    // Set logger for robot FSM (enables remote logging)
    corgi.robot_fsm_.setLogger(&logger);

    core::Publisher<power_msg::PowerStateStamped>& power_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");

    core::Publisher<motor_msg::MotorStateStamped>& motor_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Subscriber<motor_msg::MotorCmdStamped>& motor_sub = nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_data_cb);
    
    // Robot gRPC publishers and subscribers
    core::Publisher<robot_msg::RobotStateStamped>& robot_state_pub = nh.advertise<robot_msg::RobotStateStamped>("robot/state");
    core::Subscriber<robot_msg::RobotCmdStamped>& robot_cmd_sub = nh.subscribe<robot_msg::RobotCmdStamped>("robot/command", 1000, robot_cmd_data_cb);
    core::Subscriber<robot_msg::RobotRequestUpdate>& robot_request_sub = nh.subscribe<robot_msg::RobotRequestUpdate>("robot/request_update", 1000, robot_request_update_cb);
    core::Publisher<robot_msg::RobotRequestUpdate>& robot_request_pub = nh.advertise<robot_msg::RobotRequestUpdate>("robot/request_update");
    
    corgi.interruptHandler(power_pub, motor_sub, motor_pub, robot_state_pub, robot_cmd_sub, robot_request_sub, robot_request_pub);

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
    //TODO: add safe shutdown procedure(power off etc)
    sys_stop = 1;
}



