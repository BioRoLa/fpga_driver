#include "main.hpp"

/* TCP node connection setup*/
std::atomic<int> motor_message_updated{0};
std::atomic<int> fpga_message_updated{0}; //power
std::atomic<int> motor_config_message_updated{0};

std::ofstream term;
std::mutex mutex_;

// Atomic flag for safe shutdown request
std::atomic<bool> shutdown_requested{false};

motor_msg::MotorCmdStamped motor_cmd_data;
void motor_data_cb(motor_msg::MotorCmdStamped motor_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    motor_message_updated = 1;
    motor_cmd_data = motor_msg;
}

config_msg::ConfigStamped motor_config_request_data;
void motor_config_cb(config_msg::ConfigStamped motor_config_request)
{
    std::lock_guard<std::mutex> lock(mutex_);
    motor_config_message_updated = 1;
    motor_config_request_data = motor_config_request;
}

// Robot gRPC message callbacks
std::atomic<int> robot_cmd_message_updated{0};
robot_msg::RobotCmdStamped robot_cmd_data;
static uint64_t last_robot_cmd_seq = 0;

void robot_cmd_data_cb(robot_msg::RobotCmdStamped robot_cmd_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    robot_cmd_message_updated = 1;
    robot_cmd_data = robot_cmd_msg;
}

Corgi::Corgi()
    : motor_fsm_(modules_list_, powerboard1_state_, powerboard2_state_, fpga_.powerboard1_V_list_, fpga_.powerboard2_V_list_)
    , robot_fsm_(motor_fsm_, modules_list_, powerboard1_state_, powerboard2_state_, fpga_.powerboard1_V_list_, fpga_.powerboard2_V_list_)
{
    /* default value of interrupt*/
    main_irq_period_us_ = 500;
    can_irq_period_us_ = 800;
    seq = 0;
    
    /* initialize robot mode - handled by robot_fsm_ initialization */
    robot_message_updated_ = 0;

    /* powerboard1_state_ && powerboard2_state_ are already initialized to {false, false, false} in header */
    NO_CAN_TIMEDOUT_ERROR_ = true;
    NO_SWITCH_TIMEDOUT_ERROR_ = true;
    HALL_CALIBRATED_ = false;

    motor_fsm_.setTimeoutErrorFlags(&NO_CAN_TIMEDOUT_ERROR_, &NO_SWITCH_TIMEDOUT_ERROR_);
    motor_fsm_.setRobotFSM(&robot_fsm_);
    robot_fsm_.setErrorFlags(!NO_CAN_TIMEDOUT_ERROR_, !NO_SWITCH_TIMEDOUT_ERROR_);
    robot_fsm_.setLoopPeriod(main_irq_period_us_);

    load_config_();

    // The motor function-code register lives on the FPGA and survives a driver
    // restart (it is only ever written by MotorFSM::switchMode()). If the previous
    // process was killed while a module was in HALL_CALIBRATE/MOTOR mode, that
    // register stays latched even though this fresh process believes it's REST.
    // Force it back to REST here so a restart can't resume stale motor commands.
    for (auto& mod : modules_list_)
    {
        if (mod.enable_) mod.setMode(FunctionMode::REST);
    }

    console_.init(&fpga_, &modules_list_, &powerboard1_state_, &powerboard2_state_, &motor_fsm_, &robot_fsm_, &main_mtx_);

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

    YAML::Node Factors_node1_ = yaml_node_["Powerboard1_Scaling_Factor"];
    int idx1_ = 0;

    LOG_INFO << "PowerBoard1 Scaling Factor";
    for (auto f : Factors_node1_)
    {
        fpga_.powerboard1_Ifactor[idx1_] = f["Current_Factor"].as<double>();
        fpga_.powerboard1_Ioffset[idx1_] = f["Current_Offset"].as<double>();
        fpga_.powerboard1_Vfactor[idx1_] = f["Voltage_Factor"].as<double>();
        fpga_.powerboard1_Voffset[idx1_] = f["Voltage_Offset"].as<double>();
        LOG_INFO << "Index " << idx1_ << " Current Factor: " << fpga_.powerboard1_Ifactor[idx1_]
                           << ", Current Offset: " << fpga_.powerboard1_Ioffset[idx1_]
                           << ", Voltage Factor: " << fpga_.powerboard1_Vfactor[idx1_]
                           << ", Voltage Offset: " << fpga_.powerboard1_Voffset[idx1_];
        idx1_++;
    }

    LOG_INFO << "PowerBoard2 Scaling Factor";
    YAML::Node Factors_node2_ = yaml_node_["Powerboard2_Scaling_Factor"];
    int idx2_ = 0;
    for (auto f : Factors_node2_)
    {
        fpga_.powerboard2_Ifactor[idx2_] = f["Current_Factor"].as<double>();
        fpga_.powerboard2_Ioffset[idx2_] = f["Current_Offset"].as<double>();
        fpga_.powerboard2_Vfactor[idx2_] = f["Voltage_Factor"].as<double>();
        fpga_.powerboard2_Voffset[idx2_] = f["Voltage_Offset"].as<double>();
        LOG_INFO << "Index " << idx2_ << " Current Factor: " << fpga_.powerboard2_Ifactor[idx2_]
                           << ", Current Offset: " << fpga_.powerboard2_Ioffset[idx2_]
                           << ", Voltage Factor: " << fpga_.powerboard2_Vfactor[idx2_]
                           << ", Voltage Offset: " << fpga_.powerboard2_Voffset[idx2_];
        idx2_++;
    }
}

void Corgi::interruptHandler(core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                             core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                             core::Publisher<motor_msg::MotorStateStamped>& state_pub_, 
                             core::Publisher<config_msg::ConfigStamped>& config_pub_,
                             core::Subscriber<config_msg::ConfigStamped>& config_sub_, 
                             core::Publisher<robot_msg::RobotStateStamped>& robot_state_pub_,
                             core::Subscriber<robot_msg::RobotCmdStamped>& robot_cmd_sub_)


                                                       
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
            LOG_ERROR << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << fpga_.status_;
        }

        uint32_t irq0_cnt;
        uint32_t irq1_cnt;

        NiFpga_MergeStatus(&fpga_.status_,
                           NiFpga_ReadU32(fpga_.session_, NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_IRQ0_cnt, &irq0_cnt));

        NiFpga_MergeStatus(&fpga_.status_,
                           NiFpga_ReadU32(fpga_.session_, NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_IRQ0_cnt, &irq1_cnt));

        if (TimedOut)
        {
            LOG_ERROR << "IRQ timedout, IRQ_0 cnt: " << irq0_cnt << ", IRQ_1 cnt: " << irq1_cnt;
        }

        /* if an IRQ was asserted */
        if (NiFpga_IsNotError(fpga_.status_) && !TimedOut)
        {
            if (irqsAsserted & NiFpga_Irq_0)
            {
                mainLoop_(state_pb_pub_, cmd_sub_, state_pub_, config_pub_, config_sub_, robot_state_pub_, robot_cmd_sub_);
                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
            if (irqsAsserted & NiFpga_Irq_1)
            {
                if (robot_fsm_.getCurrentMode() != RobotMode::MotorConfig)
                {
                    /* Handling CAN-BUS communication */
                    canLoop_();
                }

                // Acknowledge IRQ to begin DMA acquisition
                NiFpga_MergeStatus(&fpga_.status_, NiFpga_AcknowledgeIrqs(fpga_.session_, irqsAsserted));
            }
        }
        usleep(10);

        if (shutdown_requested)
        {
            safeShutdown();
        }
    }
}

void Corgi::mainLoop_(core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                      core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                      core::Publisher<motor_msg::MotorStateStamped>& state_pub_,
                      core::Publisher<config_msg::ConfigStamped>& config_pub_,
                      core::Subscriber<config_msg::ConfigStamped>& config_sub_,
                      core::Publisher<robot_msg::RobotStateStamped>& robot_state_pub_,
                      core::Subscriber<robot_msg::RobotCmdStamped>& robot_cmd_sub_)
{
    fpga_.write_powerboard_(&powerboard1_state_, &powerboard2_state_);
    fpga_.read_powerboard_data_();

    core::spinOnce();
    
    power_msg::PowerStateStamped power_fb_msg;
    motor_msg::MotorStateStamped motor_fb_msg;
    robot_msg::RobotStateStamped robot_fb_msg;
    config_msg::ConfigStamped motor_config_reply;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Handle Robot Command if seq has changed (indicating new command)
        if (robot_cmd_message_updated == 1 && robot_cmd_data.header().seq() != last_robot_cmd_seq) {
            handleRobotCommand(robot_cmd_data);
            last_robot_cmd_seq = robot_cmd_data.header().seq();
            robot_cmd_message_updated = 0;
        }

        // Update error flags and run Robot FSM
        robot_fsm_.setErrorFlags(!NO_CAN_TIMEDOUT_ERROR_, !NO_SWITCH_TIMEDOUT_ERROR_);
        robot_fsm_.runFsm();
        
        // Run Motor FSM
        motor_fsm_.runFsm(motor_fb_msg, motor_cmd_data, motor_config_reply, motor_config_request_data);
        motor_message_updated = 0;
        HALL_CALIBRATED_ = motor_fsm_.isHallCalibrated();

        if (robot_fsm_.getCurrentMode() == RobotMode::MotorConfig && motor_config_message_updated == 1) 
        {
            config_pub_.publish(motor_config_reply);
        }
        motor_config_message_updated = 0;
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
        if (modules_list_[i].enable_ && powerboard1_state_.at(2) == true && powerboard2_state_.at(2) == true)
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
    power_dashboard_reply.set_pb1_digital(powerboard1_state_.at(0));
    power_dashboard_reply.set_pb1_signal(powerboard1_state_.at(1));
    power_dashboard_reply.set_pb1_power(powerboard1_state_.at(2));
    power_dashboard_reply.set_pb2_digital(powerboard2_state_.at(0));
    power_dashboard_reply.set_pb2_signal(powerboard2_state_.at(1));
    power_dashboard_reply.set_pb2_power(powerboard2_state_.at(2));

    if (motor_fsm_.isHallCalibrated() == true && NO_SWITCH_TIMEDOUT_ERROR_==true && NO_CAN_TIMEDOUT_ERROR_==true) power_dashboard_reply.set_clean(true);
    else power_dashboard_reply.set_clean(false);

    power_dashboard_reply.set_pb1_v_0(fpga_.powerboard1_V_list_[0]);
    power_dashboard_reply.set_pb1_i_0(fpga_.powerboard1_I_list_[0]);

    power_dashboard_reply.set_pb1_v_1(fpga_.powerboard1_V_list_[1]);
    power_dashboard_reply.set_pb1_i_1(fpga_.powerboard1_I_list_[1]);

    power_dashboard_reply.set_pb1_v_2(fpga_.powerboard1_V_list_[2]);
    power_dashboard_reply.set_pb1_i_2(fpga_.powerboard1_I_list_[2]);

    power_dashboard_reply.set_pb1_v_3(fpga_.powerboard1_V_list_[3]);
    power_dashboard_reply.set_pb1_i_3(fpga_.powerboard1_I_list_[3]);

    power_dashboard_reply.set_pb1_v_4(fpga_.powerboard1_V_list_[4]);
    power_dashboard_reply.set_pb1_i_4(fpga_.powerboard1_I_list_[4]);

    power_dashboard_reply.set_pb1_v_5(fpga_.powerboard1_V_list_[5]);
    power_dashboard_reply.set_pb1_i_5(fpga_.powerboard1_I_list_[5]);

    power_dashboard_reply.set_pb1_v_6(fpga_.powerboard1_V_list_[6]);
    power_dashboard_reply.set_pb1_i_6(fpga_.powerboard1_I_list_[6]);

    power_dashboard_reply.set_pb1_v_7(fpga_.powerboard1_V_list_[7]);
    power_dashboard_reply.set_pb1_i_7(fpga_.powerboard1_I_list_[7]);

    power_dashboard_reply.set_pb2_v_0(fpga_.powerboard2_V_list_[0]);
    power_dashboard_reply.set_pb2_i_0(fpga_.powerboard2_I_list_[0]);

    power_dashboard_reply.set_pb2_v_1(fpga_.powerboard2_V_list_[1]);
    power_dashboard_reply.set_pb2_i_1(fpga_.powerboard2_I_list_[1]);

    power_dashboard_reply.set_pb2_v_2(fpga_.powerboard2_V_list_[2]);
    power_dashboard_reply.set_pb2_i_2(fpga_.powerboard2_I_list_[2]);

    power_dashboard_reply.set_pb2_v_3(fpga_.powerboard2_V_list_[3]);
    power_dashboard_reply.set_pb2_i_3(fpga_.powerboard2_I_list_[3]);

    power_dashboard_reply.set_pb2_v_4(fpga_.powerboard2_V_list_[4]);
    power_dashboard_reply.set_pb2_i_4(fpga_.powerboard2_I_list_[4]);

    power_dashboard_reply.set_pb2_v_5(fpga_.powerboard2_V_list_[5]);
    power_dashboard_reply.set_pb2_i_5(fpga_.powerboard2_I_list_[5]);

    power_dashboard_reply.set_pb2_v_6(fpga_.powerboard2_V_list_[6]);
    power_dashboard_reply.set_pb2_i_6(fpga_.powerboard2_I_list_[6]);

    power_dashboard_reply.set_pb2_v_7(fpga_.powerboard2_V_list_[7]);
    power_dashboard_reply.set_pb2_i_7(fpga_.powerboard2_I_list_[7]);
}

void Corgi::handleRobotCommand(const robot_msg::RobotCmdStamped& robot_cmd)
{
    robot_msg::ROBOTMODE proto_requested_mode = robot_cmd.request_robot_mode();
    RobotMode requested_mode = static_cast<RobotMode>(proto_requested_mode);
    RobotMode current_mode = robot_fsm_.getCurrentMode();
    
    // Only update mode if it's different from current mode
    if (requested_mode != current_mode) {
        LOG_INFO << "[Robot Command] Mode switch requested from " << static_cast<int>(current_mode) 
                           << " to " << static_cast<int>(requested_mode);
        
        // Use RobotFSM to check if mode transition is available
        bool transition_available = robot_fsm_.requestModeTransition(requested_mode);
        
        if (transition_available) {
            LOG_INFO << "[Robot Mode] Successfully requested transition to mode " << static_cast<int>(requested_mode);
        } else {
            LOG_WARN << "[Robot Mode] Transition denied - invalid or not allowed at this time";
        }
    }
}

void Corgi::safeShutdown()
{
    LOG_INFO << "[FPGA Server] Shutdown signal received, initiating safe shutdown...";
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Step 1: Turn off all power states
        LOG_INFO << "[FPGA Server] Turning off power board...";
        powerboard1_state_[0] = false;  // PB1 digital
        powerboard1_state_[1] = false;  // PB1 signal
        powerboard1_state_[2] = false;  // PB1 power
        powerboard2_state_[0] = false;  // PB2 digital
        powerboard2_state_[1] = false;  // PB2 signal
        powerboard2_state_[2] = false;  // PB2 power
        
        // Write power off command to FPGA
        fpga_.write_powerboard_(&powerboard1_state_, &powerboard2_state_);
    }
    
    // Step 2: Wait 0.5 seconds for power down to complete
    LOG_INFO << "[FPGA Server] Waiting for power down (0.5s)...";
    usleep(500000);  // 500ms
    
    // Step 3: Set sys_stop flag to exit loop
    LOG_INFO << "[FPGA Server] Shutdown complete.";
    sys_stop = 1;
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

    // Initialize global logger with node name
    LOG_INIT("fpga_driver");

    if (argc == 3)
    {
        std::string s(argv[1]);
        if (s == "-t")
        {
            term = std::ofstream(argv[2], std::ios_base::out);
        }
    }
    
    core::NodeHandler nh;
    
    // Create log publisher and set callback for remote logging
    core::Publisher<log_msg::LogEntry>& log_pub = nh.advertise<log_msg::LogEntry>("/log", 100);
    core::GlobalLoggerImpl::instance().setPublishCallback([&log_pub](const log_msg::LogEntry& entry) {
        log_pub.publish(const_cast<log_msg::LogEntry&>(entry));
    });

    LOG_INFO << "[FPGA Server] : Launched";
    
    // Create Corgi instance (Logger is now global, no need to pass)
    Corgi corgi;

    core::Publisher<power_msg::PowerStateStamped>& power_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");

    core::Publisher<motor_msg::MotorStateStamped>& motor_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Subscriber<motor_msg::MotorCmdStamped>& motor_sub = nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_data_cb);
    
    core::Publisher<config_msg::ConfigStamped>& config_pub = nh.advertise<config_msg::ConfigStamped>("motor/config/reply");
    core::Subscriber<config_msg::ConfigStamped>& config_sub = nh.subscribe<config_msg::ConfigStamped>("motor/config/request", 1000, motor_config_cb, 10);

    // Robot gRPC publishers and subscribers
    core::Publisher<robot_msg::RobotStateStamped>& robot_state_pub = nh.advertise<robot_msg::RobotStateStamped>("robot/state");
    core::Subscriber<robot_msg::RobotCmdStamped>& robot_cmd_sub = nh.subscribe<robot_msg::RobotCmdStamped>("robot/command", 1000, robot_cmd_data_cb);
    
    corgi.interruptHandler(power_pub, motor_sub, motor_pub, config_pub, config_sub, robot_state_pub, robot_cmd_sub);

    if (NiFpga_IsError(corgi.fpga_.status_)) {
        LOG_ERROR << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << corgi.fpga_.status_;
    } else {
        endwin();
        LOG_INFO << "[FPGA Server] : Exit Safely";
    }

    return 0;
}

/* CAPTURE SYS STOP SIGNAL TO KILL PROCESS*/
void inthand(int signum)
{
    // Only set atomic flag in signal handler (async-signal-safe)
    shutdown_requested = true;
}



