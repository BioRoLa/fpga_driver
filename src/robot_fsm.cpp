#include "robot_fsm.hpp"
#include "console.hpp"
#include <iostream>
#include <sys/time.h>

RobotFSM::RobotFSM(MotorFSM& motor_fsm, std::vector<LegModule>& modules_list, bool& powerboard_state)
    : motor_fsm_(motor_fsm)
    , modules_list_(modules_list)
    , powerboard_state_(powerboard_state)
    , current_mode_(RobotMode::SystemOn)
    , previous_mode_(RobotMode::SystemOn)
    , has_can_error_(false)
    , has_switch_error_(false)
    , loop_period_us_(500)  // Default 500us
    , init_step_(0)
    , init_counter_(0)
    , config_step_(0)
{
    std::cout << green << "[Robot FSM] Initialized in SystemOn mode" << reset << std::endl;
}

void RobotFSM::runFsm()
{
    // Execute behavior for current mode
    switch (current_mode_)
    {
        case RobotMode::SystemOn:
            handleSystemOn();
            break;
            
        case RobotMode::Init:
            handleInit();
            break;
            
        case RobotMode::IDLE:
            handleIdle();
            break;
            
        case RobotMode::Standby:
            handleStandby();
            break;
            
        case RobotMode::MotorConfig:
            handleMotorConfig();
            break;
            
        default:
            std::cout << red << "[Robot FSM] Unknown mode!" << reset << std::endl;
            emergencyStop();
            break;
    }
}

bool RobotFSM::requestModeTransition(RobotMode next_mode)
{
    if (current_mode_ == next_mode)
    {
        std::cout << yellow << "[Robot FSM] Already in requested mode" << reset << std::endl;
        return true;
    }
    
    if (!isTransitionAllowed(current_mode_, next_mode))
    {
        std::cout << red << "[Robot FSM] Transition from " 
                  << static_cast<int>(current_mode_) << " to " 
                  << static_cast<int>(next_mode) << " not allowed" 
                  << reset << std::endl;
        return false;
    }
    
    // Perform state transition
    exitMode(current_mode_);
    previous_mode_ = current_mode_;
    current_mode_ = next_mode;
    enterMode(current_mode_);
    
    return true;
}

void RobotFSM::handleSystemOn()
{
    // SystemOn: Safe state
    // Ensure motors are in REST mode and power is OFF
    
    // Ensure all motors are in REST mode
    if (motor_fsm_.getCurrentMode() != FunctionMode::REST)
    {
        motor_fsm_.switchMode(FunctionMode::REST);
    }
    
    // Ensure power is OFF
    if (powerboard_state_ != false)
    {
        powerboard_state_ = false;
    }
}

void RobotFSM::handleInit()
{
    // Init: System initialization sequence
    // Steps:
    // 0. Set power ON and wait 0.5 seconds
    // 1. Check motor timeout
    // 2. Set motor FSM to SET_ZERO
    // 3. Set motor FSM to HALL_CALIBRATE
    // 4. Check motor FSM in MOTOR mode, then transition to IDLE
    
    switch (init_step_)
    {
        case 0:
            {
                // Set power ON
                if (!powerboard_state_)
                {
                    std::cout << cyan << "[Robot FSM] Init Step 0: Turning power ON..." 
                              << reset << std::endl;
                    powerboard_state_ = true;
                    init_counter_ = 0;
                }
                
                // Wait for 0.5 seconds
                init_counter_++;
                int required_cycles = 500000 / loop_period_us_;  // 0.5 seconds
                if (init_counter_ >= required_cycles)
                {
                    std::cout << cyan << "[Robot FSM] Init Step 0: Power stabilized" 
                              << reset << std::endl;
                    init_step_++;
                    init_counter_ = 0;
                }
            }
            break;
            
        case 1:
            // Check motor timeout
            {
                bool has_timeout = false;
                for (auto& module : modules_list_)
                {
                    if (module.hasTimeout())
                    {
                        has_timeout = true;
                        std::cout << red << "[Robot FSM] Init Step 1: Module " 
                                  << module.label_ << " has timeout!" << reset << std::endl;
                        break;
                    }
                }
                
                if (has_timeout)
                {
                    std::cout << red << "[Robot FSM] Init failed: Motor timeout detected" 
                              << reset << std::endl;
                    emergencyStop();
                }
                else
                {
                    std::cout << cyan << "[Robot FSM] Init Step 1: All motors responsive" 
                              << reset << std::endl;
                    init_step_++;
                }
            }
            break;
            
        case 2:
            // Set motor FSM to SET_ZERO mode
            std::cout << cyan << "[Robot FSM] Init Step 2: Setting motors to SET_ZERO..." 
                      << reset << std::endl;
            motor_fsm_.switchMode(FunctionMode::SET_ZERO);
            init_step_++;
            break;
            
        case 3:
            // Set motor FSM to HALL_CALIBRATE mode
            std::cout << cyan << "[Robot FSM] Init Step 3: Starting HALL_CALIBRATE..." 
                      << reset << std::endl;
            motor_fsm_.switchMode(FunctionMode::HALL_CALIBRATE);
            init_step_++;
            break;
            
        case 4:
            // Wait for HALL_CALIBRATE to complete and motor FSM to enter MOTOR mode
            if (motor_fsm_.getCurrentMode() == FunctionMode::MOTOR)
            {
                std::cout << green << "[Robot FSM] Init Step 4: Motors in MOTOR mode" 
                          << reset << std::endl;
                std::cout << green << "[Robot FSM] Init complete, transitioning to IDLE" 
                          << reset << std::endl;
                requestModeTransition(RobotMode::IDLE);
            }
            break;
    }
}

void RobotFSM::handleIdle()
{
    // IDLE: System ready but not active
    // Motors should be in MOTOR mode, but not receiving gRPC commands
    // Motors will hold their last command position
    
    // Ensure motors are in MOTOR mode
    if (motor_fsm_.getCurrentMode() != FunctionMode::MOTOR)
    {
        motor_fsm_.switchMode(FunctionMode::MOTOR);
    }
    
    // Check for errors - if errors detected, go back to SystemOn
    if (has_can_error_ || has_switch_error_)
    {
        std::cout << red << "[Robot FSM] Error detected in IDLE, returning to SystemOn" 
                  << reset << std::endl;
        requestModeTransition(RobotMode::SystemOn);
    }
}

void RobotFSM::handleStandby()
{
    // Standby: Ready for active control
    // Motors should be in MOTOR mode (torque control enabled)
    
    if (motor_fsm_.getCurrentMode() != FunctionMode::MOTOR)
    {
        motor_fsm_.switchMode(FunctionMode::MOTOR);
    }
    
    // Check for errors
    if (has_can_error_ || has_switch_error_)
    {
        std::cout << red << "[Robot FSM] Error detected, returning to IDLE" 
                  << reset << std::endl;
        requestModeTransition(RobotMode::IDLE);
    }
}

void RobotFSM::handleMotorConfig()
{
    // MotorConfig: Motor configuration
    //TODO: implement motor configuration steps
}

bool RobotFSM::isTransitionAllowed(RobotMode from, RobotMode to) const
{
    // Define allowed state transitions
    // This is a state machine transition table
    
    switch (from)
    {
        case RobotMode::SystemOn:
            return (to == RobotMode::Init);
            
        case RobotMode::Init:
            return (to == RobotMode::IDLE || to == RobotMode::SystemOn);
            
        case RobotMode::IDLE:
            return (to == RobotMode::Standby || 
                    to == RobotMode::MotorConfig || 
                    to == RobotMode::SystemOn);
            
        case RobotMode::Standby:
            return (to == RobotMode::IDLE);
            
        case RobotMode::MotorConfig:
            return (to == RobotMode::SystemOn);
            
        default:
            return false;
    }
}

void RobotFSM::enterMode(RobotMode new_mode)
{
    std::cout << green << "[Robot FSM] Entering mode: " 
              << static_cast<int>(new_mode) << reset << std::endl;
    
    // Reset state-specific counters
    switch (new_mode)
    {
        case RobotMode::Init:
            init_step_ = 0;
            init_counter_ = 0;
            break;
            
        case RobotMode::MotorConfig:
            config_step_ = 0;
            break;
            
        default:
            break;
    }
}

void RobotFSM::exitMode(RobotMode old_mode)
{
    std::cout << yellow << "[Robot FSM] Exiting mode: " 
              << static_cast<int>(old_mode) << reset << std::endl;
    
    // Cleanup actions when leaving a mode
    // Add specific cleanup logic here if needed
}

void RobotFSM::emergencyStop()
{
    std::cout << red << "[Robot FSM] EMERGENCY STOP!" << reset << std::endl;
    
    // Force transition to SystemOn (safe state)
    previous_mode_ = current_mode_;
    current_mode_ = RobotMode::SystemOn;
    
    // Stop all motors
    motor_fsm_.switchMode(FunctionMode::REST);
}

bool RobotFSM::isReady() const
{
    return (current_mode_ == RobotMode::Standby || 
            current_mode_ == RobotMode::IDLE);
}

void RobotFSM::setErrorFlags(bool can_error, bool switch_error)
{
    has_can_error_ = can_error;
    has_switch_error_ = switch_error;
}

void RobotFSM::setLoopPeriod(int period_us)
{
    loop_period_us_ = period_us;
}