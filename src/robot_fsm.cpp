#include "robot_fsm.hpp"
#include "console.hpp"
#include <iostream>
#include <sys/time.h>

RobotFSM::RobotFSM(MotorFSM& motor_fsm, std::vector<LegModule>& modules_list, 
                   std::vector<bool>& powerboard_state, double* powerboard_voltage)
    : motor_fsm_(motor_fsm)
    , modules_list_(modules_list)
    , powerboard_state_(powerboard_state)
    , powerboard_voltage_(powerboard_voltage)
    , current_mode_(RobotMode::SystemOn)
    , previous_mode_(RobotMode::SystemOn)
    , has_can_error_(false)
    , has_switch_error_(false)
    , loop_period_us_(500)  // Default 500us
    , init_step_(0)
    , init_counter_(0)
    , config_step_(0)
{
    LOG_INFO << "Initialized in SystemOn mode";
}

const char* RobotFSM::modeToString(RobotMode mode) const
{
    switch (mode) {
        case RobotMode::SystemOn:    return "SystemOn";
        case RobotMode::Init:        return "Init";
        case RobotMode::IDLE:        return "IDLE";
        case RobotMode::Standby:     return "Standby";
        case RobotMode::MotorConfig: return "MotorConfig";
        default:                     return "Unknown";
    }
}

void RobotFSM::runFsm()
{
    // Update timeout debounce for all modules
    for (auto& module : modules_list_)
    {
        module.updateTimeoutDebounce(loop_period_us_);
    }
    
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
            LOG_ERROR << "Unknown mode!";
            emergencyStop();
            break;
    }
}

bool RobotFSM::requestModeTransition(RobotMode next_mode)
{
    if (current_mode_ == next_mode)
    {
        LOG_DEBUG << "Already in requested mode: " << modeToString(next_mode);
        return true;
    }
    
    // Special case: SystemOn -> IDLE request will go through Init first
    if (current_mode_ == RobotMode::SystemOn && next_mode == RobotMode::IDLE)
    {
        LOG_INFO << "SystemOn -> IDLE requested, going through Init first";
        exitMode(current_mode_);
        previous_mode_ = current_mode_;
        current_mode_ = RobotMode::Init;
        enterMode(current_mode_);
        return true;
    }
    
    if (!isTransitionAllowed(current_mode_, next_mode))
    {
        LOG_WARN << "Transition from " << modeToString(current_mode_) 
                           << " to " << modeToString(next_mode) << " not allowed";
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
    
    // Ensure all power switches are OFF
    if (powerboard_state_.at(0) != false || powerboard_state_.at(1) != false || powerboard_state_.at(2) != false)
    {
        powerboard_state_.at(0) = false;  // Digital OFF
        powerboard_state_.at(1) = false;  // Signal OFF
        powerboard_state_.at(2) = false;  // Power OFF
    }
}

void RobotFSM::handleInit()
{
    // Init: System initialization sequence
    // Steps:
    // 0. Turn on digital switch and wait 1 second
    // 1. Turn on signal switch and wait 1 second
    // 2. Turn on power switch and wait 1 second
    // 3. Check motor timeout
    // 4. Set motor FSM to SET_ZERO
    // 5. Set motor FSM to HALL_CALIBRATE
    // 6. Check motor FSM in MOTOR mode, then transition to IDLE
    
    switch (init_step_)
    {
        case 0:
        case 1:
        case 2:
            // Power switch sequence: digital -> signal -> power
            if (powerSwitchSequence(init_step_, init_counter_))
            {
                init_step_++;
                init_counter_ = 0;
            }
            break;
            
        case 3:
            // Check motor timeout and emergency stop
            {
                bool has_timeout = false;
                for (auto& module : modules_list_)
                {
                    if (module.hasTimeout())
                    {
                        has_timeout = true;
                        LOG_ERROR << "Init Step 3: Module " << module.label_ << " has timeout!";
                        break;
                    }
                }
                
                if (has_timeout)
                {
                    LOG_FATAL << "Init failed: Motor timeout detected";
                    emergencyStop();
                }
                else
                {
                    LOG_INFO << "Init Step 3: All motors responsive";
                    
                    if (checkEStop())
                    {
                        LOG_ERROR << "Init failed: Emergency stop is pressed!";
                        LOG_ERROR << "Please release emergency stop button and ensure power output is normal.";
                        emergencyStop();
                    }
                    else
                    {
                        LOG_INFO << "Init Step 3: Emergency stop check passed, power output normal";
                        init_step_++;
                    }
                }
            }
            break;
            
        case 4:
            // Set motor FSM to SET_ZERO mode
            LOG_INFO << "Init Step 4: Setting motors to SET_ZERO...";
            motor_fsm_.switchMode(FunctionMode::SET_ZERO);
            init_step_++;
            break;
            
        case 5:
            // Set motor FSM to HALL_CALIBRATE mode and wait for calibration to complete
            if (motor_fsm_.getCurrentMode() != FunctionMode::HALL_CALIBRATE)
            {
                LOG_INFO << "Init Step 5: Starting HALL_CALIBRATE...";
                motor_fsm_.switchMode(FunctionMode::HALL_CALIBRATE);
            }
            
            // Wait until calibration is complete
            if (motor_fsm_.isHallCalibrated())
            {
                LOG_INFO << "Init Step 5: Hall calibration complete";
                init_step_++;
            }
            
            break;
            
        case 6:
            // Wait for HALL_CALIBRATE to complete and motor FSM to enter MOTOR mode
            if (motor_fsm_.getCurrentMode() == FunctionMode::MOTOR)
            {
                LOG_INFO << "Init Step 6: Motors in MOTOR mode";
                LOG_INFO << "Init complete, transitioning to IDLE";
                // Direct transition to IDLE without request
                exitMode(current_mode_);
                previous_mode_ = current_mode_;
                current_mode_ = RobotMode::IDLE;
                enterMode(current_mode_);
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
    
    // Check for emergency stop
    if (checkEStop())
    {
        LOG_ERROR << "Emergency stop detected in IDLE, returning to SystemOn";
        emergencyStop();
        return;
    }
    
    // Check for errors - if errors detected, go back to SystemOn
    // Use LOG_CHANGED to only log when error state changes (not every loop)
    if (has_can_error_ || has_switch_error_)
    {
        LOG_ERROR_ONCE << "Error detected in IDLE, returning to SystemOn";
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
    
    // Check for emergency stop
    if (checkEStop())
    {
        LOG_ERROR << "Emergency stop detected in Standby, returning to SystemOn";
        emergencyStop();
        return;
    }
    
    // Check for errors - only log when error state changes
    if (has_can_error_ || has_switch_error_)
    {
        LOG_ERROR_ONCE << "Error detected in Standby, returning to IDLE";
        requestModeTransition(RobotMode::IDLE);
    }
}

void RobotFSM::handleMotorConfig()
{
    // MotorConfig: Motor configuration
    // Steps:
    // 0-2. Power switch sequence (digital -> signal -> power)
    // 3. Complete configuration
    
    
    switch (config_step_)
    {
        case 0:
        case 1:
        case 2:
        // Power switch sequence: digital -> signal -> power
        if (powerSwitchSequence(config_step_, init_counter_))
        {
            config_step_++;
            init_counter_ = 0;
        }
        break;
        
        case 3:
        // Ensure motors are in CONFIG mode
        if (motor_fsm_.getCurrentMode() != FunctionMode::CONFIG)
        {
            if (motor_fsm_.switchMode(FunctionMode::CONFIG))
            {
                LOG_INFO << "Step 3: Motors set to CONFIG mode";
            }
            else
            {
                LOG_ERROR_EVERY_N(3) << "Step 3: Failed to set motors to CONFIG mode";
            }
        }   
        break;
    }
    // Check for emergency stop
    if (checkEStop() && (config_step_ == 3))
    {
        LOG_ERROR << "Emergency stop detected in MotorConfig, returning to SystemOn";
        emergencyStop();
        return;
    }
}

bool RobotFSM::isTransitionAllowed(RobotMode from, RobotMode to) const
{
    // Define allowed state transitions
    // This is a state machine transition table
    
    switch (from)
    {
        case RobotMode::SystemOn:
            return (to == RobotMode::Init ||
                    to == RobotMode::MotorConfig);
            
        case RobotMode::Init:
            return (to == RobotMode::SystemOn);
            
        case RobotMode::IDLE:
            return (to == RobotMode::Standby || 
                    to == RobotMode::MotorConfig || 
                    to == RobotMode::SystemOn);
            
        case RobotMode::Standby:
            return (to == RobotMode::IDLE ||
                    to == RobotMode::SystemOn);
            
        case RobotMode::MotorConfig:
            return (to == RobotMode::SystemOn);
            
        default:
            return false;
    }
}

void RobotFSM::enterMode(RobotMode new_mode)
{
    LOG_INFO << "Entering mode: " << modeToString(new_mode);
    
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
    LOG_INFO << "Exiting mode: " << modeToString(old_mode);
    
    // Cleanup actions when leaving a mode
    // Add specific cleanup logic here if needed
}

void RobotFSM::emergencyStop()
{
    LOG_FATAL << "EMERGENCY STOP!";
    
    // Force transition to SystemOn (safe state)
    requestModeTransition(RobotMode::SystemOn);
    
    // Stop all motors
    motor_fsm_.switchMode(FunctionMode::REST);
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

bool RobotFSM::powerSwitchSequence(int& step_counter, int& cycle_counter)
{
    // Power switch sequence: digital (step 0) -> signal (step 1) -> power (step 2)
    // Each step waits for 1 second before moving to the next
    // Returns true when the current step is complete
    
    const char* switch_names[] = {"digital", "signal", "power"};
    int switch_index = step_counter;
    
    if (switch_index < 0 || switch_index > 2)
    {
        LOG_ERROR << "Invalid power switch step: " << switch_index;
        return true;  // Skip invalid step
    }
    
    // On first cycle, check if switch is already on and turn it on if not
    if (cycle_counter == 0)
    {
        // Check if switch is already on
        if (powerboard_state_.at(switch_index))
        {
            LOG_DEBUG << "Step " << switch_index << ": " << switch_names[switch_index] 
                                << " switch already ON, skipping wait";
            return true;  // Already on, no need to wait
        }
        
        // Turn on the switch
        LOG_INFO << "Step " << switch_index << ": Turning " 
                           << switch_names[switch_index] << " switch ON...";
        powerboard_state_.at(switch_index) = true;
    }
    
    cycle_counter++;
    int required_cycles = POWER_SWITCH_STABILIZATION_TIME_US / loop_period_us_;
    
    if (cycle_counter >= required_cycles)
    {
        LOG_INFO << "Step " << switch_index << ": " 
                           << switch_names[switch_index] << " switch stabilized";
        return true;  // Step complete
    }
    
    return false;  // Still waiting
}

bool RobotFSM::checkEStop()
{
    
    if (!powerboard_voltage_) {
        LOG_ERROR << "Powerboard voltage pointer is null!";
        return false;
    }
    
    for (int i = 2; i <= 11; i++) {
        if (powerboard_voltage_[i] < ESTOP_VOLTAGE_THRESHOLD) {
            LOG_WARN << "EStop detected: Power[" << i 
                     << "] voltage = " << powerboard_voltage_[i] 
                     << "V (threshold: " << ESTOP_VOLTAGE_THRESHOLD << "V)";
            return true;
        }
    }
    
    return false;
}