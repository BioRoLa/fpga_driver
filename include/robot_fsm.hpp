#ifndef __ROBOT_FSM_H
#define __ROBOT_FSM_H

#include <vector>
#include <memory>
#include "mode.hpp"
#include "motor_fsm.hpp"
#include "leg_module.hpp"
#include "Logger.h"

/**
 * @brief Robot-level Finite State Machine
 * 
 * Manages high-level robot states (SystemOn, Init, IDLE, Standby, MotorConfig)
 * and coordinates lower-level MotorFSM for all leg modules.
 */
class RobotFSM
{
public:
    /**
     * @brief Constructor
     * @param motor_fsm Reference to the motor-level FSM
     * @param modules_list Reference to all leg modules
     * @param powerboard_state Reference to powerboard state vector [digital, signal, power]
     * @param logger Pointer to Logger instance (can be nullptr for local-only logging)
     */
    RobotFSM(MotorFSM& motor_fsm, std::vector<LegModule>& modules_list, 
             std::vector<bool>& powerboard_state, core::Logger* logger = nullptr);
    
    RobotFSM() = delete;
    
    void runFsm();

    bool requestModeTransition(RobotMode next_mode);
    RobotMode getCurrentMode() const { return current_mode_; }
    void emergencyStop();
    void setErrorFlags(bool can_error, bool switch_error);
    void setLoopPeriod(int period_us);
    void setLogger(core::Logger* logger);  // Set logger after construction

private:
    // Constants
    static constexpr int POWER_SWITCH_STABILIZATION_TIME_US = 500000;
    
    // Current state
    RobotMode current_mode_;
    RobotMode previous_mode_;
    
    // References to lower-level components
    MotorFSM& motor_fsm_;
    std::vector<LegModule>& modules_list_;
    std::vector<bool>& powerboard_state_;
    
    // Error flags
    bool has_can_error_;
    bool has_switch_error_;
    
    // Loop timing
    int loop_period_us_;  // Main loop period in microseconds
    
    // State counters for initialization sequences
    int init_step_;
    int init_counter_;  // Counter for timing in init sequence
    int config_step_;
    
    // Private state handler methods
    void handleSystemOn();
    void handleInit();
    void handleIdle();
    void handleStandby();
    void handleMotorConfig();
    
    // Transition validation
    bool isTransitionAllowed(RobotMode from, RobotMode to) const;
    
    // Helper methods
    void enterMode(RobotMode new_mode);
    void exitMode(RobotMode old_mode);
    bool powerSwitchSequence(int& step_counter, int& cycle_counter);
    
    // Convert RobotMode to string
    const char* modeToString(RobotMode mode) const;
    
    // Logger
    core::Logger* logger_;
    core::Logger default_logger_;  // Used if no logger provided
};

#endif // __ROBOT_FSM_H
