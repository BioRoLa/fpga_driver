#ifndef __ROBOT_FSM_H
#define __ROBOT_FSM_H

#include <vector>
#include <memory>
#include "mode.hpp"
#include "motor_fsm.hpp"
#include "leg_module.hpp"

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
     */
    RobotFSM(MotorFSM& motor_fsm, std::vector<LegModule>& modules_list, std::vector<bool>& powerboard_state);
    
    RobotFSM() = delete;
    
    /**
     * @brief Main FSM execution - should be called periodically
     */
    void runFsm();
    
    /**
     * @brief Request a mode transition
     * @param next_mode The desired RobotMode
     * @return true if transition is allowed and initiated, false otherwise
     */
    bool requestModeTransition(RobotMode next_mode);
    
    /**
     * @brief Get current robot mode
     */
    RobotMode getCurrentMode() const { return current_mode_; }
    
    /**
     * @brief Check if robot is ready for operation
     */
    bool isReady() const;
    
    /**
     * @brief Emergency stop - immediately transition to safe state
     */
    void emergencyStop();
    
    /**
     * @brief Set error flags for FSM logic
     */
    void setErrorFlags(bool can_error, bool switch_error);
    
    /**
     * @brief Set the main loop period in microseconds
     * @param period_us Period in microseconds (e.g., 500 for 500μs)
     */
    void setLoopPeriod(int period_us);

private:
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
};

#endif // __ROBOT_FSM_H
