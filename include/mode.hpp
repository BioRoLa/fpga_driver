#ifndef __MODE_H
#define __MODE_H

#include <cstdint>

// ============================================================================
// Motor-level enums (for individual motor control)
// ============================================================================

// Function codes for commands sent to motors
enum class FunctionMode : uint8_t
{
    REST = 0,
    CONFIG = 1,
    SET_ZERO = 2,
    HALL_CALIBRATE = 3,
    MOTOR = 4,
    CONTROL = 5,
    GET_STATE = 6,   // read-only state poll (FC_GET_STATE)
};

// Motor states returned from motor firmware
// Note: These values must match the firmware FSM state definitions
enum class MotorState : uint8_t
{
    REST = 0,
    HALL_CALIBRATE = 1,
    MOTOR = 2, 
    SET_ZERO = 3,
};

// Convert raw motor state value to corresponding FunctionMode
inline FunctionMode mapMotorStateToFunctionMode(uint8_t raw_value)
{
    switch(raw_value) {
        case 0: return FunctionMode::REST;         // Motor state 0 -> FC 0 (REST)
        case 1: return FunctionMode::HALL_CALIBRATE; // Motor state 1 -> FC 3 (HALL_CALIBRATE)
        case 2: return FunctionMode::MOTOR;        // Motor state 2 -> FC 4 (MOTOR)
        case 3: return FunctionMode::SET_ZERO;     // Motor state 3 -> FC 2 (SET_ZERO)
        default: return FunctionMode::REST;
    }
}

// ============================================================================
// Robot-level enums (for overall robot FSM control)
// ============================================================================

enum class RobotMode : uint8_t
{
    SystemOn = 0,
    Init = 1,
    IDLE = 2,
    Standby = 3,
    MotorConfig = 4,
};

#endif