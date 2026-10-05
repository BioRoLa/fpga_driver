#include "can_motor.hpp"
#include "Logger.h"
#include <cmath>

CANMotor::CANMotor(uint32_t can_id, const Motor& motor_config)
    : can_id_(can_id)
    , config_(motor_config)
    , position_bias_(motor_config.calibration_bias)
    , current_mode_(FunctionMode::REST)
    , config_sub_mode_(ConfigSubMode::REQUEST_STATE)
{
    // Initialize command data with config defaults
    control_data_.kp = config_.kp_;
    control_data_.ki = config_.ki_;
    control_data_.kd = config_.kd_;
    control_data_.kt = config_.kt_;
    control_data_.position = 0.0f;
    control_data_.torque = 0.0f;
    
    // Initialize feedback data
    feedback_data_.position = 0.0f;
    feedback_data_.velocity = 0.0f;
    feedback_data_.torque = 0.0f;
    feedback_data_.version = 0;
    feedback_data_.hall_cal_state = 0;
    feedback_data_.motor_state = FunctionMode::REST;
}

void CANMotor::setCommand(float position, float torque, float kp, float ki, float kd)
{
    control_data_.position = position;
    control_data_.torque = torque;
    control_data_.kp = kp;
    control_data_.ki = ki;
    control_data_.kd = kd;

    encodeMotorControl();
}

// Saturate a value into [x_min, x_max] before it is packed into a 12-bit
// field. Unclamped, kp=600 wrapped to ~100, a negative kp packed as ~499,
// and an out-of-range ki/torque spilled its high bits into the low nibble
// of kp/kd. NaN maps to x_min (zero gain). Position is deliberately NOT
// routed through here: it relies on the 16-bit wrap to stay modulo 2*pi.
static float saturate(float x, float x_min, float x_max, bool& clipped)
{
    if (std::isnan(x)) { clipped = true; return x_min; }
    if (x < x_min)     { clipped = true; return x_min; }
    if (x > x_max)     { clipped = true; return x_max; }
    return x;
}

void CANMotor::encodeMotorControl()
{
    bool clipped = false;
    float kp = saturate(control_data_.kp, KP_MIN, KP_MAX, clipped);
    float ki = saturate(control_data_.ki, KI_MIN, KI_MAX, clipped);
    float kd = saturate(control_data_.kd, KD_MIN, KD_MAX, clipped);
    float torque = saturate(-control_data_.torque, T_MIN, T_MAX, clipped);
    static int clip_count = 0;  // warn on the first clip, then every 1000th (1 s at 1 kHz)
    if (clipped && clip_count++ % 1000 == 0) {
        LOG_WARN << "[CANMotor 0x" << std::hex << can_id_ << std::dec
                 << "] command out of range, saturated: kp=" << control_data_.kp
                 << " ki=" << control_data_.ki << " kd=" << control_data_.kd
                 << " torque=" << control_data_.torque;
    }

    int pos_int = float_to_uint(-(control_data_.position + position_bias_), P_CMD_MIN, P_CMD_MAX, 16); // Negate to match system coordinate convention
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int ki_int = float_to_uint(ki, KI_MIN, KI_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int torque_int = float_to_uint(torque, T_MIN, T_MAX, 12);
    
    command_data_raw[0] = pos_int >> 8;
    command_data_raw[1] = pos_int & 0xFF;
    command_data_raw[2] = kp_int >> 4;
    command_data_raw[3] = ((kp_int & 0x0F) << 4) | (ki_int >> 8);
    command_data_raw[4] = ki_int & 0xFF;
    command_data_raw[5] = kd_int >> 4;
    command_data_raw[6] = ((kd_int & 0x0F) << 4) | (torque_int >> 8);
    command_data_raw[7] = torque_int & 0xFF;
}

void CANMotor::parseFeedback(const uint8_t* msg_in)
{
    std::memcpy(feedback_data_raw, msg_in, CAN_DATA_LEN);
}

void CANMotor::decodeMotorFeedback()
{
    int pos_raw = ((int)(feedback_data_raw[0]) << 8) | feedback_data_raw[1];
    int vel_raw = ((int)(feedback_data_raw[2]) << 8) | feedback_data_raw[3];
    int torque_raw = ((int)(feedback_data_raw[4]) << 8) | feedback_data_raw[5];

    feedback_data_.position = -uint_to_float(pos_raw, P_FB_MIN, P_FB_MAX, 16); // Negate to match system coordinate convention
    feedback_data_.velocity = -uint_to_float(vel_raw, V_MIN, V_MAX, 16);
    feedback_data_.torque = -uint_to_float(torque_raw, T_MIN, T_MAX, 16);
    feedback_data_.version = feedback_data_raw[7] >> 4;
    feedback_data_.hall_cal_state = feedback_data_raw[6] & 0x0F;

    // Map raw motor state to corresponding FunctionMode
    uint8_t mode_raw = feedback_data_raw[7] & 0x0F;
    feedback_data_.motor_state = mapMotorStateToFunctionMode(mode_raw);
}

void CANMotor::decodeBasedOnMode()
{
    if (current_mode_ == FunctionMode::CONFIG) {
        if (config_sub_mode_ == ConfigSubMode::REQUEST_STATE) {
            // Request state response: decode as motor feedback
            decodeMotorFeedback();
        } else {
            // Config operation: decode as config data
            decodeConfigFeedback();
        }
    } else {
        // In normal modes (MOTOR, REST, SET_ZERO, HALL_CALIBRATE), decode as motor feedback
        decodeMotorFeedback();
    }
}

void CANMotor::setConfigRead(ConfigType type, uint8_t target_addr)
{
    config_cmd_data_.config_cmd.mode = ConfigMode::READ;
    config_cmd_data_.config_cmd.type = type;
    config_cmd_data_.config_cmd.target_addr = target_addr;
    config_cmd_data_.config_cmd.value.i = 0; 

    encodeConfigCommand();
}

void CANMotor::setConfigWriteInt(uint8_t target_addr, int value)
{
    config_cmd_data_.config_cmd.mode = ConfigMode::WRITE;
    config_cmd_data_.config_cmd.type = ConfigType::INT;
    config_cmd_data_.config_cmd.target_addr = target_addr;
    config_cmd_data_.config_cmd.value.i = value;

    encodeConfigCommand();
}

void CANMotor::setConfigWriteFloat(uint8_t target_addr, float value)
{
    config_cmd_data_.config_cmd.mode = ConfigMode::WRITE;
    config_cmd_data_.config_cmd.type = ConfigType::FLOAT;
    config_cmd_data_.config_cmd.target_addr = target_addr;
    config_cmd_data_.config_cmd.value.f = value;

    encodeConfigCommand();
}

void CANMotor::encodeConfigCommand()
{
    // Copy config_cmd_data_ to command_data_raw
    // Caller should set config_cmd_data_.config_cmd fields before calling this
    std::memcpy(command_data_raw, config_cmd_data_.raw_data, CAN_DATA_LEN);
}

void CANMotor::decodeConfigFeedback()
{
    // Copy feedback_data_raw to config_fb_data_
    // Caller can access decoded data via config_fb_data_.config_fb
    std::memcpy(config_fb_data_.raw_data, feedback_data_raw, CAN_DATA_LEN);
}

void CANMotor::encodeRequestState()
{
    // Request motor state: first byte = 255, others = 0
    command_data_raw[0] = 255;
    for (int i = 1; i < CAN_DATA_LEN; i++) {
        command_data_raw[i] = 0;
    }
}

int CANMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float CANMotor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}