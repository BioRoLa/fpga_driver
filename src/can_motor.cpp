#include "can_motor.hpp"

CANMotor::CANMotor(uint32_t can_id, const Motor& motor_config)
    : can_id_(can_id)
    , config_(motor_config)
    , position_bias_(motor_config.calibration_bias)
    , current_mode_(Mode::REST)
{
    // Initialize command data with config defaults
    command_data_.control.kp = config_.kp_;
    command_data_.control.ki = config_.ki_;
    command_data_.control.kd = config_.kd_;
    command_data_.control.kt = config_.kt_;
    command_data_.control.position = 0.0f;
    command_data_.control.torque = 0.0f;
    
    // Initialize feedback data
    feedback_data_.feedback.position = 0.0f;
    feedback_data_.feedback.velocity = 0.0f;
    feedback_data_.feedback.torque = 0.0f;
    feedback_data_.feedback.version = 0;
    feedback_data_.feedback.calibrate_finish = 0;
    feedback_data_.feedback.mode_state = _REST_MODE;
}

void CANMotor::setCommand(float position, float torque, float kp, float ki, float kd)
{
    command_data_.control.position = position;
    command_data_.control.torque = torque;
    command_data_.control.kp = kp;
    command_data_.control.ki = ki;
    command_data_.control.kd = kd;
}

void CANMotor::encodeCommand(uint8_t* msg_out)
{
    int pos_int = float_to_uint(-command_data_.control.position - position_bias_, P_CMD_MIN, P_CMD_MAX, 16);
    int kp_int = float_to_uint(command_data_.control.kp, KP_MIN, KP_MAX, 12);
    int ki_int = float_to_uint(command_data_.control.ki, KI_MIN, KI_MAX, 12);
    int kd_int = float_to_uint(command_data_.control.kd, KD_MIN, KD_MAX, 12);
    int torque_int = float_to_uint(command_data_.control.torque, T_MIN, T_MAX, 12);
    
    msg_out[0] = pos_int >> 8;
    msg_out[1] = pos_int & 0xFF;
    msg_out[2] = kp_int >> 4;
    msg_out[3] = ((kp_int & 0x0F) << 4) | (ki_int >> 8);
    msg_out[4] = ki_int & 0xFF;
    msg_out[5] = kd_int >> 4;
    msg_out[6] = ((kd_int & 0x0F) << 4) | (torque_int >> 8);
    msg_out[7] = torque_int & 0xFF;
}

void CANMotor::parseFeedback(const uint8_t* msg_in)
{
    int pos_raw = ((int)(msg_in[0]) << 8) | msg_in[1];
    int vel_raw = ((int)(msg_in[2]) << 8) | msg_in[3];
    int torque_raw = ((int)(msg_in[4]) << 8) | msg_in[5];
    
    feedback_data_.feedback.position = -uint_to_float(pos_raw, P_FB_MIN, P_FB_MAX, 16);
    feedback_data_.feedback.velocity = uint_to_float(vel_raw, V_MIN, V_MAX, 16);
    feedback_data_.feedback.torque = uint_to_float(torque_raw, T_MIN, T_MAX, 16);
    feedback_data_.feedback.version = msg_in[7] >> 4;
    feedback_data_.feedback.calibrate_finish = msg_in[6] & 0x0F;
    feedback_data_.feedback.mode_state = msg_in[7] & 0x0F;
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