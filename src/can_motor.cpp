#include "can_motor.hpp"

CANMotor::CANMotor(uint32_t can_id, const Motor& motor_config)
    : can_id_(can_id)
    , config_(motor_config)
    , position_bias_(motor_config.calibration_bias)
    , current_mode_(Mode::REST)
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
    feedback_data_.calibrate_finish = 0;
    feedback_data_.mode_state = _REST_MODE;
}

void CANMotor::setCommand(float position, float torque, float kp, float ki, float kd)
{
    control_data_.position = position;
    control_data_.torque = torque;
    control_data_.kp = kp;
    control_data_.ki = ki;
    control_data_.kd = kd;

    encodeControl();
}

void CANMotor::encodeControl()
{
    int pos_int = float_to_uint(-control_data_.position + position_bias_, P_CMD_MIN, P_CMD_MAX, 16);
    int kp_int = float_to_uint(control_data_.kp, KP_MIN, KP_MAX, 12);
    int ki_int = float_to_uint(control_data_.ki, KI_MIN, KI_MAX, 12);
    int kd_int = float_to_uint(control_data_.kd, KD_MIN, KD_MAX, 12);
    int torque_int = float_to_uint(control_data_.torque, T_MIN, T_MAX, 12);
    
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
    std::memcpy(feedback_data_raw, msg_in, 8); //FIXME: substitute magic number
}

void CANMotor::decodeFeedback()
{
    int pos_raw = ((int)(feedback_data_raw[0]) << 8) | feedback_data_raw[1];
    int vel_raw = ((int)(feedback_data_raw[2]) << 8) | feedback_data_raw[3];
    int torque_raw = ((int)(feedback_data_raw[4]) << 8) | feedback_data_raw[5];

    feedback_data_.position = -uint_to_float(pos_raw, P_FB_MIN, P_FB_MAX, 16);
    feedback_data_.velocity = uint_to_float(vel_raw, V_MIN, V_MAX, 16);
    feedback_data_.torque = uint_to_float(torque_raw, T_MIN, T_MAX, 16);
    feedback_data_.version = feedback_data_raw[7] >> 4;
    feedback_data_.calibrate_finish = feedback_data_raw[6] & 0x0F;
    feedback_data_.mode_state = feedback_data_raw[7] & 0x0F;
}

void CANMotor::decodeBasedOnMode()
{
    if (current_mode_ == Mode::CONFIG) {
        // In CONFIG mode, decode as config feedback
        decodeConfig();
    } else {
        // In normal modes (MOTOR, REST, SET_ZERO, HALL_CALIBRATE), decode as motor feedback
        decodeFeedback();
    }
}

void CANMotor::setConfigRead(ConfigType type, uint8_t target_addr)
{
    config_cmd_data_.config_cmd.mode = ConfigMode::READ;
    config_cmd_data_.config_cmd.type = type;
    config_cmd_data_.config_cmd.target_addr = target_addr;
    config_cmd_data_.config_cmd.value.i = 0; 

    encodeConfig();
}

void CANMotor::setConfigWriteInt(uint8_t target_addr, int value)
{
    config_cmd_data_.config_cmd.mode = ConfigMode::WRITE;
    config_cmd_data_.config_cmd.type = ConfigType::INT;
    config_cmd_data_.config_cmd.target_addr = target_addr;
    config_cmd_data_.config_cmd.value.i = value;

    encodeConfig();
}

void CANMotor::setConfigWriteFloat(uint8_t target_addr, float value)
{
    config_cmd_data_.config_cmd.mode = ConfigMode::WRITE;
    config_cmd_data_.config_cmd.type = ConfigType::FLOAT;
    config_cmd_data_.config_cmd.target_addr = target_addr;
    config_cmd_data_.config_cmd.value.f = value;

    encodeConfig();
}

void CANMotor::encodeConfig()
{
    command_data_raw[0] = config_cmd_data_.config_cmd.mode;
    command_data_raw[1] = config_cmd_data_.config_cmd.type;
    command_data_raw[2] = config_cmd_data_.config_cmd.target_addr;

    // TODO: Verify if the motor receives the correct float value.
    // If the value is wrong (e.g., extremely large or small), implement Byte-Swapping since the motor expects Big-Endian format.

    if(config_cmd_data_.config_cmd.mode == ConfigMode::WRITE) {
        if(config_cmd_data_.config_cmd.type = ConfigType::INT){
            std::memcpy(&command_data_raw[3], &config_cmd_data_.config_cmd.value.i, 4);
        }
        else{
            std::memcpy(&command_data_raw[3], &config_cmd_data_.config_cmd.value.f, 4);
        }
    }
    else{
        std::memset(&command_data_raw[3], 0, 4);
    }
    command_data_raw[7] = 0; 
}

void CANMotor::decodeConfig()
{
    config_fb_data_.config_fb.state = feedback_data_raw[0];
    config_fb_data_.config_fb.type = feedback_data_raw[1];
    config_fb_data_.config_fb.target_addr = feedback_data_raw[2];

    if(config_fb_data_.config_fb.type == ConfigType::INT){
        std::memcpy(&config_fb_data_.config_fb.value.i, &feedback_data_raw[3], 4);
    }
    else{
        std::memcpy(&config_fb_data_.config_fb.value.f, &feedback_data_raw[3], 4);
    }

    feedback_data_.version = feedback_data_raw[7] >> 4;
    feedback_data_.mode_state = feedback_data_raw[7] & 0x0F;

    std::memcpy(config_fb_data_.raw_data, feedback_data_raw, 8);
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