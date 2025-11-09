#ifndef __CAN_MOTOR_HPP
#define __CAN_MOTOR_HPP

#include "msg.hpp"
#include "mode.hpp"
#include "can_packet.h"
#include <cstdint>
#include <cstring>

// Union for CAN message data
union CANData {
    struct MotorControl {
        float position;
        float torque;
        float kp;
        float ki;
        float kd;
        float kt;
    } control;
    
    struct MotorFeedback {
        float position;
        float velocity;
        float torque;
        uint8_t version;
        uint8_t calibrate_finish;
        uint8_t mode_state;
    } feedback;
    
    struct ConfigData {
        uint8_t raw[8];
    } config;
    
    uint8_t raw[8];
};

class CANMotor {
public:
    CANMotor(uint32_t can_id, const Motor& motor_config);
    
    // Getters
    uint32_t getCANID() const { return can_id_; }
    Mode getMode() const { return current_mode_; }
    const Motor& getConfig() const { return config_; }
    
    // Command data (for MOTOR mode)
    void setCommand(float position, float torque, float kp, float ki, float kd);
    void encodeCommand(uint8_t* msg_out);
    
    // Feedback data
    void parseFeedback(const uint8_t* msg_in);
    float getPosition() const { return feedback_data_.feedback.position - position_bias_; }
    float getVelocity() const { return feedback_data_.feedback.velocity; }
    float getTorque() const { return feedback_data_.feedback.torque; }
    uint8_t getVersion() const { return feedback_data_.feedback.version; }
    uint8_t getCalibrateFinish() const { return feedback_data_.feedback.calibrate_finish; }
    uint8_t getModeState() const { return feedback_data_.feedback.mode_state; }
    
    // Bias management
    void setPositionBias(float bias) { position_bias_ = bias; }
    float getPositionBias() const { return position_bias_; }
    
    // Mode management
    void setMode(Mode mode) { current_mode_ = mode; }
    
private:
    uint32_t can_id_;
    Motor config_;
    float position_bias_;
    Mode current_mode_;
    
    CANData command_data_;
    CANData feedback_data_;
    
    // Encoding/Decoding helpers
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

#endif