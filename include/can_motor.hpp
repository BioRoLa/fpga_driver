#ifndef __CAN_MOTOR_HPP
#define __CAN_MOTOR_HPP

#include "msg.hpp"
#include "mode.hpp"
#include "can_packet.h"
#include <cstdint>
#include <cstring>

#define CAN_DATA_LEN 8

enum ConfigMode : uint8_t
{
    READ = 0,
    WRITE = 1
};

enum ConfigType : uint8_t
{
    INT = 0,
    FLOAT = 1
};

enum ConfigState : uint8_t
{
    CONFIG_SUCCESS = 0,
    INVALID_VALUE = 1,
    READ_ONLY = 2,
    INVALID_ADDR = 3,
    INVALID_CMD = 4
};

union CONFIGData
{
    struct 
    {
        ConfigMode mode;        // 1 byte
        ConfigType type;        // 1 byte
        uint8_t target_addr;    // 1 byte
        union 
        {
            int i;              // 4 bytes
            float f;            // 4 bytes
        } value;
        uint8_t reserved;       // 1 byte
    } config_cmd;

    struct
    {
        ConfigState state;      // 1 byte
        ConfigType type;        // 1 byte
        uint8_t target_addr;    // 1 byte
        union 
        {  
            int i;              // 4 bytes
            float f;            // 4 bytes
        } value;
        uint8_t reserved;       // 1 byte
    } config_fb;

    uint8_t raw_data[8];        // 8 bytes

};

// Union for CAN message data
class CANMotor {
public:
    CANMotor(uint32_t can_id, const Motor& motor_config);
    
    // Getters
    uint32_t getCANID() const { return can_id_; }
    FunctionMode getFunctionMode() const { return current_mode_; }
    const Motor& getConfig() const { return config_; }
    
    // Command data (for MOTOR mode)
    void setCommand(float position, float torque, float kp, float ki, float kd);
    void encodeControl();                               // control_data_ -> command_data_raw
    const uint8_t* getCommandRaw() const { return command_data_raw; }
    
    // Feedback data
    void parseFeedback(const uint8_t* msg_in);          // msg_in -> feedback_data_raw (單純儲存)
    void decodeFeedback();                              // feedback_data_raw -> feedback_data_ (normal mode)
    void decodeBasedOnMode();                           // based on current_mode_
    const uint8_t* getFeedbackRaw() const { return feedback_data_raw; }
    float getPosition() const { return feedback_data_.position - position_bias_; }  // Match main branch logic: decoded - bias
    float getRawPosition() const { return feedback_data_.position; }  // Get raw position without bias
    float getVelocity() const { return feedback_data_.velocity; }
    float getTorque() const { return feedback_data_.torque; }
    uint8_t getVersion() const { return feedback_data_.version; }
    uint8_t getHallCalibrateState() const { return feedback_data_.hall_cal_state; }
    FunctionMode getMotorState() const { return feedback_data_.motor_state; }
    uint8_t getModeState() const { return static_cast<uint8_t>(feedback_data_.motor_state); }  // Legacy compatibility
    
    // Config feedback access
    const CONFIGData& getConfigFeedback() const { return config_fb_data_; }
    
    // Bias management
    void setPositionBias(float bias) { position_bias_ = bias; }
    float getPositionBias() const { return position_bias_; }
    
    // FunctionMode management
    void setMode(FunctionMode mode) { current_mode_ = mode; }
    
private:
    uint32_t can_id_;
    Motor config_;
    float position_bias_;
    FunctionMode current_mode_;

    struct 
    {
        float position;
        float torque;
        float kp;
        float ki;
        float kd;
        float kt;

    } control_data_;

    CONFIGData config_cmd_data_;
    
    uint8_t command_data_raw[8];

    struct 
    {
        float position;
        float velocity;
        float torque;
        uint8_t version;
        uint8_t hall_cal_state;
        FunctionMode motor_state;
    } feedback_data_;

    CONFIGData config_fb_data_;

    uint8_t feedback_data_raw[8];

    // Encoding/Decoding helpers
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

#endif