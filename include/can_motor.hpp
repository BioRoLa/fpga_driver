#ifndef __CAN_MOTOR_HPP
#define __CAN_MOTOR_HPP

#include "msg.hpp"
#include "mode.hpp"
#include "can_packet.h"
#include <cstdint>
#include <cstring>

#define CAN_DATA_LEN 8

// Config mode sub-states for distinguishing different operations in CONFIG mode
enum class ConfigSubMode : uint8_t
{
    REQUEST_STATE,      // Request motor state (decode as motor feedback)
    CONFIG_OPERATION    // Config read/write operation (access via union)
};

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

# pragma pack(push, 1)
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
# pragma pack(pop)

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
    void encodeMotorControl();                          // control_data_ -> command_data_raw (for motor control)
    void encodeRequestState();                          // Encode request state command (first byte = 255)
    const uint8_t* getCommandRaw() const { return command_data_raw; }
    
    // Command data getters
    float getCommandPosition() const { return control_data_.position; }
    float getCommandTorque() const { return control_data_.torque; }
    float getCommandKp() const { return control_data_.kp; }
    float getCommandKi() const { return control_data_.ki; }
    float getCommandKd() const { return control_data_.kd; }
    
    // Feedback data
    void parseFeedback(const uint8_t* msg_in);          // msg_in -> feedback_data_raw (store raw data)
    void decodeBasedOnMode();                           // Decode based on current_mode_ and config_sub_mode_
    const uint8_t* getFeedbackRaw() const { return feedback_data_raw; }
    float getPosition() const { return feedback_data_.position - position_bias_; }  // Match main branch logic: decoded - bias
    float getRawPosition() const { return feedback_data_.position; }  // Get raw position without bias
    float getVelocity() const { return feedback_data_.velocity; }
    float getTorque() const { return feedback_data_.torque; }
    uint8_t getVersion() const { return feedback_data_.version; }
    uint8_t getHallCalibrateState() const { return feedback_data_.hall_cal_state; }
    FunctionMode getMotorState() const { return feedback_data_.motor_state; }
    uint8_t getModeState() const { return static_cast<uint8_t>(feedback_data_.motor_state); }  // Legacy compatibility
    
    // Command data (for CONFIG mode)
    void setConfigRead(ConfigType type, uint8_t target_addr);
    void setConfigWriteInt(uint8_t target_addr, int value);
    void setConfigWriteFloat(uint8_t target_addr, float value);
    void encodeConfigCommand();                         // Encode config command (config_cmd_data_ -> command_data_raw)
    
    // Config feedback access
    const CONFIGData& getConfigFeedback() const { return config_fb_data_; }
    
    // Bias management
    void setPositionBias(float bias) { position_bias_ = bias; }
    float getPositionBias() const { return position_bias_; }
    
    // FunctionMode management
    void setMode(FunctionMode mode) { current_mode_ = mode; }
    
    // Config sub-mode management (only used when current_mode_ == CONFIG)
    void setConfigSubMode(ConfigSubMode sub_mode) { config_sub_mode_ = sub_mode; }
    ConfigSubMode getConfigSubMode() const { return config_sub_mode_; }
    
    // Config command data access
    CONFIGData& getConfigCommandData() { return config_cmd_data_; }
    
private:
    uint32_t can_id_;
    Motor config_;
    float position_bias_;
    FunctionMode current_mode_;
    ConfigSubMode config_sub_mode_;  // Sub-state for CONFIG mode

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
    
    // Internal decode functions (use decodeBasedOnMode() instead)
    void decodeMotorFeedback();                         // feedback_data_raw -> feedback_data_ (motor feedback)
    void decodeConfigFeedback();                        // feedback_data_raw -> config_fb_data_ (config feedback)
};

#endif