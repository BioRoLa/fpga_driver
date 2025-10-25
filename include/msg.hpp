#ifndef __MSG_H
#define __MSG_H

#include "mode.hpp"
#include <vector>
#include <math.h>
#include <string>

enum class Data_type
{
    FLOAT,
    INT
};

enum class Config_type
{
    READ,
    WRITE
};

enum class Config_state
{
    CONFIG_SUCCESS,
    INVALID_VALUE,
    READ_ONLY,
    INVALID_ADDR,
    INVALID_CMD
};

typedef union Config_data {
    int i;
    float f;
};

typedef struct Motor
{
    int CAN_ID_;
    int fw_version_;
    double kp_;
    double ki_;
    double kd_;
    double torque_ff_;
    double calibration_bias;
    double kt_;
} Motor;

// transmitted to SBRIO
typedef struct CAN_txcmd
{
    float position_;
    float torque_;
    float KP_;
    float KI_;
    float KD_;
    float KT_;
} CAN_txcmd;

typedef struct CAN_rxcmd
{
    int CAN_id_;
    float position_;
    float velocity_;
    float torque_;
    int version_;
    int calibrate_finish_;
    int mode_state_;
    Mode mode_;
} CAN_rxcmd;

typedef struct CAN_txconfig
{
    Config_type config_type_;
    Data_type data_type_;
    int target_addr_;
    Config_data data_value_;
} CAN_txconfig;

typedef struct CAN_rxconfig
{
    int CAN_id_;
    Config_state config_state_;
    Config_type config_type_;
    int target_addr_;
    Config_data data_value_;
    int mode_state_;
} CAN_rxconfig;

class Module
{
public:
    std::vector<CAN_txcmd> txdata_;
};

#endif
