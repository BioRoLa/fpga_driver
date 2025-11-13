#ifndef __MSG_H
#define __MSG_H

#include "mode.hpp"
#include <vector>
#include <math.h>
#include <string>

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

#endif
