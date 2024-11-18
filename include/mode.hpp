#ifndef __MODE_H
#define __MODE_H

#define _REST_MODE 0
#define _SET_ZERO 1
#define _HALL_CALIBRATE 2 
#define _MOTOR_MODE 3
#define _CONFIG_MODE 4


enum class Mode
{
    REST,
    SET_ZERO,
    HALL_CALIBRATE,
    MOTOR,
    CONFIG
};

enum class Behavior
{
    SET_THETA,
    TCP_SLAVE,
    CUSTOM_1,
    CUSTOM_2,
    CUSTOM_3
};

#endif