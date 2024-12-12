#ifndef __MODE_H
#define __MODE_H

#define _REST_MODE 0
<<<<<<< HEAD
#define _HALL_CALIBRATE 1
#define _MOTOR_MODE 2
#define _SET_ZERO 3
=======
#define _CONFIG_MODE 1
#define _SET_ZERO 2
#define _HALL_CALIBRATE 3
#define _MOTOR_MODE 4


>>>>>>> 8402645366b4fe92aef8ad10a2112812b90f87be

enum class Mode
{
    REST,
    CONFIG,
    SET_ZERO,
    HALL_CALIBRATE,
    MOTOR,
<<<<<<< HEAD
    CONTROL,
=======
    
>>>>>>> 8402645366b4fe92aef8ad10a2112812b90f87be
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