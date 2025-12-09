#define PI 3.14159265359f
#define P_CMD_MIN 0.0f
#define P_CMD_MAX 6.283185f  // 359.9999 deg

#define P_FB_MIN -15 * 2 * PI
#define P_FB_MAX 15 * 2 * PI  // rad

#define V_MIN -45.0f
#define V_MAX 45.0f

#define T_MIN -20.0f
#define T_MAX 20.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define KI_MIN 0.0f
#define KI_MAX 10.0f

#define KD_MIN 0.0f
#define KD_MAX 5.0f

// Integer Registers (0 ~ 7)
typedef enum {
    REG_PHASE_ORDER = 0,
    REG_CAN_ID = 1,
    REG_CAN_MASTER = 2,
    REG_CAN_TIMEOUT = 3,
    REG_M_ZERO = 4,
    REG_E_ZERO = 5,
    REG_HALL_CAL_DIR = 6,
    REG_ENCODER_LUT = 7,
    REG_INT_COUNT = 8
} MotorRegInt;

// Float Registers (0 ~ 29)
typedef enum {
    REG_Reserved0 = 0,
    REG_Reserved1 = 1,
    REG_I_BW = 2,
    REG_I_MAX = 3,
    REG_THETA_MIN = 4,
    REG_THETA_MAX = 5,
    REG_I_FW_MAX = 6,
    REG_R_NOMINAL = 7,
    REG_TEMP_MAX = 8,
    REG_I_MAX_CONT = 9,
    REG_PPAIRS = 10,
    REG_Reserved0 = 11,
    REG_Reserved1 = 12,
    REG_R_PHASE = 13,
    REG_KT = 14,
    REG_R_TH = 15,
    REG_C_TH = 16,
    REG_GR = 17,
    REG_I_CAL = 18,
    REG_P_MIN = 19,
    REG_P_MAX = 20,
    REG_V_MIN = 21,  
    REG_V_MAX = 22,
    REG_T_MIN = 23,
    REG_T_MAX = 24,
    REG_KP_MAX = 25, 
    REG_KI_MAX = 26,
    REG_KD_MAX = 27,
    REG_HALL_CAL_OFFSET = 28,
    REG_HALL_CAL_SPEED = 29,
    REG_FLOAT_COUNT = 30
} MotorRegFloat;