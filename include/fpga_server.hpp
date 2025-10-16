#include "fpga_handler.hpp"
#include "leg_module.hpp"
#include "console.hpp"
#include "fsm.hpp"

#include <NodeHandler.h>
#include <sys/time.h>
#include <fstream>
#include <yaml.h>
#include <string>
#include <vector>
#include <mutex>

#ifndef CONFIG_PATH
// #define CONFIG_PATH "/home/admin/corgi_ws/fpga_driver/config/config.yaml"
#define CONFIG_PATH "/home/hiho817/corgi_ws/fpga_driver/config/config.yaml" // Change to your local path
#endif

volatile sig_atomic_t sys_stop;
void inthand(int signum);

class Corgi
{
  public:
    Corgi();
    void load_config_();

    YAML::Node yaml_node_;
    int modules_num_;

    FpgaHandler fpga_;
    Console console_;

    Scenario scenario_;

    ModeFsm fsm_;
    std::vector<LegModule> modules_list_;
    std::vector<bool> powerboard_state_;
    std::mutex main_mtx_;

    std::ofstream MSG_Stream;

    // header msg
    struct timeval t_stamp;
    int seq;

    int main_irq_period_us_;
    int can_irq_period_us_;

    int max_timeout_cnt_;
    int timeout_cnt_;

    bool NO_SWITCH_TIMEDOUT_ERROR_;
    bool NO_CAN_TIMEDOUT_ERROR_;
    bool HALL_CALIBRATED_;

    bool digital_switch_;
    bool signal_switch_;
    bool power_switch_;
    bool stop_;
    bool vicon_trigger_;

    bool Steer_Cali;

    void interruptHandler(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                          core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                          core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                          core::Publisher<motor_msg::MotorStateStamped>& state_pub_,
                          core::Subscriber<steering_msg::SteeringCmdStamped>& steer_sub,
                          core::Publisher<steering_msg::SteeringStateStamped>& steer_pub);

    void powerboardPack(power_msg::PowerStateStamped &power_fb_msg);
    void steeringPack(steering_msg::SteeringStateStamped &steer_fb_msg);
    void mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                   core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                   core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                   core::Publisher<motor_msg::MotorStateStamped>& state_pub_,
                   core::Subscriber<steering_msg::SteeringCmdStamped>& steer_sub,
                   core::Publisher<steering_msg::SteeringStateStamped>& steer_pub);

    void canLoop_();
    void logger_init();
    void logger(int seq);
    double logbuf[100][134];
    void steering_calibration();

    int log_data;
    std::string log_path;
    std::ofstream log_stream;
  
    int32_t steering_cali_state;
    int32_t steering_state_complete;
    int32_t voltage;

    bool hall_complete;
    int32_t steer_position;

    int32_t r_hall;
    int32_t l_hall;
    int32_t zero_offset;
    double steer_current_angle;
    bool steering_state;
};
