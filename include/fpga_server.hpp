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
#define CONFIG_PATH "/home/admin/corgi_ws/fpga_driver/config/config.yaml"
#endif

volatile sig_atomic_t sys_stop;
void inthand(int signum);

class Corgi
{
  public:
    Corgi();
    void load_config_();

    YAML::Node yaml_node_;

    FpgaHandler fpga_;
    Console console_;

    Scenario scenario_;

    ModeFsm fsm_;
    std::vector<LegModule> modules_list_;
    std::vector<bool> powerboard_state_;
    std::mutex main_mtx_;

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

    void interruptHandler(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                          core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                          core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                          core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

    void powerboardPack(power_msg::PowerStateStamped &power_fb_msg);
    void mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                   core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                   core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                   core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

    void canLoop_();

    bool hall_complete;

    int32_t r_hall;
    int32_t l_hall;
    int32_t zero_offset;
};
