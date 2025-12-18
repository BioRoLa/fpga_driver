#include "fpga_handler.hpp"
#include "leg_module.hpp"
#include "console.hpp"
#include "motor_fsm.hpp"
#include "robot_fsm.hpp"

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

    std::vector<LegModule> modules_list_;
    std::vector<bool> powerboard_state_{false, false, false};  // [digital, signal, power] - Must be before motor_fsm_ and robot_fsm_
    
    MotorFSM motor_fsm_;
    RobotFSM robot_fsm_;
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

    void interruptHandler(core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                          core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                          core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

    void powerboardPack(power_msg::PowerStateStamped &power_fb_msg);
    void mainLoop_(core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                   core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                   core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

    void canLoop_();
};
