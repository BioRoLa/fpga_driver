#include "console.hpp"
#include <curses.h>
#include <iterator>

using namespace std;

mutex cons_mtx_;
int refresh_flag;

void Console::init(FpgaHandler *fpga, vector<LegModule> *mods_, std::vector<bool> *pb_state_ptr_, MotorFSM *fsm_ptr_, RobotFSM *robot_fsm_ptr_, std::mutex *mtx_ptr_)
{
    fpga_ = fpga;

    modA_ptr_ = &mods_->at(0);
    modB_ptr_ = &mods_->at(1);
    modC_ptr_ = &mods_->at(2);
    modD_ptr_ = &mods_->at(3);

    setlocale(LC_ALL, "");
    initscr();
    getmaxyx(stdscr, term_max_y_, term_max_x_);
    start_color();
    init_pair(BKGD_PAIR, COLOR_WHITE, COLOR_BLACK);
    wbkgd(stdscr, COLOR_PAIR(BKGD_PAIR));
    init_pair(CYAN_PAIR, COLOR_CYAN, COLOR_BLACK);

    frontend_rate_ = 3;
    input_panel_.init(mods_, &if_resetPanel, term_max_x_, term_max_y_);

    input_panel_.main_mtx_ = mtx_ptr_;
    input_panel_.powerboard_state_ = pb_state_ptr_;
    input_panel_.fsm_ = fsm_ptr_;
    input_panel_.robot_fsm_ = robot_fsm_ptr_;

    main_mtx_ = mtx_ptr_;
    powerboard_state_ = pb_state_ptr_;
    fsm_ = fsm_ptr_;
    robot_fsm_ = robot_fsm_ptr_;

    if_resetPanel = false;
    t_frontend_ = thread(&Console::refreshWindow, this);
    refresh_flag = 1;
}

void Console::refreshWindow()
{
    clear();

    int refresh_period_ = (int)(1 / frontend_rate_) * 1000000;
    LegModule *lm_null = 0;
    Panel p_power_("[P] Power Board ", "power", lm_null, 1, 9, 60, 40, true);
    Panel p_robot_("[R] Robot FSM ", "robot", lm_null, 1, 1, 8, 40, true);
    Panel p_config_B_("[B] RF_Module Config ", "module", modB_ptr_, 1, 30, 15, 40, true);
    Panel p_modA_("[A] LF_Module ", "module", modA_ptr_, 41, 1, (term_max_y_ - 2) / 2 - 1, 60, true);
    Panel p_modD_("[D] LH_Module ", "module", modD_ptr_, 41, (term_max_y_) / 2, (term_max_y_ - 2) / 2 - 1, 60, true);
    Panel p_modB_("[B] RF_Module ", "module", modB_ptr_, 101, 1, (term_max_y_ - 2) / 2 - 1, 60, true);
    Panel p_modC_("[C] RH_Module ", "module", modC_ptr_, 101, (term_max_y_) / 2, (term_max_y_ - 2) / 2 - 1, 60, true);

    p_power_.powerboard_state_ = powerboard_state_;
    p_robot_.robot_fsm_ = robot_fsm_;

    while (1)
    {
        cons_mtx_.lock();

        p_power_.infoDisplay(fpga_, powerboard_state_->at(0), powerboard_state_->at(1), powerboard_state_->at(2));
        p_robot_.infoDisplay(robot_fsm_->getCurrentMode());
        if(robot_fsm_->getCurrentMode() == RobotMode::MotorConfig)
        {
            p_config_B_.infoDisplayConfig();
        }
        p_modA_.infoDisplay();
        p_modB_.infoDisplay();
        p_modC_.infoDisplay();
        p_modD_.infoDisplay();
        cons_mtx_.unlock();

        usleep(0.1 * 1000 * 1000);
    }
}

void InputPanel::init(vector<LegModule> *mods_, bool *if_resetPanel, int term_max_x, int term_max_y)
{
    win_ = newwin(3, term_max_x - 1, term_max_y - 3, 1);

    modA_ptr_ = &mods_->at(0);
    modB_ptr_ = &mods_->at(1);
    modC_ptr_ = &mods_->at(2);
    modD_ptr_ = &mods_->at(3);

    thread = new std::thread(&InputPanel::inputHandler, this, win_, std::ref(mutex_));
}

void InputPanel::inputHandler(WINDOW *win_, std::mutex &input_mutex)
{
    while (1)
    {
        int ch = 0;
        int x = 1;
        do
        {
            ch = mvwgetch(win_, 1, x);
            if (ch == 'r')
            {
                reset_input_window(win_);
            }
            if (ch == 'e')
            {
                endwin();
                std::cout << "Normal FunctionMode" << std::endl;
                refresh_flag = 0;
            }
            if (ch == 'E')
            {
                refresh_flag = 1;
                refresh();
            }

        } while (ch != ':');

        string input_buf;
        keypad(win_, true);
        do
        {
            ch = mvwgetch(win_, 1, x);

            if (ch == KEY_BACKSPACE || ch == KEY_DC || ch == 127)
            {
                mvwdelch(win_, 1, x);
                mvwdelch(win_, 1, x + 1);
                mvwdelch(win_, 1, x - 1);
                wclrtoeol(win_);
                wrefresh(win_);

                if (input_buf.size() > 0)
                {
                    input_buf.erase(input_buf.size() - 1, 1);
                    x--;
                }
            }
            else
            {
                if (ch != '\n')
                {
                    input_buf.append(1, ch);
                }
                x++;
            }
        } while (ch != '\n');

        reset_input_window(win_);
        commandDecode(input_buf);
    }
}

void InputPanel::reset_input_window(WINDOW *win)
{
    werase(win);
    wclear(win);
    wrefresh(win);
}

void InputPanel::commandDecode(string buf)
{
    bool syntax_err = false;
    bool r_selected = false;  // Robot FSM selected

    bool switchFSM_success = true;

    vector<string> bufs;
    bufs = tokenizer(buf);

    if (bufs.size() >= 1)
    {
        if (bufs[0] == "R")
        {
            r_selected = true;
        }
        else
        {
            syntax_err = true;
        }

        mvwprintw(win_, 2, 1, bufs[0].c_str());
        wrefresh(win_);
    }

    cons_mtx_.lock();
    main_mtx_->lock();

    if (bufs.size() == 3)
    {
        mvwprintw(win_, 2, 3, bufs[1].c_str());
        mvwprintw(win_, 2, 5, bufs[2].c_str());
        wrefresh(win_);

        if (r_selected)
        {
            if (bufs[1] == "M")  // Robot Mode command
            {
                if (bufs[2] == "I")  // Init
                {
                    switchFSM_success = robot_fsm_->requestModeTransition(RobotMode::Init);
                }
                else if (bufs[2] == "D")  // IDLE
                {
                    switchFSM_success = robot_fsm_->requestModeTransition(RobotMode::IDLE);
                }
                else if (bufs[2] == "S")  // Standby
                {
                    switchFSM_success = robot_fsm_->requestModeTransition(RobotMode::Standby);
                }
                else if (bufs[2] == "C")  // MotorConfig
                {
                    switchFSM_success = robot_fsm_->requestModeTransition(RobotMode::MotorConfig);
                }
                else if (bufs[2] == "O")  // SystemOn (safe state)
                {
                    switchFSM_success = robot_fsm_->requestModeTransition(RobotMode::SystemOn);
                }
                else
                {
                    syntax_err = true;
                }
            }
            else if (bufs[1] == "E")  // Emergency stop
            {
                if (bufs[2] == "S")
                {
                    robot_fsm_->emergencyStop();
                    switchFSM_success = true;
                }
                else
                {
                    syntax_err = true;
                }
            }
            else
            {
                syntax_err = true;
            }
        }
    }
    else
    {
        syntax_err = true;
    }

    if (syntax_err)
    {
        mvwprintw(win_, 0, 1, "Syntax Error !");
    }
    else if (!switchFSM_success)
    {
        mvwprintw(win_, 0, 1, "Switch FunctionMode Timeout !");
    }
    else
    {
        mvwprintw(win_, 0, 1, "Command Send !");
    }

    cons_mtx_.unlock();
    main_mtx_->unlock();

    wrefresh(win_);
}

vector<string> InputPanel::tokenizer(string s)
{
    // A quick way to split strings separated via spaces.
    stringstream ss(s);
    string word;
    vector<string> bufs;
    while (ss >> word)
    {
        // cout << word << endl;
        bufs.push_back(word);
    }
    return bufs;
}

Panel::Panel(string title, string type, LegModule *lm_, int org_x, int org_y, int height, int width, bool box_on)
{
    org_x_ = org_x;
    org_y_ = org_y;
    height_ = height;
    width_ = width;
    // box_on_ = // box_on;
    type_ = type;
    title_ = title;

    md_ptr_ = lm_;

    win_ = newwin(height_, width_, org_y_, org_x_);
    refresh();

    // box(win_, 0, 0);

    string tag_(title.c_str(), title.c_str() + 3);
    title.erase(0, 3);

    // mvwprintw(win_, 0, (width_ / 2 - (title.size() + tag_.size()) / 2), tag_.c_str());
    wattron(win_, COLOR_PAIR(CYAN_PAIR));
    wattron(win_, A_BOLD);
    wattron(win_, A_STANDOUT);
    mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 - 2), tag_.c_str());
    wattroff(win_, COLOR_PAIR(CYAN_PAIR));

    mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 + 1), title.c_str());
    wattroff(win_, A_BOLD);
    wattroff(win_, A_STANDOUT);
    wrefresh(win_);
    // refresh();
}

void Panel::infoDisplay()
{
    int y_org = 2;

    // Get motors from the module
    CANMotor* motorR = md_ptr_->getMotor(0);
    CANMotor* motorL = md_ptr_->getMotor(1);
    CANMotor* motorH = md_ptr_->getMotor(2);

    // Motor_R
    mvwprintw(win_, 1, 1, "[F] Motor_R-----------------------------------------------");
    if (motorR) {
        mvwprintw(win_, 2, 1, "[C] [CAN] ID: %9d", motorR->getCANID());
        mvwprintw(win_, 3, 1, "    [tx] TIMEDOUT: %4d", md_ptr_->channel_->hasTxTimeout() ? 1 : 0);
        
        // Command data
        mvwprintw(win_, y_org + 2, 1, "[A] [tx] Pos: %4.5f", motorR->getCommandPosition());
        mvwprintw(win_, y_org + 3, 1, "[T] [tx] Trq: %4.5f", motorR->getCommandTorque());
        mvwprintw(win_, y_org + 4, 1, "[P] [tx] KP:  %4.5f", motorR->getCommandKp());
        mvwprintw(win_, y_org + 5, 1, "[I] [tx] KI:  %4.5f", motorR->getCommandKi());
        mvwprintw(win_, y_org + 6, 1, "[D] [tx] KD:  %4.5f", motorR->getCommandKd());
        
        // Feedback data
        mvwprintw(win_, 3, 30, "[rx] TIMEDOUT: %4d", md_ptr_->channel_->hasRxTimeout() ? 1 : 0);
        mvwprintw(win_, y_org + 2, 30, "[rx] Ver:   %7d", (int)motorR->getVersion());
        mvwprintw(win_, y_org + 3, 30, "[rx] FunctionMode:  %7d", (int)motorR->getModeState());
        mvwprintw(win_, y_org + 4, 30, "[rx] Pos:   %4.5f", motorR->getPosition());
        mvwprintw(win_, y_org + 5, 30, "[rx] Vel:   %4.5f", motorR->getVelocity());
        mvwprintw(win_, y_org + 6, 30, "[rx] Trq:   %4.5f", motorR->getTorque());
        mvwprintw(win_, y_org + 7, 30, "[rx] Cal:   %7d", (int)motorR->getHallCalibrateState());
    }

    // Motor L
    mvwprintw(win_, 10, 1, "[H] Motor_L-----------------------------------------------");
    if (motorL) {
        mvwprintw(win_, 11, 1, "[C] [CAN] ID: %9d", motorL->getCANID());
        mvwprintw(win_, 12, 1, "    [tx] TIMEDOUT: %4d", md_ptr_->channel_->hasTxTimeout() ? 1 : 0);
        
        // Command data
        mvwprintw(win_, y_org + 11, 1, "[A] [tx] Pos: %4.5f", motorL->getCommandPosition());
        mvwprintw(win_, y_org + 12, 1, "[T] [tx] Trq: %4.5f", motorL->getCommandTorque());
        mvwprintw(win_, y_org + 13, 1, "[P] [tx] KP:  %4.5f", motorL->getCommandKp());
        mvwprintw(win_, y_org + 14, 1, "[I] [tx] KI:  %4.5f", motorL->getCommandKi());
        mvwprintw(win_, y_org + 15, 1, "[D] [tx] KD:  %4.5f", motorL->getCommandKd());
        
        // Feedback data
        mvwprintw(win_, 12, 30, "[rx] TIMEDOUT: %4d", md_ptr_->channel_->hasRxTimeout() ? 1 : 0);
        mvwprintw(win_, y_org + 11, 30, "[rx] Ver:   %7d", (int)motorL->getVersion());
        mvwprintw(win_, y_org + 12, 30, "[rx] FunctionMode:  %7d", (int)motorL->getModeState());
        mvwprintw(win_, y_org + 13, 30, "[rx] Pos:   %4.5f", motorL->getPosition());
        mvwprintw(win_, y_org + 14, 30, "[rx] Vel:   %4.5f", motorL->getVelocity());
        mvwprintw(win_, y_org + 15, 30, "[rx] Trq:   %4.5f", motorL->getTorque());
        mvwprintw(win_, y_org + 16, 30, "[rx] Cal:   %7d", (int)motorL->getHallCalibrateState());
    }

    // Motor H
    mvwprintw(win_, 19, 1, "[H] Motor_H-----------------------------------------------");
    if (motorH) {
        mvwprintw(win_, 20, 1, "[C] [CAN] ID: %9d", motorH->getCANID());
        mvwprintw(win_, 21, 1, "    [tx] TIMEDOUT: %4d", md_ptr_->channel_->hasTxTimeout() ? 1 : 0);
        
        // Command data
        mvwprintw(win_, y_org + 20, 1, "[A] [tx] Pos: %4.5f", motorH->getCommandPosition());
        mvwprintw(win_, y_org + 21, 1, "[T] [tx] Trq: %4.5f", motorH->getCommandTorque());
        mvwprintw(win_, y_org + 22, 1, "[P] [tx] KP:  %4.5f", motorH->getCommandKp());
        mvwprintw(win_, y_org + 23, 1, "[I] [tx] KI:  %4.5f", motorH->getCommandKi());
        mvwprintw(win_, y_org + 24, 1, "[D] [tx] KD:  %4.5f", motorH->getCommandKd());
        
        // Feedback data
        mvwprintw(win_, 21, 30, "[rx] TIMEDOUT: %4d", md_ptr_->channel_->hasRxTimeout() ? 1 : 0);
        mvwprintw(win_, y_org + 20, 30, "[rx] Ver:   %7d", (int)motorH->getVersion());
        mvwprintw(win_, y_org + 21, 30, "[rx] FunctionMode:  %7d", (int)motorH->getModeState());
        mvwprintw(win_, y_org + 22, 30, "[rx] Pos:   %4.5f", motorH->getPosition());
        mvwprintw(win_, y_org + 23, 30, "[rx] Vel:   %4.5f", motorH->getVelocity());
        mvwprintw(win_, y_org + 24, 30, "[rx] Trq:   %4.5f", motorH->getTorque());
        mvwprintw(win_, y_org + 25, 30, "[rx] Cal:   %7d", (int)motorH->getHallCalibrateState());
    }
    
    wrefresh(win_);
}

void Panel::infoDisplay(FpgaHandler *fpga_, bool digital_switch, bool signal_switch, bool power_switch)
{
    mvwprintw(win_, 2, 1, "HARDWARE POWER SWITCH ----------------");
    mvwprintw(win_, 3, 1, "[D] Digital:   %4d", digital_switch);
    mvwprintw(win_, 4, 1, "[S] Signal:    %4d", signal_switch);
    mvwprintw(win_, 5, 1, "[P] Power:     %4d", power_switch);

    mvwprintw(win_, 6, 1, "Voltage Current ADC ------------------");
    mvwprintw(win_, 7, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[0], fpga_->powerboard_I_list_[0]);
    mvwprintw(win_, 8, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[1], fpga_->powerboard_I_list_[1]);
    mvwprintw(win_, 9, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[2], fpga_->powerboard_I_list_[2]);
    mvwprintw(win_, 10, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[3], fpga_->powerboard_I_list_[3]);
    mvwprintw(win_, 11, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[4], fpga_->powerboard_I_list_[4]);
    mvwprintw(win_, 12, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[5], fpga_->powerboard_I_list_[5]);
    mvwprintw(win_, 13, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[6], fpga_->powerboard_I_list_[6]);
    mvwprintw(win_, 14, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[7], fpga_->powerboard_I_list_[7]);
    mvwprintw(win_, 15, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[8], fpga_->powerboard_I_list_[8]);
    mvwprintw(win_, 16, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[9], fpga_->powerboard_I_list_[9]);
    mvwprintw(win_, 17, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[10], fpga_->powerboard_I_list_[10]);
    mvwprintw(win_, 18, 1, "Voltage: %5.5f, Current: %5.5f", fpga_->powerboard_V_list_[11], fpga_->powerboard_I_list_[11]);

    wrefresh(win_);
}

void Panel::infoDisplay(RobotMode robot_mode)
{
    // Display current Robot FSM mode
    mvwprintw(win_, 1, 1, "Current Robot FSM Mode:");
    if (robot_mode == RobotMode::SystemOn)
        mvwprintw(win_, 2, 1, "  [O] SystemOn (Safe State)");
    else if (robot_mode == RobotMode::Init)
        mvwprintw(win_, 2, 1, "  [I] Init (Initializing)  ");
    else if (robot_mode == RobotMode::IDLE)
        mvwprintw(win_, 2, 1, "  [D] IDLE (Ready)         ");
    else if (robot_mode == RobotMode::Standby)
        mvwprintw(win_, 2, 1, "  [S] Standby (Active)     ");
    else if (robot_mode == RobotMode::MotorConfig)
        mvwprintw(win_, 2, 1, "  [C] MotorConfig          ");
    
    // Display allowed transitions based on current mode
    mvwprintw(win_, 4, 1, "Allowed Transitions:");
    
    // Clear previous transition lines to avoid leftover text
    mvwprintw(win_, 5, 1, "                                        ");
    mvwprintw(win_, 6, 1, "                                        ");
    mvwprintw(win_, 7, 1, "                                        ");
    
    if (robot_mode == RobotMode::SystemOn)
    {
        mvwprintw(win_, 5, 1, "  :R M I  -> Init");
        mvwprintw(win_, 6, 1, "  :R M C  -> MotorConfig");
    }
    else if (robot_mode == RobotMode::Init)
    {
        mvwprintw(win_, 6, 1, "  :R M O  -> SystemOn");
    }
    else if (robot_mode == RobotMode::IDLE)
    {
        mvwprintw(win_, 5, 1, "  :R M S  -> Standby");
        mvwprintw(win_, 6, 1, "  :R M C  -> MotorConfig");
        mvwprintw(win_, 7, 1, "  :R M O  -> SystemOn");
    }
    else if (robot_mode == RobotMode::Standby)
    {
        mvwprintw(win_, 5, 1, "  :R M D  -> IDLE");
    }
    else if (robot_mode == RobotMode::MotorConfig)
    {
        mvwprintw(win_, 5, 1, "  :R M O  -> SystemOn");
    }
    
    mvwprintw(win_, 9, 1, "Emergency Stop: :R E S");

    wrefresh(win_);
}

void Panel::infoDisplayConfig()
{
    if (!md_ptr_) return;

    CANMotor* motorR = md_ptr_->getMotor(0);
    CANMotor* motorL = md_ptr_->getMotor(1);

    int y = 0;
    
    mvwprintw(win_, y + 1, 1, "Motor_R:");
    if (motorR) {
        const auto& cmd = motorR->getConfigCommandData().config_cmd;
        const auto& fb = motorR->getConfigFeedback().config_fb;

        mvwprintw(win_, y + 2, 1, " CMD: Mode:%d ",cmd.mode);
        mvwprintw(win_, y + 3, 1, "      Type:%d ", cmd.type);
        mvwprintw(win_, y + 4, 1, "      Addr:%d ", cmd.target_addr);
        if (cmd.type == 0) mvwprintw(win_, y + 5, 1, "      Val: %d (I)", cmd.value.i);
        else               mvwprintw(win_, y + 5, 1, "      Val: %f (F)", cmd.value.f);

        mvwprintw(win_, y + 2, 20, "  FB: State:%d ",fb.state);
        mvwprintw(win_, y + 3, 20, "      Type:%d ", fb.type);
        mvwprintw(win_, y + 4, 20, "      Addr:%d ", fb.target_addr);
        if (fb.type == 0)  mvwprintw(win_, y + 5, 20, "      Val: %d (I)", fb.value.i);
        else               mvwprintw(win_, y + 5, 20, "      Val: %f (F)", fb.value.f);
    } 
    else {
        mvwprintw(win_, y + 2, 1, " Not Connected");
    }
    mvwprintw(win_, y + 6, 1, " --------------------------------");

    mvwprintw(win_, y + 7, 1, "Motor_L:");
    if (motorL) {
        const auto& cmd = motorL->getConfigCommandData().config_cmd;
        const auto& fb = motorL->getConfigFeedback().config_fb;

        mvwprintw(win_, y + 8, 1, " CMD: Mode:%d ",cmd.mode);
        mvwprintw(win_, y + 9, 1, "      Type:%d ", cmd.type);
        mvwprintw(win_, y + 10, 1, "      Addr:%d ", cmd.target_addr);
        if (cmd.type == 0) mvwprintw(win_, y + 11, 1, "      Val: %d (I)", cmd.value.i);
        else               mvwprintw(win_, y + 11, 1, "      Val: %f (F)", cmd.value.f);

        mvwprintw(win_, y + 8, 20, "  FB: State:%d ",fb.state);
        mvwprintw(win_, y + 9, 20, "      Type:%d ", fb.type);
        mvwprintw(win_, y + 10, 20, "      Addr:%d ", fb.target_addr);
        if (fb.type == 0)  mvwprintw(win_, y + 11, 20, "      Val: %d (I)", fb.value.i);
        else               mvwprintw(win_, y + 11, 20, "      Val: %f (F)", fb.value.f);
    } 
    else {
        mvwprintw(win_, y + 8, 1, " Not Connected");
    }
    wrefresh(win_);
}

void Panel::panelTitle()
{
    string tag_(title_.c_str(), title_.c_str() + 3);
    title_.erase(0, 3);
    wattron(win_, COLOR_PAIR(CYAN_PAIR));
    wattron(win_, A_BOLD);
    wattron(win_, A_STANDOUT);
    mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 - 2), tag_.c_str());
    wattroff(win_, COLOR_PAIR(CYAN_PAIR));

    mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 + 1), title_.c_str());
    wattroff(win_, A_BOLD);
    wattroff(win_, A_STANDOUT);

    wrefresh(win_);
}

void Panel::resetPanel()
{
    werase(win_);
    wclear(win_);
    wrefresh(win_);
}
