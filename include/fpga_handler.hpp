#ifndef __FPGAHANDLER_H
#define __FPGAHANDLER_H

#include "NiFpga.h"
#include "NiFpga_FPGACANBus_4module_steering_ABAD.h"
#include "can_packet.h"
#include "Logger.h"
#include "msg.hpp"

#include <unistd.h>
#include <iostream>
#include <functional>
#include <signal.h>
#include <dlfcn.h>
#include <vector>
#include <ncurses.h>
#include <curses.h>
#include <iostream>
#include <bitset>
#include <string>
#undef OK

class FpgaHandler
{
public:
  FpgaHandler();
  ~FpgaHandler();

  NiFpga_Session session_;
  NiFpga_Status status_;
  // Fpga interrupt request
  NiFpga_IrqContext irqContext_;

  // powerboard 1
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool w_pb_digital_PB1_;
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool w_pb_signal_PB1_;
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool w_pb_power_PB1_;
  // powerboard 2
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool w_pb_digital_PB2_;
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool w_pb_signal_PB2_;
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool w_pb_power_PB2_;

  NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16 r_powerboard_data_PB1_;
  NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16 r_powerboard_data_PB2_;
  NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size size_powerboard_data_PB1_;
  NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size size_powerboard_data_PB2_;

  void setIrqPeriod(int main_loop_period, int can_loop_period);
  void write_powerboard_(std::vector<bool> *powerboard1_state_, std::vector<bool> *powerboard2_state_);

  // *********************** Unused Functions ************************ //
  // steering
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool enable_btn_;
  NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool hall;
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlU16 voltage;
  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool dir;
  NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32 encoder;
  void switch_steering(bool steering);
  NiFpga_Bool read_steer_hall();
  int32_t read_steer_encoder();
  void write_steer_vol(uint16_t vol);
  void switch_steer_dir(bool direction);
  // **************************************************************** //

  void read_powerboard_data_();

  double powerboard1_Ifactor[8];
  double powerboard1_Vfactor[8];
  double powerboard2_Ifactor[8];
  double powerboard2_Vfactor[8];
 

  double powerboard1_I_list_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  double powerboard1_V_list_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  double powerboard2_I_list_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  double powerboard2_V_list_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};


#endif