#ifndef __CAN_CHANNEL_H
#define __CAN_CHANNEL_H

#include "can_motor.hpp"
#include "NiFpga.h"
#include "NiFpga_FPGA_CANBus_4module_v3_steering.h"
#include <vector>
#include <memory>
#include <string>

class CANChannel {
public:
    CANChannel(NiFpga_Status& status, NiFpga_Session& session, 
               const std::string& channel_name);
    
    // Motor management
    void addMotor(uint32_t can_id, const Motor& config);
    CANMotor* getMotor(size_t index);
    size_t getMotorCount() const { return motors_.size(); }
    
    // Setup and configuration
    void setup(uint32_t timeout_us);
    void setMode(Mode mode);
    
    // CAN communication
    void sendCommands();
    void receiveFeedback();
    
    // Timeout Checks
    bool hasTxTimeout() const;
    bool hasRxTimeout() const;
    bool hasTimeout() const;
    
private:
    NiFpga_Status& status_;
    NiFpga_Session session_;
    std::string channel_name_;
    
    std::vector<std::unique_ptr<CANMotor>> motors_;
    
    // FPGA resource addresses (dynamically assigned based on channel_name)

    std::vector<NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32> can_ids_;
    std::vector<NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32> can_id_fcs_;
    std::vector<NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8> tx_buffers_;
    std::vector<NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8> rx_buffers_;

    NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size tx_buf_size_;
    NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size rx_buf_size_;

    NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool transmit_;
    NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool complete_;
    NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool success_;
    NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16 complete_counter_;
    
    NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool tx_timeout_;
    NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool rx_timeout_;

    NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32 timeout_us_;

    NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool port_select_;
    NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize port_select_size_;
    
    void initializeResources();
};

#endif