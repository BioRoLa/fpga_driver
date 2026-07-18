#ifndef __CAN_CHANNEL_H
#define __CAN_CHANNEL_H

#include "can_motor.hpp"
#include "NiFpga.h"
#include "NiFpga_FPGACANBus_4module_steering_ABAD.h"
#include <vector>
#include <memory>
#include <string>

class CANChannel {
public:
    CANChannel(NiFpga_Status& status, NiFpga_Session& session, 
               const std::string& channel_name);
    
    // Motor management
    // Rewrites all (or fewer) FPGA slots to point at the given motors, in order.
    // Ownership of the CANMotor objects stays with the caller (typically LegModule);
    // this only rebinds which motors this channel's fixed hardware slots currently serve.
    // motor_group.size() must be <= can_ids_.size() (hardware slot count for this channel).
    void attachMotorGroup(const std::vector<CANMotor*>& motor_group);
    CANMotor* getMotor(size_t index);
    size_t getMotorCount() const { return motors_.size(); }
    
    // Setup and configuration
    void setup(uint32_t timeout_us);
    // Mode changes (setMode/setMotorMode/setConfigSubMode) are handled at the
    // LegModule level now, not here - CANChannel just holds hardware slot
    // resources and moves whatever CANMotor state attachMotorGroup() gives it.

    // CAN communication
    void sendCommands();
    void receiveFeedback();

    // Blocks (bounded) until the FPGA reports the last triggered transmit as
    // complete. Required before a second module sharing this channel is allowed
    // to attachMotorGroup()+sendCommands() again - the existing complete_/success_
    // indicators exist for exactly this but were previously unread anywhere in
    // this driver (the non-shared design relied on the ~1ms CAN-loop tick period
    // as an implicit settle time instead). Returns false if the poll budget is
    // exhausted before completion, without blocking indefinitely.
    bool waitForTransmitComplete(uint32_t max_polls = 2000);


    // Timeout Checks (with debounce)
    void updateTimeoutDebounce(uint32_t loop_period_us);
    bool hasTxTimeout() const;
    bool hasRxTimeout() const;
    bool hasTimeout() const;
    
private:
    NiFpga_Status& status_;
    NiFpga_Session session_;
    std::string channel_name_;
    
    // Non-owning: the CANMotor objects live in LegModule (own_motors_) and persist
    // across round-robin hand-offs. This vector only tracks which motors currently
    // occupy this channel's fixed hardware slots.
    std::vector<CANMotor*> motors_;
    
    // FPGA resource addresses (dynamically assigned based on channel_name)

    std::vector<NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32> can_ids_;
    std::vector<NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32> can_id_fcs_;
    std::vector<NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8> tx_buffers_;
    std::vector<NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8> rx_buffers_;

    NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size tx_buf_size_;
    NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size rx_buf_size_;

    NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool transmit_;
    NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool complete_;
    NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool success_;
    NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16 complete_counter_;
    
    NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool tx_timeout_;
    NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool rx_timeout_;

    NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32 timeout_us_;

    NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool port_select_;
    NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize port_select_size_;
    
    // Timeout debounce
    mutable uint32_t tx_timeout_counter_us_;
    mutable uint32_t rx_timeout_counter_us_;
    static constexpr uint32_t TIMEOUT_DEBOUNCE_US_ = 500000;  // 0.5 seconds
    mutable bool tx_timeout_debounced_;
    mutable bool rx_timeout_debounced_;
    
    void initializeResources();
};

#endif
