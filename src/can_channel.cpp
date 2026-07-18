#include "can_channel.hpp"
#include "Logger.h"
#include <iostream>
#include <cstring>

CANChannel::CANChannel(NiFpga_Status& status, NiFpga_Session& session, 
                       const std::string& channel_name)
    : status_(status)
    , session_(session)
    , channel_name_(channel_name)
    , tx_timeout_counter_us_(0)
    , rx_timeout_counter_us_(0)
    , tx_timeout_debounced_(false)
    , rx_timeout_debounced_(false)
{
    initializeResources();
}

void CANChannel::initializeResources()
{
    // Map channel name to FPGA resources
    if (channel_name_ == "MOD1CAN0") {

        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID1);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID1FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN0ID1TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN0ID1RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID2);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID2FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN0ID2TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN0ID2RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID3);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID3FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN0ID3TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN0ID3RX);

        tx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN0ID1TX;
        rx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN0ID1RX;

        transmit_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD1CAN0Transmit;
        complete_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0Complete;
        success_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0success;
        complete_counter_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod1CAN0CompleteCounter;

        tx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0TXTimeout;
        rx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0RXTimeout;

        timeout_us_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0RXTimeoutus;

        port_select_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod1CAN0Select;
        port_select_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod1CAN0Select;

    }
    else if (channel_name_ == "MOD1CAN1") {

        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID1);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID1FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN1ID1TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN1ID1RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID2);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID2FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN1ID2TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN1ID2RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID3);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID3FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN1ID3TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN1ID3RX);

        tx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN1ID1TX;
        rx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN1ID1RX;

        transmit_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD1CAN1Transmit;
        complete_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1Complete;
        success_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1success;
        complete_counter_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod1CAN1CompleteCounter;

        tx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1TXTimeout;
        rx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1RXTimeout;

        timeout_us_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1RXTimeoutus;

        port_select_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod1CAN1Select;
        port_select_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod1CAN1Select;
        
    }
    else if (channel_name_ == "MOD2CAN0") {

        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID1);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID1FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN0ID1TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN0ID1RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID2);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID2FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN0ID2TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN0ID2RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID3);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID3FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN0ID3TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN0ID3RX);

        tx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN0ID1TX;
        rx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN0ID1RX;

        transmit_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD2CAN0Transmit;
        complete_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0Complete;
        success_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0success;
        complete_counter_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod2CAN0CompleteCounter;

        tx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0TXTimeout;
        rx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0RXTimeout;

        timeout_us_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0RXTimeoutus;

        port_select_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod2CAN0Select;
        port_select_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod2CAN0Select;

    }
    else if (channel_name_ == "MOD2CAN1") {

        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID1);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID1FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN1ID1TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN1ID1RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID2);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID2FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN1ID2TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN1ID2RX);
        
        can_ids_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID3);
        can_id_fcs_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID3FC);
        tx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN1ID3TX);
        rx_buffers_.push_back(NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN1ID3RX);

        tx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN1ID1TX;
        rx_buf_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN1ID1RX;

        transmit_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD2CAN1Transmit;
        complete_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1Complete;
        success_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1success;
        complete_counter_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod2CAN1CompleteCounter;

        tx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1TXTimeout;
        rx_timeout_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1RXTimeout;

        timeout_us_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1RXTimeoutus;

        port_select_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod2CAN1Select;
        port_select_size_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod2CAN1Select;

    }
    else {
        LOG_ERROR << "[CAN Channel] Unknown channel name: " << channel_name_;
    }
}

void CANChannel::attachMotorGroup(const std::vector<CANMotor*>& motor_group)
{
    if (motor_group.size() > can_ids_.size()) {
        LOG_ERROR << "[CAN Channel] attachMotorGroup: " << motor_group.size()
                  << " motors exceeds " << can_ids_.size() << " hardware slots on "
                  << channel_name_;
        return;
    }

    motors_ = motor_group;

    // Rebind slot address (CAN ID) and slot mode (FC) together with the pointer swap,
    // so a slot's on-wire identity and its data source/sink always change atomically.
    for (size_t i = 0; i < motors_.size(); ++i) {
        NiFpga_MergeStatus(&status_,
            NiFpga_WriteU32(session_, can_ids_[i], motors_[i]->getCANID()));
        NiFpga_MergeStatus(&status_,
            NiFpga_WriteU32(session_, can_id_fcs_[i],
                            static_cast<uint32_t>(motors_[i]->getFunctionMode())));
    }

    // port select reflects how many of the 3 hardware slots are currently in use
    NiFpga_Bool port_select[3] = {
        motors_.size() > 0,
        motors_.size() > 1,
        motors_.size() > 2
    };
    NiFpga_MergeStatus(&status_,
        NiFpga_WriteArrayBool(session_, port_select_, port_select, port_select_size_));
}

CANMotor* CANChannel::getMotor(size_t index)
{
    if (index < motors_.size()) {
        return motors_[index];
    }
    return nullptr;
}

void CANChannel::setup(uint32_t timeout_us)
{
    // Motor ID/FC/port-select binding now happens in attachMotorGroup(), which is
    // called both at initial bring-up and on every round-robin hand-off. setup()
    // is left only for the one-time FPGA RX timeout configuration.
    NiFpga_MergeStatus(&status_,
        NiFpga_WriteU32(session_, timeout_us_, timeout_us));
}

void CANChannel::setMode(FunctionMode mode)
{
    uint32_t mode_val = static_cast<uint32_t>(mode);
    
    for (size_t i = 0; i < motors_.size(); ++i) {
        NiFpga_MergeStatus(&status_, 
            NiFpga_WriteU32(session_, can_id_fcs_[i], mode_val));
        motors_[i]->setMode(mode);
    }
}

void CANChannel::setMotorMode(size_t index, FunctionMode mode)
{
    if (index >= motors_.size() || index >= can_id_fcs_.size()) {
        LOG_ERROR << "[CAN Channel] Invalid motor index for mode switch: " << index;
        return;
    }

    uint32_t mode_val = static_cast<uint32_t>(mode);
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(session_, can_id_fcs_[index], mode_val));
    motors_[index]->setMode(mode);
}

void CANChannel::setConfigSubMode(ConfigSubMode sub_mode)
{
    for (size_t i = 0; i < motors_.size(); ++i) {
        motors_[i]->setConfigSubMode(sub_mode);
    }
}

void CANChannel::sendCommands()
{
    for (size_t i = 0; i < motors_.size(); ++i) {
        const uint8_t* cmd_data = motors_[i]->getCommandRaw();
        NiFpga_MergeStatus(&status_, 
            NiFpga_WriteArrayU8(session_, tx_buffers_[i], cmd_data, tx_buf_size_));
    }
    
    // transmit trigger
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, transmit_, true));
}

void CANChannel::receiveFeedback()
{
    for (size_t i = 0; i < motors_.size(); ++i) {
        uint8_t rx_buffer[8] = {0};
        NiFpga_MergeStatus(&status_, 
            NiFpga_ReadArrayU8(session_, rx_buffers_[i], rx_buffer, rx_buf_size_));
        motors_[i]->parseFeedback(rx_buffer);
    }
}

void CANChannel::updateTimeoutDebounce(uint32_t loop_period_us)
{
    // Read raw timeout status from FPGA
    NiFpga_Bool tx_timeout_raw = false;
    NiFpga_Bool rx_timeout_raw = false;
    NiFpga_ReadBool(session_, tx_timeout_, &tx_timeout_raw);
    NiFpga_ReadBool(session_, rx_timeout_, &rx_timeout_raw);
    
    // Update TX timeout counter
    if (tx_timeout_raw) {
        tx_timeout_counter_us_ += loop_period_us;
        if (tx_timeout_counter_us_ >= TIMEOUT_DEBOUNCE_US_) {
            tx_timeout_debounced_ = true;
        }
    } else {
        tx_timeout_counter_us_ = 0;
        tx_timeout_debounced_ = false;
    }
    
    // Update RX timeout counter
    if (rx_timeout_raw) {
        rx_timeout_counter_us_ += loop_period_us;
        if (rx_timeout_counter_us_ >= TIMEOUT_DEBOUNCE_US_) {
            rx_timeout_debounced_ = true;
        }
    } else {
        rx_timeout_counter_us_ = 0;
        rx_timeout_debounced_ = false;
    }
}

bool CANChannel::hasTxTimeout() const
{
    return tx_timeout_debounced_;
}

bool CANChannel::hasRxTimeout() const
{
    return rx_timeout_debounced_;
}

bool CANChannel::hasTimeout() const
{
    return hasTxTimeout() || hasRxTimeout();
}
