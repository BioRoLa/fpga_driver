#include "can_channel.hpp"
#include "color.hpp"
#include <iostream>

CANChannel::CANChannel(NiFpga_Status& status, NiFpga_Session session, 
                       const std::string& channel_name)
    : status_(status)
    , session_(session)
    , channel_name_(channel_name)
{
    initializeResources();
}

void CANChannel::initializeResources()
{
    // Map channel name to FPGA resources
    if (channel_name_ == "MOD1CAN0") {

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID1);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID1FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN0ID1TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN0ID1RX);

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID2);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID2FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN0ID2TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN0ID2RX);

        tx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod1CAN0ID1TX;
        rx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod1CAN0ID1RX;

        transmit_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD1CAN0Transmit;
        complete_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0Complete;
        success_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0success;
        complete_counter_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod1CAN0CompleteCounter;

        tx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0TXTimeout;
        rx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0RXTimeout;

        timeout_us_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0RXTimeoutus;

        port_select_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod1CAN0Select;
        port_select_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod1CAN0Select;

    }
    else if (channel_name_ == "MOD1CAN1") {

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID1);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID1FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN1ID1TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN1ID1RX);

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID2);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID2FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN1ID2TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN1ID2RX);

        tx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod1CAN1ID1TX;
        rx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod1CAN1ID1RX;

        transmit_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD1CAN1Transmit;
        complete_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1Complete;
        success_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1success;
        complete_counter_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod1CAN1CompleteCounter;

        tx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1TXTimeout;
        rx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1RXTimeout;

        timeout_us_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1RXTimeoutus;

        port_select_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod1CAN1Select;
        port_select_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod1CAN1Select;
        
    }
    else if (channel_name_ == "MOD2CAN0") {

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID1);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID1FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN0ID1TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN0ID1RX);

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID2);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID2FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN0ID2TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN0ID2RX);

        tx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod2CAN0ID1TX;
        rx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod2CAN0ID1RX;

        transmit_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD2CAN0Transmit;
        complete_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0Complete;
        success_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0success;
        complete_counter_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod2CAN0CompleteCounter;

        tx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0TXTimeout;
        rx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0RXTimeout;

        timeout_us_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0RXTimeoutus;

        port_select_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod2CAN0Select;
        port_select_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod2CAN0Select;

    }
    else if (channel_name_ == "MOD2CAN1") {

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID1);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID1FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN1ID1TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN1ID1RX);

        can_ids_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID2);
        can_id_fcs_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID2FC);
        tx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN1ID2TX);
        rx_buffers_.push_back(NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN1ID2RX);

        tx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod2CAN1ID1TX;
        rx_buf_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod2CAN1ID1RX;

        transmit_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD2CAN1Transmit;
        complete_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1Complete;
        success_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1success;
        complete_counter_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod2CAN1CompleteCounter;

        tx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1TXTimeout;
        rx_timeout_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1RXTimeout;

        timeout_us_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1RXTimeoutus;

        port_select_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod2CAN1Select;
        port_select_size_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod2CAN1Select;

    }
    else {
        error_message("[CAN Channel] Unknown channel name: " + channel_name_);
    }
}

void CANChannel::addMotor(uint32_t can_id, const Motor& config)
{
    if (motors_.size() >= can_ids_.size()) {
        error_message("[CAN Channel] Cannot add more motors than hardware supports");
        return;
    }
    motors_.push_back(std::make_unique<CANMotor>(can_id, config));
}

CANMotor* CANChannel::getMotor(size_t index)
{
    if (index < motors_.size()) {
        return motors_[index].get();
    }
    return nullptr;
}

void CANChannel::setup(uint32_t timeout_us)
{
    // 設定 CAN IDs
    for (size_t i = 0; i < motors_.size(); ++i) {
        NiFpga_MergeStatus(&status_, 
            NiFpga_WriteU32(session_, can_ids_[i], motors_[i]->getCANID()));
    }
    
    // 設定 port select
    NiFpga_Bool port_select[2] = {
        motors_.size() > 0,
        motors_.size() > 1
    };
    NiFpga_MergeStatus(&status_, 
        NiFpga_WriteArrayBool(session_, port_select_, port_select, port_select_size_));
    
    // 設定 timeout
    NiFpga_MergeStatus(&status_, 
        NiFpga_WriteU32(session_, timeout_us_, timeout_us));
}

void CANChannel::setMode(Mode mode)
{
    uint32_t mode_val = static_cast<uint32_t>(mode);
    
    for (size_t i = 0; i < motors_.size(); ++i) {
        NiFpga_MergeStatus(&status_, 
            NiFpga_WriteU32(session_, can_id_fcs_[i], mode_val));
        motors_[i]->setMode(mode);
    }
}

void CANChannel::sendCommands()
{
    for (size_t i = 0; i < motors_.size(); ++i) {
        uint8_t tx_buffer[8] = {0};
        motors_[i]->encodeCommand(tx_buffer);
        NiFpga_MergeStatus(&status_, 
            NiFpga_WriteArrayU8(session_, tx_buffers_[i], tx_buffer, tx_buf_size_));
    }
    
    // 觸發傳輸
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

bool CANChannel::hasTxTimeout() const
{
    NiFpga_Bool timeout = false;
    NiFpga_ReadBool(session_, tx_timeout_, &timeout);
    return timeout;
}

bool CANChannel::hasRxTimeout() const
{
    NiFpga_Bool timeout = false;
    NiFpga_ReadBool(session_, rx_timeout_, &timeout);
    return timeout;
}

bool CANChannel::hasTimeout() const
{
    return hasTxTimeout() || hasRxTimeout();
}