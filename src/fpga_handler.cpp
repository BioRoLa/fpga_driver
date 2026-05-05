#include <fpga_handler.hpp>

FpgaHandler::FpgaHandler()
{
    status_ = NiFpga_Initialize();
    LOG_INFO << "[FPGA Handler] Fpga Initialized";

    NiFpga_MergeStatus(&status_, NiFpga_Open(NiFpga_FPGACANBus_4module_steering_ABAD_Bitfile,
                                             NiFpga_FPGACANBus_4module_steering_ABAD_Signature, "RIO0", 0, &session_));
    LOG_INFO << "[FPGA Handler] Session opened";

    NiFpga_MergeStatus(&status_, NiFpga_ReserveIrqContext(session_, &irqContext_));
    LOG_INFO << "[FPGA Handler] IRQ reserved";
    
    w_pb_digital_PB1_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Digital_PB1;
    w_pb_signal_PB1_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Signal_PB1;
    w_pb_power_PB1_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Power_PB1;

    w_pb_digital_PB2_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Digital_PB2;
    w_pb_signal_PB2_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Signal_PB2;
    w_pb_power_PB2_ = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Power_PB2;

    r_powerboard_data_PB1_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16_Data_PB1;
    r_powerboard_data_PB2_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16_Data_PB2;
    size_powerboard_data_PB1_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size_Data_PB1;
    size_powerboard_data_PB2_ = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size_Data_PB2;

    enable_btn_ =  NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_EN;
    hall = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Hall_effect;
   
    voltage = NiFpga_FPGACANBus_4module_steering_ABAD_ControlU16_input_voltage;
    dir = NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_DIR;
    encoder = NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_EncoderPosition;

    for (int i = 0; i < 8; i++)
    {
        powerboard1_V_list_[i] = 0;
        powerboard1_I_list_[i] = 0;
        powerboard2_V_list_[i] = 0;
        powerboard2_I_list_[i] = 0;
    }
}

FpgaHandler::~FpgaHandler()
{
    /* unreserve IRQ status to prevent memory leaks */
    NiFpga_MergeStatus(&status_, NiFpga_UnreserveIrqContext(session_, &irqContext_));

    /* Close the session */
    NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
    LOG_INFO << "[FPGA Handler] Session Closed";

    NiFpga_MergeStatus(&status_, NiFpga_Finalize());
    LOG_INFO << "[FPGA Handler] Fpga Finalized";
}

void FpgaHandler::setIrqPeriod(int main_loop_p, int can_loop_p)
{
    /* Set up interrupt period (microsecond) */
    /* IRQ 0 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_IRQ0_period_us, main_loop_p));

    /* IRQ 1 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_IRQ1_period_us, can_loop_p));
}

void FpgaHandler::write_powerboard_(std::vector<bool> *powerboard1_state_, std::vector<bool> *powerboard2_state_)
{
    /* PB1 */
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_PB1_, powerboard1_state_->at(0)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_PB1_, powerboard1_state_->at(1)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_PB1_, powerboard1_state_->at(2)));
    /* PB2 */
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_PB2_, powerboard2_state_->at(0)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_PB2_, powerboard2_state_->at(1)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_PB2_, powerboard2_state_->at(2)));
}

void FpgaHandler::read_powerboard_data_()
{
    uint16_t rx_arr_1[16];
    uint16_t rx_arr_2[16];
    // uint16_t *rx_arr = new uint16_t[24];
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU16(session_, NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16_Data_PB1, rx_arr_1, NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size_Data_PB1));
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU16(session_, NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16_Data_PB2, rx_arr_2, NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size_Data_PB2));

    for (int i = 0; i < 16; i++)
    {
        if (i % 2 == 0)powerboard1_I_list_[i / 2] = rx_arr_1[i] * powerboard1_Ifactor[i / 2];
        if (i % 2 == 0)powerboard2_I_list_[i / 2] = rx_arr_2[i] * powerboard2_Ifactor[i / 2];
        if (i % 2 == 1)powerboard1_V_list_[(i - 1) / 2] = rx_arr_1[i] * powerboard1_Vfactor[(i - 1) / 2];
        if (i % 2 == 1)powerboard2_V_list_[(i - 1) / 2] = rx_arr_2[i] * powerboard2_Vfactor[(i - 1) / 2];
    }
}

/////////////////////////////////////// Steering FPGA functions ///////////////////////////////////////
void FpgaHandler::switch_steering(bool steering)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, enable_btn_, steering));
}

NiFpga_Bool FpgaHandler::read_steer_hall(){
    //default = 1 (not in the range)
    NiFpga_Bool steerhall = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(session_, hall, &steerhall));
    return steerhall;
}

int32_t FpgaHandler::read_steer_encoder()
{   
    int32_t position = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadI32(session_, encoder, &position));
    return position;
}

void FpgaHandler::write_steer_vol(uint16_t vol)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU16(session_, voltage, vol));
}

void FpgaHandler::switch_steer_dir(bool direction)
{
    // default 0 = right turn
    // 1 = left turn
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, dir, direction));
}



