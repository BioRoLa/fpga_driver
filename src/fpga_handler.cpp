#include <fpga_handler.hpp>

FpgaHandler::FpgaHandler()
{
    status_ = NiFpga_Initialize();
    important_message("[FPGA Handler] Fpga Initialized");

    NiFpga_MergeStatus(&status_, NiFpga_Open(NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile,
                                             NiFpga_FPGA_CANBus_4module_v3_steering_Signature, "RIO0", 0, &session_));
    important_message("[FPGA Handler] Session opened");

    NiFpga_MergeStatus(&status_, NiFpga_ReserveIrqContext(session_, &irqContext_));
    important_message("[FPGA Handler] IRQ reserved");
    
    w_pb_digital_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Digital;
    w_pb_signal_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Signal;
    w_pb_power_ = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Power;

    r_powerboard_data_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16_Data;
    size_powerboard_data_ = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16Size_Data;

    enable_btn_ =  NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_EN;
    hall = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Hall_effect;
   
    voltage = NiFpga_FPGA_CANBus_4module_v3_steering_ControlU16_input_voltage;
    dir = NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_DIR;
    encoder = NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_EncoderPosition;

    for (int i = 0; i < 12; i++)
    {
        powerboard_V_list_[i] = 0;
        powerboard_I_list_[i] = 0;
    }
}

FpgaHandler::~FpgaHandler()
{
    /* unreserve IRQ status to prevent memory leaks */
    NiFpga_MergeStatus(&status_, NiFpga_UnreserveIrqContext(session_, &irqContext_));

    /* Close the session */
    NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
    important_message("[FPGA Handler] Session Closed");

    NiFpga_MergeStatus(&status_, NiFpga_Finalize());
    important_message("[FPGA Handler] Fpga Finalized");
}

void FpgaHandler::setIrqPeriod(int main_loop_p, int can_loop_p)
{
    /* Set up interrupt period (microsecond) */
    /* IRQ 0 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_IRQ0_period_us, main_loop_p));

    /* IRQ 1 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_IRQ1_period_us, can_loop_p));
}

void FpgaHandler::write_powerboard_(bool powerboard_state)
{
    // Simplified: single state controls all three FPGA switches
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_, powerboard_state));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_, powerboard_state));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_, powerboard_state));
}

void FpgaHandler::read_powerboard_data_()
{
    uint16_t rx_arr[24];
    // uint16_t *rx_arr = new uint16_t[24];
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU16(session_, NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16_Data, rx_arr, NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16Size_Data));

    for (int i = 0; i < 24; i++)
    {
        if (i % 2 == 0)powerboard_I_list_[i / 2] = rx_arr[i] * powerboard_Ifactor[i / 2];
        if (i % 2 == 1)powerboard_V_list_[(i - 1) / 2] = rx_arr[i] * powerboard_Vfactor[(i - 1) / 2];
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



