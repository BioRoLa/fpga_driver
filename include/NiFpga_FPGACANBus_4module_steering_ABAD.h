/*
 * Generated with the FPGA Interface C API Generator 19.0
 * for NI-RIO 19.0 or later.
 */
#ifndef __NiFpga_FPGACANBus_4module_steering_ABAD_h__
#define __NiFpga_FPGACANBus_4module_steering_ABAD_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 190
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_FPGACANBus_4module_steering_ABAD_Bitfile;
 */
#define NiFpga_FPGACANBus_4module_steering_ABAD_Bitfile "/home/admin/corgi_ws/fpga_driver/fpga_bitfile/NiFpga_FPGACANBus_4module_steering_ABAD.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_FPGACANBus_4module_steering_ABAD_Signature = "18406CCDF122D1CDA1E5559907831577";

#if NiFpga_Cpp
extern "C"
{
#endif

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_CKS_OK_PB1 = 0x18086,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_CKS_OK_PB2 = 0x18076,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Conn9_4r = 0x18026,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Conn9_5r = 0x18022,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Conn9_6r = 0x18036,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Hall_effect = 0x180EE,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0Complete = 0x1809E,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0ID1RXsuccess = 0x18112,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0ID2RXsuccess = 0x1812A,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0ID3RXsuccess = 0x18156,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0RXTimeout = 0x18176,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0TXTimeout = 0x1817A,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN0success = 0x1809A,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1Complete = 0x180B6,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1ID1RXsuccess = 0x18186,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1ID2RXsuccess = 0x1819E,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1ID3RXsuccess = 0x181CA,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1RXTimeout = 0x181EA,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1TXTimeout = 0x181F2,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod1CAN1success = 0x180B2,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0Complete = 0x180CA,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0ID1RXsuccess = 0x181FA,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0ID2RXsuccess = 0x18212,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0ID3RXsuccess = 0x1823E,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0RXTimeout = 0x1825E,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0TXTimeout = 0x18266,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN0success = 0x180C6,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1Complete = 0x180DE,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1ID1RXsuccess = 0x1826E,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1ID2RXsuccess = 0x18286,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1ID3RXsuccess = 0x182B2,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1RXTimeout = 0x182D2,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1TXTimeout = 0x182DA,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_Mod2CAN1success = 0x180DA,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_RXfinish_PB1 = 0x18092,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool_RXfinish_PB2 = 0x18082,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorBool;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU8_Info_PB1 = 0x1805E,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU8_Info_PB2 = 0x1805A,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU8;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod1CAN0CompleteCounter = 0x1816E,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod1CAN1CompleteCounter = 0x181DE,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod2CAN0CompleteCounter = 0x18252,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16_Mod2CAN1CompleteCounter = 0x182C6,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI16;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_EncoderPosition = 0x180F8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_IRQ0_cnt = 0x18004,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_IRQ1_cnt = 0x18010,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod1CAN0RXCounter = 0x1815C,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod1CAN0RXSTORCounter = 0x1817C,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod1CAN1RXCounter = 0x181D0,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod1CAN1RXSTORCounter = 0x181E0,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod2CAN0RXCounter = 0x18244,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod2CAN0RXSTORCounter = 0x18254,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod2CAN1RXCounter = 0x182B8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_Mod2CAN1RXSTORCounter = 0x182C8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32_countsnoreset = 0x18100,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorI32;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU32_Mod1CAN0RXTimeus = 0x18170,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU32_Mod1CAN1RXTimeus = 0x181EC,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU32_Mod2CAN0RXTimeus = 0x18260,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU32_Mod2CAN1RXTimeus = 0x182D4,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU32_tickscount = 0x180FC,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorU32;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Conn9_1w = 0x1802E,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Conn9_2w = 0x1802A,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Conn9_3w = 0x1801E,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_DIR = 0x180EA,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Digital_PB1 = 0x1806A,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Digital_PB2 = 0x1804A,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_EN = 0x180E6,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_IRQ0_wait_until_cleared = 0x1800E,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_IRQ1_wait_until_cleared = 0x1801A,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD1CAN0Transmit = 0x18096,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD1CAN1Transmit = 0x180AE,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD2CAN0Transmit = 0x180C2,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_MOD2CAN1Transmit = 0x180D6,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Power_PB1 = 0x18072,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Power_PB2 = 0x18052,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_ResetPosition = 0x180F2,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Signal_PB1 = 0x1806E,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_Signal_PB2 = 0x1804E,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_state = 0x1810E,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool_stop = 0x18002,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlBool;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU16_input_voltage = 0x18106,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlU16;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlI32_ResetPositionValue = 0x180F4,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlI32;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_IOrwuSec = 0x18030,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_IRQ0_period_us = 0x18008,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_IRQ1_period_us = 0x18014,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0 = 0x180A4,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID1 = 0x18124,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID1FC = 0x18114,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID2 = 0x1813C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID2FC = 0x1812C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID3 = 0x18140,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0ID3FC = 0x18150,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN0RXTimeoutus = 0x18180,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1 = 0x180A8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID1 = 0x18190,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID1FC = 0x18198,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID2 = 0x181A8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID2FC = 0x181B0,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID3 = 0x181B4,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1ID3FC = 0x181C4,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod1CAN1RXTimeoutus = 0x181E4,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0 = 0x180BC,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID1 = 0x18204,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID1FC = 0x1820C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID2 = 0x1821C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID2FC = 0x18224,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID3 = 0x18228,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0ID3FC = 0x18238,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN0RXTimeoutus = 0x18258,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1 = 0x180D0,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID1 = 0x18278,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID1FC = 0x18280,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID2 = 0x18290,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID2FC = 0x18298,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID3 = 0x1829C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1ID3FC = 0x182AC,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Mod2CAN1RXTimeoutus = 0x182CC,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_Module_Information = 0x1803C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_PowerBoard = 0x18038,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32_SteeringMotor = 0x18040,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlU32;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayBool_outputarray = 0x1810A,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayBool;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayBoolSize_outputarray = 16,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayBoolSize;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_DataArray_PB1 = 0x18088,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_DataArray_PB2 = 0x18078,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_DataRX_PB1 = 0x1808C,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_DataRX_PB2 = 0x1807C,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN0ID1RX = 0x18118,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN0ID2RX = 0x18130,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN0ID3RX = 0x1814C,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN1ID1RX = 0x1818C,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN1ID2RX = 0x181A4,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod1CAN1ID3RX = 0x181C0,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN0ID1RX = 0x18200,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN0ID2RX = 0x18218,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN0ID3RX = 0x18234,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN1ID1RX = 0x18274,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN1ID2RX = 0x1828C,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8_Mod2CAN1ID3RX = 0x182A8,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_DataArray_PB1 = 33,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_DataArray_PB2 = 33,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_DataRX_PB1 = 64,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_DataRX_PB2 = 64,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN0ID1RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN0ID2RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN0ID3RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN1ID1RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN1ID2RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod1CAN1ID3RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN0ID1RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN0ID2RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN0ID3RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN1ID1RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN1ID2RX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size_Mod2CAN1ID3RX = 8,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU8Size;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16_Data_PB1 = 0x18060,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16_Data_PB2 = 0x18054,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size_Data_PB1 = 16,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size_Data_PB2 = 16,
} NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorArrayU16Size;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod1CAN0Select = 0x180A2,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod1CAN1Select = 0x180BA,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod2CAN0Select = 0x180CE,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool_Mod2CAN1Select = 0x180E2,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBool;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod1CAN0Select = 3,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod1CAN1Select = 3,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod2CAN0Select = 3,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize_Mod2CAN1Select = 3,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayBoolSize;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_DataTxPB1 = 0x18064,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_DataTxPB2 = 0x18044,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN0ID1TX = 0x18120,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN0ID2TX = 0x18138,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN0ID3TX = 0x18144,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN1ID1TX = 0x18194,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN1ID2TX = 0x181AC,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod1CAN1ID3TX = 0x181B8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN0ID1TX = 0x18208,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN0ID2TX = 0x18220,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN0ID3TX = 0x1822C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN1ID1TX = 0x1827C,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN1ID2TX = 0x18294,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8_Mod2CAN1ID3TX = 0x182A0,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8;

typedef enum
{
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_DataTxPB1 = 12,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_DataTxPB2 = 12,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN0ID1TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN0ID2TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN0ID3TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN1ID1TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN1ID2TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod1CAN1ID3TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN0ID1TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN0ID2TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN0ID3TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN1ID1TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN1ID2TX = 8,
   NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size_Mod2CAN1ID3TX = 8,
} NiFpga_FPGACANBus_4module_steering_ABAD_ControlArrayU8Size;

#if !NiFpga_VxWorks

/* Indicator: Mod1CAN0RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXBuffer */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_Resource = 0x18160;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_Size = 8;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_Type;

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN0RXBuffer_Type* const source);

/* Indicator: Mod1CAN1RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXBuffer */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_Resource = 0x181D4;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_Size = 8;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_Type;

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod1CAN1RXBuffer_Type* const source);

/* Indicator: Mod2CAN0RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod2CAN0RXBuffer */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_Resource = 0x18248;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_Size = 8;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_Type;

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN0RXBuffer_Type* const source);

/* Indicator: Mod2CAN1RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod2CAN1RXBuffer */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_Resource = 0x182BC;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_Size = 8;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_Type;

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorClusterArray_Mod2CAN1RXBuffer_Type* const source);

/* Indicator: Mod1CAN0ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0ID1RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_Resource = 0x1811C;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID1RXFrame_Type* const source);

/* Indicator: Mod1CAN0ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0ID2RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_Resource = 0x18134;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID2RXFrame_Type* const source);

/* Indicator: Mod1CAN0ID3RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0ID3RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_Resource = 0x18148;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0ID3RXFrame_Type* const source);

/* Indicator: Mod1CAN0RXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_Resource = 0x18164;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXError_Type* const source);

/* Indicator: Mod1CAN0RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_Resource = 0x18158;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0RXFrame_Type* const source);

/* Indicator: Mod1CAN0TXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0TXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_Resource = 0x18168;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN0TXError_Type* const source);

/* Indicator: Mod1CAN1ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1ID1RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_Resource = 0x18188;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID1RXFrame_Type* const source);

/* Indicator: Mod1CAN1ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1ID2RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_Resource = 0x181A0;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID2RXFrame_Type* const source);

/* Indicator: Mod1CAN1ID3RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1ID3RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_Resource = 0x181BC;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1ID3RXFrame_Type* const source);

/* Indicator: Mod1CAN1RXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_Resource = 0x181D8;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXError_Type* const source);

/* Indicator: Mod1CAN1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_Resource = 0x181CC;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1RXFrame_Type* const source);

/* Indicator: Mod1CAN1TXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1TXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_Resource = 0x181F4;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod1CAN1TXError_Type* const source);

/* Indicator: Mod2CAN0ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0ID1RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_Resource = 0x181FC;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID1RXFrame_Type* const source);

/* Indicator: Mod2CAN0ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0ID2RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_Resource = 0x18214;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID2RXFrame_Type* const source);

/* Indicator: Mod2CAN0ID3RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0ID3RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_Resource = 0x18230;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0ID3RXFrame_Type* const source);

/* Indicator: Mod2CAN0RXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0RXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_Resource = 0x1824C;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXError_Type* const source);

/* Indicator: Mod2CAN0RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_Resource = 0x18240;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0RXFrame_Type* const source);

/* Indicator: Mod2CAN0TXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0TXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_Resource = 0x18268;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN0TXError_Type* const source);

/* Indicator: Mod2CAN1ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1ID1RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_Resource = 0x18270;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID1RXFrame_Type* const source);

/* Indicator: Mod2CAN1ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1ID2RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_Resource = 0x18288;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID2RXFrame_Type* const source);

/* Indicator: Mod2CAN1ID3RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1ID3RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_Resource = 0x182A4;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1ID3RXFrame_Type* const source);

/* Indicator: Mod2CAN1RXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1RXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_Resource = 0x182C0;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXError_Type* const source);

/* Indicator: Mod2CAN1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1RXFrame */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_Resource = 0x182B4;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1RXFrame_Type* const source);

/* Indicator: Mod2CAN1TXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1TXError */
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_Resource = 0x182DC;
const uint32_t NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_Type;


void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_Type* const destination);

void NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGACANBus_4module_steering_ABAD_IndicatorCluster_Mod2CAN1TXError_Type* const source);

#endif /* !NiFpga_VxWorks */


#if NiFpga_Cpp
}
#endif

#endif
