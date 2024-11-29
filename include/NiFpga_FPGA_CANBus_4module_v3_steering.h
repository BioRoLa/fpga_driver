/*
 * Generated with the FPGA Interface C API Generator 19.0
 * for NI-RIO 19.0 or later.
 */
#ifndef __NiFpga_FPGA_CANBus_4module_v3_steering_h__
#define __NiFpga_FPGA_CANBus_4module_v3_steering_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 190
#endif

#include "NiFpga.h"
/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile;
 */
// #define NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile "/home/tina997726/corgi_ws/fpga_driver/fpga_bitfile/NiFpga_FPGA_CANBus_4module_v3_steering.lvbitx"
#define NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile "/home/admin/corgi_ws/fpga_driver/fpga_bitfile/NiFpga_FPGA_CANBus_4module_v3_steering.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_FPGA_CANBus_4module_v3_steering_Signature = "935F14FF8253BA377DE0C1E3DE452DAF";

#if NiFpga_Cpp
extern "C"
{
#endif

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_ChecksumOK = 0x1805E,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Conn9_4r = 0x18026,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Conn9_5r = 0x18022,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Conn9_6r = 0x18036,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Hall_effect = 0x180C6,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0Complete = 0x18076,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0ID1RXsuccess = 0x180EA,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0ID2RXsuccess = 0x18102,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0RXTimeout = 0x18136,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0TXTimeout = 0x1813A,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN0success = 0x18072,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1Complete = 0x1808E,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1ID1RXsuccess = 0x18146,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1ID2RXsuccess = 0x1815E,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1RXTimeout = 0x18192,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1TXTimeout = 0x1819A,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod1CAN1success = 0x1808A,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0Complete = 0x180A2,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0ID1RXsuccess = 0x181A2,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0ID2RXsuccess = 0x181BA,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0RXTimeout = 0x181EE,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0TXTimeout = 0x181F6,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN0success = 0x1809E,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1Complete = 0x180B6,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1ID1RXsuccess = 0x181FE,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1ID2RXsuccess = 0x18216,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1RXTimeout = 0x1824A,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1TXTimeout = 0x18252,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_Mod2CAN1success = 0x180B2,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool_RXfinish = 0x1806A,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorBool;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU8_Information = 0x18046,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU8;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod1CAN0CompleteCounter = 0x1812E,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod1CAN1CompleteCounter = 0x18186,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod2CAN0CompleteCounter = 0x181E2,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16_Mod2CAN1CompleteCounter = 0x1823E,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI16;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_EncoderPosition = 0x180D0,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_IRQ0_cnt = 0x18004,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_IRQ1_cnt = 0x18010,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod1CAN0RXCounter = 0x1811C,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod1CAN0RXSTORCounter = 0x1813C,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod1CAN1RXCounter = 0x18178,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod1CAN1RXSTORCounter = 0x18188,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod2CAN0RXCounter = 0x181D4,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod2CAN0RXSTORCounter = 0x181E4,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod2CAN1RXCounter = 0x18230,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_Mod2CAN1RXSTORCounter = 0x18240,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32_countsnoreset = 0x180D8,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorI32;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU32_Mod1CAN0RXTimeus = 0x18130,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU32_Mod1CAN1RXTimeus = 0x18194,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU32_Mod2CAN0RXTimeus = 0x181F0,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU32_Mod2CAN1RXTimeus = 0x1824C,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU32_tickscount = 0x180D4,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorU32;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Conn9_1w = 0x1802E,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Conn9_2w = 0x1802A,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Conn9_3w = 0x1801E,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_DIR = 0x180C2,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Digital = 0x18052,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_EN = 0x180BE,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_IRQ0_wait_until_cleared = 0x1800E,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_IRQ1_wait_until_cleared = 0x1801A,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD1CAN0Transmit = 0x1806E,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD1CAN1Transmit = 0x18086,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD2CAN0Transmit = 0x1809A,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_MOD2CAN1Transmit = 0x180AE,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Power = 0x1805A,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_ResetPosition = 0x180CA,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_Signal = 0x18056,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_state = 0x180E6,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool_stop = 0x18002,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlBool;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU16_input_voltage = 0x180DE,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlU16;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlI32_ResetPositionValue = 0x180CC,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlI32;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_IOrwuSec = 0x18030,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_IRQ0_period_us = 0x18008,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_IRQ1_period_us = 0x18014,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0 = 0x1807C,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID1 = 0x180FC,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID1FC = 0x180EC,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID2 = 0x18114,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0ID2FC = 0x18104,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN0RXTimeoutus = 0x18140,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1 = 0x18080,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID1 = 0x18150,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID1FC = 0x18158,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID2 = 0x18168,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1ID2FC = 0x18170,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod1CAN1RXTimeoutus = 0x1818C,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0 = 0x18094,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID1 = 0x181AC,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID1FC = 0x181B4,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID2 = 0x181C4,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0ID2FC = 0x181CC,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN0RXTimeoutus = 0x181E8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1 = 0x180A8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID1 = 0x18208,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID1FC = 0x18210,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID2 = 0x18220,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1ID2FC = 0x18228,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Mod2CAN1RXTimeoutus = 0x18244,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_Module_Information = 0x1803C,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_PowerBoard = 0x18038,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32_SteeringMotor = 0x18040,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlU32;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayBool_outputarray = 0x180E2,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayBool;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayBoolSize_outputarray = 16,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayBoolSize;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_DataArray = 0x18060,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_DataRX = 0x18064,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN0ID1RX = 0x180F0,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN0ID2RX = 0x18108,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN1ID1RX = 0x1814C,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod1CAN1ID2RX = 0x18164,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN0ID1RX = 0x181A8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN0ID2RX = 0x181C0,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN1ID1RX = 0x18204,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8_Mod2CAN1ID2RX = 0x1821C,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_DataArray = 49,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_DataRX = 64,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod1CAN0ID1RX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod1CAN0ID2RX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod1CAN1ID1RX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod1CAN1ID2RX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod2CAN0ID1RX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod2CAN0ID2RX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod2CAN1ID1RX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size_Mod2CAN1ID2RX = 8,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU8Size;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16_Data = 0x18048,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16Size_Data = 24,
} NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorArrayU16Size;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod1CAN0Select = 0x1807A,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod1CAN1Select = 0x18092,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod2CAN0Select = 0x180A6,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool_Mod2CAN1Select = 0x180BA,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBool;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod1CAN0Select = 2,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod1CAN1Select = 2,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod2CAN0Select = 2,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize_Mod2CAN1Select = 2,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayBoolSize;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_DataTx = 0x1804C,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN0ID1TX = 0x180F8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN0ID2TX = 0x18110,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN1ID1TX = 0x18154,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod1CAN1ID2TX = 0x1816C,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN0ID1TX = 0x181B0,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN0ID2TX = 0x181C8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN1ID1TX = 0x1820C,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8_Mod2CAN1ID2TX = 0x18224,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8;

typedef enum
{
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_DataTx = 12,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod1CAN0ID1TX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod1CAN0ID2TX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod1CAN1ID1TX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod1CAN1ID2TX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod2CAN0ID1TX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod2CAN0ID2TX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod2CAN1ID1TX = 8,
   NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size_Mod2CAN1ID2TX = 8,
} NiFpga_FPGA_CANBus_4module_v3_steering_ControlArrayU8Size;

#if !NiFpga_VxWorks

/* Indicator: Mod1CAN0RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXBuffer */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_Resource = 0x18120;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_Size = 8;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_Type;

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN0RXBuffer_Type* const source);

/* Indicator: Mod1CAN1RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXBuffer */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_Resource = 0x1817C;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_Size = 8;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_Type;

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod1CAN1RXBuffer_Type* const source);

/* Indicator: Mod2CAN0RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod2CAN0RXBuffer */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_Resource = 0x181D8;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_Size = 8;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_Type;

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN0RXBuffer_Type* const source);

/* Indicator: Mod2CAN1RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod2CAN1RXBuffer */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_Resource = 0x18234;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_Size = 8;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_Type;

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorClusterArray_Mod2CAN1RXBuffer_Type* const source);

/* Indicator: Mod1CAN0ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0ID1RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_Resource = 0x180F4;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID1RXFrame_Type* const source);

/* Indicator: Mod1CAN0ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0ID2RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_Resource = 0x1810C;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0ID2RXFrame_Type* const source);

/* Indicator: Mod1CAN0RXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_Resource = 0x18124;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXError_Type* const source);

/* Indicator: Mod1CAN0RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_Resource = 0x18118;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0RXFrame_Type* const source);

/* Indicator: Mod1CAN0TXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0TXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_Resource = 0x18128;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN0TXError_Type* const source);

/* Indicator: Mod1CAN1ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1ID1RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_Resource = 0x18148;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID1RXFrame_Type* const source);

/* Indicator: Mod1CAN1ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1ID2RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_Resource = 0x18160;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1ID2RXFrame_Type* const source);

/* Indicator: Mod1CAN1RXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_Resource = 0x18180;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXError_Type* const source);

/* Indicator: Mod1CAN1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_Resource = 0x18174;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1RXFrame_Type* const source);

/* Indicator: Mod1CAN1TXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1TXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_Resource = 0x1819C;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod1CAN1TXError_Type* const source);

/* Indicator: Mod2CAN0ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0ID1RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_Resource = 0x181A4;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID1RXFrame_Type* const source);

/* Indicator: Mod2CAN0ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0ID2RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_Resource = 0x181BC;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0ID2RXFrame_Type* const source);

/* Indicator: Mod2CAN0RXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0RXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_Resource = 0x181DC;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXError_Type* const source);

/* Indicator: Mod2CAN0RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_Resource = 0x181D0;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0RXFrame_Type* const source);

/* Indicator: Mod2CAN0TXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN0TXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_Resource = 0x181F8;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN0TXError_Type* const source);

/* Indicator: Mod2CAN1ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1ID1RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_Resource = 0x18200;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID1RXFrame_Type* const source);

/* Indicator: Mod2CAN1ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1ID2RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_Resource = 0x18218;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1ID2RXFrame_Type* const source);

/* Indicator: Mod2CAN1RXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1RXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_Resource = 0x18238;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXError_Type* const source);

/* Indicator: Mod2CAN1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1RXFrame */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_Resource = 0x1822C;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1RXFrame_Type* const source);

/* Indicator: Mod2CAN1TXError */
/* Use NiFpga_ReadArrayU8() to access Mod2CAN1TXError */
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_Resource = 0x18254;
const uint32_t NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_Type;


void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_Type* const destination);

void NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_CANBus_4module_v3_steering_IndicatorCluster_Mod2CAN1TXError_Type* const source);

#endif /* !NiFpga_VxWorks */


#if NiFpga_Cpp
}
#endif

#endif
