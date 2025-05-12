/********************************** (C) COPYRIGHT *******************************
* File Name          : rf_test.h
* Author             : WCH
* Version            : V1.0
* Date               : 2022/03/10
* Description        : 
*******************************************************************************/

#ifndef __RF_TEST_H
#define __RF_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <CH572rf.h>
#include "CH57x_common.h"

#define  TEST_PHY_MODE   PHY_MODE_PHY_1M
#if(TEST_PHY_MODE == PHY_MODE_2G4 )

#if(PHY_2G4_MODE == 0 )
#define   AA           0x94826E8E          // 接入地址（3-4字节）
#define   AA_EX        0                   // 接入地址ex,在AA前
#define   AA_LEN       1                   // 0: 3字节; 1: 4字节; 2: 5字节
#define   PRE_LEN      1                   // 前导码长度, 1: 1字节; 3: 3字节
#define   CRC_INIT     0xFFFF              // CRC初始值
#define   CRC_POLY     0x8810              // CRC多项式
#define   CRC_LEN      2                   // CRC长度,  0: 0字节; 1: 1字节; 2: 2字节
#define   CTL_FILED    0                   // 增强模式控制字长度，0:0bit，1:9bit，2:10bit
#define   DPL_EN       0                   // 增强模式  0:关闭 ; 1:增强；增强模式最大发送长度为63字节
#define   DATA_ORDER   0                   // 数据bit顺序，0: MSB; 1: LSB
#define   MODE_2G4     PHY_2G4_1M          // 速率,0: 2M; 1: 1M
#define   CRC_XOR_EN   0                   // CRC异或使能, 0: 关闭; 1: 使能

#elif(PHY_2G4_MODE == 1 )
#define PKT_DET_CFG4( var )          { (*((PUINT32V)0x4000C120))= var; } // Demodulation parameter
#define   AA           0x94826E8E
#define   AA_EX        0
#define   AA_LEN       1
#define   PRE_LEN      1
#define   CRC_INIT     0xFFFF
#define   CRC_POLY     0x8810
#define   CRC_LEN      2
#define   CTL_FILED    0
#define   DPL_EN       0
#define   DATA_ORDER   0
#define   MODE_2G4     PHY_2G4_1M
#define   CRC_XOR_EN   0

#elif(PHY_2G4_MODE == 2 )
#define   AA           0x94826E8E
#define   AA_EX        0
#define   AA_LEN       1
#define   PRE_LEN      1
#define   CRC_INIT     0xFFFF
#define   CRC_POLY     0x8810
#define   CRC_LEN      2
#define   CTL_FILED    0
#define   DPL_EN       0
#define   DATA_ORDER   1
#define   MODE_2G4     PHY_2G4_1M
#define   CRC_XOR_EN   1

#endif

#else

#define   AA           0x71764129
#define   AA_EX        0
#define   CRC_INIT     0X555555
#define   CRC_POLY     0x80032d
#endif

typedef struct
{
    uint32_t errContinue;
    uint32_t errCount;
    uint32_t txCount;
    uint32_t rxCount;
    uint8_t testCount;
    uint8_t testData;
    int8_t  rssi;
    int8_t  rssiMax;
    int8_t  rssiMin;
    uint8_t boundEst;
    uint8_t boundConnect;
} testBound_t;

void RFRole_Init(void);
void RF_LowPower( uint32_t time);
void RF_ProcessRx( void );

#ifdef __cplusplus
}
#endif

#endif
