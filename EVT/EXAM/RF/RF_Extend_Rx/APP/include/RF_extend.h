/********************************** (C) COPYRIGHT *******************************
* File Name          : RF_extend.h
* Author             : WCH
* Version            : V1.0
* Date               : 2025/06/24
* Description        : 
*******************************************************************************/

#ifndef __RF_EXTEND_H
#define __RF_EXTEND_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <CH572rf.h>
#include "CH57x_common.h"

#define  WAIT_ACK           0               // 是否使能ACK
#define  RF_DEVICE_PERIOD   1000

#define  USER_MODE          0               //UserMode，按需配置  0:普通模式，1：修改解调参数，2：3字节地址 3：修改数据格式
#define  DPL_MODE_EN        0               //增强模式使能,按需配置
#define  MODE_2G4       PHY_2G4_2M          // 速率 0: 2M; 1: 1M

#define  RSSI_OFFSET            2           // RSSI数据偏移量，不可修改
#define  RSSI_DPL_OFFSET        4           // RSSI数据偏移量，不可修改
#define  ADDR_3BYTE             0           // 3字节地址配置，不可修改
#define  ADDR_4BYTE             1           // 4字节地址配置，不可修改
#define  ADDR_5BYTE             2           // 5字节地址配置，不可修改
#define  TEST_PHY_MODE      PHY_MODE_2G4    // 2.4G模式，不可修改
#define  CFG4_REG_VAL           0x78        // 基带寄存器配置值，不可修改
#define  CFG3_1M_ADDR3          124         // 基带寄存器配置值，不可修改
#define  CFG3_1M_ADDR4_5        156         // 基带寄存器配置值，不可修改
#define  CFG3_2M_ADDR3          128         // 基带寄存器配置值，不可修改
#define  CFG3_2M_ADDR4_5        160         // 基带寄存器配置值，不可修改

#if(USER_MODE == 0 )
#define   AA           0x94826E8E           // 接入地址（3-4字节）
#define   AA_EX        0                    // 接入地址ex,在AA前
#define   AA_LEN       ADDR_4BYTE           // 0: 3字节; 1: 4字节; 2: 5字节
#define   DATA_ORDER   0                    // 数据bit顺序，0: MSB; 1: LSB
#define   CRC_XOR_EN   0                    // CRC异或使能, 0: 关闭; 1: 使能

#elif(USER_MODE == 1 )
#define   AA           0x94826E8E
#define   AA_EX        0
#define   AA_LEN       ADDR_4BYTE
#define   DATA_ORDER   0
#define   CRC_XOR_EN   0

#elif(USER_MODE == 2 )
#define   AA           0x00826E8E
#define   AA_EX        0
#define   AA_LEN       ADDR_3BYTE            
#define   DATA_ORDER   0
#define   CRC_XOR_EN   0

#elif(USER_MODE == 3 )
#define   AA           0x94826E8E
#define   AA_EX        0
#define   AA_LEN       ADDR_4BYTE
#define   DATA_ORDER   1
#define   CRC_XOR_EN   1
#endif

#if( DPL_MODE_EN )
#define   CTL_FIELD    1                   // 增强模式控制字长度，0:0bit，1:9bit，2:10bit
#define   DPL_EN       1                   // 增强模式  0:关闭 ; 1:增强；增强模式最大发送长度为63字节
#else
#define   CTL_FIELD    0                   // 增强模式控制字长度，0:0bit，1:9bit，2:10bit
#define   DPL_EN       0                   // 增强模式  0:关闭 ; 1:增强；增强模式最大发送长度为63字节
#endif

#define   PRE_LEN      1                   // 前导码长度, 1: 1字节; 3: 3字节
#define   CRC_LEN      2                   // CRC长度,  0: 0字节; 1: 1字节; 2: 2字节
#define   CRC_INIT     0xFFFF              // CRC初始值
#define   CRC_POLY     0x8810              // CRC多项式

#define PKT_DET_CFG4( var )     { (*((PUINT32V)0x4000C120))= var; } // Demodulation parameter
#define PKT_DET_CFG3( timacq )  { (*((PUINT32V)0x4000C11C))&=~0x3ff;  (*((PUINT32V)0x4000C11C))|= timacq & 0x3ff;}// Synchronization delay value

extern volatile uint32_t gTxCount;
extern volatile uint32_t gRxCount;
extern volatile int32_t Rssi_sum;
extern volatile uint32_t gPrintCnt;

void RFRole_Init(void);

#ifdef __cplusplus
}
#endif

#endif
