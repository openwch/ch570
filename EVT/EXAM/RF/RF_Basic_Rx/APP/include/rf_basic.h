/********************************** (C) COPYRIGHT *******************************
* File Name          : RF_basic.h
* Author             : WCH
* Version            : V1.0
* Date               : 2022/03/10
* Description        : 
*******************************************************************************/

#ifndef __RF_BASIC_H
#define __RF_BASIC_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <CH572rf.h>
#include "CH57x_common.h"

#define   ACCESS_ADR   0x71762345
#define   CRC_INIT     0X555555
#define   CRC_POLY     0x80032d

#define  WAIT_ACK             0                 // 是否使能ACK(1:收到数据后回复ACK)

#define  RSSI_OFFSET          4                 // RSSI数据偏移量，不可修改
#define  RF_DEVICE_PERIOD     1000

#define PKT_DET_CFG4( var )     { (*((PUINT32V)0x4000C120))= var; } // Demodulation parameter

extern volatile uint32_t gPrintCnt;
extern volatile uint32_t gTxCount;
extern volatile uint32_t gRxCount;
extern volatile int32_t Rssi_sum;

void RFRole_Init(void);

#ifdef __cplusplus
}
#endif

#endif
