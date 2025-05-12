/********************************** (C) COPYRIGHT *******************************
* File Name          : rf_dtm.h
* Author             : WCH
* Version            : V1.0
* Date               : 2025/04/28
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

#define   TEST_PHY_MODE                 PHY_MODE_PHY_1M 
#define   PKT_DET_CFG4( var )          { (*((PUINT32V)0x4000C120))= var; } // Demodulation parameter
#define   AA           0x71764129
#define   AA_EX        0
#define   CRC_INIT     0X555555
#define   CRC_POLY     0x80032d


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
uint8_t Choose_CH( uint8_t cch );
void UART_Process_Data(void);
void PRBS9_Get( uint8_t *pData, uint16_t len );
void PRBS15_Get( uint8_t *pData, uint16_t len );
void TX_DATA( uint8_t* buf, uint8_t len );
void DtmProcess(void);

#ifdef __cplusplus
}
#endif

#endif
