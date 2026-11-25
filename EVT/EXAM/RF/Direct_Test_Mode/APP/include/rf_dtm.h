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

#define  R32_PA_CTRL      (*((PUINT32V)0x4000D05C))
#define  R32_LNA_CTRL1    (*((PUINT32V)0x4000D048))
#define  R32_LNA_CTRL2    (*((PUINT32V)0x4000D04C))

/* ==================== √¸¡Ó¬Î(’˚÷°≥ı÷µ) ==================== */
#define  UART_REC_LEN               10
#define  DTM_CMD_END_TEST           {0x01, 0x1F, 0x20, 0x00}
#define  DTM_CMD_RESET              {0x01, 0x03, 0x0C, 0x00}
#define  DTM_CMD_RX_1M              {0x01, 0x1D, 0x20, 0x01}
#define  DTM_CMD_TX_1M              {0x01, 0x1E, 0x20, 0x03}
#define  DTM_CMD_RX_2M              {0x01, 0x33}
#define  DTM_CMD_TX_2M              {0x01, 0x34}
#define  DTM_CMD_SET_POWER          {0x01, 0x01, 0xFF, 0x01}
#define  DTM_CMD_SET_PF             {0x01, 0x02, 0xFF, 0x01}
#define  DTM_CMD_SET_PA             {0x01, 0x11, 0xFF, 0x03}
#define  DTM_CMD_SET_LNA            {0x01, 0x12, 0xFF, 0x03}
#define  DTM_CMD_SINGLE_CARRIER     {0x01, 0x03, 0xFF, 0x01}

#define  DTM_TX_WAITTIME            (40u * 6u)
#define  DTM_CMD_BUF_LEN            23

#define  PRBS9_SEED                 0x01FFu
#define  PRBS15_SEED                0x7FFFu

typedef enum {
    dtm_tx = 0,
    dtm_rx,
    dtm_init = 0xff
}DTM_MODE_DEF;

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
