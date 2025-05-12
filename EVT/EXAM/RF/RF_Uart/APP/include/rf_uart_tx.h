/********************************** (C) COPYRIGHT *******************************
* File Name          : rf_uart_tx.h
* Author             : WCH
* Version            : V1.0
* Date               : 2022/03/10
* Description        : 
*******************************************************************************/

#ifndef __RF_UART_TX_H
#define __RF_UART_TX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <CH572rf.h>
#include "CH57x_common.h"
#include "buf.h"


#define  ADV_INTERVAL        20

#define  RESEND_COUNT        40

#define  BOUND_INFO_FLASH_ADDR         (1024*236)



/* rf tx status */
#define   STA_IDLE          0x00
#define   STA_BUSY          0x01
#define   STA_RESEND        0x02

void RF_UartTxInit( void );
void RF_StatusQuery( void );


#ifdef __cplusplus
}
#endif

#endif
