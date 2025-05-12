/********************************** (C) COPYRIGHT *******************************
* File Name          : rf_uart_rx.h
* Author             : WCH
* Version            : V1.0
* Date               : 2022/03/10
* Description        : 
*******************************************************************************/

#ifndef __RF_UART_RX_H
#define __RF_UART_RX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <CH572rf.h>
#include "CH57x_common.h"
#include "buf.h"

#define   RF_BUF_LEN   512


#define  CONN_INTERVAL     50
#define  CONN_TIMEOUT      100
#define  CONN_PHY_TYPE     1   // 2M

void RF_UartRxInit( void );
uint8_t RF_RxQuery( void *buf, typeBufSize *len );


#ifdef __cplusplus
}
#endif

#endif
