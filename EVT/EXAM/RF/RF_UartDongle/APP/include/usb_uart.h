/********************************** (C) COPYRIGHT *******************************
* File Name          : usb_uart.h
* Author             : WCH
* Version            : V1.0
* Date               : 2025/04/22
* Description        : 
*******************************************************************************/

#ifndef __USB_UART_H
#define __USB_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "CH57x_common.h"
#include "buf.h"

//Line Code结构
 typedef struct __PACKED _LINE_CODE
{
  uint32_t  BaudRate;   /* 波特率 */
  uint8_t StopBits;   /* 停止位计数，0：1停止位，1：1.5停止位，2：2停止位 */
  uint8_t ParityType;   /* 校验位，0：None，1：Odd，2：Even，3：Mark，4：Space */
  uint8_t DataBits;   /* 数据位计数：5，6，7，8，16 */
  uint8_t ioStaus;
}LINE_CODE, *PLINE_CODE;
extern LINE_CODE Uart0Para;

void USB_Init( void );
void USB_StatusQuery( void );
uint8_t USB_RxQuery( void *buf, typeBufSize *len );

#ifdef __cplusplus
}
#endif

#endif
