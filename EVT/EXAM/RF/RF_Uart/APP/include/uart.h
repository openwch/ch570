/********************************** (C) COPYRIGHT *******************************
 * File Name          : uart.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/06/30
 * Description        : 
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/


#ifndef BLE_DIRECTTEST_APP_INCLUDE_UART_H
#define BLE_DIRECTTEST_APP_INCLUDE_UART_H

#include "buf.h"


#define    TXD_PIN   bTXD_3  // PA0
#define    RXD_PIN   bRXD_3  // PA1

#define    DTR       (1<<2)  // PA2
#define    RTS       (1<<3)  // PA3

#define  UART_BUF_LEN   (1024*3)

enum uart_status
{
    UART_STATUS_IDLE,
    UART_STATUS_START,
    UART_STATUS_RCVING,
    UART_STATUS_RCV_END,
    UART_STATUS_SENDING,
    UART_STATUS_SEND,
    UART_STATUS_NUM,
};

#define  DATA_LEN_UART       (32)

extern uint8_t gBoundStatus;

void UART_Init(void);
uint8_t UART_RxQuery( void *buf, typeBufSize *len );
void UART_SetBuad( uint32_t buad );
void UART_SetTimer( uint16_t ms );
void UART_Send( char* data, uint16_t size);

#endif /* BLE_DIRECTTEST_APP_INCLUDE_UART_H */
