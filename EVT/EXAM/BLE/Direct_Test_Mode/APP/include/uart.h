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

#include "HAL.h"
#include "buf.h"

/*process events*/
#define UART_RECEIVE_POLL_EVT           (1<<0)

#define   UART_PROCESS_EVT              1

enum uart_status
{
    UART_STATUS_IDLE,
    UART_STATUS_RCVING,
    UART_STATUS_RCV_END,
    UART_STATUS_SENDING,

    UART_STATUS_NUM,
};

enum usb_status
{
    USB_STATUS_IDLE,
    USB_STATUS_RCVING,
    USB_STATUS_RCV_END,
    USB_STATUS_SENDING,

    USB_STATUS_NUM,
};

struct uart_process_msg {
    tmos_event_hdr_t hdr;
    uint8_t *data;
};

struct usb_process_msg {
    tmos_event_hdr_t hdr;
    uint8_t *data;
};

extern uint8_t uart_flag;

int uart_send(struct simple_buf *buf);
int uart_start_receiving(void);
void uart_task_init(void);


#endif /* BLE_DIRECTTEST_APP_INCLUDE_UART_H */
