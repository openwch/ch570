/********************************** (C) COPYRIGHT *******************************
 * File Name          : uart.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/06/30
 * Description        : 
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"
#include "HAL.h"
#include "uart.h"
#include "test_dtm.h"
#include "stdbool.h"

static tmosTaskID uart_taskid;

static uint8_t rx_buf[100];
static struct simple_buf *uart_buf = NULL;
static struct simple_buf uart_buffer;

uint8_t uart_flag;

/*********************************************************************
 * @fn      uart_buffer_create
 *
 * @brief   Create a file called uart_buffer simple buffer of the buffer
 *          and assign its address to the variable pointed to by the buffer pointer.
 *
 * @param   buf    -   a parameter buf pointing to a pointer.
 *
 * @return  none
 */
void uart_buffer_create(struct simple_buf **buf)
{
    *buf = simple_buf_create(&uart_buffer, rx_buf, sizeof(rx_buf));
}

/*********************************************************************
 * @fn      uart_start_receiving
 *
 * @brief   Serial port starts receiving.
 *
 * @param   none
 *
 * @return  none
 */
int uart_start_receiving(void)
{
    uart_flag = UART_STATUS_RCVING;
    uart_buffer_create(&uart_buf);
    PFIC_EnableIRQ(UART_IRQn);

    return 0;
}


/*********************************************************************
 * @fn      uart_send
 *
 * @brief   Serial port starts Sending.
 *
 * @param   buf    -   defined simple_buf structural object.
 *
 * @return  none
 */
int uart_send(struct simple_buf *buf)
{

    uart_flag = UART_STATUS_SENDING;

    uint8_t *send_data;
    uint16_t send_len;

    send_len = buf->len;
    send_data = simple_buf_pull(buf, buf->len);

    UART_SendString(send_data, send_len);
    uart_flag = UART_STATUS_IDLE;


//    PRINT("uart1 send %d bytes\n", send_len);
//    PRINT("[\n");
//    for(int i = 0; i < send_len; i++)
//    {
//        if(i) PRINT(" ");
//        PRINT("%#x", send_data[i]);
//    }
//    PRINT("]\n");

    return 0;
}


/*********************************************************************
 * @fn      uart_processevent
 *
 * @brief   process handle
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
tmosEvents uart_processevent(tmosTaskID task_id, tmosEvents events)
{
    if(events & SYS_EVENT_MSG)
    { // 处理HAL层消息，调用tmos_msg_receive读取消息，处理完成后删除消息。
        uint8_t *msgPtr;

        msgPtr = tmos_msg_receive(task_id);
        if(msgPtr)
        {
            /* De-allocate */
            tmos_msg_deallocate(msgPtr);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & UART_RECEIVE_POLL_EVT)
    {
        if(uart_flag == UART_STATUS_RCV_END)
        {
//            PRINT("uart recevied %d bytes\n", uart_buf->len);
//            PRINT("[\n");
//            for(int i = 0; i < uart_buf->len; i++)
//            {
//                if(i)
//                   PRINT(" ");
//                   PRINT("%#x", uart_buf->data[i]);
//            }
//            PRINT("]\n");

            struct uart_process_msg *uart_msg;

            uart_msg = (struct uart_process_msg *)      \
                tmos_msg_allocate(sizeof(struct uart_process_msg));

            if(uart_msg)
            {
                uart_msg->hdr.event = UART_PROCESS_EVT;
                uart_msg->hdr.status = true;
                uart_msg->data = (uint8_t *)uart_buf;
                tmos_msg_send(test_taskid, (uint8_t *)uart_msg );
            }
        }

        return (events ^ UART_RECEIVE_POLL_EVT);
    }

    return 0;
}

/*********************************************************************
 * @fn      uart_task_init
 *
 * @brief   Serial port initialization and task initialization.
 *
 * @param   none
 *
 * @return  none
 */
void uart_task_init(void)
{
    uart_taskid = TMOS_ProcessEventRegister(uart_processevent);

    GPIOA_SetBits(bTXD_0);
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(bRXD_0, GPIO_ModeIN_PU);

    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();
    UART_ByteTrigCfg(UART_7BYTE_TRIG);
    UART_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    tmos_start_reload_task(uart_taskid, UART_RECEIVE_POLL_EVT,
        MS1_TO_SYSTEM_TIME(100));

    uart_flag = UART_STATUS_IDLE;
    uart_start_receiving();
}

/*********************************************************************
 * @fn      UART0_IRQHandler
 *
 * @brief   UART0中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void UART_IRQHandler(void)
{
    volatile uint8_t i;

    switch(UART_GetITFlag())
    {
        case UART_II_LINE_STAT: // 线路状态错误
        {
            (void)UART_GetLinSTA();
            break;
        }

        case UART_II_RECV_RDY: // 数据达到设置触发点
            for(i = 0; i < (7 - 1); i++)
            {
                simple_buf_add_u8(uart_buf, UART_RecvByte());
            }
            break;

        case UART_II_RECV_TOUT: // 接收超时，暂时一帧数据接收完成
            while(R8_UART_RFC)
            {
                simple_buf_add_u8(uart_buf, UART_RecvByte());
            }
            uart_flag = UART_STATUS_RCV_END;
            PFIC_DisableIRQ(UART_IRQn);
            break;

        case UART_II_THR_EMPTY: // 发送缓存区空，可继续发送
            break;

        default:
            break;
    }
}
