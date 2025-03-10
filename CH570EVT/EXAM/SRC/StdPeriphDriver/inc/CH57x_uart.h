/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_uart.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH57x_UART_H__
#define __CH57x_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	LINE error and status define
 */
#define STA_ERR_BREAK     RB_LSR_BREAK_ERR    // 数据间隔错误
#define STA_ERR_FRAME     RB_LSR_FRAME_ERR    // 数据帧错误
#define STA_ERR_PAR       RB_LSR_PAR_ERR      // 奇偶校验位出错
#define STA_ERR_FIFOOV    RB_LSR_OVER_ERR     // 接收数据溢出

#define STA_TXFIFO_EMP    RB_LSR_TX_FIFO_EMP  // 当前发送FIFO空，可以继续填充发送数据
#define STA_TXALL_EMP     RB_LSR_TX_ALL_EMP   // 当前所有发送数据都发送完成
#define STA_RECV_DATA     RB_LSR_DATA_RDY     // 当前有接收到数据

/**
 * @brief  Configuration UART TrigByte num
 */
typedef enum
{
    UART_1BYTE_TRIG = 0, // 1字节触发
    UART_2BYTE_TRIG,     // 2字节触发
    UART_4BYTE_TRIG,     // 4字节触发
    UART_7BYTE_TRIG,     // 7字节触发

} UARTByteTRIGTypeDef;

/**
 * @brief   串口默认初始化配置
 */
void UART_DefInit(void);

/**
 * @brief   串口波特率配置
 *
 * @param   baudrate    - 波特率
 */
void UART_BaudRateCfg(uint32_t baudrate);

/**
 * @brief   串口字节触发中断配置
 *
 * @param   b       - 触发字节数 refer to UARTByteTRIGTypeDef
 */
void UART_ByteTrigCfg(UARTByteTRIGTypeDef b);

/**
 * @brief   串口中断配置
 *
 * @param   s       - 中断控制状态，是否使能相应中断
 * @param   i       - 中断类型
 *                    RB_IER_MODEM_CHG  - 调制解调器输入状态变化中断使能位（仅 UART0 支持）
 *                    RB_IER_LINE_STAT  - 接收线路状态中断
 *                    RB_IER_THR_EMPTY  - 发送保持寄存器空中断
 *                    RB_IER_RECV_RDY   - 接收数据中断
 */
void UART_INTCfg(FunctionalState s, uint8_t i);

/**
 * @brief   清除当前接收FIFO
 */
#define UART_CLR_RXFIFO()    (R8_UART_FCR |= RB_FCR_RX_FIFO_CLR)

/**
 * @brief   清除当前发送FIFO
 */
#define UART_CLR_TXFIFO()    (R8_UART_FCR |= RB_FCR_TX_FIFO_CLR)

/**
 * @brief   获取当前中断标志
 *
 * @return  当前中断标志
 */
#define UART_GetITFlag()     (R8_UART_IIR & RB_IIR_INT_MASK)

/**
 * @brief   获取当前通讯状态
 *
 * @return  refer to LINE error and status define
 */
#define UART_GetLinSTA()     (R8_UART_LSR)

/**
 * @brief   串口单字节发送
 *
 * @param   b       待发送的字节
 */
#define UART_SendByte(b)     (R8_UART_THR = b)

/**
 * @brief   串口多字节发送
 *
 * @param   buf     - 待发送的数据内容首地址
 * @param   l       - 待发送的数据长度
 */
void UART_SendString(uint8_t *buf, uint16_t l);

/**
 * @brief   串口读取单字节
 *
 * @return  读取到的单字节
 */
#define UART_RecvByte()    (R8_UART_RBR)

/**
 * @brief   串口读取多字节
 *
 * @param   buf     - 读取数据存放缓存区首地址
 *
 * @return  读取数据长度
 */
uint16_t UART_RecvString(uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif // __CH57x_UART_H__
