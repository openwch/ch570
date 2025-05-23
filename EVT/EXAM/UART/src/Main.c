/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 串口1收发演示
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

uint8_t TxBuff[] = "This is a tx exam\r\n";
uint8_t RxBuff[100];
uint8_t trigB;

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
__HIGH_CODE
int main()
{
    uint8_t len;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口1：先配置IO口模式，再配置串口 */
    GPIOA_SetBits(bTXD_0);
    GPIOA_ModeCfg(bRXD_0, GPIO_ModeIN_PU);      // RXD-配置上拉输入
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();

#if 1 // 测试串口发送字符串
    UART_SendString(TxBuff, sizeof(TxBuff));
#endif

#if 1 // 查询方式：接收数据后发送出去
    while(1)
    {
        len = UART_RecvString(RxBuff);
        if(len)
        {
            UART_SendString(RxBuff, len);
        }
    }

#endif

#if 0 // 中断方式：接收数据后发送出去
    UART_ByteTrigCfg(UART_7BYTE_TRIG);
    trigB = 7;
    UART_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    PFIC_EnableIRQ(UART_IRQn);
#endif

    while(1);
}

/*********************************************************************
 * @fn      UART_IRQHandler
 *
 * @brief   UART中断函数
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
            UART_GetLinSTA();
            break;
        }

        case UART_II_RECV_RDY: // 数据达到设置触发点
            for(i = 0; i != trigB; i++)
            {
                RxBuff[i] = UART_RecvByte();
                UART_SendByte(RxBuff[i]);
            }
            break;

        case UART_II_RECV_TOUT: // 接收超时，暂时一帧数据接收完成
            i = UART_RecvString(RxBuff);
            UART_SendString(RxBuff, i);
            break;

        case UART_II_THR_EMPTY: // 发送缓存区空，可继续发送
            break;

        default:
            break;
    }
}
