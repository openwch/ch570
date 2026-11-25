/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/14
 * Description        : 定时器外部事件计数例程，PA7输入脉冲计数，每1000个脉冲产生溢出中断
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试串口初始化
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOA_SetBits(bTXD_0);
    GPIOA_ModeCfg(bRXD_0, GPIO_ModeIN_PU);      // RXD-配置上拉输入
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();
}

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main(void)
{
    FlashLock();
    // 关闭两线调试
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("TMR external count @PA7\n");

    /* 定时器计数器，PA7输入脉冲计数 */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PD);

    TMR_EXTSingleCounterInit(FallEdge_To_FallEdge);
    TMR_CountOverflowCfg(1000); // 设置计数上限1000

    /* 开启计数溢出中断，计满1000个脉冲进入中断，硬件自动清零重新开始计数 */
    TMR_ClearITFlag(TMR_IT_CYC_END);
    PFIC_EnableIRQ(TMR_IRQn);
    TMR_ITCfg(ENABLE, TMR_IT_CYC_END);

    do
    {
        /* 1s打印一次当前计数值，如果送入脉冲频率较高，可能很快计数溢出，需要按实际情况修改 */
        mDelaymS(1000);
        PRINT("=%ld \n", TMR_GetCurrentCount());
    } while(1);
}

/*********************************************************************
 * @fn      TMR_IRQHandler
 *
 * @brief   TMR中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR_IRQHandler(void)
{
    if(TMR_GetITFlag(TMR_IT_CYC_END))
    {
        TMR_ClearITFlag(TMR_IT_CYC_END); // 清除中断标志
        /* 计数器计满，硬件自动清零，重新开始计数 */
        /* 用户可自行添加需要的处理 */
    }
}
