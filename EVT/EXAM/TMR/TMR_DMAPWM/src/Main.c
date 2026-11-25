/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/14
 * Description        : 定时器DMA动态加载PWM数据例程，PA7输出脉宽/占空比渐变的PWM波形
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

#define g_10us (FREQ_SYS / 100000)

__attribute__((aligned(4))) uint32_t PwmBuf[100];

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
    uint8_t i;

    FlashLock();
    // 关闭两线调试
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("TMR DMA PWM @PA7\n");

    /* 定时器DMA PWM，配置PWM口PA7 */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);

    TMR_PWMCycleCfg(g_10us * 250); 
    for(i = 0; i < 50; i++)
    {
        PwmBuf[i] = (g_10us / 2 * 10) * i;
    }
    for(i = 50; i < 100; i++)
    {
        PwmBuf[i] = (g_10us / 2 * 10) * (100 - i);
    }
    TMR_PWMInit(Low_Level, PWM_Times_16);
    TMR_DMACfg(ENABLE, (uint32_t)&PwmBuf[0], (uint32_t)&PwmBuf[100], Mode_LOOP);
    TMR_PWMEnable();
    TMR_Enable();
    /* 开启DMA完成中断，一轮DMA数据发送完毕进入中断 */
    TMR_ClearITFlag(TMR_IT_DMA_END);
    TMR_ITCfg(ENABLE, TMR_IT_DMA_END);
    PFIC_EnableIRQ(TMR_IRQn);

    while(1);
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
    if(TMR_GetITFlag(TMR_IT_DMA_END))
    {
        TMR_ClearITFlag(TMR_IT_DMA_END);
        /* LOOP模式下DMA完成一轮数据后重新循环发送，用户可在此更新PwmBuf数据 */
        /* 用户可自行添加需要的处理 */
    }
}
