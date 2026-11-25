/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/14
 * Description        : PWMX 8位数据宽度输出例程，PWM1-5输出25%占空比波形
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

#define PWMX_CYCLE_NUM   64

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

    /* 配置GPIO，PWM1-5默认复用引脚 */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA); // PA7 - PWM1
    GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeOut_PP_5mA); // PA2 - PWM2
    GPIOA_ModeCfg(GPIO_Pin_3, GPIO_ModeOut_PP_5mA); // PA3 - PWM3
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA); // PA4 - PWM4
    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeOut_PP_5mA); // PA8 - PWM5

    PWMX_CLKCfg(4);                // cycle = 4/Fsys
    PWMX_CycleCfg(PWMX_Cycle_64);  // 周期 = 64*cycle

    PWMX_ACTOUT(CH_PWM1, PWMX_CYCLE_NUM / 4, Low_Level, ENABLE); // 25% 占空比
    PWMX_ACTOUT(CH_PWM2, PWMX_CYCLE_NUM / 4, Low_Level, ENABLE); // 25% 占空比
    PWMX_ACTOUT(CH_PWM3, PWMX_CYCLE_NUM / 4, Low_Level, ENABLE); // 25% 占空比
    PWMX_ACTOUT(CH_PWM4, PWMX_CYCLE_NUM / 4, Low_Level, ENABLE); // 25% 占空比
    PWMX_ACTOUT(CH_PWM5, PWMX_CYCLE_NUM / 4, Low_Level, ENABLE); // 25% 占空比

    while(1);
}
