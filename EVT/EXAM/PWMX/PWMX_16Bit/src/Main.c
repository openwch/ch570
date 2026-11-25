/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/14
 * Description        : PWMX 16位数据宽度输出例程，PWM1-5输出不同占空比波形，并演示周期切换中断
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

uint16_t nextPWMCyc = 50000 - 1;

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

    PWMX_CLKCfg(4);                 // cycle = 4/Fsys
    PWM_16bit_CycleEnable();        // 16位宽使能
    PWMX_16bit_CycleCfg(CH_PWM_ALL, 60000 - 1); // 16 数据宽度 Ncyc=RB_PWM_CYC_VALUE+1
    PWMX_16bit_ACTOUT(CH_PWM1, 30000, Low_Level, ENABLE);  // 50%占空比
    PWMX_16bit_ACTOUT(CH_PWM2, 15000, Low_Level, ENABLE);  // 25%占空比
    PWMX_16bit_ACTOUT(CH_PWM3, 45000, Low_Level, ENABLE);  // 75%占空比
    PWMX_16bit_ACTOUT(CH_PWM4, 30000, High_Level, ENABLE); // 50%占空比
    PWMX_16bit_ACTOUT(CH_PWM5, 15000, High_Level, ENABLE); // 25%占空比

    /* 切换周期演示，在进行周期切换时需要等到上一个周期完成后再切，以PWM1为例 */
    // nextPWMCyc = 50000 / 2 - 1;
    // PWM_INTCfg(ENABLE, RB_PWM_IE_CYC);
    // PFIC_EnableIRQ(PWMX_IRQn);

    while(1);
}

/*********************************************************************
 * @fn      PWMX_IRQHandler
 *
 * @brief   PWMX中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void PWMX_IRQHandler(void)
{
    if(R8_PWM_INT_FLAG & RB_PWM_IF_CYC)
    {
        R8_PWM_INT_FLAG = RB_PWM_IF_CYC;
        PWMX_16bit_ACTOUT(CH_PWM1 | CH_PWM2 | CH_PWM3, nextPWMCyc / 2, Low_Level, ENABLE); // 切换周期时需要先改变占空比
        PWMX_16bit_CycleCfg(CH_PWM1 | CH_PWM2 | CH_PWM3, nextPWMCyc); //注意PWM1/2/3的16位周期必须同时改，PWM4/5的16位周期必须同时改，此处仅演示PWM1/2/3
        PFIC_DisableIRQ(PWMX_IRQn); // 演示一次周期切换后关闭中断
    }
}
