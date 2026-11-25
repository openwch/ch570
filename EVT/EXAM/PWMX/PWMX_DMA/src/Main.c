/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/14
 * Description        : PWMX 16位数据宽度DMA加载例程，DMA方式更新PWM周期和占空比数据
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

/* PWM1=30000 PWM2=20000 PWM3=30000 PWM4=20000 PWM5=15000 */
__attribute__((aligned(4))) uint16_t PwmBuf[8] = {30000, 20000, 30000, 20000, 15000, 0};

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

    PWMX_CLKCfg(4);                                     // cycle = 4/Fsys
    PWM_16bit_CycleEnable();                            // 16位宽使能
    PWMX_16bit_CycleCfg(CH_PWM_ALL, 60000 - 1);         // 16 数据宽度 Ncyc=RB_PWM_CYC_VALUE+1
    PWMXOutCfg(CH_PWM_ALL, ENABLE);
    /* DMA方式加载PWM数据，DMA通道选择PWM1-5 */
    PWM_DMACfg(ENABLE, (uint32_t)&PwmBuf[0], (uint32_t)&PwmBuf[6], PWM_ModeSINGLE, Mode_DMACH1_5);
    PWM_INTCfg(ENABLE, RB_PWM_IE_DMA);
    PFIC_EnableIRQ(PWMX_IRQn);

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
    if(R8_PWM_INT_FLAG & RB_PWM_IF_DMA)
    {
        R8_PWM_DMA_CTRL &= ~(RB_DMA_ENABLE); // 单次DMA传输完成后关闭DMA
        R8_PWM_INT_FLAG = RB_PWM_IF_DMA;
        PFIC_DisableIRQ(PWMX_IRQn);          // 使用单次DMA功能+中断，注意完成后关闭此中断使能，否则会一直上报中断。
    }
}
