/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/14
 * Description        : 定时器输入捕捉例程，DMA方式连续捕捉PA7上100个边沿数据
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

__attribute__((aligned(4))) uint32_t CapBuf[102];

volatile uint8_t capFlag = 0;

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
    PRINT("TMR capture @PA7\n");

    /* 定时器CAP捕捉，配置捕捉口PA7 */
    GPIOA_ResetBits(GPIO_Pin_7);
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PU);

    TMR_CapInit(Edge_To_Edge);
    TMR_CAPTimeoutCfg(0x1FFFFFF); // 设置捕捉超时时间
    TMR_DMACfg(ENABLE, (uint32_t)&CapBuf[0], (uint32_t)&CapBuf[102], Mode_Single);
    TMR_ITCfg(ENABLE, TMR_IT_DMA_END); // 开启DMA完成中断
    PFIC_EnableIRQ(TMR_IRQn);

    /* 等待100个捕捉数据通过DMA存放完成 */
    while(capFlag == 0);
    // 丢弃捕捉的前两个值
    for(i = 2; i < 102; i++)
    {
        PRINT("%08ld ", CapBuf[i] & 0x1ffffff); // 26bit, 最高位表示 高电平还是低电平
    }
    PRINT("\n");

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
        TMR_ITCfg(DISABLE, TMR_IT_DMA_END); // 使用单次DMA功能+中断，注意完成后关闭此中断使能，否则会一直上报中断。
        capFlag = 1;
    }
}
