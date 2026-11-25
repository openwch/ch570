/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/09/14
 * Description        : 定时器编码器模式例程，P前进/后退中断及计数值查询
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
    GPIOA_ModeCfg(bRXD_4, GPIO_ModeIN_PU);      // RXD-配置上拉输入
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA6);
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
    PRINT("ENCODER mode @PA7/PA2\n");

    /* ENC检测高电平信号，配置IO为下拉输入 */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PD);
    GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PD);

    /* ENC模式配置，设定计数终值为最大值，在T1,T2边沿计数模式 */
    ENC_Config(ENABLE, 0xFFFF, Mode_T1T2);

    /* 演示编码器中断 */
    ENC_ITCfg(ENABLE, RB_IE_DIR_INC); // 使能前进中断
    ENC_ITCfg(ENABLE, RB_IE_DIR_DEC); // 使能后退中断
    PFIC_EnableIRQ(ENCODE_IRQn);

    while(1)
    {
        mDelaymS(300);
        PRINT("dir:%d\n", ENC_GetCurrentDir());      // 获取编码器当前方向
        PRINT("cnt:%u\n", (unsigned)ENC_GetCurrentCount());    // 获取编码器当前计数值
    }
}

/*********************************************************************
 * @fn      ENCODER_IRQHandler
 *
 * @brief   Encoder中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void ENCODER_IRQHandler(void)
{
    if(ENC_GetITFlag(RB_IF_DIR_INC))
    {
        ENC_ClearITFlag(RB_IF_DIR_INC);
    }
    if(ENC_GetITFlag(RB_IF_DIR_DEC))
    {
        ENC_ClearITFlag(RB_IF_DIR_DEC);
    }
}
