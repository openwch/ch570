/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 定时器例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

#define  TIM_MODE      1
#define  PWM_MODE      0
#define  CAP_MODE      0
#define  COUNT_MODE    0
#define  PWM_DMAMODE   0
#define  ENC_MODE      0

__attribute__((aligned(4))) uint32_t CapBuf[100];
__attribute__((aligned(4))) uint32_t PwmBuf[100];

volatile uint8_t capFlag = 0;

#define  g_10us  (FREQ_SYS/100000)

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试初始化
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
int main()
{
    uint8_t i;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if TIM_MODE  /* 定时器，设定100ms定时器进行IO口闪灯， PA6-LED */
    GPIOA_SetBits(GPIO_Pin_6);
    GPIOA_ModeCfg(GPIO_Pin_6, GPIO_ModeOut_PP_5mA);

    TMR_TimerInit(FREQ_SYS / 10);        // 设置定时时间 100ms
    TMR_ITCfg(ENABLE, TMR_IT_CYC_END);    // 开启中断
    PFIC_EnableIRQ(TMR_IRQn);
#endif

#if PWM_MODE /* 定时器，PWM输出 */

    GPIOA_ResetBits(GPIO_Pin_7); // 配置PWM口 PA7
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    TMR_PWMInit(High_Level, PWM_Times_1);
    TMR_PWMCycleCfg(g_10us * 10); // 周期 100us  最大67108864
    TMR_PWMActDataWidth(g_10us/2 * 10);  // 占空比 50%, 修改占空比必须暂时关闭定时器
    TMR_PWMEnable();
    TMR_Enable();

#endif

#if CAP_MODE                               /* 定时器，CAP捕捉， */
    GPIOA_ResetBits(GPIO_Pin_7);           // 配置捕捉口 PA7
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PU);

    TMR_CapInit(Edge_To_Edge);
    TMR_CAPTimeoutCfg(0xFFFFFFFF); // 设置捕捉超时时间
    TMR_DMACfg(ENABLE, (uint16_t)(uint32_t)&CapBuf[0], (uint16_t)(uint32_t)&CapBuf[100], Mode_Single);
    TMR_ITCfg(ENABLE, TMR_IT_DMA_END); // 开启DMA完成中断
    PFIC_EnableIRQ(TMR_IRQn);

    while(capFlag == 0);
    capFlag = 0;
    for(i = 0; i < 100; i++)
    {
        PRINT("%08ld ", CapBuf[i] & 0x1ffffff); // 26bit, 最高位表示 高电平还是低电平
    }
    PRINT("\n");

#endif

#if COUNT_MODE   /* 定时器，计数器， */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PD);

    TMR_EXTSingleCounterInit(FallEdge_To_FallEdge);
    TMR_CountOverflowCfg(1000); // 设置计数上限1000

    /* 开启计数溢出中断，计慢1000个周期进入中断 */
    TMR_ClearITFlag(TMR_IT_CYC_END);
    PFIC_EnableIRQ(TMR_IRQn);
    TMR_ITCfg(ENABLE, TMR_IT_CYC_END);

    do
    {
        /* 1s打印一次当前计数值，如果送入脉冲频率较高，可能很快计数溢出，需要按实际情况修改 */
        mDelaymS(1000);
        PRINT("=%ld \n", TMR_GetCurrentCount());
    } while(1);

#endif

#if PWM_DMAMODE   /* 定时器,DMA PWM.*/
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);

    PRINT("TMR DMA PWM\n");
    TMR_PWMCycleCfg(g_10us * 200); // 周期 2000us
    for(i=0; i<50; i++)
    {
      PwmBuf[i]=(g_10us/2 * 10) * i;
    }
    for(i=50; i<100; i++)
    {
      PwmBuf[i]=(g_10us/2 * 10)*(100-i);
    }
    TMR_PWMInit(Low_Level, PWM_Times_16);
    TMR_DMACfg(ENABLE, (uint32_t)&PwmBuf[0], (uint32_t)&PwmBuf[100], Mode_LOOP);
    TMR_PWMEnable();
    TMR_Enable();
    /* 开启计数溢出中断，计满100个周期进入中断 */
    TMR_ClearITFlag(TMR_IT_DMA_END);
    TMR_ITCfg(ENABLE, TMR_IT_DMA_END);
    PFIC_EnableIRQ(TMR_IRQn);
#endif

#if ENC_MODE   /* 编码器.*/

    /* ENC检测高电平信号，配置IO为下拉输入 */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_PD);
    GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PD);

    /* ENC模式配置，设定计数终值为最大值，在T1,T2边沿计数模式 */
    ENC_Config(ENABLE, 0xFFFF, Mode_T1T2);

    /* 演示编码器中断 */
    ENC_ITCfg(ENABLE, RB_IE_DIR_INC);       // 使能前进中断
    ENC_ITCfg(ENABLE, RB_IE_DIR_DEC);       // 使能后退中断
    PFIC_EnableIRQ(ENCODE_IRQn);

    /* 演示编码器唤醒睡眠 */
#if 0
    PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_ENC_WAKE, Fsys_Delay_4096);
    PRINT("sleep mode sleep \n");
    DelayMs(2);
    LowPower_Sleep( RB_PWR_RAM12K | RB_PWR_XROM | RB_PWR_EXTEND);
    PRINT("wake.. \n");
#endif

    while(1)
    {
        DelayMs(300);
        PRINT("%d\n", ENC_GetCurrentDir);       // 获取编码器当前方向
        PRINT("cnt:%d\n",ENC_GetCurrentCount);  // 获取编码器当前计数值
    }
#endif

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
void TMR_IRQHandler(void) // TMR0 定时中断
{
    if(TMR_GetITFlag(TMR_IT_CYC_END))
    {
        TMR_ClearITFlag(TMR_IT_CYC_END); // 清除中断标志
#if TIM_MODE
        GPIOA_InverseBits(GPIO_Pin_6);
#endif
#if CAP_MODE
        TMR_ITCfg(DISABLE, TMR_IT_DMA_END);  // 使用单次DMA功能+中断，注意完成后关闭此中断使能，否则会一直上报中断。
        capFlag = 1;
#endif
#if COUNT_MODE
        /* 计数器计满，硬件自动清零，重新开始计数 */
        /* 用户可自行添加需要的处理 */
#endif
#if PWM_DMAMODE
        TMR_ITCfg(DISABLE, TMR_IT_DMA_END);  // 使用单次DMA功能+中断，注意完成后关闭此中断使能，否则会一直上报中断。
        PRINT("DMA end\n");
        /* DMA 结束 */
        /* 用户可自行添加需要的处理 */
#endif
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
void ENCODER_IRQHandler( void )
{
    if(ENC_GetITFlag(RB_IF_DIR_INC))
    {
        PRINT("INC\n");
        ENC_ClearITFlag(RB_IF_DIR_INC);
    }
    if(ENC_GetITFlag(RB_IF_DIR_DEC))
    {
        PRINT("DEC\n");
        ENC_ClearITFlag(RB_IF_DIR_DEC);
    }
}
