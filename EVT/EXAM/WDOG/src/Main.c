/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/09/14
 * Description        : WDOG例程：系统看门狗，1.演示溢出中断喂狗  2.演示停止喂狗后触发系统复位。
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

/* 计数周期131072/100MHz≈1.31ms */
#define WWDG_FEED_MS 100

/* 喂狗次数：喂够后停止喂狗，演示看门狗复位 */
#define WWDG_FEED_TIMES 20

/* 溢出中断次数：演示溢出后进中断喂狗 */
#define WWDG_INT_TIMES 3

volatile uint8_t wwdg_int_cnt = 0;

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
 * @fn      PrintResetCause
 *
 * @brief   打印最近一次复位原因
 *
 * @return  none
 */
void PrintResetCause(void)
{
    switch(SYS_GetLastResetSta())
    {
        case RST_STATUS_SW:
            PRINT("  reset cause: software reset SR\n");
            break;
        case RST_STATUS_RPOR:
            PRINT("  reset cause: power-on reset RPOR\n");
            break;
        case RST_STATUS_WTR:
            PRINT("  reset cause: watchdog timeout reset WTR\n");
            break;
        case RST_STATUS_MR:
            PRINT("  reset cause: external pin reset MR\n");
            break;
        case RST_STATUS_LRM1:
            PRINT("  reset cause: wake-up reset by watchdog LRM1\n");
            break;
        case RST_STATUS_GPWSM:
            PRINT("  reset cause: power-down wake-up reset GPWSM\n");
            break;
        default:
            PRINT("  reset cause: code 0x%x\n", SYS_GetLastResetSta());
            break;
    }
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
    uint8_t feeds = 0;

    FlashLock();
    // 关闭两线调试
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Time:%s; Start @ChipID=%02X\n", __TIMESTAMP__, R8_CHIP_ID);
    PrintResetCause();

    /* 1. 溢出中断喂狗演示：溢出后进中断重装载计数值 */
    PRINT("\nWWDG interrupt-only mode, waiting %d overflows ...\n", WWDG_INT_TIMES);
    WWDG_ResetCfg(DISABLE);
    WWDG_ITCfg(ENABLE);
    PFIC_EnableIRQ(WDOG_BAT_IRQn);
    WWDG_SetCounter(0);                // 重新装载计数值
    while(wwdg_int_cnt < WWDG_INT_TIMES);
    WWDG_ITCfg(DISABLE);
    PFIC_DisableIRQ(WDOG_BAT_IRQn);

    /* 2. 使能看门狗超时复位，主循环喂狗 */
    PRINT("\nSystem watchdog(WWDG) reset mode enabled, feeding %d times ...\n", WWDG_FEED_TIMES);
    WWDG_ResetCfg(ENABLE);             // 使能
    WWDG_SetCounter(0);                // 重新装载计数值

    while(feeds < WWDG_FEED_TIMES)
    {
        DelayMs(WWDG_FEED_MS);
        WWDG_SetCounter(0);            // 喂狗：重新装载计数值
        feeds++;
        PRINT("  feed %d ok\n", feeds);
    }

    /* 3. 停止喂狗，等待看门狗超时复位（8位计数满约0.33s） */
    PRINT("\nStop feeding, watchdog will reset system ...\n");
    while(1);
}

/*********************************************************************
 * @fn      WDOG_BAT_IRQHandler
 *
 * @brief   看门狗溢出中断函数，仅中断模式下重装载计数值并统计次数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void WDOG_BAT_IRQHandler(void)
{
    wwdg_int_cnt++;
    WWDG_SetCounter(0);   // 重新装载计数值
    WWDG_ClearFlag();     // 清除看门狗中断标志
}
