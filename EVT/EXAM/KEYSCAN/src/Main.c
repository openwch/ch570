/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/04
 * Description        : 低功耗按键检测例程
 *                      注意:按键扫描例程默认使用了PA3与PA8，因此ISP工具下载需取消勾选RST引脚，TXD映射到了PA7
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

volatile uint8_t KeyDownflag = 1;
uint32_t LSI_CurCnt, LSI_AfterCnt = 0;

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试初始化
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_7);
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA7, UART_RX_REMAP_PA2);
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
    uint16_t i;
    uint8_t  s;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

    /* 按键扫描配置 */
    KeyScan_Cfg(ENABLE, KEYSCAN_ALL, KEYSCAN_DIV2, KEYSCAN_REP7);

    /* 按键按下中断演示 */
    KeyScan_ITCfg(ENABLE, RB_KEY_PRESSED_IE);
    PFIC_EnableIRQ(KEYSCAN_IRQn);


    /* 按键按下唤醒睡眠演示 */
#if 0
    DelayMs(10);
    KeyPress_Wake(ENABLE);
    PRINT("sleep mode sleep \n");
    DelayMs(2);
    LowPower_Sleep( RB_PWR_RAM12K | RB_PWR_EXTEND | RB_PWR_XROM);
    PRINT("wake.. \n");
#endif

    while(1)
    {
        LSI_CurCnt = RTC_GetCycleLSI();    // RTC计数差值作按键上报超时
        if(LSI_CurCnt < LSI_AfterCnt)
        {
            if((LSI_CurCnt + RTC_MAX_COUNT) - LSI_AfterCnt > 16000)
            {
                KeyDownflag = 1;
                LSI_AfterCnt = LSI_CurCnt;
            }
        }
        if(LSI_CurCnt - LSI_AfterCnt > 16000)
        {
           KeyDownflag = 1;
           LSI_AfterCnt = LSI_CurCnt;
        }
    }
}


/*********************************************************************
 * @fn      KEYSCAN_IRQHandler
 *
 * @brief   KEYSCAN_IRQHandler
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void KEYSCAN_IRQHandler( void )
{
    if(KeyScan_GetITFlag(RB_KEY_PRESSED_IF))  // 获取中断标志
    {
        KeyScan_ClearITFlag(RB_KEY_PRESSED_IF); // 清除中断标志
    }
    if( KeyDownflag )
    {
        PRINT("KeyPressed\n");
        PRINT("KEY:%x\n",KeyValue);
        KeyDownflag = 0;
    }
}


