/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/06/24
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include <RF_extend.h>

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

/*******************************************************************************
 * @fn      process
 *
 * @brief   run in ram
 *
 * @return  None.
 */
__HIGH_CODE
__attribute__((noinline))
void process(void)
{
    while(1)
	{
        if(gPrintCnt >= RF_DEVICE_PERIOD)
		{
            gPrintCnt = 0;
#if( WAIT_ACK )
       	 	/* 打印上一窗口统计：发包数/ACK数/平均RSSI */
	        PRINT("r %d %d %d\n", (int)gTxCount, (int)gRxCount, (int)(gRxCount ? (Rssi_sum/(int)gRxCount) : 0));
	        gTxCount = 0;
	        gRxCount = 0;
	        Rssi_sum = 0;
#else
			PRINT("Recv: %d\n", gRxCount);
			gRxCount = 0;
#endif
        }
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
    FlashLock();
    // 关闭两线调试
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);
#ifdef DEBUG
    GPIOA_SetBits(bTXD_0);
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();
#endif
    PRINT("start.\n");
    PRINT("%s\n", VER_RF_LIB);
    RFRole_Init();
    process();
}

/******************************** endfile @ main ******************************/
