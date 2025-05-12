/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "rf.h"
#include "rf_uart_rx.h"
#include "usb_uart.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((used))
__HIGH_CODE
void process_main( void )
{
    while(1)
    {
        USB_StatusQuery( );
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
    HSECFG_Capacitance( HSECap_18p );
    SetSysClock( CLK_SOURCE_HSE_PLL_100MHz );
#ifdef DEBUG
    GPIOA_SetBits( bTXD_0 );
    GPIOA_ModeCfg( bTXD_0, GPIO_ModeOut_PP_5mA ); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap( ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2 );
    UART_DefInit( );
#endif
    PRINT("start.\n");
    PRINT("%s\n", VER_RF_LIB);
    USB_Init( );
    RFRole_Init( );
    RF_UartRxInit( );
    process_main( );
}

/******************************** endfile @ main ******************************/
