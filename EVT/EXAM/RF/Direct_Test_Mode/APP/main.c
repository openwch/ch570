/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/04/28
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* ͷ�ļ����� */
#include <rf_dtm.h>

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
/*********************************************************************
 * @fn      main
 *
 * @brief   ������
 *
 * @return  none
 */
int main(void)
{
    HSECFG_Capacitance(HSECap_20p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);
#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif

    GPIOA_SetBits(bTXD_0);
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA); // TXD-�������������ע������IO������ߵ�ƽ
    GPIOA_ModeCfg(bRXD_0, GPIO_ModeIN_PU);
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);

    UART_DefInit();
    UART_ByteTrigCfg( UART_4BYTE_TRIG );
    UART_INTCfg( ENABLE,  RB_IER_RECV_RDY );;
    PFIC_EnableIRQ( UART_IRQn );
    PFIC_EnableIRQ( BLEL_IRQn );
    PFIC_EnableIRQ( BLEB_IRQn );
    RFRole_Init();
    DtmProcess();
}

/******************************** endfile @ main ******************************/
