/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/21
 * Description        : 比较器功能演示
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

volatile uint8_t cmp_ItFlag = 0;
const CMPOutSelTypeDef ITMode = cmp_out_sel_rise;

#define OUT_TO_TIM_CAP  1
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
    HSECFG_Capacitance( HSECap_18p );
    SetSysClock( CLK_SOURCE_HSE_PLL_100MHz );

    /* 配置串口1：先配置IO口模式，再配置串口 */
    GPIOA_SetBits( bTXD_0 );
    GPIOA_ModeCfg( bTXD_0, GPIO_ModeOut_PP_5mA );          // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();
    PRINT( "Start @ChipID=%02X\n", R8_CHIP_ID );

    GPIOA_ModeCfg( GPIO_Pin_7, GPIO_ModeIN_Floating );   // CMP_P0 = PA7 配置为浮空输入
    CMP_Init( cmp_sw_3, cmp_nref_level_800 );  //内部参考电压档位800mv

#if OUT_TO_TIM_CAP //比较器输出到定时器捕获
    CMP_OutToTIMCAPCfg( ENABLE );
    TMR_CAPTimeoutCfg( 0x02000000 ); // 设置捕捉超时时间
    R8_TMR_INT_FLAG |= RB_TMR_IF_DATA_ACT;
    TMR_CapInit( RiseEdge_To_RiseEdge );
#endif

    CMP_Enable();

    CMP_INTCfg( ITMode, ENABLE );
    CMP_ClearITStatus();
    PFIC_EnableIRQ( CMP_IRQn );

    while (1)
    {
        if (cmp_ItFlag)
        {
            DelayUs (1);
            if (CMP_ReadAPROut())
            {
                PRINT ("CMP_1\n");
            }
            else
            {
                PRINT ("CMP_0\n");
            }
            cmp_ItFlag = 0;
            CMP_INTCfg (ITMode, ENABLE);
        }
#if OUT_TO_TIM_CAP
        if (R8_TMR_INT_FLAG & RB_TMR_IF_DATA_ACT)
        {
            TMR_ClearITFlag( TMR_IT_DATA_ACT );
            PRINT ("c:%ld\r\n", R32_TMR_FIFO);
        }
#endif
    }
}

/*********************************************************************
 * @fn      CMP_IRQHandler
 *
 * @brief   CMP中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void CMP_IRQHandler( void )
{
    if( CMP_GetITStatus() )
    {
        CMP_ClearITStatus();
        CMP_INTCfg( ITMode, DISABLE );
        cmp_ItFlag = 1;
    }
}
