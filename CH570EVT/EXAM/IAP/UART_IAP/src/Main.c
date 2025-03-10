/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : FALSH读写例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"
#include "iap.h"

IAPDataFlashInfo_t p_image_flash;

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

#if USE_FLASHROM_FLAG
    FLASH_ROM_READ(IAP_FLAG_DATAFLASH_ADD, &p_image_flash, 4);
    if ((p_image_flash.ImageFlag != FLAG_USER_CALL_IAP))
    {
        jumpApp();
    }
#else
    //初始化引脚为上拉输入。为了减小程序大小，采用寄存器编写。
    R32_PA_PD_DRV &= ~GPIO_Pin_4;
    R32_PA_PU |= GPIO_Pin_4;
    R32_PA_DIR &= ~GPIO_Pin_4;
    //
    DelayMs(10);
    if (GPIOA_ReadPortPin(GPIO_Pin_4))
    {
        DelayMs(5);
        if (GPIOA_ReadPortPin(GPIO_Pin_4))
        {
            //启动前判断是否进入iap
            jumpApp();
        }
    }
#endif
    /* uart初始化，根据需要更改成自己的uart */
    GPIOA_SetBits( bTXD_0 );

    /* 为了节约代码空间，初始化尽量都使用寄存器 */
    R32_PA_PD_DRV &= ((~bTXD_0) & (~bRXD_0));
    /* GPIOA_ModeCfg( bTXD1, GPIO_ModeOut_PP_5mA ); */
    //R32_PA_PD_DRV &= ~bTXD1;
    R32_PA_DIR    |= bTXD_0;

    /* GPIOA_ModeCfg(bRXD1, GPIO_ModeIN_PU); */
    //R32_PA_PD_DRV &= ~bRXD1;
    R32_PA_PU     |= bRXD_0;
    R32_PA_DIR    &= ~bRXD_0;

    UART_BaudRateCfg( 115200 );
    R16_PIN_ALTERNATE_H &= ~(RB_UART_TXD);
    R8_UART_FCR = (2<<6) | RB_FCR_FIFO_EN;   // FIFO打开，触发点4字节
    R8_UART_LCR = RB_LCR_WORD_SZ;
    R8_UART_IER = RB_IER_TXD_EN;
    R8_UART_DIV = 1;

    Main_Circulation();
}
