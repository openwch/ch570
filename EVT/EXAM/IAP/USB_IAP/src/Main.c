/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/03/15
 * Description        : USB IAP例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"
#include "iap.h"

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   IAP主循环,程序放ram中运行，提升速度.
 *
 * @param   None.
 *
 * @return  None.
 */
__attribute__((section(".highcode")))
void Main_Circulation()
{
    uint16_t j = 0;
    while (1)
    {
        j++;
        if (j > 5)//100us处理一次数据
        {
            j = 0;
            USB_DevTransProcess();//采用查询方式进行usb操作，不使用中断。
        }
        DelayUs(20);
        g_tcnt++;
        if (g_tcnt > 3000000)
        {
            //1分钟没有操作，进入app
            R8_USB_CTRL = RB_UC_RESET_SIE;
            R16_PIN_ALTERNATE &= ~(RB_UDP_PU_EN | RB_PIN_USB_EN);
            DelayMs(10);
            jumpApp();
        }
    }
}

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
            //启动前判断是否进入iap，没有按键按下
            jumpApp();
        }
    }
#endif
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;//使用USB要关掉两线调试
    /* USB初始化 */
    R8_USB_CTRL = 0x00; // 先设定模式,取消 RB_UC_CLR_ALL

    R8_UEP4_1_MOD = RB_UEP4_RX_EN | RB_UEP4_TX_EN | RB_UEP1_RX_EN | RB_UEP1_TX_EN; // 端点4 OUT+IN,端点1 OUT+IN
    R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN; // 端点2 OUT+IN,端点3 OUT+IN

    R16_UEP0_DMA = (uint16_t)(uint32_t)EP0_Databuf;
    R16_UEP1_DMA = (uint16_t)(uint32_t)EP1_Databuf;
    R16_UEP2_DMA = (uint16_t)(uint32_t)EP2_Databuf;
    R16_UEP3_DMA = (uint16_t)(uint32_t)EP3_Databuf;

    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

    R8_USB_DEV_AD = 0x00;
    R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;  // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    R16_PIN_ALTERNATE |= RB_PIN_USB_EN | RB_UDP_PU_EN;              // 防止USB端口浮空及上拉电阻
    R8_USB_INT_FG = 0xFF;                                           // 清中断标志
    R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;                    // 允许USB端口
    R8_USB_INT_EN = 0;  //禁止usb中断，采用查询方式

    /* 进入highcode主循环 */
    Main_Circulation();
}
