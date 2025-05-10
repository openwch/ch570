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

#define WRProt_Size    0x08  /* 代码包含大小，单位4KB */

uint8_t TestBuf[1024];

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
    uint16_t i;
    uint8_t  s;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if 1 // 获取唯一ID，MAC地址，读写Flash-ROM

    PRINT("GET_UNIQUE_ID...\n");
    GET_UNIQUE_ID(TestBuf);
    for(i = 0; i < 8; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    PRINT("FlashMACADDRESS...\n");
    GetMACAddress(TestBuf);
    for(i = 0; i < 6; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    PRINT("FLASH_READ 20k...\n");
    FLASH_ROM_READ(20 * 1024, TestBuf, 128);
    for(i = 0; i < 128; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    s = FLASH_ROM_ERASE(20 * 1024, 4096);
    PRINT("FLASH_ROM_ERASE=%02x\n", s);
    PRINT("FLASH_READ 20k...\n");
    FLASH_ROM_READ(20 * 1024, TestBuf, 128);
    for(i = 0; i < 128; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    for(i = 0; i < 128; i++)
    {
        TestBuf[i] = 0x70 + i;
    }
    s = FLASH_ROM_WRITE(20 * 1024, TestBuf, 128);
    PRINT("FlashWrite=%02x\n", s);
    s = FLASH_ROM_VERIFY(20 * 1024, TestBuf, 128);
    PRINT("FlashVerify=%02x\n", s);

    PRINT("FLASH_READ...\n");
    FLASH_ROM_READ(20 * 1024, TestBuf, 128);
    for(i = 0; i < 128; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

#endif

#if 0 /* 修改用户配置值 */
    s = UserOptionByteConfig(ENABLE, ENABLE, ENABLE, WRProt_Size);
    if(s)
        PRINT("ERR\n");
    else
    {
        PRINT("suc\n");
        mDelaymS(10);
        UserOptionByte_Active();
    }
#endif

#if 0 /* 关闭两线调试接口 */
    s = UserOptionByteClose_SWD();
    if(s)
        PRINT("ERR\n");
    else
    {
        PRINT("suc\n");
        mDelaymS(10);
        UserOptionByte_Active();
    }
#endif

    while(1);
}
