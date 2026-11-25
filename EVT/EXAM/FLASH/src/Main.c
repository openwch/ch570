/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : FLASH读写例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

#define WRProt_Size    0x08  /* 代码包含大小，单位4KB */

/* -------------------- Flash-ROM(Code Flash) 操作地址与长度 -------------------- */
#define FLASH_ROM_TEST_ADDR     (20 * 1024)             /* Flash-ROM 操作起始地址 */
#define FLASH_ROM_TEST_LEN      (128)                   /* Flash-ROM 读写/校验长度，需为4字节整数倍 */
#define FLASH_ROM_ERASE_LEN     (4096)                  /* Flash-ROM 擦除长度 */         

/* Flash-ROM 可操作范围: [FLASH_ROM_RANGE_BASE, FLASH_ROM_RANGE_END)*/
#define FLASH_ROM_RANGE_BASE    (20 * 1024)
#define FLASH_ROM_RANGE_END     (0x70000)      

__attribute__((aligned(4))) uint8_t TestBuf[1024];


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
int main(void)
{
    uint8_t  s;
	uint32_t i;
	uint32_t rom_addr      = FLASH_ROM_TEST_ADDR;
    uint32_t rom_len       = FLASH_ROM_TEST_LEN;
    uint32_t rom_er_len    = FLASH_ROM_ERASE_LEN;

    FlashLock();

    // 关闭两线调试
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

    FlashUnlock();
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

    /* 第一次操作前，判断 Flash-ROM 操作地址范围是否超出可操作范围 */
    if((rom_addr < FLASH_ROM_RANGE_BASE) ||
       ((rom_addr + rom_len) > FLASH_ROM_RANGE_END) ||
       ((rom_addr + rom_er_len) > FLASH_ROM_RANGE_END))
    {
        PRINT("Flash-ROM operation out of range!\n");
        while(1);
    }

    PRINT("FLASH_READ 20k...\n");
    FLASH_ROM_READ(rom_addr, TestBuf, rom_len);
    for(i = 0; i < rom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    s = FLASH_ROM_ERASE(rom_addr, rom_er_len);
    PRINT("FLASH_ROM_ERASE=%02x\n", s);
    PRINT("FLASH_READ 20k...\n");
    FLASH_ROM_READ(rom_addr, TestBuf, rom_len);
    for(i = 0; i < rom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    for(i = 0; i < rom_len; i++)
    {
        TestBuf[i] = 0x70 + i;
    }
    s = FLASH_ROM_WRITE(rom_addr, TestBuf, rom_len);//Codeflash写入必须是4字节的整数倍
    PRINT("FlashWrite=%02x\n", s);
    s = FLASH_ROM_VERIFY(rom_addr, TestBuf, rom_len);
    PRINT("FlashVerify=%02x\n", s);

    PRINT("FLASH_READ...\n");
    FLASH_ROM_READ(rom_addr, TestBuf, rom_len);
    for(i = 0; i < rom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

#endif

#if 0 /* 修改用户配置值 */
    s = UserOptionByteConfig(ENABLE, ENABLE, WRProt_Size); 
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

    FlashLock();
    while(1);
}
