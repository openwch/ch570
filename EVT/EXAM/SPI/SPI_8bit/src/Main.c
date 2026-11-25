/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/15
 * Description        : SPI例程：普通模式数据收发，单字节/FIFO多字节/DMA方式，宏切换主机/从机
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

/* 主从模式定义 */
#define SPI_MASTER_MODE     1
#define SPI_SLAVE_MODE      0

/* 主从模式选择 */
#define SPI_MODE            SPI_MASTER_MODE      // 切换模式

/* ---- 硬件引脚配置：PA4-SCS PA5-SCK PA6-MISO PA7-MOSI ---- */
#define SPI_SCS_PIN      GPIO_Pin_4
#define SPI_SCK_PIN      GPIO_Pin_5
#define SPI_MISO_PIN     GPIO_Pin_6
#define SPI_MOSI_PIN     GPIO_Pin_7

__attribute__((aligned(4))) uint8_t spiBuff[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
__attribute__((aligned(4))) uint8_t spiBuffrev[16];

void DebugInit(void)
{
    GPIOA_SetBits(bTXD_0);
    GPIOA_ModeCfg(bRXD_0, GPIO_ModeIN_PU);      // RXD-配置上拉输入
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();
}

int main(void)
{
    uint8_t i;

    FlashLock();
    // 关闭两线调试
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if(SPI_MODE == SPI_MASTER_MODE)
    /* ---- 主机模式 ---- */
    PRINT("1.spi master mode send data ...\n");
    DelayMs(100);

    GPIOA_SetBits(SPI_SCS_PIN);
    GPIOA_ModeCfg(SPI_SCS_PIN | SPI_SCK_PIN | SPI_MOSI_PIN, GPIO_ModeOut_PP_5mA);
    SPI_MasterDefInit();

    // 单字节发送
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterSendByte(0x55);
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(1);
    GPIOA_ResetBits(SPI_SCS_PIN);
    i = SPI_MasterRecvByte();
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(2);
    PRINT("receive %x\n", i);

    // FIFO 连续发送
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterTrans(spiBuff, 8);
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(2);
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterRecv(spiBuffrev, 8);
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(2);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");

    // DMA 连续发送
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterDMATrans(spiBuff, 12);
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(2);
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterDMARecv(spiBuffrev, 12);
    GPIOA_SetBits(SPI_SCS_PIN);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");
#else
    /* ---- 从机模式 ---- */
    PRINT("1.spi slave mode \n");
    GPIOA_ModeCfg(SPI_SCS_PIN | SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN, GPIO_ModeIN_PU);
    SPI_SlaveInit();

    // 单字节接收与回发
    i = SPI_SlaveRecvByte();
    SPI_SlaveSendByte(0xAA);
    PRINT("receive %x\n", i);

    // FIFO 连续接收与回发
    SPI_SlaveRecv(spiBuffrev, 8);
    SPI_SlaveTrans(spiBuffrev, 8);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");

    // DMA 连续接收与回发
    SPI_SlaveDMARecv(spiBuffrev, 12);
    SPI_SlaveDMATrans(spiBuffrev, 12);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");
#endif

    PRINT("END ...\n");
    while(1);
}
