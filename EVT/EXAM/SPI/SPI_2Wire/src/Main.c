/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/15
 * Description        : SPI例程：两线模式(8bit)数据收发，主机发送/接收与从机接收/发送
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

/* 主从模式定义 */
#define SPI_MASTER_MODE       1
#define SPI_SLAVE_MODE        0

/* 主从数据方向定义 */
#define MASTER_SEND_MODE      1 // 主机发送模式，从机接收模式
#define MASTER_RECEIVE_MODE   0 // 主机接收模式，从机发送模式

/* 主从模式选择 */
#define SPI_MODE            SPI_MASTER_MODE          // 切换模式
#define SPI_DIR             MASTER_SEND_MODE         // 切换方向

/* ---- 硬件引脚配置：PA4-SCS PA5-SCK PA6-MISO PA7-MOSI ---- */
/* 注意：572两线模式主机数据线为MOSI、从机数据线为MISO */
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
    PRINT("1.spi 2wire master mode, data on MOSI(PA7) ...\n");
    DelayMs(100);

#if SPI_DIR == MASTER_SEND_MODE
    /* 主机发送模式：数据由MOSI(PA7)输出，接从机MISO(PA6) */
    GPIOA_SetBits(SPI_SCS_PIN);
    GPIOA_ModeCfg(SPI_SCS_PIN | SPI_SCK_PIN | SPI_MOSI_PIN, GPIO_ModeOut_PP_5mA);

    SPI_2WIRE_MasterOutputInit();
    // 单字节发送
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterSendByte(0x55);
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(10);

    // FIFO 连续发送
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterTrans(spiBuff, 8);
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(10);

    // DMA 连续发送
    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterDMATrans(spiBuff, 12);
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(10);
    PRINT("2wire master send END ...\n");
#else
    /* 主机接收模式：数据由MOSI(PA7)输入，接从机MISO(PA6) */
    GPIOA_SetBits(SPI_SCS_PIN);
    GPIOA_ModeCfg(SPI_SCS_PIN | SPI_SCK_PIN, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(SPI_MOSI_PIN, GPIO_ModeIN_PU);
    SPI_2WIRE_MasterReceiveInit();
    GPIOA_ResetBits(SPI_SCS_PIN);
    i = SPI_MasterRecvByte();
    GPIOA_SetBits(SPI_SCS_PIN);
    DelayMs(2);
    PRINT("receive %x\n", i);

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

    GPIOA_ResetBits(SPI_SCS_PIN);
    SPI_MasterDMARecv(spiBuffrev, 12);
    GPIOA_SetBits(SPI_SCS_PIN);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");
    PRINT("2wire master receive END ...\n");
#endif
#else
    /* ---- 从机模式 ---- */
    PRINT("1.spi 2wire slave mode, data on MISO(PA6) \n");
    GPIOA_ModeCfg(SPI_SCS_PIN | SPI_SCK_PIN | SPI_MISO_PIN, GPIO_ModeIN_PU);

#if SPI_DIR == MASTER_SEND_MODE    // 主机发，从机收模式：数据由MISO(PA6)输入，接主机MOSI(PA7)
    SPI_2WIRE_SlaveInputInit();

    i = SPI_SlaveRecvByte();
    DelayMs(2);
    PRINT("receive %x\n", i);

    SPI_SlaveRecv(spiBuffrev, 8);
    DelayMs(2);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");

    SPI_SlaveDMARecv(spiBuffrev, 12);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");
    PRINT("END ...\n");
#else                            // 主机收，从机发模式：数据由MISO(PA6)输出，接主机MOSI(PA7)
    GPIOA_ModeCfg(SPI_SCS_PIN | SPI_SCK_PIN, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(SPI_MISO_PIN, GPIO_ModeOut_PP_5mA);
    SPI_2WIRE_SlaveOutputInit();
    SPI_SlaveSendByte(0xAA);
    SPI_SlaveTrans(spiBuff, 8);
    SPI_SlaveDMATrans(spiBuff, 12);
    PRINT("END ...\n");
#endif
#endif

    while(1);
}
