/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : SPI演示 Master/Slave 模式数据收发
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

#define  SPI_NORMAL_MODE      1    // SPI 普通3线模式
#define  SPI_2WIRE_MODE       0    // SPI 2线模式 (主机的MOSI，从机的MISO)

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

int main()
{
    uint8_t i;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if SPI_NORMAL_MODE  // SPI普通3线模式
#if 1
    /* 主机模式 */
    PRINT("1.spi master mode send data ...\n");
    DelayMs(100);

        /* SPI */
        GPIOA_SetBits(GPIO_Pin_4);
        GPIOA_ModeCfg(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
        SPI_MasterDefInit();

        // 单字节发送
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterSendByte(0x55);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(1);
        GPIOA_ResetBits(GPIO_Pin_4);
        i = SPI_MasterRecvByte();
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(2);
        PRINT("receive %x\n",i);

        // FIFO 连续发送
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterTrans(spiBuff, 8);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(2);
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterRecv(spiBuffrev, 8);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(2);
        PRINT("FIFO recv ");
        for(i = 0; i < 8; i++)
        {
            PRINT(" %x", spiBuffrev[i]);
        }
        PRINT("\n");

        // DMA 连续发送
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterDMATrans(spiBuff, 12);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(2);
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterDMARecv(spiBuffrev, 12);
        GPIOA_SetBits(GPIO_Pin_4);
        PRINT("DMA recv ");
        for(i = 0; i < 12; i++)
        {
            PRINT(" %x", spiBuffrev[i]);
        }
        PRINT("\n");
    PRINT("END ...\n");
    while(1);
#else
    /* 设备模式 */
    PRINT("1.spi0 mul slave mode \n");
    GPIOA_ModeCfg(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, GPIO_ModeIN_PU);
    SPI_SlaveInit();

    i = SPI_SlaveRecvByte();
    SPI_SlaveSendByte(0xAA);
    PRINT("receive %x\n",i);

    SPI_SlaveRecv(spiBuffrev, 8);
    SPI_SlaveTrans(spiBuffrev, 8);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");

    SPI_SlaveDMARecv(spiBuffrev, 12);
    SPI_SlaveDMATrans(spiBuffrev, 12);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiBuffrev[i]);
    }
    PRINT("\n");

    PRINT("END ...\n");

    while(1);
#endif
#endif


#if SPI_2WIRE_MODE    // SPI 2线模式

#define  MASTER_SEND_MODE        1   // 主机发送模式，从机接收模式
#define  MASTER_RECEIVE_MODE     0   // 主机接收模式，从机发送模式

#if 0
#if MASTER_SEND_MODE
    /* 主机模式 */
    PRINT("1.spi 2wire master mode send data ...\n");
    DelayMs(100);

        /* SPI */
        GPIOA_SetBits(GPIO_Pin_4);
        GPIOA_ModeCfg(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7, GPIO_ModeOut_PP_5mA);

        SPI_2WIRE_MasterOutputInit();
        // 单字节发送
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterSendByte(0x55);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(10);

        // FIFO 连续发送
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterTrans(spiBuff, 8);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(10);

        // DMA 连续发送
        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterDMATrans(spiBuff, 12);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(10);
        PRINT("END ...\n");
        while(1);
#endif
#if MASTER_RECEIVE_MODE
        GPIOA_SetBits(GPIO_Pin_4);
        GPIOA_ModeCfg(GPIO_Pin_4 | GPIO_Pin_5, GPIO_ModeOut_PP_5mA);
        GPIOA_ModeCfg( GPIO_Pin_7, GPIO_ModeIN_PU);
        SPI_2WIRE_MasterReceiveInit();
        GPIOA_ResetBits(GPIO_Pin_4);
        i = SPI_MasterRecvByte();
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(2);
        PRINT("receive %x\n",i);

        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterRecv(spiBuffrev, 8);
        GPIOA_SetBits(GPIO_Pin_4);
        DelayMs(2);
        PRINT("FIFO recv ");
        for(i = 0; i < 8; i++)
        {
            PRINT(" %x", spiBuffrev[i]);
        }
        PRINT("\n");

        GPIOA_ResetBits(GPIO_Pin_4);
        SPI_MasterDMARecv(spiBuffrev, 12);
        GPIOA_SetBits(GPIO_Pin_4);
        PRINT("DMA recv ");
        for(i = 0; i < 12; i++)
        {
            PRINT(" %x", spiBuffrev[i]);
        }
        PRINT("\n");
        PRINT("END ...\n");
        while(1);
#endif
#else
    /* 设备模式 */
    PRINT("1.spi mul slave mode \n");
    GPIOA_ModeCfg(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6, GPIO_ModeIN_PU);

#if MASTER_SEND_MODE    // 主机模式
    SPI_2WIRE_SlaveInputInit();

    i = SPI_SlaveRecvByte();
    DelayMs(2);
    PRINT("receive %x\n",i);

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
#endif
#if MASTER_RECEIVE_MODE
    GPIOA_ModeCfg(GPIO_Pin_6, GPIO_ModeOut_PP_5mA);
    SPI_2WIRE_SlaveOutputInit();
    SPI_SlaveSendByte(0xAA);
    SPI_SlaveTrans(spiBuff, 8);
    SPI_SlaveDMATrans(spiBuff, 12);
    PRINT("END ...\n");
#endif
    while(1);
#endif
#endif
}

