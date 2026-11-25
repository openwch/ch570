/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_basic.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/15
 * Description        : 2.4G库基本模式接收测试例程(RX端)
 *
 *                      1. 获取信号强度
 *                      可读取当前数据包的RSSI值，按接收窗统计平均值
 *
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "rf_basic.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
rfipRx_t gRxParam;

__attribute__((__aligned__(4))) uint8_t RxBuf[264]; // 接收DMA buf不能小于264字节

#define  TX_POWER         LL_TX_PWR_7_DBM   // 发射功率

#define  TEST_DATA_LEN    4               // 数据长度
#define  TEST_FREQUENCY   16              // 通信频点

rfipTx_t gTxParam;

__attribute__((__aligned__(4))) uint8_t TxBuf[64];

volatile uint32_t gTxCount;
volatile uint32_t gRxCount;
volatile int32_t Rssi_sum = 0;
volatile uint32_t gPrintCnt = 0;

#if( WAIT_ACK )
/******************************** 发送相关函数 ********************************/
/* 以下发送相关代码仅用于ACK应答 */

/*******************************************************************************
 * @brief   配置发送的频点
 *
 * @param   f - 需要配置的频点
 *
 * @return  None.
 */
__HIGH_CODE
void rf_tx_set_frequency( uint32_t f )
{
    gTxParam.frequency = f;
}

/*******************************************************************************
 * @brief   配置发送的地址
 *
 * @param   sync_word - 需要配置的接入地址
 *
 * @return  None.
 */
__HIGH_CODE
void rf_tx_set_sync_word( uint32_t sync_word )
{
    gTxParam.accessAddress = sync_word;
}

/*******************************************************************************
 * @brief   rf发送数据子程序
 *
 * @param   pBuf - 发送的DMA地址
 *
 * @return  None.
 */
__HIGH_CODE
void rf_tx_start( uint8_t *pBuf )
{
    // 配置发送的频点
    gTxParam.frequency = TEST_FREQUENCY;
    // 发送的DMA地址
    gTxParam.txDMA = (uint32_t)pBuf;
    gTxParam.waitTime = 40*2;
    RFIP_StartTx( &gTxParam );
}

/*******************************************************************************
 * @brief   ACK应答, 接收到数据后重新以发送方式回复ACK
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
static void RF_SendAck( void )
{
    TxBuf[0] = 0x0d;                  // 与官方例程一致的ACK包内容
    TxBuf[1] = 0;
    rf_tx_start( TxBuf );
}
#endif

/******************************** 接收相关函数 ********************************/
/**
 * @brief   配置接收的地址
 *
 * @param   sync_word - 需要配置的接入地址
 *
 * @return  None.
 */
__HIGH_CODE
void rf_rx_set_sync_word( uint32_t sync_word )
{
    gRxParam.accessAddress = sync_word;
}

/*******************************************************************************
 * @fn      rf_rx_set_frequency
 *
 * @param   f - 需要配置的频点
 *
 * @return  None.
 */
__HIGH_CODE
void rf_rx_set_frequency( uint32_t f )
{
    gRxParam.frequency = f;
}

/*******************************************************************************
 * @fn      rf_rx_start
 *
 * @brief   rf接收数据子程序
 *
 * @return  None.
 */
__HIGH_CODE
void rf_rx_start( void )
{
    // 配置接收的频点
    gRxParam.frequency = TEST_FREQUENCY;
    // 配置接收的超时时间，0则无超时
    gRxParam.timeOut = 0;
    RFIP_SetRx( &gRxParam );
}

/*******************************************************************************
 * @fn      rf_rx_process_data
 *
 * @brief   rf接收数据处理
 *
 * @return  None.
 */
__HIGH_CODE
void rf_rx_process_data( void )
{
    gRxCount ++;
    {
        uint8_t *pData = (uint8_t *)gRxParam.rxDMA;
        Rssi_sum += (int8_t)pData[TEST_DATA_LEN+RSSI_OFFSET];
    }
}

/*******************************************************************************
 * @fn      LLE_IRQHandler
 *
 * @brief   LLE_IRQHandler
 *
 * @return  None.
 */
__INTERRUPT
__HIGH_CODE
void LLE_IRQHandler( void )
{
    LLE_LibIRQHandler( );
}

/*******************************************************************************
 * @fn      BB_IRQHandler
 *
 * @brief   BB_IRQHandler
 *
 * @return  None.
 */
__INTERRUPT
__HIGH_CODE
void BB_IRQHandler( void )
{
    BB_LibIRQHandler( );
}

/*******************************************************************************
 * @fn      RF_ProcessCallBack
 *
 * @brief   rf中断处理程序
 *
 * @param   sta - 中断状态.
 *          id - 保留
 *
 * @return  None.
 */
__HIGH_CODE
void RF_ProcessCallBack( rfRole_States_t sta,uint8_t id )
{
    if( sta&RF_STATE_RX )
    {
        rf_rx_process_data();
#if( WAIT_ACK )
        RF_SendAck( );            // 应答ACK，发送完成后在TX_FINISH中重新进入接收
#else
        rf_rx_start( );
#endif
    }
    if( sta&RF_STATE_RX_CRCERR )
    {
        rf_rx_start( );
    }
    if( sta&RF_STATE_TX_FINISH )
    {
#if( WAIT_ACK )
        rf_rx_start( );           // ACK发送完成，重新进入接收
#endif
        gTxCount ++;
    }
    if( sta&RF_STATE_TIMEOUT )
    {
        /* rx地址匹配但接收超时，重新进入接收 */
        rf_rx_start( );
    }
}

/*******************************************************************************
 * @fn      TMR_IRQHandler
 *
 * @brief   TMR中断函数，500ms统计一次收包数和平均RSSI
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR_IRQHandler(void) // TMR 定时中断
{
    if(TMR_GetITFlag(TMR_IT_CYC_END))
    {
        TMR_ClearITFlag(TMR_IT_CYC_END); // 清除中断标志
        gPrintCnt++;
    }
}

/*******************************************************************************
 * @fn      RFRole_Init
 *
 * @brief   RF应用层初始化
 *
 * @param   None.
 *
 * @return  None.
 */
void RFRole_Init(void)
{
    sys_safe_access_enable( );
    R32_MISC_CTRL = (R32_MISC_CTRL&(~(0x3f<<24)))|(0xe<<24);
    sys_safe_access_disable( );
    PKT_DET_CFG4(0x3f);
    {
        rfRoleConfig_t conf ={0};
        conf.rfProcessCB = RF_ProcessCallBack;
        conf.processMask = RF_STATE_RX|RF_STATE_RX_CRCERR|RF_STATE_TX_FINISH|RF_STATE_TIMEOUT;
        RFRole_BasicInit( &conf );
    }
    TPROPERTIES_CFG Properties;
    {
        Properties.cfgVal = PHY_MODE_PHY_2M;        // BLE 2M模式
        PRINT("cfgVal=%x\n",Properties.cfgVal);
    }

    // RX相关参数，全局变量
    {
        gRxParam.accessAddress = ACCESS_ADR;        // Rx接入地址
        gRxParam.crcInit = CRC_INIT;
        gRxParam.crcPoly = CRC_POLY;
        gRxParam.properties = Properties.cfgVal;
        gRxParam.rxDMA = (uint32_t)RxBuf;
        gRxParam.rxMaxLen = TEST_DATA_LEN;
    }

#if( WAIT_ACK )
    // TX相关参数，全局变量(用于ACK应答)
    {
        gTxParam.accessAddress = ACCESS_ADR;        // Tx接入地址
        gTxParam.crcInit = CRC_INIT;
        gTxParam.crcPoly = CRC_POLY;
        gTxParam.properties = Properties.cfgVal;
        gTxParam.waitTime = 80*2;
        gTxParam.txPowerVal = TX_POWER;             // Tx发射功率
        gTxParam.txLen = TEST_DATA_LEN;

        if(gTxParam.txPowerVal > LL_TX_PWR_6_DBM)
        {
            sys_safe_access_enable( );
            R32_MISC_CTRL |= (7<<6);
            sys_safe_access_disable( );
        }
        else
        {
            sys_safe_access_enable( );
            R32_MISC_CTRL = ((R32_MISC_CTRL &~(7<<6)) | (5<<6));
            sys_safe_access_disable( );
        }
    }
#endif

    PFIC_EnableIRQ( BLEB_IRQn );
    PFIC_EnableIRQ( BLEL_IRQn );

    gRxCount = 0;
    Rssi_sum = 0;
    /* 打印演示配置信息 */
    PRINT("\n==============================================\n");
    PRINT("  2.4G RF Basic RX\n");
#if( WAIT_ACK )
    PRINT("  mode     : RX + ACK reply\n");
#else
    PRINT("  mode     : RX only\n");
#endif
    PRINT("  freq     : ch %d\n", TEST_FREQUENCY);
    PRINT("  phy      : %s\n", PHY_MODE_PHY_2M ? "2M" : "1M");
    PRINT("  len      : %d bytes\n", TEST_DATA_LEN);
    PRINT("==============================================\n");
    rf_rx_start();
    TMR_TimerInit( GetSysClock() / RF_DEVICE_PERIOD );
    TMR_ITCfg(ENABLE, TMR_IT_CYC_END);// 开启中断
    PFIC_EnableIRQ(TMR_IRQn);
}

/******************************** endfile @rf ******************************/
