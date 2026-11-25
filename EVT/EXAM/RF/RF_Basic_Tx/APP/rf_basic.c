/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_basic.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/15
 * Description        : 2.4G库基本模式发送测试例程(TX端)
 *
 *                      1. 功率设置
 *                      TX_POWER，支持-25dBm ~ +7dBm 动态调整
 *
 *                      2.发送状态切换稳定时间
 *                      waitTime，如果需要切换通道发送，稳定时间不低于80us
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
rfipTx_t gTxParam;

__attribute__((__aligned__(4))) uint8_t TxBuf[64];

#define  TEST_DATA_LEN    4                 // 数据长度
#define  TEST_FREQUENCY   16                // 通信频点
#define  TX_POWER         LL_TX_PWR_7_DBM   // 发射功率

rfipRx_t gRxParam;

__attribute__((__aligned__(4))) uint8_t RxBuf[264]; // 接收DMA buf不能小于264字节(用于接收ACK)

uint32_t volatile gTxCount = 0;
uint32_t volatile gRxCount = 0;
int32_t  volatile Rssi_sum = 0;

int8_t gRssi=0;
volatile uint32_t gPrintCnt = 0;

/******************************** 发送相关函数 ********************************/
/**
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
    gTxParam.waitTime = 40*2; // 如果需要切换通道发送，稳定时间不低于80us
    RFIP_StartTx( &gTxParam );
}

#if( WAIT_ACK )
/******************************** ACK接收相关函数 ********************************/
/*******************************************************************************
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
 * @fn      rf_rx_start
 *
 * @brief   rf接收数据子程序(用于发送后打开ACK接收窗口等待应答)
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
 * @brief   打开ACK接收窗口等待接收端应答(应答接收封装)
 *
 * @param   None.
 *
 * @return  None.
 */
static void RF_TxWaitAckStart( void )
{
    rf_rx_start( );
}

/*******************************************************************************
 * @fn      rf_rx_process_data
 *
 * @brief   rf接收数据处理(接收ACK包统计应答数与信号强度)
 *
 * @return  None.
 */
__HIGH_CODE
void rf_rx_process_data( void )
{
    uint8_t *pData = (uint8_t *)gRxParam.rxDMA;
    uint8_t AckLen = pData[1];
    gRssi = (int8_t)pData[AckLen+RSSI_OFFSET];
    gRxCount ++;
    Rssi_sum += gRssi;
}
#endif

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
#if( WAIT_ACK )
        rf_rx_process_data();
#endif
    }
    if( sta&RF_STATE_TX_FINISH )
    {
#if( WAIT_ACK )
        RF_TxWaitAckStart( );     // 打开ACK接收窗口等待应答
#endif
        gTxCount ++;
    }
}

/*******************************************************************************
 * @fn      TMR_IRQHandler
 *
 * @brief   TMR中断函数，定时发送测试数据
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
        // 初始化发送的数据
        TxBuf[0] = 0x55;
        TxBuf[1] = TEST_DATA_LEN;        // 此字节固定为数据长度，从此往后有TEST_DATA_LEN字节数据
        TxBuf[2] ++;
        TxBuf[3] = 3;
        TxBuf[4] = 4;
        rf_tx_start( TxBuf );   // 发送计数由TX_FINISH回调统计，此处不重复累加

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

    // TX相关参数，全局变量
    {
        gTxParam.accessAddress = ACCESS_ADR;         // Tx接入地址
        gTxParam.crcInit = CRC_INIT;
        gTxParam.crcPoly = CRC_POLY;
        gTxParam.properties = Properties.cfgVal;
        gTxParam.waitTime = 80*2;
        gTxParam.txPowerVal = TX_POWER;             // Tx发射功率
        gTxParam.txLen = TEST_DATA_LEN;

        if(gTxParam.txPowerVal > LL_TX_PWR_6_DBM)
        {
            sys_safe_access_enable();
            R32_MISC_CTRL |= (7<<6);
            sys_safe_access_disable();
        }
        else
        {
            sys_safe_access_enable();
            R32_MISC_CTRL = ((R32_MISC_CTRL &~(7<<6)) | (5<<6));
            sys_safe_access_disable();
        }
    }

#if( WAIT_ACK )
    // RX相关参数，全局变量(用于接收ACK)
    {
        gRxParam.accessAddress = ACCESS_ADR;        // Rx接入地址
        gRxParam.crcInit = CRC_INIT;
        gRxParam.crcPoly = CRC_POLY;
        gRxParam.properties = Properties.cfgVal;
        gRxParam.rxDMA = (uint32_t)RxBuf;
        gRxParam.rxMaxLen = TEST_DATA_LEN;
    }
#endif

    PFIC_EnableIRQ( BLEB_IRQn );
    PFIC_EnableIRQ( BLEL_IRQn );

    /* 打印演示配置信息 */
    PRINT("\n==============================================\n");
    PRINT("  2.4G RF Basic TX\n");
#if( WAIT_ACK )
    PRINT("  mode     : TX + ACK wait\n");
#else
    PRINT("  mode     : TX only (single send)\n");
#endif
    PRINT("  freq     : ch %d\n", TEST_FREQUENCY);
    PRINT("  phy      : %s\n", PHY_MODE_PHY_2M ? "2M" : "1M");
    PRINT("  len      : %d bytes\n", TEST_DATA_LEN);
    PRINT("==============================================\n");
    gTxCount = 0;
    gRxCount = 0;
    Rssi_sum = 0;
    TMR_TimerInit( GetSysClock() / RF_DEVICE_PERIOD );
    TMR_ITCfg(ENABLE, TMR_IT_CYC_END); // 开启中断
    PFIC_EnableIRQ(TMR_IRQn);
}

/******************************** endfile @rf ******************************/
