/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_extend.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/09/15
 * Description        : 2.4G库扩展模式发送测试例程(TX端)
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
#include <RF_extend.h>

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
rfipTx_t gTxParam;
rfipRx_t gRxParam;

__attribute__((__aligned__(4))) uint8_t TxBuf[64];
__attribute__((__aligned__(4))) uint8_t RxBuf[264]; // 接收DMA buf不能小于264字节

#define  TEST_DATA_LEN    4         // 数据长度
#define  TEST_FREQUENCY   16        // 通信通道
#define  TX_POWER         LL_TX_PWR_7_DBM // 发射功率

volatile uint32_t gTxCount;
volatile uint32_t gRxCount;
volatile int32_t Rssi_sum = 0;
volatile uint32_t gPrintCnt = 0;

int8_t gRssi=0;

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
    gTxParam.waitTime = 80*2;
    RFIP_StartTx( &gTxParam );
}

/******************************** 接收相关函数 ********************************/
/* 以下接收相关代码仅用于发送后打开ACK接收窗口等待应答 */

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

#if( WAIT_ACK )
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
#endif

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
    gRxCount ++;
    {
        uint8_t *pData = (uint8_t *)gRxParam.rxDMA;
        #if(DPL_EN)
            Rssi_sum += (int8_t)pData[TEST_DATA_LEN+RSSI_DPL_OFFSET];
        #else
            Rssi_sum += (int8_t)pData[TEST_DATA_LEN+RSSI_OFFSET];
        #endif
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
void RF_ProcessCallBack( rfRole_States_t sta,uint8_t id  )
{
    if( sta&RF_STATE_RX )
    {
        rf_rx_process_data();
    }
    if( sta&RF_STATE_RX_CRCERR )
    {
        /* ACK错包，等待超时后由定时发送重新开始 */
    }
    if( sta&RF_STATE_TX_FINISH )
    {
#if( WAIT_ACK )
        RF_TxWaitAckStart( );     // 打开ACK接收窗口等待应答
#endif
        gTxCount ++;
    }
    if( sta&RF_STATE_TIMEOUT )
    {
        /* ACK等待超时，如果是发送则是发送失败(中断上下文，不做打印) */
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
        gPrintCnt++;
        // 发送数据
        TxBuf[0] = TEST_DATA_LEN;
        TxBuf[1] ++;
        TxBuf[2] = 0x02;
        TxBuf[3] = 0x03;
        rf_tx_start( TxBuf );
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

#if(USER_MODE == 1 )
    PKT_DET_CFG4(CFG4_REG_VAL);
#endif
    if(MODE_2G4 == PHY_2G4_1M)
    {
        #if( AA_LEN== ADDR_3BYTE )
            PKT_DET_CFG3(CFG3_1M_ADDR3); //3字节地址
        #else
            PKT_DET_CFG3(CFG3_1M_ADDR4_5); //4&5字节地址
        #endif
    }
    else {
        #if( AA_LEN== ADDR_3BYTE )
            PKT_DET_CFG3(CFG3_2M_ADDR3); //3字节地址
        #else
            PKT_DET_CFG3(CFG3_2M_ADDR4_5); //4&5字节地址
        #endif
    }

    {
        rfRoleConfig_t conf ={0};
        conf.rfProcessCB = RF_ProcessCallBack;
        conf.processMask = RF_STATE_RX|RF_STATE_RX_CRCERR|RF_STATE_TX_FINISH|RF_STATE_TIMEOUT;
        RFRole_BasicInit( &conf );
    }
    TPROPERTIES_CFG Properties;
    {
       Properties.cfgVal =  TEST_PHY_MODE |BB_WHITENING_OFF;
        #if(TEST_PHY_MODE == PHY_MODE_2G4 )
        Properties.lengthCrc = CRC_LEN;
        Properties.ctlField = CTL_FIELD;
        Properties.lengthAA = AA_LEN;
        Properties.lengthPreamble = PRE_LEN;
        Properties.dplEnable = DPL_EN;
        Properties.mode2G4 = MODE_2G4;
        Properties.bitOrderData = DATA_ORDER;
        Properties.crcXOREnable = CRC_XOR_EN;
#endif
        PRINT("cfgVal=%x\n",Properties.cfgVal);
    }

    // TX相关参数，全局变量
    {
        gTxParam.accessAddress = AA;
        gTxParam.accessAddressEx = AA_EX;
        gTxParam.crcInit = CRC_INIT;
        gTxParam.crcPoly = CRC_POLY;
        gTxParam.properties = Properties.cfgVal;
        gTxParam.waitTime = 80*2;
        gTxParam.txPowerVal = TX_POWER;
        gTxParam.txLen = TEST_DATA_LEN;
    }
    
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

    // RX相关参数，全局变量(用于接收ACK)
    {
        gRxParam.accessAddress = AA;
        gRxParam.accessAddressEx = AA_EX;
        gRxParam.crcInit = CRC_INIT;
        gRxParam.crcPoly = CRC_POLY;
        gRxParam.properties = Properties.cfgVal;
        gRxParam.rxDMA = (uint32_t)RxBuf;
        gRxParam.rxMaxLen = TEST_DATA_LEN;
    }
    PFIC_EnableIRQ( BLEB_IRQn );
    PFIC_EnableIRQ( BLEL_IRQn );

    gTxCount = 0;
    gRxCount = 0;
    Rssi_sum = 0;
    /* 打印演示配置信息 */
    PRINT("\n==============================================\n");
    PRINT("  2.4G RF Extend TX\n");
#if( WAIT_ACK )
    PRINT("  mode     : TX + ACK wait\n");
#else
    PRINT("  mode     : TX only (single send)\n");
#endif
    PRINT("  freq     : ch %d\n", TEST_FREQUENCY);
    PRINT("  phy      : %s\n", (MODE_2G4 == PHY_2G4_2M) ? "2M" : "1M");
    PRINT("  len      : %d bytes\n", TEST_DATA_LEN);
    PRINT("==============================================\n");
    TMR_TimerInit( GetSysClock() / RF_DEVICE_PERIOD );
    TMR_ITCfg(ENABLE, TMR_IT_CYC_END); // 开启中断
    PFIC_EnableIRQ(TMR_IRQn);
}

/******************************** endfile @rf ******************************/
