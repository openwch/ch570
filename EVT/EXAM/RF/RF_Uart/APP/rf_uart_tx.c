/********************************** (C) COPYRIGHT *******************************
 * File Name          : rf_basic.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/08/15
 * Description        : 无线串口-发送端，注：RF发送失败 RESEND_COUNT 次数会丢弃数据包
 *
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "rf.h"
#include "rf_uart_tx.h"
#include "uart.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

rfTxBuf_t gTxBuf;
uint32_t gRfRxCount;
uint16_t gInterval;
uint16_t gTimeoutMax;
uint16_t gTimeout;
uint16_t gServerData;

uint8_t gTxDataSeq;
uint8_t gRfStatus;
uint8_t gBoundStatus;
uint8_t volatile gRxDataFlag;
uint8_t getDataProbe;
rfPackage_t *pPkt_t;

static void rfProcessRx( rfPackage_t *pPkt );
static void rfProcessTx( void );
static void rfProcessTimeout( void );

// tf status callbacks
rfStatusCBs_t rfCBs =
{
    rfProcessRx,
    rfProcessTx,
    rfProcessTimeout,
    rfProcessTimeout,
};

/*******************************************************************************
 * @fn      rf_disconnect
 *
 * @brief   断开连接
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
static void rf_disconnect( void )
{
    gBoundStatus = BOUND_STATUS_IDLE;
    UART_SetTimer( ADV_INTERVAL );
    rf_tx_set_sync_word( AA );
    rf_tx_set_frequency( DEF_FREQUENCY );

    rf_rx_set_sync_word( AA );
    rf_rx_set_frequency( DEF_FREQUENCY );
    PRINT("disconnect.\n" );
}

/*******************************************************************************
 * @fn      rf_bound
 *
 * @brief   断开连接
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
static void rf_bound( bound_rsp_t *rsp )
{

    rfBoundInfo_t info;

    gTimeout = 0;
    gBoundStatus = BOUND_STATUS_WAIT;
    gInterval = rsp->interval;
    gTimeoutMax = (rsp->timeout*10/rsp->interval);
    gServerData = rsp->severData;
    rf_tx_set_sync_word( rsp->accessaddr );
    rf_tx_set_frequency( rsp->channel );
    rf_tx_set_phy_type( rsp->phy );
    rf_rx_set_sync_word( rsp->accessaddr );
    rf_rx_set_frequency( rsp->channel );
    rf_rx_set_phy_type( rsp->phy );
    info.head = 0x55aa;
    info.serverData = gServerData;
    FLASH_ROM_ERASE( BOUND_INFO_FLASH_ADDR, 4096 );
    FLASH_ROM_WRITE( BOUND_INFO_FLASH_ADDR,&info,4 );
    PRINT("bound success.%x %x\n",rsp->accessaddr,rsp->channel );
}

/*******************************************************************************
 * @fn      RF_ProcessRx
 *
 * @brief
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
static void rfProcessRx( rfPackage_t *pPkt )
{
    if( gTxDataSeq == pPkt->seq )
    {
        if( pPkt->type == PKT_DATA_RSP_ACK )
        {
            // 数据发送成功
            if( pPkt->length > PKT_DATA_OFFSET+1 )
            {
                gRxDataFlag = 1;
                pPkt_t = pPkt;
            }
        }
        else if( pPkt->type == PKT_CMD_BOUND_RSP )
        {
            rf_bound( (bound_rsp_t *)(pPkt+1) );
        }
        else if( pPkt->type == PKT_CMD_RSP_STATUS )
        {
            rfRsp_t *pRsp_t = (rfRsp_t *)(pPkt+1);

            if( gBoundStatus == BOUND_STATUS_WAIT )
            {
                gBoundStatus = BOUND_STATUS_EST;
                UART_SetTimer( gInterval );
            }
            // 状态应答为对端设备波特率
            if( pRsp_t->opcode == OPCODE_BSP )
            {
                UART_SetBuad( pRsp_t->buad_t.BaudRate );
                if(pRsp_t->buad_t.ioStaus&0x20)
                {
                    GPIOA_SetBits(DTR);
                }
                else
                {
                    GPIOA_ResetBits(DTR);
                }
                if(pRsp_t->buad_t.ioStaus&0x40)
                {
                    GPIOA_SetBits(RTS);
                }
                else
                {
                    GPIOA_ResetBits(RTS);
                }
            }
            else if( pRsp_t->opcode == OPCODE_DATA )
            {
                gRxDataFlag = 1;
                pPkt_t = pPkt;
                getDataProbe = 6;
            }
        }
        else
        {

        }
        gRfStatus = RF_STATUS_IDLE;
        gTxBuf.status = STA_IDLE;
        gTxDataSeq ++;
        gTimeout = 0;
    }
}

/*******************************************************************************
 * @fn      RF_ProcessTx
 *
 * @brief
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
static void rfProcessTx( void )
{
    if( gRfStatus == RF_STATUS_TX )
    {
        gRfStatus = RF_STATUS_WAITRSP;
    }
    else if( gRfStatus == RF_STATUS_RETX )
    {
        gRfStatus = RF_STATUS_REWAIT;
    }
    else if( gRfStatus == RF_STATUS_GETS )
    {

    }
    rf_rx_start( 150 );
}

/*******************************************************************************
 * @fn      RF_ProcessTimeout
 *
 * @brief
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
static  void rfProcessTimeout( void )
{
    if( gRfStatus == RF_STATUS_WAITRSP )
    {
        gTxBuf.resendCount = RESEND_COUNT;
        gTxBuf.status = STA_RESEND;
    }
    else if( gRfStatus == RF_STATUS_REWAIT )
    {
        gTxBuf.status = STA_RESEND;
    }
    else
    {
        if( gBoundStatus == BOUND_STATUS_WAIT )
        {
            if( ++ gTimeout > BOUND_EST_COUNT )
            {
                rf_disconnect( );
            }
        }
        else if( gBoundStatus == BOUND_STATUS_EST )
        {
            if( ++ gTimeout > gTimeoutMax )
            {
                rf_disconnect( );
            }
        }
        // 获取状态可以丢弃，不重传
        gTxBuf.status = STA_IDLE;
    }
}


/*******************************************************************************
 * @fn      RF_StatusQuery
 *
 * @brief   状态处理
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void RF_StatusQuery( void )
{
    uint8_t s;

    if( gRxDataFlag )
    {
        rfRsp_t *pRsp_t = (rfRsp_t *)(pPkt_t+1);

        UART_Send(pRsp_t->other.rspData,pPkt_t->length-PKT_DATA_OFFSET-1);
        gRxDataFlag = 0;
    }
    if( gTxBuf.status == STA_IDLE )
    {
        rfPackage_t *pPkt_t = (rfPackage_t *)gTxBuf.TxBuf;

        gTxBuf.len  = DATA_LEN_MAX_TX;
        s = UART_RxQuery( (void *)(pPkt_t+1), &gTxBuf.len );
        // 发送数据
        if( s == 0 )
        {
            gRfStatus = RF_STATUS_TX;
            pPkt_t->type = PKT_DATA_FLAG;
            pPkt_t->length = gTxBuf.len + PKT_DATA_OFFSET;
            gTxBuf.status = STA_BUSY;
            pPkt_t->seq = gTxDataSeq;
            pPkt_t->resv = 0;
            rf_tx_start( gTxBuf.TxBuf, 60 );
        }
        else if( s == 0x80 )
        {
            if( gBoundStatus )
            {
                // 获取状态
                gRfStatus = RF_STATUS_GETS;
                pPkt_t->type = PKT_CMD_GET_STATUS;
                pPkt_t->length = PKT_DATA_OFFSET;
            }
            else
            {
                // 请求绑定
                bound_req_t *pReq_t = (bound_req_t *)&gTxBuf.TxBuf[PKT_HEAD_LEN];

                gRfStatus = RF_STATUS_REQ;
                pPkt_t->type = PKT_CMD_BOUND_REQ;
                pPkt_t->length = PKT_DATA_OFFSET + sizeof(bound_req_t);
                pReq_t->interval = ADV_INTERVAL;
                pReq_t->severData = gServerData;
                gTxDataSeq = 0;
            }
            gTxBuf.status = STA_BUSY;
            pPkt_t->seq = gTxDataSeq;
            pPkt_t->resv = 0;
            rf_tx_start( gTxBuf.TxBuf, 60 );
        }
        else
        {
            if( getDataProbe )
            {
                getDataProbe--;
                gRfStatus = RF_STATUS_GETS;
                pPkt_t->type = PKT_CMD_GET_STATUS;
                pPkt_t->length = PKT_DATA_OFFSET;
                gTxBuf.status = STA_BUSY;
                pPkt_t->seq = gTxDataSeq;
                pPkt_t->resv = 0;
                rf_tx_start( gTxBuf.TxBuf, 60 );
            }
        }
    }
    else if( gTxBuf.status == STA_RESEND )
    {
        if( gTxBuf.resendCount )
        {
            if( gTxBuf.resendCount < RESEND_COUNT/2 )
            {
                PRINT("*%d %d\n",gTxBuf.resendCount,gTxDataSeq);
            }
            gRfStatus = RF_STATUS_RETX;
            if( gTxBuf.resendCount != 0xFF ) gTxBuf.resendCount--;
            gTxBuf.status = STA_BUSY;
            rf_tx_start( gTxBuf.TxBuf, 60 );
        }
        else
        {
            // 发送失败丢弃
            PRINT(" resend fail.%d\n",gTxBuf.TxBuf[1]);
            gTxBuf.status = STA_IDLE;
        }
    }
}

/*******************************************************************************
 * @fn      RF_UartTxInit
 *
 * @brief   RF uart发送应用初始化
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void RF_UartTxInit( void )
{
    rfBoundInfo_t *pInfo;
    PRINT("----------------- rf uart tx mode -----------------\n");
    gTxDataSeq = 0;
    gRxDataFlag = 0;
    gBoundStatus = BOUND_STATUS_IDLE;
    gTxBuf.status = 0;

    pInfo = (rfBoundInfo_t *)(BOUND_INFO_FLASH_ADDR);
    if( pInfo->head == BOUND_INFO_HEAD )
    {
        gServerData = pInfo->serverData;
    }
    else {
        gServerData = 0;
    }
    PRINT("gServerData = %x \n",gServerData);
    RFRole_RegisterStatusCbs( &rfCBs );
}

/******************************** endfile @rf ******************************/
