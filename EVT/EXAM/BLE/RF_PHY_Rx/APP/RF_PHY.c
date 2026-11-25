/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_PHY.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "RF_PHY.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

#define RF_AUTO_MODE       1

uint8_t rf_task_id;
uint8_t tx_data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
uint8_t rx_data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
uint8_t rx_len;
int8_t rssi;

// volatile uint8_t tx_end_flag = 0;   /* 未被使用 */
volatile uint8_t rx_end_flag = 0;

/*********************************************************************
 * @fn      RF_Wait_Rx_End
 *
 * @brief   自动模式等待应答发送完成，必须在RAM中等待，等待时可以执行用户代码，但需要注意执行的代码必须运行在RAM中，否则影响发送
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline))
void RF_Wait_Rx_End()
{
    uint32_t i=0;
    while(!rx_end_flag)
    {
        i++;
        __nop();
        __nop();
        // 约5ms超时
        if(i>(FREQ_SYS/1000))
        {
            rx_end_flag = TRUE;
        }
    }
}

/*********************************************************************
 * @fn      RF_2G4StatusCallback
 *
 * @brief   RF 状态回调，此函数在中断中调用。注意：不可在此函数中直接调用RF接收或者发送API，需要使用事件的方式调用
 *          在此回调中直接使用或调用函数涉及到的变量需注意，此函数在中断中调用。
 *
 * @param   sta     - 状态类型
 * @param   crc     - crc校验结果
 * @param   rxBuf   - 数据buf指针
 *
 * @return  none
 */
void RF_2G4StatusCallback(uint8_t sta, uint8_t crc, uint8_t *rxBuf)
{
    switch(sta)
    {
        case RX_MODE_RX_DATA:
        {
            if (crc == 0) 
            {
#if(RF_AUTO_MODE)
                RF_Wait_Rx_End();
#endif
                rssi = (int8_t)rxBuf[0];
                rx_len = rxBuf[1];
                tmos_memcpy(rx_data,&rxBuf[2],rx_len);
                tmos_set_event(rf_task_id, RF_RX_DATA_EVT);
            } 
            else 
            {
                if (crc & (1<<0)) 
                {
                    //crc error
                }

                if (crc & (1<<1)) 
                {
                    //match type error
                }
            }
#if(!RF_AUTO_MODE)
            tmos_set_event(rf_task_id, RF_RX_EVT);
#endif
            break;
        }
        case RX_MODE_TX_FINISH:
        {
#if(RF_AUTO_MODE)
            rx_end_flag = TRUE;
            tmos_set_event(rf_task_id, RF_RX_EVT);
#endif
            break;
        }
        case RX_MODE_TX_FAIL:
        {
#if(RF_AUTO_MODE)
            rx_end_flag = TRUE;
            tmos_set_event(rf_task_id, RF_RX_EVT);
#endif
            break;
        }
    }
}

/*********************************************************************
 * @fn      RF_ProcessEvent
 *
 * @brief   RF 事件处理
 *
 * @param   task_id - 任务ID
 * @param   events  - 事件标志
 *
 * @return  未完成事件
 */
uint16_t RF_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(task_id)) != NULL)
        {
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
    if(events & RF_RX_EVT)
    {
        uint8_t state;
        RF_Shut();
        tx_data[0]++;
#if(RF_AUTO_MODE)
        rx_end_flag = FALSE;
#endif
        state = RF_Rx(tx_data, 10, 0xFF, 0xFF);
        PRINT("RX mode.state = %x\n", state);
        return events ^ RF_RX_EVT;
    }
    if(events & RF_RX_DATA_EVT)
    {
        uint8_t i;
        PRINT("rx recv, rssi: %d\n", (int8_t)rssi);
        PRINT("len:%d-", rx_len);
        
        for (i = 0; i < rx_len; i++) 
        {
            PRINT("%x ", rx_data[i]);
        }
        PRINT("\n");
        return events ^ RF_RX_DATA_EVT;
    }
    return 0;
}

/*********************************************************************
 * @fn      RF_Init
 *
 * @brief   RF 初始化
 *
 * @return  none
 */
void RF_Init(void)
{
    uint8_t    state;
    rfConfig_t rf_Config;

    tmos_memset(&rf_Config, 0, sizeof(rfConfig_t));
    rf_task_id = TMOS_ProcessEventRegister(RF_ProcessEvent);
    rf_Config.accessAddress = 0x71764129; // 禁止使用0x55555555以及0xAAAAAAAA ( 建议不超过24次位反转，且不超过连续的6个0或1 )
    rf_Config.CRCInit = 0x555555;
    rf_Config.Channel = 39;
    rf_Config.Frequency = 2480000;
#if(RF_AUTO_MODE)
    rf_Config.LLEMode = LLE_MODE_AUTO;
#else
    rf_Config.LLEMode = LLE_MODE_BASIC | LLE_MODE_EX_CHANNEL; // 使能 LLE_MODE_EX_CHANNEL 表示 选择 rf_Config.Frequency 作为通信频点
#endif
    rf_Config.rfStatusCB = RF_2G4StatusCallback;
    rf_Config.RxMaxlen = 251;
    state = RF_Config(&rf_Config);
    PRINT("rf 2.4g init: %x\n", state);

    PRINT("\n==============================================\n");
    PRINT("  2.4G RF PHY RX\n");
#if( RF_AUTO_MODE )
    PRINT("  mode     : AUTO\n");
#else
    PRINT("  mode     : BASIC\n");
#endif
    PRINT("  freq     : %d.%03d MHz\n", rf_Config.Frequency / 1000, rf_Config.Frequency % 1000);
    PRINT("  channel  : %d\n", rf_Config.Channel);
    PRINT("  addr     : %08x\n", rf_Config.accessAddress);
    PRINT("  rxMaxlen : %d\n", rf_Config.RxMaxlen);
    PRINT("==============================================\n");
   { // RX mode
#if(RF_AUTO_MODE)
       rx_end_flag = FALSE;
#endif
       state = RF_Rx(tx_data, 10, 0xFF, 0xFF);
       PRINT("RX mode.state = %x\n", state);
   }
}

/******************************** endfile @ main ******************************/
