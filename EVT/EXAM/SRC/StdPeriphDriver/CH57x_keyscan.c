/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_keyscan.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/12/17
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

/*********************************************************************
 * @fn      KeyScan_Cfg
 *
 * @brief   配置按键扫描功能
 *
 * @param   s            -  设置是否开启按键扫描功能
 * @param   keyScanPin   -  设置参与按键扫描的IO使能
 * @param   ClkDiv       -  设置扫描时钟分频，时钟来源LSI
 * @param   Rep          -  设置扫描到相同按键值次数
 *
 * @return  none
 */
void KeyScan_Cfg(uint8_t s, uint16_t keyScanPin, uint16_t ClkDiv, uint16_t Rep)
{
    if(s == DISABLE)
    {
        R16_KEY_SCAN_CTRL &= ~(RB_SCAN_START_EN);
    }
    else
    {
        R16_KEY_SCAN_CTRL |= keyScanPin | ClkDiv | Rep;
        R16_KEY_SCAN_CTRL |= RB_SCAN_START_EN;
    }
}

/*********************************************************************
 * @fn      KeyPress_Wake
 *
 * @brief   按键按下唤醒睡眠使能
 *
 * @param   s            -  设置是否开启按键唤醒功能
 *
 * @return  none
 */
void KeyPress_Wake(uint8_t s)
{
    if(s == DISABLE)
    {
        sys_safe_access_enable();
        R8_SLP_CLK_OFF0 &= ~(RB_SLP_KEYSCAN_WAKE);
        sys_safe_access_disable();
    }
    else
    {
        sys_safe_access_enable();
        R8_SLP_CLK_OFF0 |= RB_SLP_KEYSCAN_WAKE;
        sys_safe_access_disable();
    }
}
