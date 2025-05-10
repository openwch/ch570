/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_pwr.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH57x_PWR_H__
#define __CH57x_PWR_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ROM_CFG_ADR_HW       0x7F00C            // config address for hardware config for LDO&OSC and etc

/**
 * @brief  wakeup mode define, select wakeup delay
 */
typedef enum
{
    Fsys_Delay_3584 = 0,
    Fsys_Delay_512,
    Fsys_Delay_64,
    Fsys_Delay_1,
    Fsys_Delay_8191,
    Fsys_Delay_7168,
    Fsys_Delay_6144,
    Fsys_Delay_4096,

} WakeUP_ModeypeDef;

/**
 * @brief  wakeup mode define
 */
typedef enum
{
    /* 下面等级将使用低功耗监控，1uA消耗 */
    LPLevel_1V8 = 0,
    LPLevel_2V0,
    LPLevel_2V2,
    LPLevel_2V4,

} VolM_LevelypeDef;

/**
 * @brief   外设时钟控制位
 *
 * @param   s       - 是否打开对应外设时钟
 * @param   perph   - please refer to Peripher CLK control bit define
 */
void PWR_PeriphClkCfg(FunctionalState s, uint16_t perph);

/**
 * @brief   睡眠唤醒源配置
 *
 * @param   s       - 是否打开此外设睡眠唤醒功能
 * @param   perph   - 需要设置的唤醒源
 *                    RB_SLP_USB_WAKE   -  USB 为唤醒源
 *                    RB_SLP_RTC_WAKE   -  RTC 为唤醒源
 *                    RB_SLP_GPIO_WAKE  -  GPIO 为唤醒源
 *                    RB_SLP_BAT_WAKE   -  BAT 为唤醒源
 * @param   mode    - refer to WakeUP_ModeypeDef
 */
void PWR_PeriphWakeUpCfg(FunctionalState s, uint8_t perph, WakeUP_ModeypeDef mode);

/**
 * @brief   电源监控
 *
 * @param   s       - 是否打开此功能
 * @param   vl      - refer to VolM_LevelypeDef
 */
void PowerMonitor(FunctionalState s, VolM_LevelypeDef vl);

/**
 * @brief   低功耗-Idle模式
 */
void LowPower_Idle(void);

/**
 * @brief   低功耗-Halt模式，此低功耗切到HSI/5时钟运行，唤醒后需要用户自己重新选择系统时钟源
 */
void LowPower_Halt(void);

/**
 * @brief   低功耗-Sleep模式，此低功耗切到HSI/5时钟运行，唤醒后需要用户自己重新选择系统时钟源
 *          @note 注意调用此函数，DCDC功能强制关闭，唤醒后可以手动再次打开
 *
 * @param   rm      - 供电模块选择
 *                    RB_PWR_RAM2K  -   2K retention SRAM 供电
 *                    RB_PWR_RAM16K -   16K main SRAM 供电
 *                    RB_PWR_EXTEND -   USB 和 BLE 单元保留区域供电
 *                    RB_PWR_XROM   -   FlashROM 供电
 *                    NULL          -   以上单元都断电
 */
void LowPower_Sleep(uint16_t rm);

/**
 * @brief   低功耗-Shutdown模式，此低功耗切到HSI/5时钟运行，唤醒后需要用户自己重新选择系统时钟源
 *          @note 注意调用此函数，DCDC功能强制关闭，唤醒后可以手动再次打开
 *
 * @param   rm      - 供电模块选择
 *                    RB_PWR_RAM2K  -   2K retention SRAM 供电
 *                    RB_PWR_RAM16K -   16K main SRAM 供电
 *                    NULL          -   以上单元都断电
 */
void LowPower_Shutdown(uint16_t rm);

#ifdef __cplusplus
}
#endif

#endif // __CH57x_PWR_H__
