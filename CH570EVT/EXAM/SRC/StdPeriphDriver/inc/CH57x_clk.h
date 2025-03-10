/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_clk.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH57x_CLK_H__
#define __CH57x_CLK_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  系统主频定义
 */
typedef enum
{
    CLK_SOURCE_LSI = 0xC0,

    CLK_SOURCE_HSE_16MHz = (0x02),
    CLK_SOURCE_HSE_8MHz = (0x04),
    CLK_SOURCE_HSE_6_4MHz = (0x05),
    CLK_SOURCE_HSE_4MHz = (0x08),
    CLK_SOURCE_HSE_2MHz = (0x10),
    CLK_SOURCE_HSE_1MHz = (0x0),

    CLK_SOURCE_HSE_PLL_100MHz = (0x40 | 6),
    CLK_SOURCE_HSE_PLL_75MHz = (0x40 | 8),
    CLK_SOURCE_HSE_PLL_60MHz = (0x40 | 10),
    CLK_SOURCE_HSE_PLL_50MHz = (0x40 | 12),
    CLK_SOURCE_HSE_PLL_40MHz = (0x40 | 15),
    CLK_SOURCE_HSE_PLL_30MHz = (0x40 | 20),
    CLK_SOURCE_HSE_PLL_25MHz = (0x40 | 24),
    CLK_SOURCE_HSE_PLL_24MHz = (0x40 | 25),
    CLK_SOURCE_HSE_PLL_20MHz = (0x40 | 30),

} SYS_CLKTypeDef;

/**
 * @brief  32M晶振电流挡位
 */
typedef enum
{
    HSE_RCur_75 = 0,
    HSE_RCur_100,
    HSE_RCur_125,
    HSE_RCur_150

} HSECurrentTypeDef;

/**
 * @brief  32M晶振内部电容挡位
 */
typedef enum
{
    HSECap_10p = 0,
    HSECap_12p,
    HSECap_14p,
    HSECap_16p,
    HSECap_18p,
    HSECap_20p,
    HSECap_22p,
    HSECap_24p,
    HSECap_2p,
    HSECap_4p,
    HSECap_6p,
    HSECap_8p

} HSECapTypeDef;

#define RTC_MAX_COUNT             0xA8C00000

#define MAX_DAY                   0x00004000
#define MAX_2_SEC                 0x0000A8C0
//#define	 MAX_SEC		0x545FFFFF

#define BEGYEAR                   2020
#define IsLeapYear(yr)            (!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))
#define YearLength(yr)            (IsLeapYear(yr) ? 366 : 365)
#define monthLength(lpyr, mon)    (((mon) == 1) ? (28 + (lpyr)) : (((mon) > 6) ? (((mon) & 1) ? 31 : 30) : (((mon) & 1) ? 30 : 31)))

/**
 * @brief  the total number of cycles captured by the oscillator define
 */
typedef enum
{
    Count_1 = 0,
    Count_2,
    Count_4,
    Count_32,
    Count_64,
    Count_128,
    Count_1024,
    Count_2047,
} RTC_OSCCntTypeDef;

/**
 * @brief  rtc timer mode period define
 */
typedef enum
{
    Period_4096 = 0,
    Period_8192,
    Period_16384,
    Period_32768,
    Period_65536,
    Period_131072,
    Period_262144,
    Period_524288,
} RTC_TMRCycTypeDef;

/**
 * @brief  rtc interrupt event define
 */
typedef enum
{
    RTC_TRIG_EVENT = 0, // RTC 触发事件
    RTC_TMR_EVENT,      // RTC 周期定时事件

} RTC_EVENTTypeDef;

/**
 * @brief  rtc interrupt mode define
 */
typedef enum
{
    RTC_TRIG_MODE = 0, // RTC 触发模式
    RTC_TMR_MODE,      // RTC 周期定时模式

} RTC_MODETypeDef;

/**
 * @brief   HSE晶体 偏置电流配置
 *
 * @param   c   - 75%,100%,125%,150%
 */
void HSECFG_Current(HSECurrentTypeDef c);

/**
 * @brief   HSE晶体 负载电容配置
 *
 * @param   c   - refer to HSECapTypeDef
 */
void HSECFG_Capacitance(HSECapTypeDef c);

/**
 * @brief   初始化 RTC时钟, 捕获周期数越高,初始化时间越长,时钟精度越高
 *
 * @param   cnt     - the total number of cycles captured by the oscillator
 *
 * @return  RTC时钟, 24~42KHz
 */
uint32_t RTC_InitClock(RTC_OSCCntTypeDef cnt);

/**
 * @brief   RTC时钟初始化当前时间
 *
 * @param   y       - 配置年，MAX_Y = BEGYEAR + 44
 * @param   mon     - 配置月，MAX_MON = 12
 * @param   d       - 配置日，MAX_D = 31
 * @param   h       - 配置小时，MAX_H = 23
 * @param   m       - 配置分钟，MAX_M = 59
 * @param   s       - 配置秒，MAX_S = 59
 */
void RTC_InitTime(uint16_t y, uint16_t mon, uint16_t d, uint16_t h, uint16_t m, uint16_t s);

/**
 * @brief   获取当前时间
 *
 * @param   py      - 获取到的年，MAX_Y = BEGYEAR + 44
 * @param   pmon    - 获取到的月，MAX_MON = 12
 * @param   pd      - 获取到的日，MAX_D = 31
 * @param   ph      - 获取到的小时，MAX_H = 23
 * @param   pm      - 获取到的分钟，MAX_M = 59
 * @param   ps      - 获取到的秒，MAX_S = 59
 */
void RTC_GetTime(uint16_t *py, uint16_t *pmon, uint16_t *pd, uint16_t *ph, uint16_t *pm, uint16_t *ps);

/**
 * @brief   基于LSI时钟，配置当前RTC 周期数
 *
 * @param   cyc     - 配置周期计数初值，MAX_CYC = 0xA8BFFFFF = 2831155199
 */
void RTC_SetCycleLSI(uint32_t cyc);

/**
 * @brief   基于LSI时钟，获取当前RTC 周期数
 *
 * @return  当前周期数，MAX_CYC = 0xA8BFFFFF = 2831155199
 */
uint32_t RTC_GetCycleLSI(void);

/**
 * @brief   RTC定时模式配置（注意定时基准固定为32768Hz）
 *
 * @param   t   - refer to RTC_TMRCycTypeDef
 */
void RTC_TRIGFunCfg(uint32_t cyc);

/**
 * @brief   RTC定时模式配置（注意定时基准固定为32768Hz）
 *
 * @param   t   - refer to RTC_TMRCycTypeDef
 */
void RTC_TMRFunCfg(RTC_TMRCycTypeDef t);

/**
 * @brief   RTC 模式功能关闭
 *
 * @param   m   - 需要关闭的当前模式
 */
void RTC_ModeFunDisable(RTC_MODETypeDef m);

/**
 * @brief   获取RTC中断标志
 *
 * @param   f   - refer to RTC_EVENTTypeDef
 *
 * @return  中断标志状态
 */
uint8_t RTC_GetITFlag(RTC_EVENTTypeDef f);

/**
 * @brief   清除RTC中断标志
 *
 * @param   f   - refer to RTC_EVENTTypeDef
 */
void RTC_ClearITFlag(RTC_EVENTTypeDef f);

/**
 * @brief   低频时钟电源配置
 */
void LClk_Cfg(FunctionalState s);


#ifdef __cplusplus
}
#endif

#endif // __CH57x_CLK_H__
