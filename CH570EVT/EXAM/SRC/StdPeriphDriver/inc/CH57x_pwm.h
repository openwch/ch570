/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_pwm.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH57x_PWM_H__
#define __CH57x_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  channel of PWM define
 */
#define CH_PWM1     0x01  // PWM1   通道
#define CH_PWM2     0x02  // PWM2   通道
#define CH_PWM3     0x04  // PWM3   通道
#define CH_PWM4     0x08  // PWM4   通道
#define CH_PWM5     0x10  // PWM5   通道
#define CH_PWM_ALL  0x1F  // PWM1-5 通道

/**
 * @brief  DMA channel of PWM
 */
typedef enum
{
    Mode_DMACH1_3 = 0, // DMA选择1、2、3通道输出
    Mode_DMACH4_5,     // DMA选择4、5通道输出
    Mode_DMACH1_5,     // DMA选择1、2、3、4、5通道输出
} PWM_DMAChannel;

/**
 * @brief  channel of PWM define
 */
typedef enum
{
    High_Level = 0, // 默认低电平，高电平有效
    Low_Level,      // 默认高电平，低电平有效
} PWMX_PolarTypeDef;

/**
 * @brief  Configuration PWM4_11 Cycle size
 */
typedef enum
{
    PWMX_Cycle_256 = 0, // 256 个PWMX周期
    PWMX_Cycle_255,     // 255 个PWMX周期
    PWMX_Cycle_128,     // 128 个PWMX周期
    PWMX_Cycle_127,     // 127 个PWMX周期
    PWMX_Cycle_64,      // 64 个PWMX周期
    PWMX_Cycle_63,      // 63 个PWMX周期
} PWMX_CycleTypeDef;

/**
 * @brief  Configuration DMA mode
 */
typedef enum
{
    PWM_ModeSINGLE = 0, // 单次模式
    PWM_ModeLOOP,       // 循环模式
} PWM_DMAModeTypeDef;

/**
 * @brief   PWM 通道基准时钟配置
 *
 * @param   d   - 通道基准时钟 = d*Tsys
 */
#define PWMX_CLKCfg(d)    (R16_PWM_CLOCK_DIV = d)

/**
 * @brief   PWM 8位周期配置
 *
 * @param   cyc - refer to PWMX_CycleTypeDef
 */
void PWMX_CycleCfg(PWMX_CycleTypeDef cyc);

/**
 * @brief   PWM 16位周期配置
 *
 * @param   ch  - select channel of pwm, refer to channel of PWM define
 *          cyc - 16位周期
 */
void PWMX_16bit_CycleCfg(uint8_t ch, uint16_t cyc);

/**
 * @brief   PWM 16位数据位宽使能
 */
#define PWM_16bit_CycleEnable()  (R8_PWM_CONFIG |= (3 << 1))

/**
 * @brief   设置 PWM1 8位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM1_ActDataWidth(d)     (R8_PWM1_DATA = d)

/**
 * @brief   设置 PWM2 8位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM2_ActDataWidth(d)     (R8_PWM2_DATA = d)

/**
 * @brief   设置 PWM3 8位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM3_ActDataWidth(d)     (R8_PWM3_DATA = d)

/**
 * @brief   设置 PWM4 8位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM4_ActDataWidth(d)     (R8_PWM4_DATA = d)

/**
 * @brief   设置 PWM5 8位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM5_ActDataWidth(d)     (R8_PWM5_DATA = d)

/**
 * @brief   设置 PWM1 16位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM1_16bit_ActDataWidth(d)     (R16_PWM1_DATA = d)

/**
 * @brief   设置 PWM2 16位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM2_16bit_ActDataWidth(d)     (R16_PWM2_DATA = d)

/**
 * @brief   设置 PWM3 16位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM3_16bit_ActDataWidth(d)     (R16_PWM3_DATA = d)

/**
 * @brief   设置 PWM4 16位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM4_16bit_ActDataWidth(d)     (R16_PWM4_DATA = d)

/**
 * @brief   设置 PWM5 16位有效数据脉宽
 *
 * @param   d   - 有效数据脉宽
 */
#define PWM5_16bit_ActDataWidth(d)     (R16_PWM5_DATA = d)

/**
 * @brief   PWM 8位输出波形配置
 *
 * @param   ch      - select channel of pwm, refer to channel of PWM define
 * @param   da      - effective pulse width
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_ACTOUT(uint8_t ch, uint8_t da, PWMX_PolarTypeDef pr, FunctionalState s);

/**
 * @brief   PWM 16位输出波形配置
 *
 * @param   ch      - select channel of pwm, refer to channel of PWM define
 * @param   da      - effective pulse width
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_16bit_ACTOUT(uint8_t ch, uint16_t da, PWMX_PolarTypeDef pr, FunctionalState s);

/**
 * @brief   PWM 交替输出模式配置
 *
 * @param   ch      - select group of PWM alternate output
 *                    RB_PWM4_5_STAG_EN     -  PWM4 和 PWM5 通道交替输出
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_AlterOutCfg(uint8_t ch, FunctionalState s);

/**
 * @brief   PWM 同步输出模式配置
 *
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_SyncOutCfg(FunctionalState s);

/**
 * @brief   配置PWM DMA功能
 *
 * @param   s           - 是否打开DMA功能
 * @param   startAddr   - DMA 起始地址
 * @param   endAddr     - DMA 结束地址
 * @param   m           - 配置DMA模式
 */
void PWM_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, PWM_DMAModeTypeDef m, PWM_DMAChannel ch);

#ifdef __cplusplus
}
#endif

#endif // __CH57x_PWM_H__
