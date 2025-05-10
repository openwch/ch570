/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_timer.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH57x_TIMER_H__
#define __CH57x_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DataBit_25            (1 << 25)

/**
 * @brief  TMR interrupt bit define
 */
#define TMR_IT_CYC_END     0x01  // 周期结束标志：捕捉-超时，定时-周期结束，PWM-周期结束
#define TMR_IT_DATA_ACT    0x02  // 数据有效标志：捕捉-新数据，PWM-有效电平结束
#define TMR_IT_FIFO_HF     0x04  // FIFO 使用过半：捕捉- FIFO>=4， PWM- FIFO<4
#define TMR_IT_DMA_END     0x08  // DMA 结束，支持TMR-TMR3
#define TMR_IT_FIFO_OV     0x10  // FIFO 溢出：捕捉- FIFO满， PWM- FIFO空

/**
 * @brief  ENC interrupt bit define
 */
#define RB_IE_DIR_INC      0x01  // 前进中断使能
#define RB_IE_DIR_DEC      0x02  // 后退中断使能

/**
 * @brief  Configuration PWM effective level repeat times
 */
typedef enum
{
    PWM_Times_1 = 0, // PWM 有效输出重复1次数
    PWM_Times_4,     // PWM 有效输出重复4次数
    PWM_Times_8,     // PWM 有效输出重复8次数
    PWM_Times_16,    // PWM 有效输出重复16次数
} PWM_RepeatTsTypeDef;

/**
 * @brief  Configuration Cap mode
 */
typedef enum
{
    CAP_NULL = 0,         // 不捕捉 & 不计数
    Edge_To_Edge,         // 任意边沿之间  &  计数任意边沿
    FallEdge_To_FallEdge, // 下降沿到下降沿  & 计数下降沿
    RiseEdge_To_RiseEdge, // 上升沿到上升沿  &  计数上升沿
} CapModeTypeDef;

/**
 * @brief  Configuration DMA mode
 */
typedef enum
{
    Mode_Single = 0, // 单次模式
    Mode_LOOP,       // 循环模式
} DMAModeTypeDef;

/**
 * @brief  Configuration ENC mode
 */
typedef enum
{
    Mode_IDLE = 0,   // IDLE模式
    Mode_T2 ,        // T2边沿计数模式
    Mode_T1 ,        // T1边沿计数模式
    Mode_T1T2 ,      // T1和T2边沿计数模式
} ENCModeTypeDef;

/**
 * @brief   定时功能初始化
 *
 * @param   t       - 定时时间，基于当前系统时钟Tsys, 最长定时周期 67108864
 */
void TMR_TimerInit(uint32_t t);

/**
 * @brief   边沿计数功能初始化
 *
 * @param   cap     - 采集计数类型
 */
void TMR_EXTSingleCounterInit(CapModeTypeDef cap);

/**
 * @brief   设置计数统计溢出大小，最大67108863
 *
 * @param   cyc     - 计数统计溢出大小
 */
#define TMR_CountOverflowCfg(cyc)    (R32_TMR_CNT_END = (cyc + 2))

/**
 * @brief   获取当前计数值，最大67108863
 *
 * @return  当前计数值
 */
#define TMR_GetCurrentCount()        R32_TMR_COUNT

/**
 * @brief   PWM0 通道输出波形周期配置, 最大67108863
 *
 * @param   cyc     - 输出波形周期
 */
#define TMR_PWMCycleCfg(cyc)         (R32_TMR_CNT_END = cyc)

/**
 * @brief   PWM 输出初始化
 *
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   ts      - set pwm repeat times, refer to PWM_RepeatTsTypeDef
 */
void TMR_PWMInit(PWMX_PolarTypeDef pr, PWM_RepeatTsTypeDef ts);

/**
 * @brief   PWM0 有效数据脉宽, 最大67108864
 *
 * @param   d       - 有效数据脉宽
 */
#define TMR_PWMActDataWidth(d)    (R32_TMR_FIFO = d)

/**
 * @brief   CAP0 捕捉电平超时配置, 最大33554432
 *
 * @param   cyc     - 捕捉电平超时
 */
#define TMR_CAPTimeoutCfg(cyc)    (R32_TMR_CNT_END = cyc)

/**
 * @brief   外部信号捕捉功能初始化
 *
 * @param   cap     - select capture mode, refer to CapModeTypeDef
 */
void TMR_CapInit(CapModeTypeDef cap);

/**
 * @brief   获取脉冲数据
 *
 * @return  脉冲数据
 */
#define TMR_CAPGetData()        R32_TMR_FIFO

/**
 * @brief   获取当前已捕获数据个数
 *
 * @return  当前已捕获数据个数
 */
#define TMR_CAPDataCounter()    R8_TMR_FIFO_COUNT

/**
 * @brief   配置DMA功能
 *
 * @param   s           - 是否打开DMA功能
 * @param   startAddr   - DMA 起始地址
 * @param   endAddr     - DMA 结束地址
 * @param   m           - 配置DMA模式
 */
void TMR_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m);

/**
 * @brief   配置ENC功能
 *
 * @param   s           - 是否开启编码器功能
 * @param   encReg      - 编码器模式终值(最大值0xFFFF)
 * @param   m           - 配置ENC模式
 *
 * @return  none
 */
void ENC_Config(uint8_t s, uint32_t encReg, ENCModeTypeDef m);

/**
 * @brief   关闭 TMR PWM输出
 */
#define TMR_PWMDisable()           (R8_TMR_CTRL_MOD &= ~RB_TMR_OUT_EN)

/**
 * @brief   开启 TMR PWM输出
 */
#define TMR_PWMEnable()           (R8_TMR_CTRL_MOD |= RB_TMR_OUT_EN)

/**
 * @brief   关闭 TMR
 */
#define TMR_Disable()           (R8_TMR_CTRL_MOD &= ~RB_TMR_COUNT_EN)

/**
 * @brief   开启 TMR
 */
#define TMR_Enable()            (R8_TMR_CTRL_MOD |= RB_TMR_COUNT_EN)

/**
 * @brief   TMR中断配置
 *
 * @param   s       - 使能/关闭
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR_ITCfg(s, f)         ((s) ? (R8_TMR_INTER_EN |= f) : (R8_TMR_INTER_EN &= ~f))

/**
 * @brief   清除TMR中断标志
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR_ClearITFlag(f)      (R8_TMR_INT_FLAG = f)

/**
 * @brief   查询中断标志状态
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR_GetITFlag(f)        (R8_TMR_INT_FLAG & f)

/**
 * @brief   获取编码器当前方向
 *
 * @return  方向值  0:前进  1:后退
 */
#define ENC_GetCurrentDir       (R8_ENC_REG_CTRL>>5 & 0x01)

/**
 * @brief   获取编码器当前计数值
 */
#define ENC_GetCurrentCount      R32_ENC_REG_CCNT

/**
 * @brief   编码器模式读计数并清0
 */
#define ENC_GetCountandReset()  R8_ENC_REG_CTRL |= RB_RD_CLR_EN

/**
 * @brief   ENC中断配置
 *
 * @param   s       - 使能/关闭
 * @param   f       - refer to ENC interrupt bit define
 */
#define ENC_ITCfg(s, f)         ((s) ? (R8_ENC_INTER_EN |= f) : (R8_ENC_INTER_EN &= ~f))

/**
 * @brief   清除ENC中断标志
 *
 * @param   f       - refer to ENC interrupt bit define
 */
#define ENC_ClearITFlag(f)      (R8_ENC_INT_FLAG = f)

/**
 * @brief   查询中断标志状态
 *
 * @param   f       - refer to ENC interrupt bit define
 */
#define ENC_GetITFlag(f)        (R8_ENC_INT_FLAG & f)

#ifdef __cplusplus
}
#endif

#endif // __CH57x_TIMER_H__
