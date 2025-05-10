/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_gpio.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH57x_GPIO_H__
#define __CH57x_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	GPIO_pins_define
 */
#define GPIO_Pin_0      (0x00000001) /*!< Pin 0 selected */
#define GPIO_Pin_1      (0x00000002) /*!< Pin 1 selected */
#define GPIO_Pin_2      (0x00000004) /*!< Pin 2 selected */
#define GPIO_Pin_3      (0x00000008) /*!< Pin 3 selected */
#define GPIO_Pin_4      (0x00000010) /*!< Pin 4 selected */
#define GPIO_Pin_5      (0x00000020) /*!< Pin 5 selected */
#define GPIO_Pin_6      (0x00000040) /*!< Pin 6 selected */
#define GPIO_Pin_7      (0x00000080) /*!< Pin 7 selected */
#define GPIO_Pin_8      (0x00000100) /*!< Pin 8 selected */
#define GPIO_Pin_9      (0x00000200) /*!< Pin 9 selected */
#define GPIO_Pin_10     (0x00000400) /*!< Pin 10 selected */
#define GPIO_Pin_11     (0x00000800) /*!< Pin 11 selected */
#define GPIO_Pin_12     (0x00001000) /*!< Pin 12 selected */
#define GPIO_Pin_13     (0x00002000) /*!< Pin 13 selected */
#define GPIO_Pin_14     (0x00004000) /*!< Pin 14 selected */
#define GPIO_Pin_15     (0x00008000) /*!< Pin 15 selected */
#define GPIO_Pin_16     (0x00010000) /*!< Pin 16 selected */
#define GPIO_Pin_17     (0x00020000) /*!< Pin 17 selected */
#define GPIO_Pin_18     (0x00040000) /*!< Pin 18 selected */
#define GPIO_Pin_19     (0x00080000) /*!< Pin 19 selected */
#define GPIO_Pin_20     (0x00100000) /*!< Pin 20 selected */
#define GPIO_Pin_21     (0x00200000) /*!< Pin 21 selected */
#define GPIO_Pin_22     (0x00400000) /*!< Pin 22 selected */
#define GPIO_Pin_23     (0x00800000) /*!< Pin 23 selected */
#define GPIO_Pin_All    (0xFFFFFFFF) /*!< All pins selected */

/**
 * @brief   GPIO_pins_remap_define
 */
#define REMAP_RXD_PA2   0x00  /*!<默认映射（RXD/PA2） */
#define REMAP_RXD_PA3   0x01  /*!<重映射（RXD/PA3） */
#define REMAP_RXD_PA0   0x02  /*!<重映射（RXD/PA0） */
#define REMAP_RXD_PA1   0x03  /*!<重映射（RXD/PA1） */
#define REMAP_RXD_PA4   0x04  /*!<重映射（RXD/PA4） */
#define REMAP_RXD_PA9   0x05  /*!<重映射（RXD/PA9） */
#define REMAP_RXD_PA10  0x06  /*!<重映射（RXD/PA10） */
#define REMAP_RXD_PA11  0x07  /*!<重映射（RXD/PA11） */

#define REMAP_TXD_PA3   0x00  /*!<默认映射（TXD/PA3） */
#define REMAP_TXD_PA2   0x08  /*!<重映射（TXD/PA2） */
#define REMAP_TXD_PA1   0x10  /*!<重映射（TXD/PA1） */
#define REMAP_TXD_PA0   0x18  /*!<重映射（TXD/PA0） */
#define REMAP_TXD_PA7   0x20  /*!<重映射（TXD/PA7） */
#define REMAP_TXD_PA8   0x28  /*!<重映射（TXD/PA8） */
#define REMAP_TXD_PA11  0x30  /*!<重映射（TXD/PA11） */
#define REMAP_TXD_PA10  0x38  /*!<重映射（TXD/PA10） */

#define REMAP_TMR_DEFAULT   0x00   /*!<默认映射（PWM0/PA7，CAP_IN1/PA7，CAP_IN2/PA2） */
#define REMAP_TMR_MODE1     0x40   /*!<重映射1 （PWM0/PA2，CAP_IN1/PA2，CAP_IN2/PA7） */
#define REMAP_TMR_MODE2     0x80   /*!<重映射2（PWM0/PA4，CAP_IN1/PA4，CAP_IN2/PA9） */
#define REMAP_TMR_MODE3     0xC0   /*!<重映射3（PWM0/PA9，CAP_IN1/PA9，CAP_IN2/PA4） */

#define REMAP_I2C_DEFAULT   0x00  /*!<默认映射（SCL/PA8，SDA/PA9） */
#define REMAP_I2C_MODE1     0x200 /*!<重映射1 （SCL/PA0，SDA/PA1） */
#define REMAP_I2C_MODE2     0x400 /*!<重映射2 （SCL/PA3，SDA/PA2） */
#define REMAP_I2C_MODE3     0x600 /*!<重映射3 （SCL/PA5，SDA/PA6） */

/**
 * @brief  Configuration GPIO Mode
 */
typedef enum
{
    GPIO_ModeIN_Floating, //浮空输入
    GPIO_ModeIN_PU,       //上拉输入
    GPIO_ModeIN_PD,       //下拉输入
    GPIO_ModeOut_PP_5mA,  //推挽输出最大5mA
    GPIO_ModeOut_PP_20mA, //推挽输出最大20mA

} GPIOModeTypeDef;

/**
 * @brief  Configuration GPIO IT Mode
 */
typedef enum
{
    GPIO_ITMode_LowLevel,  //低电平触发
    GPIO_ITMode_HighLevel, //高电平触发
    GPIO_ITMode_FallEdge,  //下降沿触发
    GPIO_ITMode_RiseEdge,  //上升沿触发

} GPIOITModeTpDef;

/**
 * @brief   GPIOA端口引脚模式配置
 *
 * @param   pin     - PA0-PA15
 * @param   mode    - 输入输出类型
 */
void GPIOA_ModeCfg(uint32_t pin, GPIOModeTypeDef mode);

/**
 * @brief   GPIOA端口引脚输出置低
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_ResetBits(pin)      (R32_PA_CLR = pin)

/**
 * @brief   GPIOA端口引脚输出置高
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_SetBits(pin)        (R32_PA_SET = pin)

/**
 * @brief   GPIOA端口引脚输出电平翻转
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_InverseBits(pin)    (R32_PA_OUT ^= pin)

/**
 * @brief   GPIOA端口32位数据返回，低16位有效
 *
 * @return  GPIOA端口32位数据
 */
#define GPIOA_ReadPort()          (R32_PA_PIN)

/**
 * @brief   GPIOA端口引脚状态，0-引脚低电平，(!0)-引脚高电平
 *
 * @param   pin     - PA0-PA15
 *
 * @return  GPIOA端口引脚状态
 */
#define GPIOA_ReadPortPin(pin)    (R32_PA_PIN & (pin))

/**
 * @brief   GPIOA引脚中断模式配置
 *
 * @param   pin     - PA0-PA15
 * @param   mode    - 触发类型
 */
void GPIOA_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode);

/**
 * @brief   读取GPIOA端口中断标志状态
 *
 * @return  GPIOA端口中断标志状态
 */
#define GPIOA_ReadITFlagPort()       (R16_PA_INT_IF)

/**
 * @brief   读取GPIOA端口引脚中断标志状态
 *
 * @param   pin     - PA0-PA15
 *
 * @return  GPIOA端口引脚中断标志状态
 */
#define GPIOA_ReadITFlagBit(pin)     (R16_PA_INT_IF & (pin))

/**
 * @brief   清除GPIOA端口引脚中断标志状态
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_ClearITFlagBit(pin)    (R16_PA_INT_IF = pin)

/**
 * @brief   外设功能引脚映射
 *
 * @param   s       - 是否使能映射
 * @param   perph   - 写具体的映射关系，详见GPIO_pins_remap_define
 */
void GPIOPinRemap(FunctionalState s, uint16_t perph);

/**
 * @brief   I/O pin数字功能控制
 *
 * @param   s       - 是否打开对应I/O pin数字功能
 * @param   pin     - PA0-PA15
 */
void GPIOADigitalCfg(FunctionalState s, uint16_t pin);


#ifdef __cplusplus
}
#endif

#endif // __CH57x_GPIO_H__
