/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/02/18
 * Description 		  : ADC功能演示
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"

/*********************************************************************
 * @fn      ADC_8bit_Sample
 *
 * @brief   PWM模拟8位ADC采样,采样脚(PA2)采样范围（0 ~ (VDD-800)mV）
 *
 * @return  采样值，对应电压的计算公式为：（采样值*VDD/512）
 */
uint32_t ADC_8bit_Sample()
{
    uint32_t vref=0;
    uint32_t cmp_count=1;
    /* 配置GPIO */
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);  // PA4 - PWM4
    PWM_16bit_CycleDisable();
    PWMX_CLKCfg(1);                                  // cycle = Fsys
    PWMX_CycleCfg(PWMX_Cycle_256);
    CMP_Init( cmp_sw_2, cmp_nref_level_800 );// CMP_P0(PA7),CMP_N(PA2)
    CMP_Enable();
    vref = 128;
    while(1)
    {
        uint8_t cmp_out=0;
        uint8_t cmp_out_1ast=0;
        PWMX_ACTOUT(CH_PWM4, vref, High_Level, ENABLE);

        DelayUs(760);
        cmp_out_1ast = CMP_ReadAPROut();
        while(1)
        {
            DelayUs(20);
            cmp_out = CMP_ReadAPROut();
            if(cmp_out == cmp_out_1ast)
            {
                break;
            }
            cmp_out_1ast = cmp_out;
        }

        if(cmp_count==128)
        {
            if (cmp_out)
            {
                return (vref*2-1);
            }
            else
            {
                return (vref*2+1);
            }
        }
        else
        {
            cmp_count<<=1;
            if (cmp_out)
            {
                vref -= 128/cmp_count;
            }
            else
            {
                vref += 128/cmp_count;
            }
        }
    }
}

/*********************************************************************
 * @fn      ADC_9bit_Sample
 *
 * @brief   PWM模拟9位ADC采样,采样脚(PA2)采样范围（0 ~ (VDD-800)mV）
 *
 * @return  采样值，对应电压的计算公式为：（采样值*VDD/1024）
 */
uint32_t ADC_9bit_Sample()
{
    uint32_t vref=0;
    uint32_t cmp_count=1;
    /* 配置GPIO */
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);  // PA4 - PWM4
    PWMX_CLKCfg(1);                                  // cycle = Fsys
    PWM_16bit_CycleEnable();                          // 16位宽使能
    PWMX_16bit_CycleCfg(CH_PWM4, 512);           // 16 数据宽度时的PWM时钟周期
    CMP_Init( cmp_sw_2, cmp_nref_level_800 );// CMP_P0(PA7),CMP_N(PA2)
    CMP_Enable();
    vref = 256;
    while(1)
    {
        uint8_t cmp_out=0;
        uint8_t cmp_out_1ast=0;
        PWMX_16bit_ACTOUT(CH_PWM4, vref, High_Level, ENABLE);
        DelayUs(230);
        cmp_out_1ast = CMP_ReadAPROut();
        while(1)
        {
            DelayUs(20);
            cmp_out = CMP_ReadAPROut();
            if(cmp_out == cmp_out_1ast)
            {
                break;
            }
            cmp_out_1ast = cmp_out;
        }

        if(cmp_count==256)
        {
            if (cmp_out)
            {
                return (vref*2-1);
            }
            else
            {
                return (vref*2+1);
            }
        }
        else
        {
            cmp_count<<=1;
            if (cmp_out)
            {
                vref -= 256/cmp_count;
            }
            else
            {
                vref += 256/cmp_count;
            }
        }
    }
}

/*********************************************************************
 * @fn      ADC_4bit_Sample
 *
 * @brief   CMP模拟4位ADC采样,采样脚(PA7)采样范围（0~825mV）
 *
 * @return  采样值，对应电压的计算公式为：（采样值*25mV）
 */
uint32_t ADC_4bit_Sample()
{
    uint32_t vref=0;
    uint32_t cmp_count=1;
    vref = 8;
    while(1)
    {
        uint8_t cmp_out=0;
        uint8_t cmp_out_1ast=0;
        CMP_Init( cmp_sw_3, vref-1 );// CMP_P0(PA7),CMP_N(CMP_VERF)
        CMP_Enable();
        DelayUs(1);
        cmp_out_1ast = CMP_ReadAPROut();
        while(1)
        {
            cmp_out = CMP_ReadAPROut();
            if(cmp_out == cmp_out_1ast)
            {
                break;
            }
            cmp_out_1ast = cmp_out;
        }

        if(cmp_count==8)
        {
            if (cmp_out)
            {
                if(vref==15)
                {
                    vref = 16;
                }
                else
                {
                    return (vref*2+1);
                }
            }
            else
            {
                return (vref*2-1);
            }
        }
        else
        {
            cmp_count<<=1;
            if (cmp_out)
            {
                vref += 8/cmp_count;
            }
            else
            {
                vref -= 8/cmp_count;
            }
        }
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main()
{
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);
    /* 配置串口1：先配置IO口模式，再配置串口 */
    GPIOA_SetBits( bTXD_0 );
    GPIOA_ModeCfg( bTXD_0, GPIO_ModeOut_PP_5mA );          // TXD-配置推挽输出，注意先让IO口输出高电平
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();
    PRINT( "Start @ChipID=%02X\n", R8_CHIP_ID );

    PRINT ("ADC_9bit_Sample %d\n",ADC_9bit_Sample()*3300/1024);
    PRINT ("ADC_8bit_Sample %d\n",ADC_8bit_Sample()*3300/512);
    PRINT ("ADC_4bit_Sample %d\n",ADC_4bit_Sample()*25);

    while(1);
}
