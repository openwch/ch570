/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH57x_cmp.c
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
 * @fn      CMP_Init
 *
 * @brief   Comparator initialization
 *
 * @param   s   - comparator channel input selection, refer to CMPSwTypeDef
 * @param   v   - comparator negative end Vref level, refer to CMPNrefLevelTypeDef
 *
 * @return  none
 */
void CMP_Init (CMPSwTypeDef s, CMPNrefLevelTypeDef v)
{
    R8_CMP_CTRL_0 = (R8_CMP_CTRL_0 & ~((0x03 << 2) | (0x0F << 4))) |
        ((s & 0x03) << 2) |
        ((v & 0x0F) << 4);
}

/*********************************************************************
 * @fn      CMP_OutToTIMCAPCfg
 *
 * @brief   Enable/disable comparator output to TIM capture
 *
 * @param   s   - ENABLE or DISABLE
 *
 * @return  none
 */
void CMP_OutToTIMCAPCfg(FunctionalState s)
{
    if (s)
    {
        R8_CMP_CTRL_0 |= RB_CMP_CAP;
    }
    else
    {
        R8_CMP_CTRL_0 &= ~RB_CMP_CAP;
    }
}

/*********************************************************************
 * @fn      CMP_INTCfg
 *
 * @brief   Comparator interrupt configuration
 *
 * @param   sel - comparator output selection, refer to CMPOutSelTypeDef
 * @param   s   - ENABLE or DISABLE
 *
 * @return  none
 */
void CMP_INTCfg (CMPOutSelTypeDef sel, FunctionalState s)
{
    R8_CMP_CTRL_1 = (R8_CMP_CTRL_1 & ~(0x03 << 2)) | ((uint8_t)sel << 2);
    if (s)
    {
        R8_CMP_CTRL_1 |= RB_CMP_IE;
    }
    else
    {
        R8_CMP_CTRL_1 &= ~RB_CMP_IE;
    }
}

