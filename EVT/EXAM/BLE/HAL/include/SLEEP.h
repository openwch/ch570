/********************************** (C) COPYRIGHT *******************************
 * File Name          : SLEEP.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/11/12
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#ifndef __SLEEP_H
#define __SLEEP_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */
typedef void (*pfnLowPowerGapProcessCB_t)( void );

extern uint16_t LSIWakeup_MaxTime;
/*********************************************************************
 * FUNCTIONS
 */

/**
 * @brief   ����˯�߻��ѵķ�ʽ   - RTC���ѣ�����ģʽ
 */
extern void HAL_SleepInit(void);

/**
 * @brief   ����˯��
 *
 * @param   time    - ���ѵ�ʱ��㣨RTC����ֵ��
 *
 * @return  state.
 */
extern uint32_t CH57x_LowPower(uint32_t time);

/**
 * @brief   ��ȡ��ǰ��ǰ����ʱ��
 *
 * @param   none.
 */
extern uint16_t GET_WakeUpLSIMaxTime(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
