/********************************** (C) COPYRIGHT *******************************
 * File Name          : beacon_main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/17
 * Description        : BLE Beacon Main Application Entry Point
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CONFIG.h"
#include "HAL.h"
#include "beacon.h"
#include "shtxx_sensor.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x03};
#endif

/* UART RX buffer for configuration */
#define UART_CFG_BUF_SIZE  256
static uint8_t uart_cfg_buf[UART_CFG_BUF_SIZE];
static uint16_t uart_cfg_idx = 0;

/*********************************************************************
 * @fn      UART_IRQHandler
 *
 * @brief   UART interrupt handler for configuration
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void UART_IRQHandler(void)
{
    volatile uint8_t i;
    uint8_t rx_byte;

    switch(UART_GetITFlag())
    {
        case UART_II_LINE_STAT:
            UART_GetLinSTA();
            break;

        case UART_II_RECV_RDY:
            // Receive byte
            rx_byte = UART_RecvByte();

            // Store in buffer
            if(uart_cfg_idx < UART_CFG_BUF_SIZE)
            {
                uart_cfg_buf[uart_cfg_idx++] = rx_byte;
            }
            else
            {
                uart_cfg_idx = 0;  // Buffer overflow, reset
            }
            break;

        case UART_II_RECV_TOUT:
            // Timeout: process received data
            if(uart_cfg_idx > 0)
            {
                Beacon_UartRxCallback(uart_cfg_buf, uart_cfg_idx);
                uart_cfg_idx = 0;
            }
            break;

        case UART_II_THR_EMPTY:
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   Main loop
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline))
void Main_Circulation()
{
    while(1)
    {
        TMOS_SystemProcess();
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main function
 *
 * @return  none
 */
int main(void)
{
    // Disable debug pins (to free up pins for I2C)
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;

    // Configure oscillator
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);

#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif

    // Initialize UART for configuration
    GPIOA_SetBits(bTXD_0);
    GPIOA_ModeCfg(bRXD_0, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(bTXD_0, GPIO_ModeOut_PP_5mA);
    UART_Remap(ENABLE, UART_TX_REMAP_PA3, UART_RX_REMAP_PA2);
    UART_DefInit();

    // Enable UART interrupts for configuration
    UART_ByteTrigCfg(UART_1BYTE_TRIG);
    UART_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT | RB_IER_RECV_TOUT);
    PFIC_EnableIRQ(UART_IRQn);

    PRINT("\n\n");
    PRINT("*****************************************\n");
    PRINT("* WCH CH570 BLE Beacon with SHTXX      *\n");
    PRINT("* Version: %s                   *\n", VER_LIB);
    PRINT("*****************************************\n");
    PRINT("\n");

    // Initialize BLE stack
    CH57x_BLEInit();
    HAL_Init();

    // Initialize GAP Broadcaster Role
    GAPRole_BroadcasterInit();

    // Initialize Beacon Application
    Beacon_Init();

    PRINT("System initialized. Ready for configuration via UART.\n");
    PRINT("UART: PA3=TX, PA2=RX, 115200 baud\n");
    PRINT("I2C:  PA8=SDA, PA9=SCL\n");
    PRINT("\n");

    // Main loop
    Main_Circulation();
}

/******************************** endfile @ main ******************************/
