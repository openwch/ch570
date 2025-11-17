/********************************** (C) COPYRIGHT *******************************
 * File Name          : shtxx_sensor.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/17
 * Description        : SHTXX (SHT30/SHT31/SHT35) Temperature & Humidity Sensor Driver
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __SHTXX_SENSOR_H__
#define __SHTXX_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "CH57x_common.h"

/*********************************************************************
 * CONSTANTS
 */

/* SHTxx I2C Address */
#define SHTXX_ADDR_DEFAULT      0x44    // Default I2C address (ADDR pin = LOW)
#define SHTXX_ADDR_ALT          0x45    // Alternate address (ADDR pin = HIGH)

/* SHTxx Commands */
#define SHTXX_CMD_MEASURE_HIGH  0x2C06  // High repeatability measurement
#define SHTXX_CMD_MEASURE_MED   0x2C0D  // Medium repeatability measurement
#define SHTXX_CMD_MEASURE_LOW   0x2C10  // Low repeatability measurement
#define SHTXX_CMD_READ_STATUS   0xF32D  // Read status register
#define SHTXX_CMD_CLEAR_STATUS  0x3041  // Clear status register
#define SHTXX_CMD_SOFT_RESET    0x30A2  // Soft reset
#define SHTXX_CMD_HEATER_EN     0x306D  // Enable heater
#define SHTXX_CMD_HEATER_DIS    0x3066  // Disable heater

/* Measurement timeout (ms) */
#define SHTXX_TIMEOUT_MS        100

/*********************************************************************
 * TYPEDEFS
 */

/* Measurement data structure */
typedef struct {
    int16_t  temperature_raw;   // Raw temperature value
    uint16_t humidity_raw;      // Raw humidity value
    float    temperature_c;     // Temperature in Celsius
    float    humidity_rh;       // Relative humidity in %
    uint8_t  crc_valid;         // CRC check result
} shtxx_data_t;

/* Sensor configuration */
typedef struct {
    uint8_t i2c_addr;          // I2C address (7-bit)
    uint8_t repeatability;     // Measurement repeatability (0=low, 1=med, 2=high)
    uint8_t heater_enabled;    // Heater state
} shtxx_config_t;

/*********************************************************************
 * FUNCTIONS
 */

/**
 * @brief   Initialize SHTXX sensor
 * @param   config - Pointer to sensor configuration
 * @return  0 on success, negative error code on failure
 */
int shtxx_init(shtxx_config_t *config);

/**
 * @brief   Perform soft reset of sensor
 * @return  0 on success, negative error code on failure
 */
int shtxx_soft_reset(void);

/**
 * @brief   Read temperature and humidity
 * @param   data - Pointer to data structure to store results
 * @return  0 on success, negative error code on failure
 */
int shtxx_read_measurement(shtxx_data_t *data);

/**
 * @brief   Enable/disable heater
 * @param   enable - 1 to enable, 0 to disable
 * @return  0 on success, negative error code on failure
 */
int shtxx_set_heater(uint8_t enable);

/**
 * @brief   Read status register
 * @param   status - Pointer to store status value
 * @return  0 on success, negative error code on failure
 */
int shtxx_read_status(uint16_t *status);

/**
 * @brief   Calculate CRC-8 for data validation
 * @param   data - Pointer to data buffer
 * @param   len - Length of data
 * @return  Calculated CRC-8 value
 */
uint8_t shtxx_calculate_crc8(const uint8_t *data, uint8_t len);

/**
 * @brief   Convert raw values to physical units
 * @param   data - Pointer to data structure
 */
void shtxx_convert_values(shtxx_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __SHTXX_SENSOR_H__ */
