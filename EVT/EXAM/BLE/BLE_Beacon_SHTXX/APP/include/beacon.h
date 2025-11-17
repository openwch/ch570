/********************************** (C) COPYRIGHT *******************************
 * File Name          : beacon.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/17
 * Description        : BLE Beacon Application with SHTXX Sensor Integration
 *                      Supports Eddystone-UID, URL, and TLM formats
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __BEACON_H__
#define __BEACON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "CONFIG.h"

/*********************************************************************
 * CONSTANTS
 */

/* Beacon modes */
#define BEACON_MODE_EDDYSTONE_UID   0   // Eddystone-UID
#define BEACON_MODE_EDDYSTONE_URL   1   // Eddystone-URL
#define BEACON_MODE_EDDYSTONE_TLM   2   // Eddystone-TLM (with sensor data)
#define BEACON_MODE_IBEACON         3   // Apple iBeacon
#define BEACON_MODE_SENSOR_CUSTOM   4   // Custom format with sensor data

/* Configuration defaults */
#define DEFAULT_BEACON_MODE         BEACON_MODE_EDDYSTONE_UID
#define DEFAULT_TX_POWER            0   // 0 dBm
#define DEFAULT_ADV_INTERVAL        1600 // 1000ms (units of 625us)
#define DEFAULT_SENSOR_ENABLED      1   // Enable sensor by default

/* Eddystone service UUID */
#define EDDYSTONE_SERVICE_UUID      0xFEAA

/* Configuration magic number for validation */
#define CONFIG_MAGIC                0x57434848  // "WCH"

/* UART configuration commands */
#define CMD_GET_CONFIG              0x01
#define CMD_SET_CONFIG              0x02
#define CMD_SAVE_CONFIG             0x03
#define CMD_RESET_CONFIG            0x04
#define CMD_GET_SENSOR              0x05
#define CMD_START_BEACON            0x10
#define CMD_STOP_BEACON             0x11

/* Response codes */
#define RESP_OK                     0x00
#define RESP_ERROR                  0xFF
#define RESP_INVALID_CMD            0xFE
#define RESP_INVALID_PARAM          0xFD

/*********************************************************************
 * TYPEDEFS
 */

/* Beacon configuration structure */
typedef struct {
    uint32_t magic;                 // Magic number for validation
    uint8_t  beacon_mode;           // Beacon mode (see BEACON_MODE_xxx)
    uint8_t  sensor_enabled;        // Enable sensor data in broadcast
    int8_t   tx_power;              // TX power in dBm
    uint16_t adv_interval;          // Advertising interval (units of 625us)

    // Eddystone-UID parameters
    uint8_t  namespace_id[10];      // 10-byte namespace ID
    uint8_t  instance_id[6];        // 6-byte instance ID

    // Eddystone-URL parameters
    uint8_t  url_scheme;            // URL scheme (0=http://, 1=https://, etc.)
    uint8_t  url[17];               // Encoded URL (max 17 bytes)
    uint8_t  url_length;            // URL length

    // iBeacon parameters
    uint8_t  ibeacon_uuid[16];      // 16-byte proximity UUID
    uint16_t ibeacon_major;         // Major version
    uint16_t ibeacon_minor;         // Minor version

    // Sensor configuration
    uint8_t  sensor_update_rate;    // Sensor update rate in seconds
    uint8_t  shtxx_addr;            // SHTXX I2C address
    uint8_t  shtxx_repeatability;   // Measurement repeatability

    uint8_t  reserved[8];           // Reserved for future use
} beacon_config_t;

/* Beacon state */
typedef enum {
    BEACON_STATE_IDLE,
    BEACON_STATE_INITIALIZING,
    BEACON_STATE_ADVERTISING,
    BEACON_STATE_STOPPED,
    BEACON_STATE_ERROR,
    BEACON_STATE_CONFIG_MODE
} beacon_state_t;

/* Sensor broadcast data */
typedef struct {
    int16_t  temperature;           // Temperature in 0.01°C
    uint16_t humidity;              // Humidity in 0.01%
    uint16_t battery_voltage;       // Battery voltage in mV
    uint32_t uptime;                // Uptime in seconds
} sensor_broadcast_t;

/*********************************************************************
 * EVENTS
 */
#define SBP_START_DEVICE_EVT            0x0001
#define SBP_PERIODIC_EVT                0x0002
#define SBP_SENSOR_READ_EVT             0x0004
#define SBP_CONFIG_TIMEOUT_EVT          0x0008
#define SBP_UPDATE_ADV_DATA_EVT         0x0010

/*********************************************************************
 * FUNCTIONS
 */

/**
 * @brief   Initialize Beacon Application
 */
void Beacon_Init(void);

/**
 * @brief   Beacon Application Task event processor
 * @param   task_id - The TMOS assigned task ID
 * @param   events - events to process
 * @return  events not processed
 */
uint16_t Beacon_ProcessEvent(uint8_t task_id, uint16_t events);

/**
 * @brief   Get current beacon configuration
 * @param   config - Pointer to store configuration
 */
void Beacon_GetConfig(beacon_config_t *config);

/**
 * @brief   Set beacon configuration
 * @param   config - Pointer to new configuration
 * @return  0 on success, negative on error
 */
int Beacon_SetConfig(beacon_config_t *config);

/**
 * @brief   Save configuration to flash
 * @return  0 on success, negative on error
 */
int Beacon_SaveConfig(void);

/**
 * @brief   Load configuration from flash
 * @return  0 on success, negative on error
 */
int Beacon_LoadConfig(void);

/**
 * @brief   Reset configuration to defaults
 */
void Beacon_ResetConfig(void);

/**
 * @brief   Start beacon advertising
 * @return  0 on success, negative on error
 */
int Beacon_Start(void);

/**
 * @brief   Stop beacon advertising
 */
void Beacon_Stop(void);

/**
 * @brief   Update advertising data with current sensor values
 */
void Beacon_UpdateAdvData(void);

/**
 * @brief   Get current sensor data
 * @param   data - Pointer to store sensor data
 * @return  0 on success, negative on error
 */
int Beacon_GetSensorData(sensor_broadcast_t *data);

/**
 * @brief   UART RX callback for configuration
 * @param   data - Received data buffer
 * @param   len - Data length
 */
void Beacon_UartRxCallback(uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BEACON_H__ */
