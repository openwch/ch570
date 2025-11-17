/********************************** (C) COPYRIGHT *******************************
 * File Name          : beacon.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/17
 * Description        : BLE Beacon Application with SHTXX Sensor Integration
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "beacon.h"
#include "shtxx_sensor.h"
#include "CH57x_common.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t Beacon_TaskID = INVALID_TASK_ID;
static beacon_state_t beacon_state = BEACON_STATE_IDLE;
static beacon_config_t beacon_config;
static sensor_broadcast_t sensor_data;
static shtxx_data_t shtxx_data;

/* Flash address for configuration storage */
#define CONFIG_FLASH_ADDR   (FLASH_ROM_MAX_SIZE - EEPROM_BLOCK_SIZE)

/* Advertising data buffers */
static uint8_t advertData[31];
static uint8_t advertDataLen = 0;

/* UART RX buffer */
#define UART_RX_BUF_SIZE    256
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint16_t uart_rx_index = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Beacon_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void Beacon_BuildAdvData(void);
static void Beacon_BuildEddystoneUID(void);
static void Beacon_BuildEddystoneURL(void);
static void Beacon_BuildEddystoneTLM(void);
static void Beacon_BuildiBeacon(void);
static void Beacon_BuildSensorCustom(void);
static void Beacon_ReadSensor(void);
static void Beacon_ProcessUartData(uint8_t *data, uint16_t len);
static void Beacon_SendUartResponse(uint8_t cmd, uint8_t status, uint8_t *data, uint16_t len);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @brief   Reset configuration to defaults
 */
void Beacon_ResetConfig(void)
{
    // Clear configuration
    tmos_memset(&beacon_config, 0, sizeof(beacon_config_t));

    // Set defaults
    beacon_config.magic = CONFIG_MAGIC;
    beacon_config.beacon_mode = DEFAULT_BEACON_MODE;
    beacon_config.sensor_enabled = DEFAULT_SENSOR_ENABLED;
    beacon_config.tx_power = DEFAULT_TX_POWER;
    beacon_config.adv_interval = DEFAULT_ADV_INTERVAL;

    // Default Eddystone-UID
    uint8_t default_namespace[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    uint8_t default_instance[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
    tmos_memcpy(beacon_config.namespace_id, default_namespace, 10);
    tmos_memcpy(beacon_config.instance_id, default_instance, 6);

    // Default Eddystone-URL: "https://wch.cn"
    beacon_config.url_scheme = 1;  // https://
    const char url_enc[] = "wch\x00";  // .cn/com is encoded as 0x00
    beacon_config.url_length = sizeof(url_enc);
    tmos_memcpy(beacon_config.url, url_enc, sizeof(url_enc));

    // Sensor configuration
    beacon_config.sensor_update_rate = 5;  // 5 seconds
    beacon_config.shtxx_addr = SHTXX_ADDR_DEFAULT;
    beacon_config.shtxx_repeatability = 2;  // High repeatability

    PRINT("Beacon: Configuration reset to defaults\n");
}

/**
 * @brief   Load configuration from flash
 */
int Beacon_LoadConfig(void)
{
    beacon_config_t *flash_config = (beacon_config_t *)CONFIG_FLASH_ADDR;

    // Check magic number
    if(flash_config->magic == CONFIG_MAGIC)
    {
        tmos_memcpy(&beacon_config, flash_config, sizeof(beacon_config_t));
        PRINT("Beacon: Configuration loaded from flash\n");
        return 0;
    }
    else
    {
        PRINT("Beacon: No valid configuration in flash, using defaults\n");
        Beacon_ResetConfig();
        return -1;
    }
}

/**
 * @brief   Save configuration to flash
 */
int Beacon_SaveConfig(void)
{
    // Erase flash page
    FLASH_ROM_ERASE(CONFIG_FLASH_ADDR, EEPROM_BLOCK_SIZE);

    // Write configuration
    FLASH_ROM_WRITE(CONFIG_FLASH_ADDR, &beacon_config, sizeof(beacon_config_t));

    PRINT("Beacon: Configuration saved to flash\n");
    return 0;
}

/**
 * @brief   Get current beacon configuration
 */
void Beacon_GetConfig(beacon_config_t *config)
{
    if(config != NULL)
    {
        tmos_memcpy(config, &beacon_config, sizeof(beacon_config_t));
    }
}

/**
 * @brief   Set beacon configuration
 */
int Beacon_SetConfig(beacon_config_t *config)
{
    if(config == NULL)
    {
        return -1;
    }

    // Validate configuration
    if(config->magic != CONFIG_MAGIC)
    {
        PRINT("Beacon: Invalid configuration magic\n");
        return -2;
    }

    // Copy configuration
    tmos_memcpy(&beacon_config, config, sizeof(beacon_config_t));

    // Update advertising data if already advertising
    if(beacon_state == BEACON_STATE_ADVERTISING)
    {
        Beacon_UpdateAdvData();
    }

    PRINT("Beacon: Configuration updated\n");
    return 0;
}

/**
 * @brief   Build Eddystone-UID advertising data
 */
static void Beacon_BuildEddystoneUID(void)
{
    uint8_t idx = 0;

    // Flags
    advertData[idx++] = 0x02;  // Length
    advertData[idx++] = 0x01;  // Type: Flags
    advertData[idx++] = 0x06;  // Flags: General Discoverable + BR/EDR Not Supported

    // Complete list of 16-bit Service UUIDs
    advertData[idx++] = 0x03;  // Length
    advertData[idx++] = 0x03;  // Type: Complete list of 16-bit UUIDs
    advertData[idx++] = 0xAA;  // Eddystone UUID (0xFEAA)
    advertData[idx++] = 0xFE;

    // Service Data
    advertData[idx++] = 0x17;  // Length (23 bytes)
    advertData[idx++] = 0x16;  // Type: Service Data
    advertData[idx++] = 0xAA;  // Eddystone UUID
    advertData[idx++] = 0xFE;
    advertData[idx++] = 0x00;  // Frame Type: UID

    advertData[idx++] = beacon_config.tx_power;  // Calibrated TX power

    // Namespace ID (10 bytes)
    tmos_memcpy(&advertData[idx], beacon_config.namespace_id, 10);
    idx += 10;

    // Instance ID (6 bytes)
    tmos_memcpy(&advertData[idx], beacon_config.instance_id, 6);
    idx += 6;

    // RFU (2 bytes)
    advertData[idx++] = 0x00;
    advertData[idx++] = 0x00;

    advertDataLen = idx;
}

/**
 * @brief   Build Eddystone-URL advertising data
 */
static void Beacon_BuildEddystoneURL(void)
{
    uint8_t idx = 0;

    // Flags
    advertData[idx++] = 0x02;
    advertData[idx++] = 0x01;
    advertData[idx++] = 0x06;

    // Complete list of 16-bit Service UUIDs
    advertData[idx++] = 0x03;
    advertData[idx++] = 0x03;
    advertData[idx++] = 0xAA;
    advertData[idx++] = 0xFE;

    // Service Data
    uint8_t service_data_len = 4 + beacon_config.url_length;
    advertData[idx++] = service_data_len + 1;  // Length
    advertData[idx++] = 0x16;  // Type: Service Data
    advertData[idx++] = 0xAA;  // Eddystone UUID
    advertData[idx++] = 0xFE;
    advertData[idx++] = 0x10;  // Frame Type: URL

    advertData[idx++] = beacon_config.tx_power;  // TX power
    advertData[idx++] = beacon_config.url_scheme;  // URL Scheme

    // URL
    tmos_memcpy(&advertData[idx], beacon_config.url, beacon_config.url_length);
    idx += beacon_config.url_length;

    advertDataLen = idx;
}

/**
 * @brief   Build Eddystone-TLM advertising data with sensor data
 */
static void Beacon_BuildEddystoneTLM(void)
{
    uint8_t idx = 0;

    // Flags
    advertData[idx++] = 0x02;
    advertData[idx++] = 0x01;
    advertData[idx++] = 0x06;

    // Complete list of 16-bit Service UUIDs
    advertData[idx++] = 0x03;
    advertData[idx++] = 0x03;
    advertData[idx++] = 0xAA;
    advertData[idx++] = 0xFE;

    // Service Data
    advertData[idx++] = 0x11;  // Length (17 bytes)
    advertData[idx++] = 0x16;  // Type: Service Data
    advertData[idx++] = 0xAA;  // Eddystone UUID
    advertData[idx++] = 0xFE;
    advertData[idx++] = 0x20;  // Frame Type: TLM

    advertData[idx++] = 0x00;  // TLM version

    // Battery voltage (mV) - big endian
    advertData[idx++] = (sensor_data.battery_voltage >> 8) & 0xFF;
    advertData[idx++] = sensor_data.battery_voltage & 0xFF;

    // Temperature (sensor data in 0.01°C, convert to 8.8 fixed point)
    int16_t temp_fixed = (sensor_data.temperature * 256) / 100;
    advertData[idx++] = (temp_fixed >> 8) & 0xFF;
    advertData[idx++] = temp_fixed & 0xFF;

    // Advertisement count (use humidity as custom data)
    advertData[idx++] = (sensor_data.humidity >> 24) & 0xFF;
    advertData[idx++] = (sensor_data.humidity >> 16) & 0xFF;
    advertData[idx++] = (sensor_data.humidity >> 8) & 0xFF;
    advertData[idx++] = sensor_data.humidity & 0xFF;

    // Time since boot (0.1s resolution) - big endian
    uint32_t uptime_decisec = sensor_data.uptime * 10;
    advertData[idx++] = (uptime_decisec >> 24) & 0xFF;
    advertData[idx++] = (uptime_decisec >> 16) & 0xFF;
    advertData[idx++] = (uptime_decisec >> 8) & 0xFF;
    advertData[idx++] = uptime_decisec & 0xFF;

    advertDataLen = idx;
}

/**
 * @brief   Build iBeacon advertising data
 */
static void Beacon_BuildiBeacon(void)
{
    uint8_t idx = 0;

    // Flags
    advertData[idx++] = 0x02;
    advertData[idx++] = 0x01;
    advertData[idx++] = 0x06;

    // Manufacturer Specific Data
    advertData[idx++] = 0x1A;  // Length (26 bytes)
    advertData[idx++] = 0xFF;  // Type: Manufacturer Specific Data
    advertData[idx++] = 0x4C;  // Apple Company ID (0x004C)
    advertData[idx++] = 0x00;
    advertData[idx++] = 0x02;  // iBeacon type
    advertData[idx++] = 0x15;  // iBeacon data length (21 bytes)

    // Proximity UUID (16 bytes)
    tmos_memcpy(&advertData[idx], beacon_config.ibeacon_uuid, 16);
    idx += 16;

    // Major (2 bytes, big endian)
    advertData[idx++] = (beacon_config.ibeacon_major >> 8) & 0xFF;
    advertData[idx++] = beacon_config.ibeacon_major & 0xFF;

    // Minor (2 bytes, big endian)
    advertData[idx++] = (beacon_config.ibeacon_minor >> 8) & 0xFF;
    advertData[idx++] = beacon_config.ibeacon_minor & 0xFF;

    // Measured Power
    advertData[idx++] = beacon_config.tx_power;

    advertDataLen = idx;
}

/**
 * @brief   Build custom sensor broadcast format
 */
static void Beacon_BuildSensorCustom(void)
{
    uint8_t idx = 0;

    // Flags
    advertData[idx++] = 0x02;
    advertData[idx++] = 0x01;
    advertData[idx++] = 0x06;

    // Complete Local Name
    const char name[] = "WCH-Sensor";
    advertData[idx++] = sizeof(name);
    advertData[idx++] = 0x09;  // Complete Local Name
    tmos_memcpy(&advertData[idx], name, sizeof(name) - 1);
    idx += sizeof(name) - 1;

    // Manufacturer Specific Data
    advertData[idx++] = 0x0D;  // Length
    advertData[idx++] = 0xFF;  // Type: Manufacturer Specific Data
    advertData[idx++] = 0xD7;  // WCH Company ID (0x07D7)
    advertData[idx++] = 0x07;

    // Temperature (2 bytes, signed, 0.01°C)
    advertData[idx++] = (sensor_data.temperature >> 8) & 0xFF;
    advertData[idx++] = sensor_data.temperature & 0xFF;

    // Humidity (2 bytes, unsigned, 0.01%)
    advertData[idx++] = (sensor_data.humidity >> 8) & 0xFF;
    advertData[idx++] = sensor_data.humidity & 0xFF;

    // Battery voltage (2 bytes, mV)
    advertData[idx++] = (sensor_data.battery_voltage >> 8) & 0xFF;
    advertData[idx++] = sensor_data.battery_voltage & 0xFF;

    // Uptime (4 bytes, seconds)
    advertData[idx++] = (sensor_data.uptime >> 24) & 0xFF;
    advertData[idx++] = (sensor_data.uptime >> 16) & 0xFF;
    advertData[idx++] = (sensor_data.uptime >> 8) & 0xFF;
    advertData[idx++] = sensor_data.uptime & 0xFF;

    advertDataLen = idx;
}

/**
 * @brief   Build advertising data based on current mode
 */
static void Beacon_BuildAdvData(void)
{
    switch(beacon_config.beacon_mode)
    {
        case BEACON_MODE_EDDYSTONE_UID:
            Beacon_BuildEddystoneUID();
            break;

        case BEACON_MODE_EDDYSTONE_URL:
            Beacon_BuildEddystoneURL();
            break;

        case BEACON_MODE_EDDYSTONE_TLM:
            Beacon_BuildEddystoneTLM();
            break;

        case BEACON_MODE_IBEACON:
            Beacon_BuildiBeacon();
            break;

        case BEACON_MODE_SENSOR_CUSTOM:
            Beacon_BuildSensorCustom();
            break;

        default:
            Beacon_BuildEddystoneUID();
            break;
    }

    PRINT("Beacon: Advertising data built (len=%d, mode=%d)\n",
          advertDataLen, beacon_config.beacon_mode);
}

/**
 * @brief   Update advertising data
 */
void Beacon_UpdateAdvData(void)
{
    // Build new advertising data
    Beacon_BuildAdvData();

    // Update GAP advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, advertDataLen, advertData);

    PRINT("Beacon: Advertising data updated\n");
}

/**
 * @brief   Read sensor data
 */
static void Beacon_ReadSensor(void)
{
    int ret;

    if(!beacon_config.sensor_enabled)
    {
        return;
    }

    // Read SHTXX sensor
    ret = shtxx_read_measurement(&shtxx_data);
    if(ret == 0)
    {
        // Convert to broadcast format
        sensor_data.temperature = (int16_t)(shtxx_data.temperature_c * 100);
        sensor_data.humidity = (uint16_t)(shtxx_data.humidity_rh * 100);

        PRINT("Sensor: T=%.2f°C, RH=%.2f%%\n",
              shtxx_data.temperature_c, shtxx_data.humidity_rh);
    }
    else
    {
        PRINT("Sensor: Read failed (ret=%d)\n", ret);
    }

    // Update uptime
    sensor_data.uptime = TMOS_GetSystemClock() / 1000;  // Convert to seconds

    // Read battery voltage (ADC)
    // For now, use a placeholder value
    sensor_data.battery_voltage = 3000;  // 3.0V
}

/**
 * @brief   Get current sensor data
 */
int Beacon_GetSensorData(sensor_broadcast_t *data)
{
    if(data == NULL)
    {
        return -1;
    }

    tmos_memcpy(data, &sensor_data, sizeof(sensor_broadcast_t));
    return 0;
}

/**
 * @brief   Start beacon advertising
 */
int Beacon_Start(void)
{
    uint8_t advertising_enable = TRUE;

    // Read sensor data first
    if(beacon_config.sensor_enabled)
    {
        Beacon_ReadSensor();
    }

    // Build advertising data
    Beacon_BuildAdvData();

    // Set advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, advertDataLen, advertData);

    // Set advertising interval
    GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, beacon_config.adv_interval);
    GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, beacon_config.adv_interval);

    // Start advertising
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);

    beacon_state = BEACON_STATE_ADVERTISING;

    // Start sensor update timer if sensor enabled
    if(beacon_config.sensor_enabled)
    {
        tmos_start_task(Beacon_TaskID, SBP_SENSOR_READ_EVT,
                       beacon_config.sensor_update_rate * 1600);
    }

    PRINT("Beacon: Started advertising\n");
    return 0;
}

/**
 * @brief   Stop beacon advertising
 */
void Beacon_Stop(void)
{
    uint8_t advertising_enable = FALSE;

    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);

    beacon_state = BEACON_STATE_STOPPED;

    // Stop sensor timer
    tmos_stop_task(Beacon_TaskID, SBP_SENSOR_READ_EVT);

    PRINT("Beacon: Stopped advertising\n");
}

/**
 * @brief   Send UART response
 */
static void Beacon_SendUartResponse(uint8_t cmd, uint8_t status, uint8_t *data, uint16_t len)
{
    uint8_t response[UART_RX_BUF_SIZE];
    uint16_t idx = 0;

    // Header
    response[idx++] = 0xAA;  // Start byte
    response[idx++] = 0x55;  // Start byte
    response[idx++] = cmd;   // Command
    response[idx++] = status;  // Status

    // Length
    response[idx++] = (len >> 8) & 0xFF;
    response[idx++] = len & 0xFF;

    // Data
    if(data != NULL && len > 0)
    {
        tmos_memcpy(&response[idx], data, len);
        idx += len;
    }

    // Checksum (simple XOR)
    uint8_t checksum = 0;
    for(uint16_t i = 0; i < idx; i++)
    {
        checksum ^= response[i];
    }
    response[idx++] = checksum;

    // Send via UART
    UART_SendString(response, idx);
}

/**
 * @brief   Process UART configuration data
 */
static void Beacon_ProcessUartData(uint8_t *data, uint16_t len)
{
    if(len < 6)  // Minimum: Header(2) + CMD(1) + Status(1) + Len(2)
    {
        return;
    }

    // Check header
    if(data[0] != 0xAA || data[1] != 0x55)
    {
        return;
    }

    uint8_t cmd = data[2];
    uint16_t data_len = (data[4] << 8) | data[5];

    PRINT("UART: Received cmd=0x%02X, len=%d\n", cmd, data_len);

    switch(cmd)
    {
        case CMD_GET_CONFIG:
            Beacon_SendUartResponse(CMD_GET_CONFIG, RESP_OK,
                                   (uint8_t *)&beacon_config, sizeof(beacon_config_t));
            break;

        case CMD_SET_CONFIG:
            if(data_len == sizeof(beacon_config_t))
            {
                beacon_config_t *new_config = (beacon_config_t *)&data[6];
                int ret = Beacon_SetConfig(new_config);
                Beacon_SendUartResponse(CMD_SET_CONFIG,
                                       ret == 0 ? RESP_OK : RESP_ERROR, NULL, 0);
            }
            else
            {
                Beacon_SendUartResponse(CMD_SET_CONFIG, RESP_INVALID_PARAM, NULL, 0);
            }
            break;

        case CMD_SAVE_CONFIG:
            Beacon_SaveConfig();
            Beacon_SendUartResponse(CMD_SAVE_CONFIG, RESP_OK, NULL, 0);
            break;

        case CMD_RESET_CONFIG:
            Beacon_ResetConfig();
            Beacon_SendUartResponse(CMD_RESET_CONFIG, RESP_OK, NULL, 0);
            break;

        case CMD_GET_SENSOR:
            Beacon_ReadSensor();
            Beacon_SendUartResponse(CMD_GET_SENSOR, RESP_OK,
                                   (uint8_t *)&sensor_data, sizeof(sensor_broadcast_t));
            break;

        case CMD_START_BEACON:
            Beacon_Start();
            Beacon_SendUartResponse(CMD_START_BEACON, RESP_OK, NULL, 0);
            break;

        case CMD_STOP_BEACON:
            Beacon_Stop();
            Beacon_SendUartResponse(CMD_STOP_BEACON, RESP_OK, NULL, 0);
            break;

        default:
            Beacon_SendUartResponse(cmd, RESP_INVALID_CMD, NULL, 0);
            break;
    }
}

/**
 * @brief   UART RX callback
 */
void Beacon_UartRxCallback(uint8_t *data, uint16_t len)
{
    if(uart_rx_index + len > UART_RX_BUF_SIZE)
    {
        uart_rx_index = 0;  // Buffer overflow, reset
    }

    tmos_memcpy(&uart_rx_buf[uart_rx_index], data, len);
    uart_rx_index += len;

    // Process complete packets
    Beacon_ProcessUartData(uart_rx_buf, uart_rx_index);
    uart_rx_index = 0;  // Reset for next packet
}

/**
 * @brief   Initialize Beacon Application
 */
void Beacon_Init(void)
{
    Beacon_TaskID = TMOS_ProcessEventRegister(Beacon_ProcessEvent);

    // Load configuration from flash (or use defaults)
    Beacon_LoadConfig();

    // Initialize SHTXX sensor if enabled
    if(beacon_config.sensor_enabled)
    {
        shtxx_config_t sensor_config = {
            .i2c_addr = beacon_config.shtxx_addr,
            .repeatability = beacon_config.shtxx_repeatability,
            .heater_enabled = 0
        };
        shtxx_init(&sensor_config);
    }

    // Initialize sensor data
    tmos_memset(&sensor_data, 0, sizeof(sensor_broadcast_t));

    // Setup GAP Broadcaster Role
    {
        uint8_t advertising_enable = FALSE;  // Start disabled
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);
    }

    // Setup advertising parameters
    {
        uint16_t advInt = beacon_config.adv_interval;
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_MAX, advInt);
    }

    beacon_state = BEACON_STATE_IDLE;

    PRINT("Beacon: Initialized (mode=%d, sensor=%d)\n",
          beacon_config.beacon_mode, beacon_config.sensor_enabled);

    // Start device
    tmos_set_event(Beacon_TaskID, SBP_START_DEVICE_EVT);
}

/**
 * @brief   Beacon event processor
 */
uint16_t Beacon_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;
        if((pMsg = tmos_msg_receive(Beacon_TaskID)) != NULL)
        {
            Beacon_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
            tmos_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & SBP_START_DEVICE_EVT)
    {
        // Start broadcaster
        GAPRole_BroadcasterStartDevice(Beacon_TaskID, NULL);

        // Auto-start if configured
        Beacon_Start();

        return (events ^ SBP_START_DEVICE_EVT);
    }

    if(events & SBP_SENSOR_READ_EVT)
    {
        // Read sensor
        Beacon_ReadSensor();

        // Update advertising data if in TLM or custom mode
        if(beacon_config.beacon_mode == BEACON_MODE_EDDYSTONE_TLM ||
           beacon_config.beacon_mode == BEACON_MODE_SENSOR_CUSTOM)
        {
            Beacon_UpdateAdvData();
        }

        // Restart timer
        if(beacon_config.sensor_enabled && beacon_state == BEACON_STATE_ADVERTISING)
        {
            tmos_start_task(Beacon_TaskID, SBP_SENSOR_READ_EVT,
                           beacon_config.sensor_update_rate * 1600);
        }

        return (events ^ SBP_SENSOR_READ_EVT);
    }

    return 0;
}

/**
 * @brief   Process TMOS messages
 */
static void Beacon_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        default:
            break;
    }
}

/******************************** endfile @ beacon ******************************/
