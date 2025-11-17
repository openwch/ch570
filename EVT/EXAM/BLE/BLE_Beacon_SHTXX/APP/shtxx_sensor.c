/********************************** (C) COPYRIGHT *******************************
 * File Name          : shtxx_sensor.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/17
 * Description        : SHTXX (SHT30/SHT31/SHT35) Temperature & Humidity Sensor Driver
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "shtxx_sensor.h"
#include "CH57x_common.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
static shtxx_config_t sensor_config = {
    .i2c_addr = SHTXX_ADDR_DEFAULT,
    .repeatability = 2,  // High repeatability by default
    .heater_enabled = 0
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/**
 * @brief   Write command to SHTXX sensor
 * @param   cmd - 16-bit command
 * @return  0 on success, negative on failure
 */
static int shtxx_write_command(uint16_t cmd)
{
    uint8_t cmd_buf[2];
    int ret;

    cmd_buf[0] = (cmd >> 8) & 0xFF;  // MSB
    cmd_buf[1] = cmd & 0xFF;         // LSB

    // Wait for I2C bus to be ready
    while(I2C_GetFlagStatus(I2C_FLAG_BUSY) != RESET);

    // Send start condition
    I2C_GenerateSTART(ENABLE);
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

    // Send device address for write
    I2C_Send7bitAddress(sensor_config.i2c_addr << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Send command MSB
    I2C_SendData(cmd_buf[0]);
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Send command LSB
    I2C_SendData(cmd_buf[1]);
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Send stop condition
    I2C_GenerateSTOP(ENABLE);

    return 0;
}

/**
 * @brief   Read data from SHTXX sensor
 * @param   buf - Buffer to store data
 * @param   len - Number of bytes to read
 * @return  0 on success, negative on failure
 */
static int shtxx_read_data(uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;

    // Wait for I2C bus to be ready
    while(I2C_GetFlagStatus(I2C_FLAG_BUSY) != RESET);

    // Send start condition
    I2C_GenerateSTART(ENABLE);
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

    // Send device address for read
    I2C_Send7bitAddress(sensor_config.i2c_addr << 1, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // Enable ACK
    I2C_AcknowledgeConfig(ENABLE);

    // Read data bytes
    for(i = 0; i < len; i++)
    {
        if(i == len - 1)
        {
            // Last byte: send NACK
            I2C_AcknowledgeConfig(DISABLE);
        }

        // Wait for data
        while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
        buf[i] = I2C_ReceiveData();
    }

    // Send stop condition
    I2C_GenerateSTOP(ENABLE);

    return 0;
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @brief   Calculate CRC-8 for data validation
 */
uint8_t shtxx_calculate_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i, j;

    for(i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

/**
 * @brief   Convert raw values to physical units
 */
void shtxx_convert_values(shtxx_data_t *data)
{
    // Temperature conversion formula: T = -45 + 175 * (T_raw / 65535)
    data->temperature_c = -45.0f + 175.0f * ((float)data->temperature_raw / 65535.0f);

    // Humidity conversion formula: RH = 100 * (RH_raw / 65535)
    data->humidity_rh = 100.0f * ((float)data->humidity_raw / 65535.0f);

    // Clamp humidity to valid range
    if(data->humidity_rh < 0.0f) data->humidity_rh = 0.0f;
    if(data->humidity_rh > 100.0f) data->humidity_rh = 100.0f;
}

/**
 * @brief   Initialize SHTXX sensor
 */
int shtxx_init(shtxx_config_t *config)
{
    int ret;

    if(config != NULL)
    {
        sensor_config = *config;
    }

    // Initialize I2C peripheral
    GPIOA_ModeCfg(GPIO_Pin_8 | GPIO_Pin_9, GPIO_ModeIN_PU);
    I2C_Init(I2C_Mode_I2C, 400000, I2C_DutyCycle_16_9, I2C_Ack_Enable,
             I2C_AckAddr_7bit, 0x00);

    // Wait for sensor to be ready (power-up time ~1ms)
    DelayMs(10);

    // Perform soft reset
    ret = shtxx_soft_reset();
    if(ret != 0)
    {
        PRINT("SHTXX: Soft reset failed\n");
        return ret;
    }

    // Wait for reset to complete
    DelayMs(10);

    PRINT("SHTXX: Sensor initialized (addr=0x%02X)\n", sensor_config.i2c_addr);

    return 0;
}

/**
 * @brief   Perform soft reset of sensor
 */
int shtxx_soft_reset(void)
{
    return shtxx_write_command(SHTXX_CMD_SOFT_RESET);
}

/**
 * @brief   Read temperature and humidity
 */
int shtxx_read_measurement(shtxx_data_t *data)
{
    uint8_t read_buf[6];  // 2 bytes temp + CRC + 2 bytes hum + CRC
    uint8_t crc_temp, crc_hum;
    uint16_t cmd;
    int ret;

    if(data == NULL)
    {
        return -1;
    }

    // Select measurement command based on repeatability
    switch(sensor_config.repeatability)
    {
        case 0:  cmd = SHTXX_CMD_MEASURE_LOW; break;
        case 1:  cmd = SHTXX_CMD_MEASURE_MED; break;
        default: cmd = SHTXX_CMD_MEASURE_HIGH; break;
    }

    // Send measurement command
    ret = shtxx_write_command(cmd);
    if(ret != 0)
    {
        return ret;
    }

    // Wait for measurement to complete
    // High repeatability: 15ms, Medium: 6ms, Low: 4ms
    DelayMs(20);

    // Read measurement data
    ret = shtxx_read_data(read_buf, 6);
    if(ret != 0)
    {
        return ret;
    }

    // Extract temperature (first 2 bytes + CRC)
    data->temperature_raw = (read_buf[0] << 8) | read_buf[1];
    crc_temp = shtxx_calculate_crc8(read_buf, 2);

    // Extract humidity (next 2 bytes + CRC)
    data->humidity_raw = (read_buf[3] << 8) | read_buf[4];
    crc_hum = shtxx_calculate_crc8(&read_buf[3], 2);

    // Verify CRC
    data->crc_valid = (crc_temp == read_buf[2]) && (crc_hum == read_buf[5]);

    if(!data->crc_valid)
    {
        PRINT("SHTXX: CRC check failed\n");
        return -2;
    }

    // Convert to physical units
    shtxx_convert_values(data);

    return 0;
}

/**
 * @brief   Enable/disable heater
 */
int shtxx_set_heater(uint8_t enable)
{
    uint16_t cmd = enable ? SHTXX_CMD_HEATER_EN : SHTXX_CMD_HEATER_DIS;
    int ret;

    ret = shtxx_write_command(cmd);
    if(ret == 0)
    {
        sensor_config.heater_enabled = enable;
    }

    return ret;
}

/**
 * @brief   Read status register
 */
int shtxx_read_status(uint16_t *status)
{
    uint8_t read_buf[3];  // 2 bytes status + CRC
    uint8_t crc;
    int ret;

    if(status == NULL)
    {
        return -1;
    }

    // Send read status command
    ret = shtxx_write_command(SHTXX_CMD_READ_STATUS);
    if(ret != 0)
    {
        return ret;
    }

    DelayMs(1);

    // Read status data
    ret = shtxx_read_data(read_buf, 3);
    if(ret != 0)
    {
        return ret;
    }

    // Extract status
    *status = (read_buf[0] << 8) | read_buf[1];

    // Verify CRC
    crc = shtxx_calculate_crc8(read_buf, 2);
    if(crc != read_buf[2])
    {
        PRINT("SHTXX: Status CRC check failed\n");
        return -2;
    }

    return 0;
}

/******************************** endfile @ shtxx_sensor ******************************/
