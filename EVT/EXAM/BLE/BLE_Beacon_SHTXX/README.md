# WCH CH570 BLE Beacon with SHTXX Sensor

A comprehensive BLE beacon example with SHTXX (SHT30/SHT31/SHT35) temperature and humidity sensor integration. Supports multiple beacon formats including Eddystone-UID, Eddystone-URL, Eddystone-TLM, iBeacon, and custom sensor broadcast.

## Features

- **Multiple Beacon Formats**
  - Eddystone-UID (unique beacon identifier)
  - Eddystone-URL (broadcast web links)
  - Eddystone-TLM (telemetry with sensor data)
  - Apple iBeacon
  - Custom sensor broadcast format

- **SHTXX Sensor Integration**
  - I2C temperature and humidity measurement
  - Configurable measurement repeatability
  - CRC data validation
  - Integration with beacon telemetry

- **UART Configuration Interface**
  - Read/write beacon configuration
  - Save configuration to flash
  - Real-time sensor monitoring
  - Start/stop beacon advertising

- **Python GUI Configuration Tool**
  - Intuitive graphical interface
  - COM port connection management
  - Beacon mode configuration
  - Sensor monitoring
  - JSON configuration import/export
  - Flash programming

## Hardware Requirements

- WCH CH570 development board
- SHTXX sensor (SHT30/SHT31/SHT35)
- USB-UART adapter (for configuration)

## Pin Connections

| Function | GPIO Pin | Description |
|----------|----------|-------------|
| UART TX  | PA3      | Configuration TX |
| UART RX  | PA2      | Configuration RX |
| I2C SDA  | PA8      | SHTXX Data |
| I2C SCL  | PA9      | SHTXX Clock |

### SHTXX Sensor Wiring

```
CH570 Board          SHTXX Sensor
-----------          ------------
  3.3V     --------->   VDD
  GND      --------->   GND
  PA8 (SDA) ------->   SDA
  PA9 (SCL) ------->   SCL
```

## Software Architecture

### State Machine

```
┌──────────────┐
│     IDLE     │
└──────┬───────┘
       │ Initialize
       v
┌──────────────┐
│INITIALIZING  │
└──────┬───────┘
       │ Start Device
       v
┌──────────────┐     Stop      ┌──────────────┐
│ ADVERTISING  │<-------------->│   STOPPED    │
└──────┬───────┘                └──────────────┘
       │
       │ Config Request
       v
┌──────────────┐
│ CONFIG_MODE  │
└──────────────┘
```

### Event Flow

1. **System Initialization**
   - BLE stack initialization
   - SHTXX sensor initialization
   - Load configuration from flash
   - Register TMOS events

2. **Beacon Advertising**
   - Build advertising data based on mode
   - Start BLE broadcaster
   - Periodic sensor updates (if enabled)
   - Update advertising data with sensor values

3. **Configuration Mode**
   - UART command reception
   - Process configuration commands
   - Send responses
   - Update beacon parameters

4. **Sensor Monitoring**
   - Periodic I2C sensor reading
   - Data validation (CRC check)
   - Value conversion to physical units
   - Integration with beacon data

## UART Protocol

### Packet Format

```
┌────┬────┬────┬────┬─────┬─────┬──────┬──────┬─────────┐
│ AA │ 55 │CMD │STA │LEN_H│LEN_L│ DATA │ ... │CHECKSUM │
└────┴────┴────┴────┴─────┴─────┴──────┴──────┴─────────┘
  2B Header  CMD  STA    2B Len     N bytes      1B XOR
```

### Commands

| Command | Code | Description |
|---------|------|-------------|
| GET_CONFIG | 0x01 | Read current configuration |
| SET_CONFIG | 0x02 | Write new configuration |
| SAVE_CONFIG | 0x03 | Save config to flash |
| RESET_CONFIG | 0x04 | Reset to defaults |
| GET_SENSOR | 0x05 | Read sensor data |
| START_BEACON | 0x10 | Start advertising |
| STOP_BEACON | 0x11 | Stop advertising |

### Response Codes

| Code | Description |
|------|-------------|
| 0x00 | Success (RESP_OK) |
| 0xFD | Invalid parameter |
| 0xFE | Invalid command |
| 0xFF | General error |

## Configuration Structure

### JSON Format

```json
{
  "beacon_mode": 0,
  "beacon_mode_name": "Eddystone-UID",
  "sensor_enabled": true,
  "tx_power_dbm": 0,
  "adv_interval_ms": 1000,
  "eddystone_uid": {
    "namespace_id": "00010203040506070809",
    "instance_id": "000000000001"
  },
  "eddystone_url": {
    "scheme": 1,
    "url": "wch.cn"
  },
  "ibeacon": {
    "uuid": "00000000000000000000000000000000",
    "major": 0,
    "minor": 0
  },
  "sensor": {
    "update_rate_sec": 5,
    "i2c_address": "0x44",
    "repeatability": 2
  }
}
```

## Python GUI Usage

### Installation

1. Install Python 3.7 or later
2. Install dependencies:
   ```bash
   cd Python_GUI
   pip install -r requirements.txt
   ```

### Running the GUI

```bash
python beacon_config.py
```

### GUI Workflow

1. **Connect to Device**
   - Select COM port from dropdown
   - Click "Refresh" to scan for ports
   - Click "Connect" to establish connection

2. **Read Configuration**
   - Click "Read Config" to fetch current settings
   - Configuration displayed in JSON format
   - UI fields automatically populated

3. **Configure Beacon**
   - Select beacon mode (Eddystone-UID, URL, TLM, iBeacon, Custom)
   - Configure mode-specific parameters
   - Set TX power and advertising interval
   - Configure sensor settings

4. **Save Configuration**
   - Click "Write Config" to send to device
   - Click "Save to Flash" to persist configuration
   - Or save to JSON file for later use

5. **Monitor Sensor**
   - Switch to "Sensor Monitor" tab
   - Click "Read Once" for single reading
   - Click "Start Monitor" for continuous updates
   - View temperature, humidity, battery, uptime

6. **Control Beacon**
   - Click "Start Beacon" to begin advertising
   - Click "Stop Beacon" to stop advertising

## Beacon Modes

### 1. Eddystone-UID

Broadcasts a unique beacon identifier.

**Parameters:**
- Namespace ID (10 bytes): Group identifier
- Instance ID (6 bytes): Individual beacon ID

**Use Cases:**
- Indoor positioning
- Proximity detection
- Asset tracking

### 2. Eddystone-URL

Broadcasts a compressed URL.

**Parameters:**
- URL Scheme: http://, https://, etc.
- Encoded URL: Domain and path

**Use Cases:**
- Physical web
- Marketing campaigns
- Information beacons

**URL Encoding:**
- Schemes: `http://www.`, `https://www.`, `http://`, `https://`
- Expansions: `.com`, `.org`, `.edu`, `.net`, `.info`, etc.

### 3. Eddystone-TLM (Telemetry)

Broadcasts sensor and battery data.

**Data Included:**
- Battery voltage (mV)
- Temperature (from SHTXX sensor)
- Humidity (encoded in advertisement count)
- Uptime (deciseconds)

**Use Cases:**
- Environmental monitoring
- Sensor networks
- IoT applications

### 4. iBeacon

Apple's beacon format for iOS devices.

**Parameters:**
- UUID (16 bytes): Application identifier
- Major (2 bytes): Group identifier
- Minor (2 bytes): Individual beacon ID

**Use Cases:**
- iOS apps
- Retail applications
- Museum guides

### 5. Custom Sensor Broadcast

WCH-specific format with full sensor data.

**Data Included:**
- Temperature (0.01°C resolution)
- Humidity (0.01% resolution)
- Battery voltage (mV)
- Uptime (seconds)

**Use Cases:**
- Custom applications
- Maximum sensor data
- WCH-specific solutions

## Example Configurations

### 1. Basic Eddystone-UID Beacon

```json
{
  "beacon_mode": 0,
  "sensor_enabled": false,
  "tx_power_dbm": 0,
  "adv_interval_ms": 1000,
  "eddystone_uid": {
    "namespace_id": "AABBCCDDEEFF0011223344",
    "instance_id": "112233445566"
  }
}
```

### 2. Eddystone-URL for Website

```json
{
  "beacon_mode": 1,
  "sensor_enabled": false,
  "tx_power_dbm": 4,
  "adv_interval_ms": 500,
  "eddystone_url": {
    "scheme": 1,
    "url": "wch.cn"
  }
}
```

### 3. Environmental Sensor Beacon

```json
{
  "beacon_mode": 2,
  "sensor_enabled": true,
  "tx_power_dbm": 0,
  "adv_interval_ms": 2000,
  "sensor": {
    "update_rate_sec": 10,
    "i2c_address": "0x44",
    "repeatability": 2
  }
}
```

### 4. iBeacon for iOS App

```json
{
  "beacon_mode": 3,
  "sensor_enabled": false,
  "tx_power_dbm": -4,
  "adv_interval_ms": 100,
  "ibeacon": {
    "uuid": "E2C56DB5DFFB48D2B060D0F5A71096E0",
    "major": 1,
    "minor": 100
  }
}
```

## Building the Firmware

1. Open project in MounRiver Studio
2. Select `BLE_Beacon_SHTXX.wvproj`
3. Build the project
4. Flash using WCHISPTool

## Troubleshooting

### Sensor Not Detected

- Check I2C connections (PA8=SDA, PA9=SCL)
- Verify sensor power supply (3.3V)
- Check I2C address (default 0x44)
- Try alternate address (0x45)

### UART Communication Issues

- Verify baud rate (115200)
- Check TX/RX connections
- Ensure correct COM port selected
- Try different USB cable

### Beacon Not Visible

- Use BLE scanner app (nRF Connect, LightBlue)
- Check advertising interval settings
- Verify TX power setting
- Ensure beacon is started

### Configuration Not Saving

- Click "Save to Flash" after "Write Config"
- Wait for confirmation message
- Power cycle device to verify persistence

## Advanced Features

### Flash Configuration Storage

Configuration is stored at the end of flash memory and persists across power cycles.

**Flash Layout:**
```
┌─────────────────┐
│  Application    │
│     Code        │
├─────────────────┤
│   Free Space    │
├─────────────────┤
│ Configuration   │  <- Last EEPROM block
│   (1 block)     │
└─────────────────┘
```

### Sensor Data Integration

When sensor is enabled and beacon mode supports it:

1. Sensor read timer triggers every N seconds
2. I2C transaction reads temperature and humidity
3. Data validation via CRC-8
4. Values converted to physical units
5. Advertising data updated with new values
6. Beacon continues broadcasting updated data

### Power Consumption

Estimated current consumption:

- **Idle**: ~2 mA
- **Advertising (1 Hz)**: ~3-5 mA
- **With Sensor (every 5s)**: ~4-6 mA
- **Sleep mode**: <10 µA (when implemented)

### Range and TX Power

| TX Power | Approximate Range | Current Draw |
|----------|------------------|--------------|
| -20 dBm  | ~5 m            | Low          |
| -12 dBm  | ~10 m           | Low-Medium   |
| -4 dBm   | ~20 m           | Medium       |
| 0 dBm    | ~30 m           | Medium-High  |
| +4 dBm   | ~50 m           | High         |

## API Reference

### Beacon Functions

```c
void Beacon_Init(void);
int Beacon_Start(void);
void Beacon_Stop(void);
int Beacon_SetConfig(beacon_config_t *config);
void Beacon_GetConfig(beacon_config_t *config);
int Beacon_SaveConfig(void);
int Beacon_LoadConfig(void);
void Beacon_ResetConfig(void);
void Beacon_UpdateAdvData(void);
int Beacon_GetSensorData(sensor_broadcast_t *data);
```

### SHTXX Functions

```c
int shtxx_init(shtxx_config_t *config);
int shtxx_soft_reset(void);
int shtxx_read_measurement(shtxx_data_t *data);
int shtxx_set_heater(uint8_t enable);
int shtxx_read_status(uint16_t *status);
uint8_t shtxx_calculate_crc8(const uint8_t *data, uint8_t len);
void shtxx_convert_values(shtxx_data_t *data);
```

## License

Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.

This software and binary are for use with microcontrollers manufactured by Nanjing Qinheng Microelectronics.

## Support

- Email: tech@wch.cn
- Website: https://www.wch.cn
- Forum: https://www.wch.cn/bbs

## Version History

### v1.0 (2025-01-17)
- Initial release
- Multiple beacon format support
- SHTXX sensor integration
- Python GUI configuration tool
- UART protocol implementation
- JSON configuration support
- Flash persistence

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing style
- Comments in English
- Test with real hardware
- Update documentation

## Acknowledgments

Inspired by:
- Inplay nanoBeacon IN100
- Google Eddystone specification
- Apple iBeacon specification
- Sensirion SHTXX datasheet
