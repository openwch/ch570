# BLE Beacon State Machine Documentation

## Overview

The BLE Beacon with SHTXX sensor operates using a state-driven architecture managed by the TMOS (Tiny Multi-task Operating System). This document describes the state machine, transitions, and event handling.

## State Definitions

### 1. BEACON_STATE_IDLE
**Description:** Initial state after power-on or reset.

**Characteristics:**
- No BLE advertising active
- Sensors not actively reading
- Waiting for initialization

**Valid Transitions:**
- → BEACON_STATE_INITIALIZING (on Init event)

**Activities:**
- None

---

### 2. BEACON_STATE_INITIALIZING
**Description:** System initialization in progress.

**Characteristics:**
- BLE stack initializing
- I2C peripheral setup
- SHTXX sensor initialization
- Loading configuration from flash

**Valid Transitions:**
- → BEACON_STATE_ADVERTISING (on successful init)
- → BEACON_STATE_ERROR (on init failure)

**Activities:**
- Initialize BLE broadcaster role
- Configure I2C for SHTXX
- Load configuration from flash
- Initialize sensor
- Build initial advertising data

---

### 3. BEACON_STATE_ADVERTISING
**Description:** Active beacon broadcasting.

**Characteristics:**
- BLE advertising active
- Periodic sensor updates (if enabled)
- UART configuration commands accepted
- Advertising data may update dynamically

**Valid Transitions:**
- → BEACON_STATE_STOPPED (on Stop command)
- → BEACON_STATE_CONFIG_MODE (on Config command)
- → BEACON_STATE_ERROR (on critical error)

**Activities:**
- Broadcast BLE advertising packets
- Periodic sensor readings
- Update advertising data with sensor values
- Process UART configuration commands
- Maintain uptime counter

---

### 4. BEACON_STATE_STOPPED
**Description:** Beacon advertising intentionally stopped.

**Characteristics:**
- BLE advertising disabled
- Sensor reading stopped
- UART configuration still active
- Can be restarted

**Valid Transitions:**
- → BEACON_STATE_ADVERTISING (on Start command)

**Activities:**
- Process UART configuration commands
- Await start command

---

### 5. BEACON_STATE_CONFIG_MODE
**Description:** Configuration update in progress.

**Characteristics:**
- May temporarily pause advertising
- UART configuration active
- Configuration being modified

**Valid Transitions:**
- → BEACON_STATE_ADVERTISING (after config applied)
- → BEACON_STATE_STOPPED (if stopped during config)

**Activities:**
- Receive configuration data via UART
- Validate configuration
- Apply new settings
- Update advertising parameters

---

### 6. BEACON_STATE_ERROR
**Description:** Error condition detected.

**Characteristics:**
- Advertising stopped
- Error logged
- System may require reset

**Valid Transitions:**
- → BEACON_STATE_IDLE (on manual reset)
- → BEACON_STATE_INITIALIZING (on auto-recovery)

**Activities:**
- Log error condition
- Attempt recovery if possible
- Await manual intervention

---

## State Transition Diagram

```
                    ┌─────────────────┐
                    │   POWER ON /    │
                    │     RESET       │
                    └────────┬────────┘
                             │
                             v
                    ┌─────────────────┐
                    │      IDLE       │
                    └────────┬────────┘
                             │
                             │ SBP_START_DEVICE_EVT
                             v
                    ┌─────────────────┐
                    │  INITIALIZING   │
                    └────────┬────────┘
                             │
                ┌────────────┴────────────┐
                │                         │
        Success │                         │ Failure
                v                         v
       ┌─────────────────┐      ┌─────────────────┐
       │  ADVERTISING    │      │     ERROR       │
       └────────┬────────┘      └─────────────────┘
                │
                │
    ┌───────────┼───────────┐
    │           │           │
    │ Config    │ Stop      │
    │ Request   │ Command   │
    │           │           │
    v           │           v
┌──────────┐    │     ┌──────────┐
│ CONFIG   │    │     │ STOPPED  │
│  MODE    │    │     └────┬─────┘
└────┬─────┘    │          │
     │          │          │ Start
     │ Done     │          │ Command
     │          │          │
     └──────────┴──────────┘
```

## Event Processing

### TMOS Events

#### SBP_START_DEVICE_EVT (0x0001)
**Trigger:** Application initialization complete

**Handler Actions:**
1. Call `GAPRole_BroadcasterStartDevice()`
2. Initialize broadcaster callbacks
3. Transition to ADVERTISING
4. Call `Beacon_Start()`

**State Changes:**
- IDLE → ADVERTISING

---

#### SBP_PERIODIC_EVT (0x0002)
**Trigger:** Periodic timer (optional, currently unused)

**Handler Actions:**
1. Perform periodic maintenance
2. Restart timer if needed

**State Changes:**
- None (stays in current state)

---

#### SBP_SENSOR_READ_EVT (0x0004)
**Trigger:** Sensor update timer expires

**Handler Actions:**
1. Call `Beacon_ReadSensor()`
2. Read SHTXX via I2C
3. Validate CRC
4. Convert to physical units
5. Update `sensor_data` structure
6. If TLM or Custom mode: call `Beacon_UpdateAdvData()`
7. Restart timer

**State Changes:**
- None (stays in ADVERTISING)

---

#### SBP_CONFIG_TIMEOUT_EVT (0x0008)
**Trigger:** Configuration timeout (optional, currently unused)

**Handler Actions:**
1. Return to previous state
2. Discard incomplete configuration

**State Changes:**
- CONFIG_MODE → ADVERTISING or STOPPED

---

#### SBP_UPDATE_ADV_DATA_EVT (0x0010)
**Trigger:** Advertising data needs update

**Handler Actions:**
1. Call `Beacon_BuildAdvData()`
2. Update GAP advertising data
3. Continue advertising with new data

**State Changes:**
- None (stays in ADVERTISING)

---

## UART Command State Flow

### GET_CONFIG (0x01)
```
User sends GET_CONFIG
         ↓
Current state preserved
         ↓
Read beacon_config from memory
         ↓
Serialize to bytes
         ↓
Send UART response
         ↓
Continue in current state
```

### SET_CONFIG (0x02)
```
User sends SET_CONFIG + data
         ↓
Parse configuration bytes
         ↓
Validate magic number
         ↓
Update beacon_config
         ↓
If ADVERTISING: call UpdateAdvData()
         ↓
Send UART response (OK/ERROR)
         ↓
Continue in current state
```

### SAVE_CONFIG (0x03)
```
User sends SAVE_CONFIG
         ↓
Current state preserved
         ↓
Erase flash page
         ↓
Write config to flash
         ↓
Send UART response (OK/ERROR)
         ↓
Continue in current state
```

### START_BEACON (0x10)
```
User sends START_BEACON
         ↓
Call Beacon_Start()
         ↓
Read sensor if enabled
         ↓
Build advertising data
         ↓
Enable GAP advertising
         ↓
Start sensor timer if enabled
         ↓
State: STOPPED → ADVERTISING
         ↓
Send UART response (OK)
```

### STOP_BEACON (0x11)
```
User sends STOP_BEACON
         ↓
Call Beacon_Stop()
         ↓
Disable GAP advertising
         ↓
Stop sensor timer
         ↓
State: ADVERTISING → STOPPED
         ↓
Send UART response (OK)
```

## Sensor Update Flow

When sensor is enabled and beacon is advertising:

```
Timer expires (every N seconds)
         ↓
SBP_SENSOR_READ_EVT triggered
         ↓
shtxx_write_command(MEASURE)
         ↓
Wait 20ms for measurement
         ↓
shtxx_read_data(6 bytes)
         ↓
Extract temperature (2 bytes + CRC)
         ↓
Extract humidity (2 bytes + CRC)
         ↓
Validate CRC
         ↓
Convert to physical units
         ↓
Update sensor_data structure
         ↓
If mode == TLM or CUSTOM:
    ↓
    Beacon_BuildAdvData()
    ↓
    GAPRole_SetParameter(ADVERT_DATA)
         ↓
Restart timer
         ↓
Continue ADVERTISING
```

## Configuration Update Sequence

### GUI Configuration Flow

```
User modifies settings in GUI
         ↓
Clicks "Write Config"
         ↓
GUI: update_config_from_ui()
         ↓
GUI: serialize to bytes
         ↓
GUI: send_command(SET_CONFIG)
         ↓
Device: Beacon_ProcessUartData()
         ↓
Device: Beacon_SetConfig()
         ↓
Device: Validate magic
         ↓
Device: Copy to beacon_config
         ↓
Device: If ADVERTISING, UpdateAdvData()
         ↓
Device: Send response
         ↓
GUI: Show success message
         ↓
Optional: User clicks "Save to Flash"
         ↓
GUI: send_command(SAVE_CONFIG)
         ↓
Device: Beacon_SaveConfig()
         ↓
Device: Flash erase + write
         ↓
Device: Send response
         ↓
GUI: Show "Saved to flash"
```

## Error Handling

### I2C Communication Error
```
shtxx_read_measurement() fails
         ↓
Return error code (-1, -2)
         ↓
Print error to UART debug
         ↓
sensor_data unchanged
         ↓
Advertising continues with old data
         ↓
Retry on next timer
```

### CRC Validation Failure
```
CRC check fails
         ↓
Set data.crc_valid = 0
         ↓
Return error (-2)
         ↓
Print "CRC check failed"
         ↓
Discard invalid data
         ↓
Keep previous valid data
         ↓
Retry on next timer
```

### Flash Write Error
```
FLASH_ROM_WRITE() fails
         ↓
Return error from Beacon_SaveConfig()
         ↓
Send UART response (ERROR)
         ↓
GUI shows error message
         ↓
Configuration in RAM still valid
         ↓
User can retry
```

### UART Protocol Error
```
Invalid packet received
         ↓
Check header (AA 55)
         ↓
If invalid: discard packet
         ↓
If valid but unknown CMD:
    ↓
    Send RESP_INVALID_CMD
         ↓
If valid but wrong length:
    ↓
    Send RESP_INVALID_PARAM
         ↓
Reset uart_rx_index
         ↓
Wait for next packet
```

## Timing Diagrams

### Boot Sequence
```
Time  Event
----  -----
0ms   Power on
1ms   main() starts
2ms   System clock configured (100MHz)
3ms   UART initialized (115200 baud)
5ms   BLE stack CH57x_BLEInit()
50ms  HAL_Init()
52ms  GAPRole_BroadcasterInit()
55ms  Beacon_Init()
56ms    - Load config from flash
60ms    - SHTXX sensor init
65ms    - shtxx_soft_reset()
75ms    - Init complete
76ms  SBP_START_DEVICE_EVT posted
77ms  GAPRole_BroadcasterStartDevice()
80ms  Beacon_Start()
81ms    - Read sensor
100ms   - Build adv data
101ms   - Start advertising
102ms STATE: ADVERTISING
5102ms First sensor update (if 5s rate)
```

### Sensor Read Cycle
```
Time   Event
-----  -----
0ms    Timer expires: SBP_SENSOR_READ_EVT
1ms    Beacon_ReadSensor() called
2ms    I2C: Send START
3ms    I2C: Send address + MEASURE cmd
5ms    I2C: Send STOP
25ms   Wait for measurement (20ms)
26ms   I2C: Send START
27ms   I2C: Send address + READ
28ms   I2C: Read 6 bytes (temp + RH + CRCs)
30ms   I2C: Send STOP
31ms   Validate CRC
32ms   Convert to physical units
33ms   Update sensor_data
34ms   If TLM/Custom: BuildAdvData()
35ms   If TLM/Custom: SetParameter(ADVERT_DATA)
36ms   Restart timer (5000ms)
5036ms Next sensor read
```

## Best Practices

### State Management
1. Always check current state before transitions
2. Clean up resources when leaving a state
3. Use atomic operations for state changes
4. Log state transitions for debugging

### Event Handling
1. Process events in order of priority
2. Keep event handlers short and fast
3. Use timers for periodic tasks
4. Avoid blocking operations in event handlers

### Error Recovery
1. Implement graceful degradation
2. Retry failed operations with exponential backoff
3. Log errors for debugging
4. Provide user feedback via UART/LED

### Power Management
1. Use appropriate advertising intervals
2. Reduce sensor update rate when possible
3. Implement sleep mode between events
4. Optimize I2C clock speed

## Debug and Monitoring

### UART Debug Output
```c
PRINT("Beacon: Started advertising\n");
PRINT("Sensor: T=%.2f°C, RH=%.2f%%\n", temp, humidity);
PRINT("UART: Received cmd=0x%02X\n", cmd);
```

### Recommended Debug Points
1. State transitions
2. Configuration changes
3. Sensor read results
4. UART command processing
5. Flash operations
6. Error conditions

### LED Status Indicators (Optional)
- **Slow blink**: ADVERTISING
- **Fast blink**: CONFIG_MODE
- **Solid on**: ERROR
- **Off**: STOPPED

## Conclusion

This state machine provides a robust, event-driven architecture for the BLE beacon with sensor integration. The modular design allows easy extension with new beacon formats, sensors, or communication protocols.

For questions or improvements, please refer to the main README.md or contact support.
