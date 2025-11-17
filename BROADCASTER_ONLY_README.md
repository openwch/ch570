# CH570 Broadcaster-Only Mode Conversion

## Overview

This repository has been converted to **broadcaster-only mode**. The BLE peripheral application now only broadcasts advertising packets and does **not accept connections**.

## What is Broadcaster-Only Mode?

In BLE broadcaster-only mode, the device:
- **Broadcasts** BLE advertising packets continuously
- **Does NOT accept connections** from central devices
- **Does NOT provide GATT services** (no read/write characteristics)
- **Lower power consumption** compared to full peripheral mode
- Useful for beacon applications, sensor broadcasts, etc.

## Changes Made

### 1. Modified Files

#### `/EVT/EXAM/BLE/Peripheral/APP/peripheral.c`

**Key Modifications:**

1. **Advertising Type Changed to Non-Connectable**
   - Set `GAPROLE_ADV_EVENT_TYPE` to `GAP_ADTYPE_ADV_NONCONN_IND`
   - This makes the device broadcast-only (non-connectable)

2. **Device Name Updated**
   - Changed from "Simple Peripheral" to "Broadcaster Only CH570"
   - Visible in BLE scan results

3. **Scan Response Data Modified**
   - Removed connection interval parameters (not needed for broadcaster)
   - Updated device name in scan response
   - Kept TX power level information

4. **Connection Handling Disabled**
   - All connection event handlers commented out
   - No connection parameter updates
   - No PHY updates
   - No RSSI reading (requires connection)

5. **GATT Services Disabled**
   - `GGS_AddService()` - Disabled
   - `GATTServApp_AddService()` - Disabled
   - `SimpleProfile_AddService()` - Disabled
   - Profile characteristics initialization disabled

6. **Bond Manager Disabled**
   - Bonding/pairing configuration removed (no connections = no bonding)

7. **State Change Callback Updated**
   - Only handles advertising states
   - Removed connection state handling
   - Added warning if connection is attempted

## Advertising Data Structure

The device broadcasts the following information:

### Advertising Packet
- **Flags:** General discoverable, BR/EDR not supported
- **Service UUID:** SimpleProfile service UUID (0xFFF0)

### Scan Response Packet
- **Complete Name:** "Broadcaster Only CH570"
- **TX Power Level:** 0 dBm

## How It Works

1. **Initialization** (`main()` in `peripheral_main.c`)
   - BLE stack initialized
   - HAL initialized
   - GAP Peripheral Role initialized
   - Peripheral application initialized

2. **Advertising Starts**
   - Device begins broadcasting non-connectable advertising packets
   - Advertising interval: 50ms (80 * 0.625ms)
   - Broadcasts on all 3 advertising channels (37, 38, 39)

3. **Continuous Broadcasting**
   - Device continuously broadcasts advertising packets
   - No connection events occur
   - Central devices can scan and see the device but cannot connect

## Testing

To verify broadcaster-only mode is working:

1. **Use a BLE Scanner App** (nRF Connect, LightBlue, etc.)
   - Scan for BLE devices
   - Look for "Broadcaster Only CH570"
   - Device should appear as "Non-Connectable" or advertising type 0x03
   - Connection attempts should fail or not be possible

2. **Expected Behavior**
   - ✅ Device appears in BLE scan results
   - ✅ Device name shows "Broadcaster Only CH570"
   - ✅ TX power shows 0 dBm
   - ❌ Cannot connect to device
   - ❌ No GATT services accessible

## Reverting to Peripheral Mode

To revert to full peripheral (connectable) mode:

1. Uncomment all sections marked with `// BROADCASTER MODE:`
2. Change advertising type back to `GAP_ADTYPE_ADV_IND` (0x00)
3. Remove the `GAPROLE_ADV_EVENT_TYPE` parameter setting
4. Restore original device name and scan response data

## Technical Details

### Advertising Parameters
- **Type:** `GAP_ADTYPE_ADV_NONCONN_IND` (0x03)
- **Interval:** 50ms (DEFAULT_ADVERTISING_INTERVAL = 80 * 0.625ms)
- **Channels:** 37, 38, 39 (all advertising channels)
- **Discoverable Mode:** General discoverable

### Disabled Features
- ❌ GATT Server
- ❌ Connection handling
- ❌ Bonding/Pairing
- ❌ Connection parameter updates
- ❌ PHY updates
- ❌ RSSI reading
- ❌ Notifications/Indications
- ❌ Periodic tasks (were for notifications)

### Enabled Features
- ✅ Advertising broadcast
- ✅ Scan response
- ✅ Scan request callbacks
- ✅ Device name in advertising data

## Use Cases

Broadcaster-only mode is ideal for:
- **BLE Beacons** (iBeacon, Eddystone)
- **Sensor Broadcasting** (temperature, humidity, etc.)
- **Asset Tracking** (location beacons)
- **Proximity Detection** (presence detection)
- **One-way Data Broadcasting** (announcements, status)

## Notes

- This conversion maintains the same BLE library and HAL
- The main loop and TMOS event system remain unchanged
- Low-power sleep mode can still be configured in `CONFIG.h`
- Advertising can be started/stopped programmatically if needed

## References

- CH570/CH572 BLE Development Manual (Chinese): `/EVT/EXAM/BLE/沁恒低功耗蓝牙软件开发参考手册.PDF`
- Datasheet: `/Datasheet/CH572DS1_en.PDF`
- BLE Core Specification v5.0+

---

**Modified:** 2025
**Original Author:** WCH (Nanjing Qinheng Microelectronics)
**Modification:** Broadcaster-Only Mode Conversion
