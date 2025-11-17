# Quick Start Guide
## WCH CH570 BLE Beacon with SHTXX Sensor

Get your beacon up and running in 15 minutes!

## Prerequisites

### Hardware
- [ ] WCH CH570 development board
- [ ] SHTXX sensor (SHT30/SHT31/SHT35)
- [ ] USB-UART adapter (CH340, FT232, etc.)
- [ ] Breadboard and jumper wires
- [ ] Micro-USB cable (for power)

### Software
- [ ] MounRiver Studio IDE (for firmware)
- [ ] WCHISPTool (for flashing)
- [ ] Python 3.7+ (for GUI)
- [ ] BLE Scanner app (nRF Connect or LightBlue)

## Step 1: Hardware Setup (5 minutes)

### Connect SHTXX Sensor

```
CH570 Board          SHTXX Sensor
-----------          ------------
  3.3V     --------->   VDD
  GND      --------->   GND
  PA8      --------->   SDA
  PA9      --------->   SCL
```

**Important Notes:**
- Ensure solid connections
- Use short wires (<10 cm) for I2C
- Check sensor ADDR pin (LOW=0x44, HIGH=0x45)

### Connect USB-UART Adapter

```
CH570 Board          USB-UART Adapter
-----------          ----------------
  PA3 (TX) --------->   RX
  PA2 (RX) --------->   TX
  GND      --------->   GND
```

**Important Notes:**
- Cross-connect TX/RX
- Don't connect VCC (power from USB)

### Power Connection

Connect CH570 board to PC via Micro-USB for power.

## Step 2: Build and Flash Firmware (5 minutes)

### Option A: Using Pre-built Binary (Fastest)

1. Download `beacon_shtxx.hex` from releases
2. Open WCHISPTool
3. Select CH570 chip
4. Load .hex file
5. Click "Download"
6. Wait for completion

### Option B: Build from Source

1. Open MounRiver Studio
2. File → Open Projects from File System
3. Navigate to `BLE_Beacon_SHTXX` folder
4. Select `BLE_Beacon_SHTXX.wvproj`
5. Project → Build Project
6. Flash using WCHISPTool

## Step 3: Install Python GUI (3 minutes)

```bash
cd Python_GUI
pip install -r requirements.txt
```

That's it! Just pyserial is needed.

## Step 4: First Run (2 minutes)

### Start the GUI

```bash
python beacon_config.py
```

### Connect to Device

1. Click "Refresh" to scan COM ports
2. Select your USB-UART port (usually highest number)
3. Click "Connect"
4. You should see "Status: Connected" in green

### Read Current Configuration

1. Click "Read Config" button
2. Configuration should appear in the text area
3. UI fields should populate automatically

**If you see an error:**
- Check UART connections (TX↔RX crossed)
- Verify correct COM port
- Ensure device is powered on
- Try clicking Connect again

## Step 5: Test Beacon (5 minutes)

### Start with Default Configuration

The device boots with Eddystone-UID beacon mode by default.

1. Open BLE scanner app on your phone
   - iOS: LightBlue Explorer
   - Android: nRF Connect

2. Scan for BLE devices

3. Look for:
   - Eddystone-UID beacon
   - Namespace: `00:01:02:03:04:05:06:07:08:09`
   - Instance: `00:00:00:00:00:01`

**You should see it!** 🎉

### Try Different Beacon Modes

#### Eddystone-URL (Website Beacon)

1. In GUI, go to "Beacon Config" tab
2. Select "Eddystone-URL" from Mode dropdown
3. URL Scheme: `https://www.`
4. URL: `wch.cn`
5. Click "Write Config"
6. Click "Start Beacon"

Scan again - you should see a URL beacon!

#### Sensor Beacon (Eddystone-TLM)

1. Go to "Beacon Config" tab
2. Select "Eddystone-TLM (with sensor)"
3. Check "Enable Sensor Data in Broadcast"
4. Click "Write Config"
5. Click "Save to Flash" (persist configuration)

Now go to "Sensor Monitor" tab:
1. Click "Read Once"
2. You should see temperature and humidity!

Scan with BLE app - you should see TLM frame with sensor data.

## Verification Checklist

### Hardware
- [ ] SHTXX sensor detected (check GUI sensor tab)
- [ ] Temperature reading looks reasonable (15-30°C typical)
- [ ] Humidity reading makes sense (30-70% typical)
- [ ] No I2C errors in serial console

### Beacon Broadcasting
- [ ] Beacon visible in BLE scanner
- [ ] RSSI shows reasonable value (-40 to -80 dBm)
- [ ] Beacon name/ID matches configuration
- [ ] Advertising interval approximately correct

### Configuration
- [ ] GUI can read configuration
- [ ] GUI can write configuration
- [ ] Configuration persists after power cycle
- [ ] Sensor settings can be changed

## Common Issues

### "Sensor: Read failed"

**Symptoms:** Error in serial console or GUI

**Solutions:**
1. Check I2C wiring (PA8=SDA, PA9=SCL)
2. Verify 3.3V power to sensor
3. Try alternate I2C address (0x45)
4. Check for loose connections

### "UART: No response"

**Symptoms:** GUI commands timeout

**Solutions:**
1. Verify TX/RX are crossed (TX→RX, RX→TX)
2. Check correct COM port selected
3. Ensure 115200 baud rate
4. Try different USB cable
5. Restart device

### "Beacon not visible"

**Symptoms:** BLE scanner doesn't find beacon

**Solutions:**
1. Ensure "Start Beacon" was clicked
2. Check advertising interval (try 100ms for testing)
3. Increase TX power (try +4 dBm)
4. Refresh BLE scanner
5. Try different scanner app
6. Check device isn't in error state

### "Configuration not saving"

**Symptoms:** Settings lost after power cycle

**Solutions:**
1. Click "Save to Flash" after "Write Config"
2. Wait for success message
3. Don't disconnect power immediately after save
4. Check flash write wasn't blocked

## Next Steps

### Customize Your Beacon

1. **Change Beacon IDs**
   - Edit Namespace/Instance for Eddystone-UID
   - Edit UUID/Major/Minor for iBeacon

2. **Optimize Power**
   - Increase advertising interval
   - Reduce sensor update rate
   - Lower TX power if range permits

3. **Add More Sensors**
   - Modify `beacon.c` to add sensor drivers
   - Update advertising data structure
   - Rebuild firmware

### Create Your Application

1. **iOS App**
   - Use CoreLocation for iBeacon
   - Parse Eddystone frames

2. **Android App**
   - Use Android Beacon Library
   - Scan for Eddystone

3. **Gateway**
   - Raspberry Pi with BLE dongle
   - Scan and upload to cloud
   - Dashboard for monitoring

### Advanced Configuration

See full documentation:
- [README.md](README.md) - Complete documentation
- [STATE_MACHINE.md](STATE_MACHINE.md) - Architecture details
- [Python_GUI/examples/](Python_GUI/examples/) - Configuration templates

## Example Use Cases

### 1. Museum Guide
- **Mode:** Eddystone-URL
- **URL:** Links to exhibit information
- **Interval:** 500ms (responsive)
- **TX Power:** 0 dBm (medium range)

### 2. Environmental Monitor
- **Mode:** Eddystone-TLM
- **Sensor:** Enabled, 10s update
- **Interval:** 2000ms (power efficient)
- **Application:** Read TLM frames for temp/humidity

### 3. Indoor Navigation
- **Mode:** Eddystone-UID
- **Namespace:** Building identifier
- **Instance:** Unique per beacon
- **Interval:** 1000ms (balanced)

### 4. Retail Proximity
- **Mode:** iBeacon
- **UUID:** App identifier
- **Major/Minor:** Store/section
- **Interval:** 100ms (very responsive)

## Performance Tips

### Battery Life Optimization
```
Mode: Eddystone-UID
Interval: 5000ms
TX Power: -12 dBm
Sensor: Disabled or 60s updates
Expected: >1 year on CR2032
```

### Maximum Range
```
Mode: Any
Interval: 1000ms
TX Power: +4 dBm
Antenna: External if possible
Expected: 50-100m open space
```

### Fastest Response
```
Mode: iBeacon
Interval: 100ms
TX Power: 0 dBm
Expected: <100ms detection
```

## Support

### Debug Output

Connect to UART (115200 baud) and monitor:
```
Beacon: Initialized (mode=0, sensor=1)
SHTXX: Sensor initialized (addr=0x44)
Sensor: T=24.35°C, RH=45.23%
Beacon: Started advertising
```

### Community

- **Forum:** https://www.wch.cn/bbs
- **Email:** tech@wch.cn
- **GitHub:** Open issues for bugs

### Documentation

- Full README with all features
- State machine documentation
- API reference
- Example configurations

## Success!

You now have a fully functional BLE beacon with environmental sensing!

**What you can do:**
- ✅ Broadcast multiple beacon formats
- ✅ Read temperature and humidity
- ✅ Configure via GUI
- ✅ Save settings to flash
- ✅ Monitor sensor in real-time

**Next steps:**
- Build your application
- Deploy multiple beacons
- Create IoT solutions
- Share your project!

---

**Need help?** Check the full [README.md](README.md) or ask on the forum!

**Found a bug?** Open an issue on GitHub!

**Built something cool?** Share it with the community!

Happy beaconing! 🚀
