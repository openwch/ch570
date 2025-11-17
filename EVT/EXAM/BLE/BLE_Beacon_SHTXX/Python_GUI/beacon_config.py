#!/usr/bin/env python3
"""
WCH CH570 BLE Beacon Configuration Tool
Inspired by Inplay nanoBeacon IN100 GUI

Author: WCH
Version: 1.0
Date: 2025-01-17
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import struct
import json
import threading
import time
from datetime import datetime

# Configuration constants
CONFIG_MAGIC = 0x57434848  # "WCH"

# Beacon modes
BEACON_MODES = {
    0: "Eddystone-UID",
    1: "Eddystone-URL",
    2: "Eddystone-TLM (with sensor)",
    3: "iBeacon",
    4: "Custom Sensor Broadcast"
}

# UART Commands
CMD_GET_CONFIG = 0x01
CMD_SET_CONFIG = 0x02
CMD_SAVE_CONFIG = 0x03
CMD_RESET_CONFIG = 0x04
CMD_GET_SENSOR = 0x05
CMD_START_BEACON = 0x10
CMD_STOP_BEACON = 0x11

# Response codes
RESP_OK = 0x00
RESP_ERROR = 0xFF
RESP_INVALID_CMD = 0xFE
RESP_INVALID_PARAM = 0xFD

# URL schemes for Eddystone
URL_SCHEMES = {
    0: "http://www.",
    1: "https://www.",
    2: "http://",
    3: "https://"
}

URL_ENCODINGS = {
    ".com/": 0x00,
    ".org/": 0x01,
    ".edu/": 0x02,
    ".net/": 0x03,
    ".info/": 0x04,
    ".biz/": 0x05,
    ".gov/": 0x06,
    ".com": 0x07,
    ".org": 0x08,
    ".edu": 0x09,
    ".net": 0x0A,
    ".info": 0x0B,
    ".biz": 0x0C,
    ".gov": 0x0D
}


class BeaconConfig:
    """Beacon configuration data structure"""

    def __init__(self):
        self.magic = CONFIG_MAGIC
        self.beacon_mode = 0
        self.sensor_enabled = 1
        self.tx_power = 0
        self.adv_interval = 1600
        self.namespace_id = bytes([0] * 10)
        self.instance_id = bytes([0] * 6)
        self.url_scheme = 1
        self.url = b"wch.cn"
        self.url_length = len(self.url)
        self.ibeacon_uuid = bytes([0] * 16)
        self.ibeacon_major = 0
        self.ibeacon_minor = 0
        self.sensor_update_rate = 5
        self.shtxx_addr = 0x44
        self.shtxx_repeatability = 2
        self.reserved = bytes([0] * 8)

    def to_bytes(self):
        """Convert configuration to bytes"""
        data = struct.pack('<I', self.magic)
        data += struct.pack('<B', self.beacon_mode)
        data += struct.pack('<B', self.sensor_enabled)
        data += struct.pack('<b', self.tx_power)
        data += struct.pack('<H', self.adv_interval)
        data += self.namespace_id[:10]
        data += self.instance_id[:6]
        data += struct.pack('<B', self.url_scheme)
        url_padded = self.url[:17] + bytes([0] * (17 - len(self.url)))
        data += url_padded
        data += struct.pack('<B', self.url_length)
        data += self.ibeacon_uuid[:16]
        data += struct.pack('<H', self.ibeacon_major)
        data += struct.pack('<H', self.ibeacon_minor)
        data += struct.pack('<B', self.sensor_update_rate)
        data += struct.pack('<B', self.shtxx_addr)
        data += struct.pack('<B', self.shtxx_repeatability)
        data += self.reserved[:8]
        return data

    def from_bytes(self, data):
        """Parse configuration from bytes"""
        offset = 0
        self.magic = struct.unpack('<I', data[offset:offset + 4])[0]
        offset += 4
        self.beacon_mode = struct.unpack('<B', data[offset:offset + 1])[0]
        offset += 1
        self.sensor_enabled = struct.unpack('<B', data[offset:offset + 1])[0]
        offset += 1
        self.tx_power = struct.unpack('<b', data[offset:offset + 1])[0]
        offset += 1
        self.adv_interval = struct.unpack('<H', data[offset:offset + 2])[0]
        offset += 2
        self.namespace_id = data[offset:offset + 10]
        offset += 10
        self.instance_id = data[offset:offset + 6]
        offset += 6
        self.url_scheme = struct.unpack('<B', data[offset:offset + 1])[0]
        offset += 1
        self.url = data[offset:offset + 17].rstrip(b'\x00')
        offset += 17
        self.url_length = struct.unpack('<B', data[offset:offset + 1])[0]
        offset += 1
        self.ibeacon_uuid = data[offset:offset + 16]
        offset += 16
        self.ibeacon_major = struct.unpack('<H', data[offset:offset + 2])[0]
        offset += 2
        self.ibeacon_minor = struct.unpack('<H', data[offset:offset + 2])[0]
        offset += 2
        self.sensor_update_rate = struct.unpack('<B', data[offset:offset + 1])[0]
        offset += 1
        self.shtxx_addr = struct.unpack('<B', data[offset:offset + 1])[0]
        offset += 1
        self.shtxx_repeatability = struct.unpack('<B', data[offset:offset + 1])[0]
        offset += 1
        self.reserved = data[offset:offset + 8]

    def to_json(self):
        """Convert to JSON-serializable dict"""
        return {
            "beacon_mode": self.beacon_mode,
            "beacon_mode_name": BEACON_MODES.get(self.beacon_mode, "Unknown"),
            "sensor_enabled": bool(self.sensor_enabled),
            "tx_power_dbm": self.tx_power,
            "adv_interval_ms": int(self.adv_interval * 0.625),
            "eddystone_uid": {
                "namespace_id": self.namespace_id.hex(),
                "instance_id": self.instance_id.hex()
            },
            "eddystone_url": {
                "scheme": self.url_scheme,
                "url": self.url.decode('ascii', errors='ignore')
            },
            "ibeacon": {
                "uuid": self.ibeacon_uuid.hex(),
                "major": self.ibeacon_major,
                "minor": self.ibeacon_minor
            },
            "sensor": {
                "update_rate_sec": self.sensor_update_rate,
                "i2c_address": f"0x{self.shtxx_addr:02X}",
                "repeatability": self.shtxx_repeatability
            }
        }

    def from_json(self, data):
        """Load from JSON dict"""
        self.beacon_mode = data.get("beacon_mode", 0)
        self.sensor_enabled = 1 if data.get("sensor_enabled", True) else 0
        self.tx_power = data.get("tx_power_dbm", 0)
        self.adv_interval = int(data.get("adv_interval_ms", 1000) / 0.625)

        if "eddystone_uid" in data:
            self.namespace_id = bytes.fromhex(data["eddystone_uid"]["namespace_id"])
            self.instance_id = bytes.fromhex(data["eddystone_uid"]["instance_id"])

        if "eddystone_url" in data:
            self.url_scheme = data["eddystone_url"]["scheme"]
            self.url = data["eddystone_url"]["url"].encode('ascii')
            self.url_length = len(self.url)

        if "ibeacon" in data:
            self.ibeacon_uuid = bytes.fromhex(data["ibeacon"]["uuid"])
            self.ibeacon_major = data["ibeacon"]["major"]
            self.ibeacon_minor = data["ibeacon"]["minor"]

        if "sensor" in data:
            self.sensor_update_rate = data["sensor"]["update_rate_sec"]
            addr_str = data["sensor"]["i2c_address"]
            self.shtxx_addr = int(addr_str, 16) if addr_str.startswith("0x") else int(addr_str)
            self.shtxx_repeatability = data["sensor"]["repeatability"]


class BeaconConfigGUI:
    """Main GUI Application"""

    def __init__(self, root):
        self.root = root
        self.root.title("WCH CH570 Beacon Configuration Tool")
        self.root.geometry("900x700")

        self.serial_port = None
        self.config = BeaconConfig()
        self.sensor_monitor_running = False

        self.setup_ui()

    def setup_ui(self):
        """Setup the user interface"""
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Connection tab
        self.connection_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.connection_frame, text="Connection")
        self.setup_connection_tab()

        # Beacon Configuration tab
        self.beacon_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.beacon_frame, text="Beacon Config")
        self.setup_beacon_tab()

        # Sensor tab
        self.sensor_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.sensor_frame, text="Sensor Monitor")
        self.setup_sensor_tab()

        # Status bar
        self.status_bar = ttk.Label(self.root, text="Disconnected", relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def setup_connection_tab(self):
        """Setup connection tab"""
        # COM Port selection
        port_frame = ttk.LabelFrame(self.connection_frame, text="UART Connection", padding=10)
        port_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(port_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(port_frame, width=30, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        self.refresh_ports()

        ttk.Button(port_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        ttk.Button(port_frame, text="Connect", command=self.connect).grid(row=0, column=3, padx=5)
        ttk.Button(port_frame, text="Disconnect", command=self.disconnect).grid(row=0, column=4, padx=5)

        # Connection status
        self.connection_status = ttk.Label(port_frame, text="Status: Disconnected", foreground="red")
        self.connection_status.grid(row=1, column=0, columnspan=5, pady=5)

        # Device info
        info_frame = ttk.LabelFrame(self.connection_frame, text="Device Information", padding=10)
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.info_text = tk.Text(info_frame, height=15, width=80)
        self.info_text.pack(fill=tk.BOTH, expand=True)

        # Buttons
        btn_frame = ttk.Frame(self.connection_frame)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Button(btn_frame, text="Read Config", command=self.read_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Write Config", command=self.write_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Save to Flash", command=self.save_to_flash).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Reset to Defaults", command=self.reset_config).pack(side=tk.LEFT, padx=5)

    def setup_beacon_tab(self):
        """Setup beacon configuration tab"""
        # Beacon Mode
        mode_frame = ttk.LabelFrame(self.beacon_frame, text="Beacon Mode", padding=10)
        mode_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(mode_frame, text="Mode:").grid(row=0, column=0, sticky=tk.W)
        self.mode_combo = ttk.Combobox(mode_frame, width=30, state='readonly',
                                       values=list(BEACON_MODES.values()))
        self.mode_combo.grid(row=0, column=1, padx=5, pady=5)
        self.mode_combo.current(0)
        self.mode_combo.bind("<<ComboboxSelected>>", self.on_mode_change)

        ttk.Label(mode_frame, text="TX Power (dBm):").grid(row=1, column=0, sticky=tk.W)
        self.tx_power_spin = ttk.Spinbox(mode_frame, from_=-20, to=4, width=10)
        self.tx_power_spin.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)
        self.tx_power_spin.set(0)

        ttk.Label(mode_frame, text="Adv Interval (ms):").grid(row=2, column=0, sticky=tk.W)
        self.adv_interval_spin = ttk.Spinbox(mode_frame, from_=100, to=10000, width=10, increment=100)
        self.adv_interval_spin.grid(row=2, column=1, sticky=tk.W, padx=5, pady=5)
        self.adv_interval_spin.set(1000)

        self.sensor_enable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(mode_frame, text="Enable Sensor Data in Broadcast",
                       variable=self.sensor_enable_var).grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=5)

        # Configuration panels (will be shown/hidden based on mode)
        self.config_panels = {}

        # Eddystone-UID panel
        self.config_panels['eddystone_uid'] = self.create_eddystone_uid_panel()

        # Eddystone-URL panel
        self.config_panels['eddystone_url'] = self.create_eddystone_url_panel()

        # iBeacon panel
        self.config_panels['ibeacon'] = self.create_ibeacon_panel()

        # Sensor settings panel
        self.create_sensor_settings_panel()

        # File operations
        file_frame = ttk.LabelFrame(self.beacon_frame, text="Configuration File", padding=10)
        file_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Button(file_frame, text="Load from JSON", command=self.load_from_json).pack(side=tk.LEFT, padx=5)
        ttk.Button(file_frame, text="Save to JSON", command=self.save_to_json).pack(side=tk.LEFT, padx=5)

        # Show default panel
        self.on_mode_change()

    def create_eddystone_uid_panel(self):
        """Create Eddystone-UID configuration panel"""
        frame = ttk.LabelFrame(self.beacon_frame, text="Eddystone-UID Parameters", padding=10)
        frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(frame, text="Namespace ID (10 bytes hex):").grid(row=0, column=0, sticky=tk.W)
        self.namespace_entry = ttk.Entry(frame, width=30)
        self.namespace_entry.grid(row=0, column=1, padx=5, pady=5)
        self.namespace_entry.insert(0, "00010203040506070809")

        ttk.Label(frame, text="Instance ID (6 bytes hex):").grid(row=1, column=0, sticky=tk.W)
        self.instance_entry = ttk.Entry(frame, width=30)
        self.instance_entry.grid(row=1, column=1, padx=5, pady=5)
        self.instance_entry.insert(0, "000000000001")

        return frame

    def create_eddystone_url_panel(self):
        """Create Eddystone-URL configuration panel"""
        frame = ttk.LabelFrame(self.beacon_frame, text="Eddystone-URL Parameters", padding=10)
        frame.pack(fill=tk.X, padx=10, pady=5)
        frame.pack_forget()  # Hide initially

        ttk.Label(frame, text="URL Scheme:").grid(row=0, column=0, sticky=tk.W)
        self.url_scheme_combo = ttk.Combobox(frame, width=20, state='readonly',
                                             values=list(URL_SCHEMES.values()))
        self.url_scheme_combo.grid(row=0, column=1, padx=5, pady=5)
        self.url_scheme_combo.current(1)  # https://www.

        ttk.Label(frame, text="URL (without scheme):").grid(row=1, column=0, sticky=tk.W)
        self.url_entry = ttk.Entry(frame, width=30)
        self.url_entry.grid(row=1, column=1, padx=5, pady=5)
        self.url_entry.insert(0, "wch.cn")

        return frame

    def create_ibeacon_panel(self):
        """Create iBeacon configuration panel"""
        frame = ttk.LabelFrame(self.beacon_frame, text="iBeacon Parameters", padding=10)
        frame.pack(fill=tk.X, padx=10, pady=5)
        frame.pack_forget()  # Hide initially

        ttk.Label(frame, text="UUID (16 bytes hex):").grid(row=0, column=0, sticky=tk.W)
        self.ibeacon_uuid_entry = ttk.Entry(frame, width=40)
        self.ibeacon_uuid_entry.grid(row=0, column=1, padx=5, pady=5)
        self.ibeacon_uuid_entry.insert(0, "00000000000000000000000000000000")

        ttk.Label(frame, text="Major:").grid(row=1, column=0, sticky=tk.W)
        self.ibeacon_major_spin = ttk.Spinbox(frame, from_=0, to=65535, width=10)
        self.ibeacon_major_spin.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)
        self.ibeacon_major_spin.set(0)

        ttk.Label(frame, text="Minor:").grid(row=2, column=0, sticky=tk.W)
        self.ibeacon_minor_spin = ttk.Spinbox(frame, from_=0, to=65535, width=10)
        self.ibeacon_minor_spin.grid(row=2, column=1, sticky=tk.W, padx=5, pady=5)
        self.ibeacon_minor_spin.set(0)

        return frame

    def create_sensor_settings_panel(self):
        """Create sensor settings panel"""
        frame = ttk.LabelFrame(self.beacon_frame, text="Sensor Settings", padding=10)
        frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(frame, text="Update Rate (seconds):").grid(row=0, column=0, sticky=tk.W)
        self.sensor_update_spin = ttk.Spinbox(frame, from_=1, to=60, width=10)
        self.sensor_update_spin.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)
        self.sensor_update_spin.set(5)

        ttk.Label(frame, text="I2C Address:").grid(row=1, column=0, sticky=tk.W)
        self.i2c_addr_combo = ttk.Combobox(frame, width=10, state='readonly',
                                          values=["0x44", "0x45"])
        self.i2c_addr_combo.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)
        self.i2c_addr_combo.current(0)

        ttk.Label(frame, text="Repeatability:").grid(row=2, column=0, sticky=tk.W)
        self.repeatability_combo = ttk.Combobox(frame, width=10, state='readonly',
                                               values=["Low", "Medium", "High"])
        self.repeatability_combo.grid(row=2, column=1, sticky=tk.W, padx=5, pady=5)
        self.repeatability_combo.current(2)

    def setup_sensor_tab(self):
        """Setup sensor monitoring tab"""
        # Sensor data display
        data_frame = ttk.LabelFrame(self.sensor_frame, text="Current Sensor Readings", padding=10)
        data_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.temp_label = ttk.Label(data_frame, text="Temperature: -- °C", font=('Arial', 16))
        self.temp_label.pack(pady=10)

        self.humidity_label = ttk.Label(data_frame, text="Humidity: -- %", font=('Arial', 16))
        self.humidity_label.pack(pady=10)

        self.battery_label = ttk.Label(data_frame, text="Battery: -- mV", font=('Arial', 14))
        self.battery_label.pack(pady=10)

        self.uptime_label = ttk.Label(data_frame, text="Uptime: --", font=('Arial', 14))
        self.uptime_label.pack(pady=10)

        # Control buttons
        btn_frame = ttk.Frame(self.sensor_frame)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Button(btn_frame, text="Read Once", command=self.read_sensor).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Start Monitor", command=self.start_sensor_monitor).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Stop Monitor", command=self.stop_sensor_monitor).pack(side=tk.LEFT, padx=5)

        ttk.Button(btn_frame, text="Start Beacon", command=self.start_beacon).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Stop Beacon", command=self.stop_beacon).pack(side=tk.LEFT, padx=5)

    def on_mode_change(self, event=None):
        """Handle beacon mode change"""
        mode_name = self.mode_combo.get()
        mode_idx = list(BEACON_MODES.values()).index(mode_name)

        # Hide all panels
        for panel in self.config_panels.values():
            panel.pack_forget()

        # Show relevant panel
        if mode_idx == 0:  # Eddystone-UID
            self.config_panels['eddystone_uid'].pack(fill=tk.X, padx=10, pady=5)
        elif mode_idx == 1:  # Eddystone-URL
            self.config_panels['eddystone_url'].pack(fill=tk.X, padx=10, pady=5)
        elif mode_idx == 3:  # iBeacon
            self.config_panels['ibeacon'].pack(fill=tk.X, padx=10, pady=5)

    def refresh_ports(self):
        """Refresh available COM ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{port.device} - {port.description}" for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)

    def connect(self):
        """Connect to serial port"""
        if self.serial_port and self.serial_port.is_open:
            messagebox.showwarning("Warning", "Already connected!")
            return

        port_str = self.port_combo.get()
        if not port_str:
            messagebox.showerror("Error", "Please select a COM port")
            return

        port = port_str.split(" - ")[0]

        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            self.connection_status.config(text="Status: Connected", foreground="green")
            self.status_bar.config(text=f"Connected to {port}")
            messagebox.showinfo("Success", f"Connected to {port}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {str(e)}")
            self.serial_port = None

    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.connection_status.config(text="Status: Disconnected", foreground="red")
            self.status_bar.config(text="Disconnected")
            messagebox.showinfo("Info", "Disconnected")

    def send_command(self, cmd, data=None):
        """Send command to device"""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Not connected!")
            return None

        # Build packet
        packet = bytearray([0xAA, 0x55, cmd, 0x00])  # Header + cmd + status

        # Add data length and data
        if data:
            packet.extend(struct.pack('>H', len(data)))
            packet.extend(data)
        else:
            packet.extend(struct.pack('>H', 0))

        # Calculate checksum
        checksum = 0
        for byte in packet:
            checksum ^= byte
        packet.append(checksum)

        # Send packet
        self.serial_port.write(packet)
        time.sleep(0.1)

        # Read response
        response = self.serial_port.read(1000)
        if len(response) >= 7:
            # Parse response
            if response[0] == 0xAA and response[1] == 0x55:
                resp_cmd = response[2]
                resp_status = response[3]
                resp_len = struct.unpack('>H', response[4:6])[0]
                resp_data = response[6:6 + resp_len] if resp_len > 0 else None

                return resp_status, resp_data

        return None, None

    def read_config(self):
        """Read configuration from device"""
        status, data = self.send_command(CMD_GET_CONFIG)

        if status == RESP_OK and data:
            try:
                self.config.from_bytes(data)
                self.update_ui_from_config()
                self.info_text.delete(1.0, tk.END)
                self.info_text.insert(tk.END, json.dumps(self.config.to_json(), indent=2))
                messagebox.showinfo("Success", "Configuration read from device")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to parse config: {str(e)}")
        else:
            messagebox.showerror("Error", "Failed to read configuration from device")

    def write_config(self):
        """Write configuration to device"""
        self.update_config_from_ui()

        status, _ = self.send_command(CMD_SET_CONFIG, self.config.to_bytes())

        if status == RESP_OK:
            messagebox.showinfo("Success", "Configuration written to device")
        else:
            messagebox.showerror("Error", "Failed to write configuration")

    def save_to_flash(self):
        """Save configuration to device flash"""
        status, _ = self.send_command(CMD_SAVE_CONFIG)

        if status == RESP_OK:
            messagebox.showinfo("Success", "Configuration saved to flash")
        else:
            messagebox.showerror("Error", "Failed to save configuration")

    def reset_config(self):
        """Reset device configuration to defaults"""
        if messagebox.askyesno("Confirm", "Reset configuration to defaults?"):
            status, _ = self.send_command(CMD_RESET_CONFIG)

            if status == RESP_OK:
                messagebox.showinfo("Success", "Configuration reset to defaults")
                self.read_config()
            else:
                messagebox.showerror("Error", "Failed to reset configuration")

    def read_sensor(self):
        """Read sensor data from device"""
        status, data = self.send_command(CMD_GET_SENSOR)

        if status == RESP_OK and data and len(data) >= 12:
            temp = struct.unpack('<h', data[0:2])[0] / 100.0
            humidity = struct.unpack('<H', data[2:4])[0] / 100.0
            battery = struct.unpack('<H', data[4:6])[0]
            uptime = struct.unpack('<I', data[6:10])[0]

            self.temp_label.config(text=f"Temperature: {temp:.2f} °C")
            self.humidity_label.config(text=f"Humidity: {humidity:.2f} %")
            self.battery_label.config(text=f"Battery: {battery} mV")

            hours = uptime // 3600
            minutes = (uptime % 3600) // 60
            seconds = uptime % 60
            self.uptime_label.config(text=f"Uptime: {hours:02d}:{minutes:02d}:{seconds:02d}")

        else:
            messagebox.showerror("Error", "Failed to read sensor data")

    def start_sensor_monitor(self):
        """Start continuous sensor monitoring"""
        if not self.sensor_monitor_running:
            self.sensor_monitor_running = True
            threading.Thread(target=self.sensor_monitor_thread, daemon=True).start()
            messagebox.showinfo("Info", "Sensor monitoring started")

    def stop_sensor_monitor(self):
        """Stop sensor monitoring"""
        self.sensor_monitor_running = False
        messagebox.showinfo("Info", "Sensor monitoring stopped")

    def sensor_monitor_thread(self):
        """Sensor monitoring thread"""
        while self.sensor_monitor_running:
            self.read_sensor()
            time.sleep(2)

    def start_beacon(self):
        """Start beacon advertising"""
        status, _ = self.send_command(CMD_START_BEACON)

        if status == RESP_OK:
            messagebox.showinfo("Success", "Beacon started")
        else:
            messagebox.showerror("Error", "Failed to start beacon")

    def stop_beacon(self):
        """Stop beacon advertising"""
        status, _ = self.send_command(CMD_STOP_BEACON)

        if status == RESP_OK:
            messagebox.showinfo("Success", "Beacon stopped")
        else:
            messagebox.showerror("Error", "Failed to stop beacon")

    def update_ui_from_config(self):
        """Update UI elements from configuration"""
        self.mode_combo.current(self.config.beacon_mode)
        self.tx_power_spin.set(self.config.tx_power)
        self.adv_interval_spin.set(int(self.config.adv_interval * 0.625))
        self.sensor_enable_var.set(bool(self.config.sensor_enabled))

        self.namespace_entry.delete(0, tk.END)
        self.namespace_entry.insert(0, self.config.namespace_id.hex())

        self.instance_entry.delete(0, tk.END)
        self.instance_entry.insert(0, self.config.instance_id.hex())

        self.url_scheme_combo.current(self.config.url_scheme)
        self.url_entry.delete(0, tk.END)
        self.url_entry.insert(0, self.config.url.decode('ascii', errors='ignore'))

        self.ibeacon_uuid_entry.delete(0, tk.END)
        self.ibeacon_uuid_entry.insert(0, self.config.ibeacon_uuid.hex())

        self.ibeacon_major_spin.set(self.config.ibeacon_major)
        self.ibeacon_minor_spin.set(self.config.ibeacon_minor)

        self.sensor_update_spin.set(self.config.sensor_update_rate)
        self.i2c_addr_combo.current(0 if self.config.shtxx_addr == 0x44 else 1)
        self.repeatability_combo.current(self.config.shtxx_repeatability)

        self.on_mode_change()

    def update_config_from_ui(self):
        """Update configuration from UI elements"""
        mode_name = self.mode_combo.get()
        self.config.beacon_mode = list(BEACON_MODES.values()).index(mode_name)

        self.config.tx_power = int(self.tx_power_spin.get())
        self.config.adv_interval = int(int(self.adv_interval_spin.get()) / 0.625)
        self.config.sensor_enabled = 1 if self.sensor_enable_var.get() else 0

        try:
            self.config.namespace_id = bytes.fromhex(self.namespace_entry.get())[:10]
            self.config.instance_id = bytes.fromhex(self.instance_entry.get())[:6]
        except:
            pass

        self.config.url_scheme = self.url_scheme_combo.current()
        url_str = self.url_entry.get()
        self.config.url = url_str.encode('ascii')
        self.config.url_length = len(self.config.url)

        try:
            self.config.ibeacon_uuid = bytes.fromhex(self.ibeacon_uuid_entry.get())[:16]
        except:
            pass

        self.config.ibeacon_major = int(self.ibeacon_major_spin.get())
        self.config.ibeacon_minor = int(self.ibeacon_minor_spin.get())

        self.config.sensor_update_rate = int(self.sensor_update_spin.get())
        self.config.shtxx_addr = 0x44 if self.i2c_addr_combo.current() == 0 else 0x45
        self.config.shtxx_repeatability = self.repeatability_combo.current()

    def load_from_json(self):
        """Load configuration from JSON file"""
        filename = filedialog.askopenfilename(
            title="Load Configuration",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )

        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    self.config.from_json(data)
                    self.update_ui_from_config()
                    messagebox.showinfo("Success", f"Configuration loaded from {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load configuration: {str(e)}")

    def save_to_json(self):
        """Save configuration to JSON file"""
        self.update_config_from_ui()

        filename = filedialog.asksaveasfilename(
            title="Save Configuration",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )

        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.config.to_json(), f, indent=2)
                    messagebox.showinfo("Success", f"Configuration saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save configuration: {str(e)}")


def main():
    root = tk.Tk()
    app = BeaconConfigGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
