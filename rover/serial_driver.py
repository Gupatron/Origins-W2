"""
SerialDriver module for sending servo angles over a serial link.
Port and baudrate are configurable via a JSON config file (config.json) in the same folder.
If not specified in config (using keys 'serial_port' and 'baud_rate'), defaults to /dev/ttyUSB0 and 230400.
This module can be imported and used in other scripts, such as servo.py.
"""

import json
import time
import serial
import os

# Default values if not in config
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 230400

# Config file name (assumed in the same directory as this script)
CONFIG_FILE = "config.json"

def load_config():
    """Load config from JSON file in the same directory."""
    config_path = os.path.join(os.path.dirname(__file__), CONFIG_FILE)
    if not os.path.exists(config_path):
        print(f"[config] File {CONFIG_FILE} not found. Using defaults.")
        return {}
    try:
        with open(config_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"[config] Error loading {CONFIG_FILE}: {e}. Using defaults.")
        return {}

class SerialDriver:
    def __init__(self, port=None, baud=None):
        config = load_config()
        self.port = port or config.get("serial_port", DEFAULT_PORT)
        self.baud = baud or config.get("baud_rate", DEFAULT_BAUD)
        self.ser = None
        self._ensure_open()

    def _ensure_open(self):
        if self.ser is not None:
            return
        tries = 0
        while self.ser is None and tries < 10:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            except Exception as e:
                print(f"[serial] Open failed ({e}); retrying...")
                time.sleep(0.5)
                tries += 1
        if self.ser:
            # Give the MCU a moment to reset if applicable
            time.sleep(1.5)

    def send_angle(self, angle_deg: int):
        self._ensure_open()
        if not self.ser:
            return
        # Transmit as single byte (0..255). Assumes angle fits in one byte.
        try:
            self.ser.write(bytes([int(angle_deg) & 0xFF]))
        except Exception as e:
            print(f"[serial] Write error: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None  # Force reopen on next call

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass