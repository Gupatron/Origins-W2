#!/usr/bin/env python3
"""
VESC motor control – using firmware PID for RPM control
"""
import time
import glob
import serial
import threading
import json
from pyvesc import SetDutyCycle, SetRPM, encode

# ----------------------------------------------------------------------
# Config (same file rover.py uses)
# ----------------------------------------------------------------------
config = json.load(open('config.json'))
BAUD = 230400
POLE_PAIRS = config.get('pole_pairs', 23.65)  # keep old default
MAX_DUTY = config.get('max_duty_percent', 30) / 100.0  # default 30 %
RAMP_TIME = config.get('ramp_time_s', 0.3)
MAX_ERPM = 100000  # Example max ERPM; adjust based on motor specs

# ----------------------------------------------------------------------
# CRC / packing helpers
# ----------------------------------------------------------------------
def _crc16_xmodem(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def pack_comm(payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("Use long frame for payload >255")
    start = b"\x02"
    length = bytes([len(payload)])
    crc = _crc16_xmodem(payload)
    crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    end = b"\x03"
    return start + length + payload + crc_bytes + end

# ----------------------------------------------------------------------
# Port discovery
# ----------------------------------------------------------------------
def pick_port():
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        raise SystemExit("No VESC serial ports found.")
    return ports[0]

# ----------------------------------------------------------------------
# Smooth ramp for duty or RPM
# ----------------------------------------------------------------------
def ramp_command(ser, serial_lock, start, stop, ramp_time, is_duty=True):
    steps = max(1, int(ramp_time / 0.05))
    for i in range(steps + 1):
        val = start + (stop - start) * (i / steps)
        try:
            with serial_lock:
                if is_duty:
                    ser.write(encode(SetDutyCycle(int(val * 100000))))
                else:
                    ser.write(encode(SetRPM(int(val))))
                ser.flush()
            time.sleep(0.05)
        except serial.SerialException:
            break

# ----------------------------------------------------------------------
# Throttle class – uses VESC firmware for inner loop PID
# ----------------------------------------------------------------------
class Throttle:
    def __init__(self, ser, serial_lock, cfg, feedback):
        self.ser = ser
        self.serial_lock = serial_lock
        self.cfg = cfg
        self.feedback = feedback
        self.lock = threading.Lock()
        self.running = False
        self.use_rpm_mode = False  # False: duty mode, True: RPM mode
        self.target_duty = 0.0
        self.target_erpm = 0
        self.current_command = 0.0  # Current duty or ERPM
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

    def _control_loop(self):
        period = 1.0 / self.cfg.get('vese_frequency', 200)  # Aim for 200 Hz
        while not self.stop_event.is_set():
            loop_start = time.perf_counter()
            # Get target
            with self.lock:
                running = self.running
                target = self.target_erpm if self.use_rpm_mode else self.target_duty
            # Cap targets
            if self.use_rpm_mode:
                target = max(min(target, MAX_ERPM), -MAX_ERPM)
            else:
                target = max(min(target, MAX_DUTY), -MAX_DUTY)
            # Ramp if starting/stopping
            if running and not getattr(self, "_last_running", False):
                ramp_command(self.ser, self.serial_lock,
                             getattr(self, "_last_command", 0), target, RAMP_TIME,
                             is_duty=not self.use_rpm_mode)
            elif not running and getattr(self, "_last_running", False):
                ramp_command(self.ser, self.serial_lock,
                             getattr(self, "_last_command", 0), 0, RAMP_TIME,
                             is_duty=not self.use_rpm_mode)
            else:
                # Send command
                try:
                    with self.serial_lock:
                        if self.use_rpm_mode:
                            self.ser.write(encode(SetRPM(int(target))))
                        else:
                            self.ser.write(encode(SetDutyCycle(int(target * 100000))))
                        self.ser.flush()
                except serial.SerialException as e:
                    print(f"[CONTROL] write error: {e}")
            # Bookkeeping
            self._last_running = running
            self._last_command = target
            with self.lock:
                self.current_command = target
            # Tight timing
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, period - elapsed))

    def set_duty(self, desired_duty):
        with self.lock:
            self.use_rpm_mode = False
            self.target_duty = desired_duty
            self.running = abs(desired_duty) > 0

    def set_rpm(self, desired_rpm):
        erpm = desired_rpm * POLE_PAIRS
        with self.lock:
            self.use_rpm_mode = True
            self.target_erpm = erpm
            self.running = abs(desired_rpm) > 0
        # In firmware PID, error is handled internally; approximate error from feedback
        current_rpm = self.feedback.get_rpm()
        error = desired_rpm - current_rpm
        return error, current_rpm

    def get_current_command(self):
        with self.lock:
            return self.current_command

    def stop(self):
        self.stop_event.set()
        self.thread.join()

# ----------------------------------------------------------------------
# MotorRpmFeedback class – poll at 100 Hz since ESC returns at 100 Hz
# ----------------------------------------------------------------------
class MotorRpmFeedback:
    def __init__(self, ser, serial_lock, cfg):
        self.ser = ser
        self.serial_lock = serial_lock
        self.cfg = cfg
        self.lock = threading.Lock()
        self.omega_erpm = 0
        self.omega_rpm = 0
        self.update_count = 0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._rpm_loop, daemon=True)
        self.thread.start()

    def _rpm_loop(self):
        period = 1.0 / 100  # Set to 100 Hz as per ESC limit
        pkt = pack_comm(b"\x04")
        while not self.stop_event.is_set():
            loop_start = time.perf_counter()
            # Request
            try:
                with self.serial_lock:
                    self.ser.write(pkt)
                    self.ser.flush()
                with self.serial_lock:
                    buf = self.ser.read(512)
            except serial.SerialException as e:
                print(f"[RPM] serial error: {e}")
                buf = b""
            # Parse
            if buf and len(buf) >= 29:
                erpm = int.from_bytes(buf[25:29], "big", signed=True)
                rpm = erpm / POLE_PAIRS
                with self.lock:
                    self.omega_erpm = erpm
                    self.omega_rpm = rpm
                    self.update_count += 1
            # Timing
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, period - elapsed))

    def get_rpm(self):
        with self.lock:
            return self.omega_rpm

    def stop(self):
        self.stop_event.set()
        self.thread.join()

# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
if __name__ == "__main__":
    port = pick_port()
    ser = serial.Serial(port, BAUD, timeout=0.005)
    serial_lock = threading.Lock()
    feedback = MotorRpmFeedback(ser, serial_lock, config)
    throttle = Throttle(ser, serial_lock, config, feedback)
    
    try:
        print("=== VESC Duty Cycle Input with RPM Feedback at 100Hz ===")
        duty = float(input("Enter duty cycle (e.g., 0.05 for 5%): "))
        throttle.set_duty(duty)
        print(f"Set duty to {duty}. Printing RPM at 100Hz (press Ctrl+C to stop)...")
        
        while True:
            print(f"Current RPM: {feedback.get_rpm():.2f}")
            time.sleep(0.01)  # 100Hz
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        throttle.set_duty(0.0)  # Ramp to stop
        time.sleep(RAMP_TIME + 0.1)  # Wait for ramp down
        throttle.stop()
        feedback.stop()
        ser.close()