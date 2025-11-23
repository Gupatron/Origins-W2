"""
Servo control thread for Jetson Nano.
Reads desired servo targets from DataBuffer and transmits them over a serial link
(e.g., to an Arduino that actually generates the PWM). Runs in its own thread.
- Accepts either absolute degree input (buffer.servo_angle) or "offset" input
  (buffer.servo_offset in range [-40, +40]) which is mapped to a degree range.
This keeps the GUI and transport decoupled: GUI publishes servo_angle or jogs,
the rover process updates the DataBuffer via Zenoh, and this thread streams
changes to the micro that drives the servo.
If you do not use an external micro and want the Jetson to drive PWM directly,
you can replace the SerialDriver class with a GPIO driver (Jetson.GPIO or pigpio).
"""
import time
import threading
from serial_driver import SerialDriver  # Import from the separate serial_driver.py file

class Steering:
    OFFSET_MIN = -40
    OFFSET_MAX = 40
    SERVO_MIN = 50  # degrees (as understood by your MCU)
    SERVO_MAX = 130  # degrees

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def map_range(x, in_min, in_max, out_min, out_max):
        if in_max == in_min:
            raise ValueError("Input range cannot be zero width")
        frac = (x - in_min) / (in_max - in_min)
        return out_min + frac * (out_max - out_min)

    @staticmethod
    def offset_to_angle(offset):
        o = Steering.clamp(int(offset), Steering.OFFSET_MIN, Steering.OFFSET_MAX)
        angle = Steering.map_range(o, Steering.OFFSET_MIN, Steering.OFFSET_MAX, Steering.SERVO_MIN, Steering.SERVO_MAX)
        return int(round(Steering.clamp(angle, Steering.SERVO_MIN, Steering.SERVO_MAX)))

    def __init__(self, buffer, serial_port=None, baud=None):
        self.buffer = buffer
        self.stop_event = threading.Event()
        self.driver = SerialDriver(port=serial_port, baud=baud)  # Pass None to use config/defaults
        self.last_sent = None
        self.last_offset_seen = None
        self.thread = threading.Thread(target=self._servo_loop, daemon=True)

    def start(self):
        if not self.thread.is_alive():
            self.stop_event.clear()  # Reset stop event if restarting
            self.thread = threading.Thread(target=self._servo_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join()
        self.driver.close()

    def _servo_loop(self):
        try:
            while not self.stop_event.is_set():
                with self.buffer.lock:
                    # Prefer absolute angle if it's been set; otherwise map offset.
                    desired_angle = int(self.buffer.servo_angle)
                    # Track and map offset if GUI uses offsets
                    if self.buffer.servo_offset != self.last_offset_seen:
                        mapped = self.offset_to_angle(self.buffer.servo_offset)
                        # If the GUI only updates offset, keep angle in sync
                        desired_angle = mapped
                        self.buffer.servo_angle = mapped
                        self.last_offset_seen = self.buffer.servo_offset
                    desired_angle = self.clamp(desired_angle, self.SERVO_MIN, self.SERVO_MAX)
                if desired_angle != self.last_sent:
                    self.driver.send_angle(desired_angle)
                    self.last_sent = desired_angle
                # Optional: store a tiny history for plotting/debug
                with self.buffer.lock:
                    self.buffer.servo_angles.append(desired_angle)
                    self.buffer.times.append(time.time())
                time.sleep(0.02)  # 50 Hz update loop
        finally:
            self.driver.close()


# ----------------------------------------------------------------------
# Mock Buffer for standalone testing
# ----------------------------------------------------------------------
class Buffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.servo_angle = 90  # Default neutral position
        self.servo_offset = 0
        self.servo_angles = []
        self.times = []


if __name__ == "__main__":
    buffer = Buffer()
    controller = Steering(buffer)
    controller.start()
    try:
        print("Starting servo test sequence (independent of Zenoh). Ensure hardware is safe to operate.")
        # Sweep from SERVO_MIN to SERVO_MAX in steps of 1
        print("Sweeping servo from 50 to 130 degrees:")
        for angle in range(Steering.SERVO_MIN, Steering.SERVO_MAX + 1):
            with buffer.lock:
                buffer.servo_angle = angle
            print(f"Servo angle: {angle}°")
            time.sleep(0.1)  # Increased delay for observable movement during sweep
        # Discrete moves: 50 -> 90 -> 130
        positions = [50, 90, 130]
        print("\nMoving to discrete positions:")
        for angle in positions:
            with buffer.lock:
                buffer.servo_angle = angle
            print(f"Servo angle: {angle}°")
            time.sleep(1)  # Longer delay to observe each position
        # Move back to 90 at the end
        with buffer.lock:
            buffer.servo_angle = 90
        print("Moving back to neutral position: 90°")
        time.sleep(1)
        print("Test sequence complete. Servo returned to neutral.")
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        controller.stop()