# rover_wifi.py
import zenoh
import time
import struct
from rover_databuffer import DataBuffer
import sys

rover_buffer = DataBuffer()

class Message:
    def __init__(self, motor_rpm: float, wheel1_rpm: float, wheel2_rpm: float, wheel3_rpm: float, wheel4_rpm: float, theta: float, send_timestamp: float, drifting: bool):
        self.motor_rpm = motor_rpm
        self.wheel1_rpm = wheel1_rpm
        self.wheel2_rpm = wheel2_rpm
        self.wheel3_rpm = wheel3_rpm
        self.wheel4_rpm = wheel4_rpm
        self.theta = theta
        self.send_timestamp = send_timestamp
        self.drifting = drifting

    def to_binary(self) -> bytes:
        return struct.pack('ddddddd?', self.motor_rpm, self.wheel1_rpm, self.wheel2_rpm, self.wheel3_rpm, self.wheel4_rpm, self.theta, self.send_timestamp, self.drifting)

    @classmethod
    def from_binary(cls, data: bytes) -> 'Message':
        return cls(*struct.unpack('ddddddd?', data))

def append_to_buffer(msg: Message):
    current_time = time.time()
    with rover_buffer.lock:
        rover_buffer.Motor_RPM.append(msg.motor_rpm)
        rover_buffer.Wheel1_RPM.append(msg.wheel1_rpm)
        rover_buffer.Wheel2_RPM.append(msg.wheel2_rpm)
        rover_buffer.Wheel3_RPM.append(msg.wheel3_rpm)
        rover_buffer.Wheel4_RPM.append(msg.wheel4_rpm)
        rover_buffer.Theta.append(msg.theta)
        rover_buffer.Send_Timestamp.append(msg.send_timestamp)
        rover_buffer.Receive_Timestamp.append(current_time)
        rover_buffer.Drifting.append(msg.drifting)

def print_latest(elapsed):
    with rover_buffer.lock:
        if not rover_buffer.Motor_RPM:
            return
        drifting_val = 1 if rover_buffer.Drifting[-1] else 0
        print(f"{{{rover_buffer.Motor_RPM[-1]}, {rover_buffer.Wheel1_RPM[-1]}, {rover_buffer.Wheel2_RPM[-1]}, {rover_buffer.Wheel3_RPM[-1]}, {rover_buffer.Wheel4_RPM[-1]}, {rover_buffer.Theta[-1]}, {drifting_val}, {elapsed:.1f} s}}")

if __name__ == "__main__":
    # Open Zenoh session (default config should work over WiFi)
    conf = zenoh.Config()
    session = zenoh.open(conf)

    # Declare key expressions
    to_rover_key = 'base/to/rover'
    from_rover_key = 'rover/to/base'

    # Publisher for sending replies to base
    pub = session.declare_publisher(from_rover_key)

    # Subscriber for messages from base
    def message_callback(sample):
        msg = Message.from_binary(sample.payload.to_bytes())
        append_to_buffer(msg)

    sub = session.declare_subscriber(to_rover_key, message_callback)

    # Keep running to listen (use Ctrl+C to stop, or adjust for production)
    print("Rover listening...")
    print_frequency = 0.005  # User can change this value (in seconds)
    elapsed = 0.0
    try:
        while True:
            print_latest(elapsed)
            time.sleep(print_frequency)
            elapsed += print_frequency
    except KeyboardInterrupt:
        pass

    # Clean up
    sub.undeclare()
    pub.undeclare()
    session.close()
