# rover_wifi.py
import zenoh
import time
import struct
from rover_databuffer import DataBuffer
import sys

rover_buffer = DataBuffer()

class Message:
    def __init__(self, omega: float, theta: float, look_theta: float):
        self.omega = omega
        self.theta = theta
        self.look_theta = look_theta

    def to_binary(self) -> bytes:
        return struct.pack('ddd', self.omega, self.theta, self.look_theta)

    @classmethod
    def from_binary(cls, data: bytes) -> 'Message':
        return cls(*struct.unpack('ddd', data))

def append_to_buffer(msg: Message):
    with rover_buffer.lock:
        rover_buffer.Omega.append(msg.omega)
        rover_buffer.Theta.append(msg.theta)
        rover_buffer.look_theta.append(msg.look_theta)

def print_latest():
    with rover_buffer.lock:
        if not rover_buffer.Omega:
            return
        print(f"{{{rover_buffer.Omega[-1]}, {rover_buffer.Theta[-1]}, {rover_buffer.look_theta[-1]}}}")

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
    try:
        while True:
            print_latest()
            time.sleep(print_frequency)
    except KeyboardInterrupt:
        pass

    # Clean up
    sub.undeclare()
    pub.undeclare()
    session.close()
