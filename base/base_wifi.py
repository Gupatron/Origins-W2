# base_wifi.py
import zenoh
import time
import struct
from base_databuffer import DataBuffer

base_buffer = DataBuffer()

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

def append_dummy_to_buffer():
    with base_buffer.lock:
        base_buffer.Omega.append(10.0)
        base_buffer.Theta.append(20.0)
        base_buffer.look_theta.append(30.0)

def get_latest_message_from_buffer() -> Message | None:
    with base_buffer.lock:
        if not base_buffer.Omega:
            return None
        return Message(
            base_buffer.Omega[-1],
            base_buffer.Theta[-1],
            base_buffer.look_theta[-1]
        )

def send_message(pub, msg: Message):
    if msg:
        pub.put(msg.to_binary())

def print_latest():
    with base_buffer.lock:
        if not base_buffer.Omega:
            return
        print(f"{{{base_buffer.Omega[-1]}, {base_buffer.Theta[-1]}, {base_buffer.look_theta[-1]}}}")

if __name__ == "__main__":
    # Open Zenoh session (default config should work over WiFi)
    conf = zenoh.Config()
    session = zenoh.open(conf)

    # Declare key expressions
    to_rover_key = 'base/to/rover'
    from_rover_key = 'rover/to/base'

    # Subscriber for replies from rover
    def reply_callback(sample):
        print(f"Received reply: {sample.payload.to_bytes().decode('utf-8')}")

    sub = session.declare_subscriber(from_rover_key, reply_callback)

    # Publisher for sending to rover
    pub = session.declare_publisher(to_rover_key)

    print_frequency = 0.005  # User can change this value (in seconds)

    counter = 0.0

    try:
        while True:
            with base_buffer.lock:
                base_buffer.Omega.append(10.0 + counter)
                base_buffer.Theta.append(20.0 + counter)
                base_buffer.look_theta.append(30.0 + counter)
            msg = get_latest_message_from_buffer()
            send_message(pub, msg)
            print_latest()
            time.sleep(print_frequency)
            counter += 1.0
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        sub.undeclare()
        pub.undeclare()
        session.close()