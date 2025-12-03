
import zenoh
import struct
from rover_databuffer import DataBuffer
import sys

# Global DataBuffer instance (shared with anything that imports this module)
rover_buffer = DataBuffer()

# Global Zenoh objects
session = None
sub = None
pub = None

class Message:
    def __init__(self, omega: float, theta: float, look_theta: float):
        self.omega = omega
        self.theta = theta
        self.look_theta = look_theta

    def to_binary(self) -> bytes:
        return struct.pack('ddd', self.omega, self.theta, self.look_theta)

    @classmethod
    def from_binary(cls, data: bytes) -> 'Message':
        if len(data) != 24:
            raise ValueError(f"Incorrect data size: expected 24 bytes, got {len(data)}")
        return cls(*struct.unpack('ddd', data))

def append_to_buffer(msg: Message):
    """Appends message data to the global rover_buffer safely."""
    with rover_buffer.lock:
        rover_buffer.Omega.append(msg.omega)
        rover_buffer.Theta.append(msg.theta)
        rover_buffer.look_theta.append(msg.look_theta)
        
        # Optional: Limit buffer size to prevent memory leaks over long runs
        # Keeps only the last 1000 items
        if len(rover_buffer.Omega) > 1000:
            rover_buffer.Omega.pop(0)
            rover_buffer.Theta.pop(0)
            rover_buffer.look_theta.pop(0)

def init_zenoh():
    """Initializes the Zenoh session and subscribers."""
    global session, sub, pub
    
    # 1. Config and Session
    conf = zenoh.Config()
    session = zenoh.open(conf)

    # 2. Define Callback
    def message_callback(sample):
        try:
            msg = Message.from_binary(sample.payload.to_bytes())
            append_to_buffer(msg)
        except Exception as e:
            print(f"[Wifi Error] decoding packet: {e}")

    # 3. Declare Pub/Sub
    to_rover_key = 'base/to/rover'
    from_rover_key = 'rover/to/base'
    
    sub = session.declare_subscriber(to_rover_key, message_callback)
    pub = session.declare_publisher(from_rover_key)
    
    print(f"Zenoh connected. Listening on '{to_rover_key}'...")

def close_zenoh():
    """Cleans up the Zenoh session."""
    global sub, pub, session
    if sub: sub.undeclare()
    if pub: pub.undeclare()
    if session: session.close()
    print("Zenoh session closed.")

# This block only runs if you run 'python rover_wifi.py' directly.
# It allows this file to still work as a standalone listener for testing.
if __name__ == "__main__":
    import time
    init_zenoh()
    try:
        while True:
            # Simple print loop for standalone testing
            with rover_buffer.lock:
                if rover_buffer.Omega:
                    print(f"Latest: {rover_buffer.Omega[-1]}, {rover_buffer.Theta[-1]}")
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        close_zenoh()
