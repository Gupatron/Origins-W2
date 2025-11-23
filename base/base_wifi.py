# base_wifi.py
import zenoh
import time
import struct
from base_databuffer import DataBuffer
import sys
import pygame

base_buffer = DataBuffer()

class Message:
    def __init__(self, theta_left: float, theta_right: float, rpm_left: float, rpm_right: float, send_timestamp: float, left_trigger_pressed: bool):
        self.theta_left = theta_left
        self.theta_right = theta_right
        self.rpm_left = rpm_left
        self.rpm_right = rpm_right
        self.send_timestamp = send_timestamp
        self.left_trigger_pressed = left_trigger_pressed

    def to_binary(self) -> bytes:
        return struct.pack('ddddd?', self.theta_left, self.theta_right, self.rpm_left, self.rpm_right, self.send_timestamp, self.left_trigger_pressed)

    @classmethod
    def from_binary(cls, data: bytes) -> 'Message':
        return cls(*struct.unpack('ddddd?', data))

def append_controller_to_buffer(controller):
    current_time = time.time()
    # Read left stick X and map to -40 to 40 with deadzone
    left_stick_x = controller.get_axis(0)
    left_theta = left_stick_x * 40
    if abs(left_theta) <= 1:
        left_theta = 0

    # Read right stick X and map to -40 to 40 with deadzone
    right_stick_x = controller.get_axis(3)
    right_theta = right_stick_x * 40
    if abs(right_theta) <= 1:
        right_theta = 0

    # Read triggers (map from -1..1 to 0..1)
    left_trigger = (controller.get_axis(2) + 1) / 2
    right_trigger = (controller.get_axis(5) + 1) / 2

    # Map triggers to 0-100
    left_rpm = left_trigger * 100
    right_rpm = right_trigger * 100

    # Boolean flag for left trigger pressed
    left_trigger_pressed = left_trigger > 0

    with base_buffer.lock:
        base_buffer.Theta_Left.append(left_theta)
        base_buffer.Theta_Right.append(right_theta)
        base_buffer.RPM_Left.append(left_rpm)
        base_buffer.RPM_Right.append(right_rpm)
        base_buffer.Left_Trigger_Pressed.append(left_trigger_pressed)
        base_buffer.Send_Timestamp.append(current_time)

def get_latest_message_from_buffer() -> Message | None:
    with base_buffer.lock:
        if not base_buffer.Theta_Left:
            return None
        return Message(
            base_buffer.Theta_Left[-1],
            base_buffer.Theta_Right[-1],
            base_buffer.RPM_Left[-1],
            base_buffer.RPM_Right[-1],
            base_buffer.Send_Timestamp[-1],
            base_buffer.Left_Trigger_Pressed[-1]
        )

def send_message(pub, msg: Message):
    if msg:
        pub.put(msg.to_binary())

def print_latest():
    with base_buffer.lock:
        if not base_buffer.Theta_Left:
            return
        pressed_val = 1 if base_buffer.Left_Trigger_Pressed[-1] else 0
        print(f"{{{base_buffer.Theta_Left[-1]}, {base_buffer.Theta_Right[-1]}, {base_buffer.RPM_Left[-1]}, {base_buffer.RPM_Right[-1]}, {pressed_val}}}")

if __name__ == "__main__":
    # Initialize Pygame
    pygame.init()

    # Initialize the joystick subsystem
    pygame.joystick.init()

    # Check for connected joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick connected.")
        pygame.quit()
        sys.exit()

    # Assume the first joystick is the PS4 controller
    controller = pygame.joystick.Joystick(0)
    controller.init()

    print("PS4 Controller connected: ", controller.get_name())

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

    try:
        while True:
            # Pump events to keep the joystick responsive
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            append_controller_to_buffer(controller)
            msg = get_latest_message_from_buffer()
            send_message(pub, msg)
            print_latest()
            time.sleep(print_frequency)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        sub.undeclare()
        pub.undeclare()
        session.close()
        controller.quit()
        pygame.quit()