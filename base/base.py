# base.py
import threading
import zenoh
import time
import struct
import pygame
from base_databuffer import DataBuffer
import controller_config

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

def send_message(pub, msg: Message):
    if msg:
        pub.put(msg.to_binary())

def print_latest(buffer: DataBuffer):
    with buffer.lock:
        if not buffer.Omega:
            return
        print(f"{{{buffer.Omega[-1]}, {buffer.Theta[-1]}, {buffer.look_theta[-1]}}}")

def reply_callback(sample):
    print(f"Received reply: {sample.payload.to_bytes().decode('utf-8')}")

class Controller:
    def __init__(self, databuffer: DataBuffer, frequency: float = 20.0):
        self.buffer = databuffer
        self.loop_period = 1.0 / frequency
        self.look_theta = 0.0

        # Tunable parameters
        self.MAX_OMEGA = 200.0
        self.MAX_LOOK_RATE = 180.0
        self.STICK_DEADZONE = 0.12
        self.TRIGGER_DEADZONE = 0.08

        # Initialize Pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected!")

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        print(f"Controller connected: {self.joy.get_name()}")

        # Axis mapping
        if controller_config.IS_PS4:
            self.AXIS_LEFT_X     = 0
            self.AXIS_RIGHT_X    = 3
            self.AXIS_LEFT_TRIG  = 2
            self.AXIS_RIGHT_TRIG = 5
        else:  # Xbox / Generic
            self.AXIS_LEFT_X     = 0
            self.AXIS_RIGHT_X    = 4
            self.AXIS_TRIGGERS   = 2

        self.is_ps4 = controller_config.IS_PS4

    def _apply_deadzone(self, value: float, deadzone: float) -> float:
        return 0.0 if abs(value) < deadzone else value

    def run(self):
        while True:
            pygame.event.pump()

            # === Left Stick → Steering Theta (50° → 130°) ===
            left_x = self._apply_deadzone(self.joy.get_axis(self.AXIS_LEFT_X), self.STICK_DEADZONE)
            theta = 90.0 + 40.0 * left_x

            # === Right Stick → Look Theta (wraps 0–360°) ===
            right_x = self._apply_deadzone(self.joy.get_axis(self.AXIS_RIGHT_X), self.STICK_DEADZONE)
            delta_look = right_x * self.MAX_LOOK_RATE * self.loop_period
            self.look_theta = (self.look_theta + delta_look) % 360.0

            # === Triggers → Omega ===
            if self.is_ps4:
                lt_raw = (self.joy.get_axis(self.AXIS_LEFT_TRIG) + 1.0) / 2.0
                rt_raw = (self.joy.get_axis(self.AXIS_RIGHT_TRIG) + 1.0) / 2.0
            else:
                trig = self.joy.get_axis(self.AXIS_TRIGGERS)
                lt_raw = max(0.0, trig)
                rt_raw = max(0.0, -trig)

            lt = max(0.0, lt_raw - self.TRIGGER_DEADZONE) / (1.0 - self.TRIGGER_DEADZONE)
            rt = max(0.0, rt_raw - self.TRIGGER_DEADZONE) / (1.0 - self.TRIGGER_DEADZONE)

            if lt > 0.1 and rt > 0.1:
                omega = 0.0
            elif lt > 0.1:
                omega = -lt * self.MAX_OMEGA
            elif rt > 0.1:
                omega = rt * self.MAX_OMEGA
            else:
                omega = 0.0

            left_trigger_pressed = lt > 0.1

            # === Write to buffer ===
            with self.buffer.lock:
                now = time.time()
                self.buffer.Theta.append(theta)
                self.buffer.Omega.append(omega)
                self.buffer.look_theta.append(self.look_theta)
                self.buffer.Left_Trigger_Pressed.append(left_trigger_pressed)
                self.buffer.Motor_RPM.append(omega)
                self.buffer.RPM_Left.append(omega if omega < 0 else 0)
                self.buffer.RPM_Right.append(omega if omega > 0 else 0)
                self.buffer.Send_Timestamp.append(now)

                for lst in (self.buffer.Wheel1_RPM, self.buffer.Wheel2_RPM,
                            self.buffer.Wheel3_RPM, self.buffer.Wheel4_RPM,
                            self.buffer.Drifting, self.buffer.Theta_Left,
                            self.buffer.Theta_Right):
                    lst.append(0)

            # === Debug output ===
            print(f"{{{omega}, {theta}, {self.look_theta}}}")

            time.sleep(self.loop_period)

def controller_thread(buffer: DataBuffer):
    try:
        controller = Controller(databuffer=buffer, frequency=20.0)
        controller.run()
    except Exception as e:
        print(f"Controller thread error: {e}")
    finally:
        pygame.quit()

def sender_thread(buffer: DataBuffer, session, pub):
    last_sent_ts = 0.0
    send_frequency = 0.01  # Check every 10ms, but only send on new data
    try:
        while True:
            sent = False
            with buffer.lock:
                if buffer.Send_Timestamp and buffer.Send_Timestamp[-1] > last_sent_ts:
                    msg = Message(
                        buffer.Omega[-1],
                        buffer.Theta[-1],
                        buffer.look_theta[-1]
                    )
                    last_sent_ts = buffer.Send_Timestamp[-1]
                    send_message(pub, msg)
                    print_latest(buffer)
                    sent = True
            if not sent:
                time.sleep(send_frequency)
    except Exception as e:
        print(f"Sender thread error: {e}")

if __name__ == "__main__":
    # Open Zenoh session
    conf = zenoh.Config()
    session = zenoh.open(conf)

    # Declare key expressions
    to_rover_key = 'base/to/rover'
    from_rover_key = 'rover/to/base'

    # Subscriber for replies from rover
    sub = session.declare_subscriber(from_rover_key, reply_callback)

    # Publisher for sending to rover
    pub = session.declare_publisher(to_rover_key)

    # Shared data buffer
    buffer = DataBuffer()

    # Create threads
    t_controller = threading.Thread(target=controller_thread, args=(buffer,))
    t_sender = threading.Thread(target=sender_thread, args=(buffer, session, pub))

    # Start threads
    t_controller.start()
    t_sender.start()

    try:
        # Main thread sleeps to keep program running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Clean up Zenoh
        sub.undeclare()
        pub.undeclare()
        session.close()