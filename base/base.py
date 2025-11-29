# base.py
import threading
import zenoh
import time
import pygame
from base_databuffer import DataBuffer
from controller import Controller
from base_wifi import Message, send_message, get_latest_message_from_buffer, print_latest, reply_callback

def controller_thread(buffer: DataBuffer):
    try:
        controller = Controller(databuffer=buffer, frequency=20.0)
        controller.run()
    except Exception as e:
        print(f"Controller thread error: {e}")

def sender_thread(buffer: DataBuffer, pub):
    send_frequency = 0.05  # Adjusted to match controller frequency (20Hz); user can tune this
    try:
        while True:
            msg = get_latest_message_from_buffer(buffer)
            if msg:
                send_message(pub, msg)
                print_latest(buffer)
            time.sleep(send_frequency)
    except Exception as e:
        print(f"Sender thread error: {e}")

if __name__ == "__main__":
    # Initialize Pygame in main thread
    pygame.init()
    pygame.joystick.init()

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
    t_sender = threading.Thread(target=sender_thread, args=(buffer, pub))

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
        pygame.quit()