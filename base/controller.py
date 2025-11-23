import pygame
import time

# Initialize Pygame
pygame.init()

# Initialize the joystick subsystem
pygame.joystick.init()

# Check for connected joysticks
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick connected.")
    pygame.quit()
    exit()

# Assume the first joystick is the PS4 controller
controller = pygame.joystick.Joystick(0)
controller.init()

print("PS4 Controller connected: ", controller.get_name())

try:
    while True:
        # Pump events to keep the joystick responsive
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

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

        # Print the values
        print(f"Left Theta: {left_theta:.2f}")
        print(f"Right Theta: {right_theta:.2f}")
        print(f"Left RPM: {left_rpm:.2f}")
        print(f"Right RPM: {right_rpm:.2f}")
        print(f"Left Trigger Pressed: {left_trigger_pressed}")
        print("---")

        # Sleep for a short time to avoid flooding the console
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    controller.quit()
    pygame.quit()