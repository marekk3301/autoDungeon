import time
import threading
import queue
import RPi.GPIO as GPIO
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Constants
PCA9685_ADDRESS = 0x40
NEUTRAL_THROTTLE = 0.2
UP_THROTTLE = 0.5
DOWN_THROTTLE = -0.5

# GPIO pins for hall sensors
HALL_SENSOR_PINS = [5, 6, 7, 8]

# Number of servos
NUM_SERVOS = 4

# Command queue
command_queue = queue.Queue()

# Servo state tracking
servo_states = {i: {'target': None, 'current': 'stopped', 'throttle': NEUTRAL_THROTTLE, 'position': 0} for i in range(1, NUM_SERVOS + 1)}

# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Set frequency to 50 Hz for servo control

# Initialize servos
continuous_servos = [servo.ContinuousServo(pca.channels[i]) for i in range(NUM_SERVOS)]

# Initialize GPIO
def init_gpio():
    GPIO.setmode(GPIO.BCM)
    for pin in HALL_SENSOR_PINS:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Command parser
def command_parser(command):
    command = command.strip().split()
    if len(command) == 0:
        return
    cmd = command[0].lower()
    if cmd == 'move':
        if len(command) != 3:
            print("Usage: move <servo_id> <up|down>")
            return
        try:
            servo_id = int(command[1])
            direction = command[2].lower()
            if direction == 'up':
                throttle = UP_THROTTLE
            elif direction == 'down':
                throttle = DOWN_THROTTLE
            else:
                print("Invalid direction. Use 'up' or 'down'.")
                return
            if 1 <= servo_id <= NUM_SERVOS:
                command_queue.put(('move', servo_id, throttle))
            else:
                print(f"Invalid servo ID. Use values between 1 and {NUM_SERVOS}.")
        except ValueError:
            print("Invalid servo ID. Use an integer.")
    elif cmd == 'stop':
        if len(command) == 1:
            command_queue.put(('stop',))
        elif len(command) == 2:
            try:
                servo_id = int(command[1])
                if 1 <= servo_id <= NUM_SERVOS:
                    command_queue.put(('stop', servo_id))
                else:
                    print(f"Invalid servo ID. Use values between 1 and {NUM_SERVOS}.")
            except ValueError:
                print("Invalid servo ID. Use an integer.")
        else:
            print("Usage: stop [servo_id]")
    elif cmd == 'home':
        command_queue.put(('home',))
    elif cmd == 'print':
        if len(command) == 1:
            for servo_id, state in servo_states.items():
                print(f"Servo {servo_id}: Position = {state['position']}")
        elif len(command) == 2:
            try:
                servo_id = int(command[1])
                if 1 <= servo_id <= NUM_SERVOS:
                    print(f"Servo {servo_id}: Position = {servo_states[servo_id]['position']}")
                else:
                    print(f"Invalid servo ID. Use values between 1 and {NUM_SERVOS}.")
            except ValueError:
                print("Invalid servo ID. Use an integer.")
        else:
            print("Usage: print [servo_id]")
    else:
        print("Unknown command. Use 'move <servo_id> <up|down>', 'stop [servo_id]', 'home', or 'print [servo_id]'.")

# Servo control thread
def servo_control_thread():
    while True:
        command = command_queue.get()
        if command[0] == 'move':
            servo_id, throttle = command[1], command[2]
            if servo_states[servo_id]['current'] == 'stopped':
                servo_states[servo_id]['current'] = 'moving'
                servo_states[servo_id]['throttle'] = throttle
                continuous_servos[servo_id - 1].throttle = throttle
                print(f"Servo {servo_id} moving {'up' if throttle == UP_THROTTLE else 'down'}")
            else:
                print(f"Servo {servo_id} is already moving.")
        elif command[0] == 'stop':
            if len(command) == 1:
                for servo_id in servo_states:
                    if servo_states[servo_id]['current'] == 'moving':
                        servo_states[servo_id]['current'] = 'stopped'
                        servo_states[servo_id]['throttle'] = NEUTRAL_THROTTLE
                        continuous_servos[servo_id - 1].throttle = NEUTRAL_THROTTLE
                        print(f"Servo {servo_id} stopped.")
            else:
                servo_id = command[1]
                if servo_states[servo_id]['current'] == 'moving':
                    servo_states[servo_id]['current'] = 'stopped'
                    servo_states[servo_id]['throttle'] = NEUTRAL_THROTTLE
                    continuous_servos[servo_id - 1].throttle = NEUTRAL_THROTTLE
                    print(f"Servo {servo_id} stopped.")
        elif command[0] == 'home':
            print("Homing all servos...")
            for servo_id in servo_states:
                servo_states[servo_id]['current'] = 'homing'
                servo_states[servo_id]['throttle'] = UP_THROTTLE
                continuous_servos[servo_id - 1].throttle = UP_THROTTLE
            while any(state['current'] == 'homing' for state in servo_states.values()):
                for servo_id in servo_states:
                    if servo_states[servo_id]['current'] == 'homing':
                        if read_hall_sensor(HALL_SENSOR_PINS[servo_id - 1]):
                            servo_states[servo_id]['current'] = 'stopped'
                            servo_states[servo_id]['throttle'] = NEUTRAL_THROTTLE
                            continuous_servos[servo_id - 1].throttle = NEUTRAL_THROTTLE
                            servo_states[servo_id]['position'] = 0
                            print(f"Servo {servo_id} homed.")
                time.sleep(0.01)
            print("All servos homed.")

# Sensor reading thread
def sensor_reading_thread():
    last_sensor_state = [False] * NUM_SERVOS
    while True:
        for servo_id in servo_states:
            if servo_states[servo_id]['current'] == 'moving':
                current_sensor_state = read_hall_sensor(HALL_SENSOR_PINS[servo_id - 1])
                if current_sensor_state and not last_sensor_state[servo_id - 1]:
                    if servo_states[servo_id]['throttle'] == UP_THROTTLE:
                        servo_states[servo_id]['position'] += 1
                    elif servo_states[servo_id]['throttle'] == DOWN_THROTTLE:
                        servo_states[servo_id]['position'] -= 1
#                    print(f"Servo {servo_id}: Position = {servo_states[servo_id]['position']}")
                last_sensor_state[servo_id - 1] = current_sensor_state
        time.sleep(0.01)

# Read hall sensor
def read_hall_sensor(pin):
    return not GPIO.input(pin)  # Adjusted to handle active low sensors

# User input thread
def user_input_thread():
    while True:
        command = input()
        command_parser(command)

# Main function
def main():
    init_gpio()

    # Start threads
    threading.Thread(target=servo_control_thread, daemon=True).start()
    threading.Thread(target=sensor_reading_thread, daemon=True).start()
    threading.Thread(target=user_input_thread, daemon=True).start()

    # Keep the main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
        for servo in continuous_servos:
            servo.throttle = NEUTRAL_THROTTLE
        GPIO.cleanup()
        pca.deinit()

if __name__ == "__main__":
    main()
