import socket
import gpiod
import time
import threading

# GPIO pin numbers
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22
SERVO_PIN = 18

# Open GPIO chip
chip = gpiod.Chip('gpiochip0')

# Get GPIO lines
dir_line = chip.get_line(DIR_PIN)
step_line = chip.get_line(STEP_PIN)
enable_line = chip.get_line(ENABLE_PIN)
servo_line = chip.get_line(SERVO_PIN)

# Set up GPIO lines
dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)
servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)

# Socket configuration
server_ip = '192.168.110.140'
server_port = 9999

# Global control variables
task_in_progress = False  # Flag to ensure sequential task execution


# Stepper motor control
def move_motor(duration, direction, delay):
    """Move the motor in a specified direction for a specific duration."""
    dir_line.set_value(direction)  # 0 for CW, 1 for CCW
    enable_line.set_value(0)  # Enable the motor
    start_time = time.time()

    while time.time() - start_time < duration:
        step_line.set_value(1)
        time.sleep(delay)
        step_line.set_value(0)
        time.sleep(delay)

    enable_line.set_value(1)  # Disable the motor


# Servo motor control
def set_servo(angle, duration):
    """Move the servo motor to a specified angle and hold for a duration."""
    pulse_width = (angle / 270) * (0.0025 - 0.0005) + 0.0005
    start_time = time.time()

    while time.time() - start_time < duration:
        servo_line.set_value(1)
        time.sleep(pulse_width)
        servo_line.set_value(0)
        time.sleep(0.02 - pulse_width)


# Handle command from socket
def handle_command(command):
    """Handle the received command."""
    global task_in_progress

    if task_in_progress:
        print("Task already in progress. Ignoring new command.")
        return

    if command == b'0':
        task_in_progress = True
        print("OK: Starting motor and servo actions (reverse of ERROR).")

        # Stepper motor and servo actions for `0` command
        motor_thread = threading.Thread(target=move_motor, args=(2, 0, 0.001))  # 2 seconds, CCW, 1ms delay
        servo_thread = threading.Thread(target=set_servo, args=(90, 2))  # 2 seconds to 90 degrees

        motor_thread.start()
        servo_thread.start()

        motor_thread.join()  # Wait for motor to finish
        servo_thread.join()  # Wait for servo to finish

        # Return the servo to its original position
        print("Returning servo to original position.")
        set_servo(135, 1)  # 135 degrees, hold for 1 second

        print("Task complete.")
        task_in_progress = False

    elif command == b'1':
        task_in_progress = True
        print("ERROR: Starting motor and servo actions.")

        # Stepper motor and servo actions for `1` command
        motor_thread = threading.Thread(target=move_motor, args=(2, 0, 0.001))  # 2 seconds, CW, 1ms delay
        servo_thread = threading.Thread(target=set_servo, args=(180, 2))  # 2 seconds to 180 degrees

        motor_thread.start()
        servo_thread.start()

        motor_thread.join()  # Wait for motor to finish
        servo_thread.join()  # Wait for servo to finish

        # Return the servo to its original position
        print("Returning servo to original position.")
        set_servo(135, 1)  # 135 degrees, hold for 1 second

        print("Task complete.")
        task_in_progress = False

    else:
        print("Unknown command received.")


# Main server loop
def main():
    global task_in_progress

    # Set up server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((server_ip, server_port))
    server_socket.listen(1)
    print(f"Waiting for connection on {server_ip}:{server_port}...")
    connection, address = server_socket.accept()
    print(f"Connected by {address}")

    try:
        while True:
            data = connection.recv(1024)
            if not data:
                break
            handle_command(data)
    except KeyboardInterrupt:
        print("Server interrupted.")
    finally:
        connection.close()
        server_socket.close()
        print("Server closed.")

        # Release GPIO resources
        enable_line.set_value(1)  # Disable motor
        dir_line.release()
        step_line.release()
        enable_line.release()
        servo_line.release()
        print("GPIO resources released.")


if __name__ == "__main__":
    main()
