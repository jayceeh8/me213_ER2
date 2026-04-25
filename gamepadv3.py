import pygame
import serial
import time
import threading

SERIAL_PORT = 'COM5'     # Change to your HC-05 COM port
BAUD_RATE = 9600

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No gamepad found")
    exit()

gamepad = pygame.joystick.Joystick(0)
gamepad.init()
print(f"Connected: {gamepad.get_name()}")

DRIVE_AXIS = 3
STEER_AXIS = 0
DEADZONE = 0.1

BUTTON_A = 0
BUTTON_X = 2
LEFT_BUMPER = 4
RIGHT_BUMPER = 5

def apply_deadzone(value):
    return 0.0 if abs(value) < DEADZONE else value

def read_serial():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"[Arduino]: {line}")
        except serial.SerialException:
            print("[!] Bluetooth connection lost.")
            break
        except Exception as e:
            print(f"[!] Read error: {e}")
            break

serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

last_drive_command = ""
last_arm_command = ""
drive_command = ""
arm_command = ""

def send(cmd):
    ser.write((cmd + '\n').encode('utf-8'))
    print(f"[Sent]: {cmd}")

while True:
    pygame.event.pump()

    estop      = gamepad.get_button(BUTTON_A)
    auto_trigger = gamepad.get_button(BUTTON_X)
    arm_up     = gamepad.get_button(LEFT_BUMPER)
    arm_down   = gamepad.get_button(RIGHT_BUMPER)

    if estop:
        drive_command = "STOP"
        arm_command   = ""

    elif auto_trigger:
        drive_command = "DRIVE_STOP"
        arm_command = "AUTO"

    else:
        drive = apply_deadzone(gamepad.get_axis(DRIVE_AXIS))
        steer = apply_deadzone(gamepad.get_axis(STEER_AXIS))

        left_speed  = int((drive - steer) * -255)
        right_speed = int((drive + steer) * -255)

        left_speed  = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))

        drive_command = f"L{left_speed},{right_speed}"

        if arm_up and not arm_down:
            arm_command = "ARM_UP"
        elif arm_down and not arm_up:
            arm_command = "ARM_DOWN"
        else:
            arm_command = "ARM_STOP"

    # Only send if command changed
    if drive_command != last_drive_command:
        send(drive_command)
        last_drive_command = drive_command

    if arm_command != last_arm_command:
        send(arm_command)
        last_arm_command = arm_command

    time.sleep(0.05)