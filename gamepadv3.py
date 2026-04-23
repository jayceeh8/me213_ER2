import pygame
import serial
import time
import threading

SERIAL_PORT = 'COM3'
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

def apply_deadzone(value):
    return 0.0 if abs(value) < DEADZONE else value

def read_serial():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"[ENCODER] {line}")

serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

last_drive_command = ""
last_arm_command = ""

DRIVE_AXIS = 3
STEER_AXIS = 0
DEADZONE = 0.1

BUTTON_A = 0
BUTTON_X = 2
LEFT_BUMPER = 4
RIGHT_BUMPER = 5

while True:
    pygame.event.pump()
    
    estop = gamepad.get_button(BUTTON_A) 
    auto_trigger = gamepad.get_button(BUTTON_X)
    arm_up = gamepad.get_button(LEFT_BUMPER)
    arm_down = gamepad.get_button(RIGHT_BUMPER)

    if estop:
        if drive_command != "STOP" or arm_command != "STOP":
            ser.write("STOP\n".encode())
            print("EMERGENCY STOP DRIVE")
            drive_command = "STOP"
            arm_command = "STOP"
            last_drive_command = "STOP"
            last_arm_command = "STOP"
    elif auto_trigger:
        if last_arm_command != "AUTO":
            ser.write("AUTO\n".encode())
            print("Autonomous mode triggered")
            arm_command = "AUTO"
            last_arm_command = "AUTO"
    else:
        drive = apply_deadzone(gamepad.get_axis(DRIVE_AXIS))
        steer = apply_deadzone(gamepad.get_axis(STEER_AXIS))

        # CHANGE SIGN HERE IF MOTORS ARE SPINNING THE WRONG WAY
        left_speed = int((drive - steer) * -256)
        right_speed = int((drive + steer) * -255)

        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))

        drive_command = f"L{left_speed}, R{right_speed}\n"
        
        if arm_up and not arm_down:
            arm_command = "ARM_DOWN"
            print("Arm up manual")
        elif arm_down and not arm_up:
            arm_command = "ARM UP"
            print("Arm down manual")
        else:
            arm_command = "ARM_STOP"

    if drive_command != last_drive_command:
        ser.write(drive_command.encode())
        print(f"Sent: {drive_command}")
        last_drive_command = drive_command
    
    if arm_command != last_arm_command:
        ser.write(arm_command.encode())
        print(f"Sent: {arm_command}")
        last_arm_command = arm_command

    time.sleep(0.05)