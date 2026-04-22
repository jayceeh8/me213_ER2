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

def read_serial():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"[ENCODER] {line}")

serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

last_command = ""

JOYSTICK_AXIS = 3
DEADZONE = 0.1

while True:
    pygame.event.pump()
    
    estop = gamepad.get_button(0) # Button A
    auto_trigger = gamepad.get_button(2) # Button X

    if estop:
        if last_command != "STOP":
            ser.write("STOP\n".encode())
            print("EMERGENCY STOP")
            last_command = "STOP"
    elif auto_trigger:
        if last_command != "AUTO":
            ser.write("AUTO\n".encode())
            print("Autonomous mode triggered")
            last_command = "AUTO"
    else:
        axis_value = gamepad.get_axis(JOYSTICK_AXIS)

        if abs(axis_value) < DEADZONE:
            axis_value = 0.0
        
        speed = int(axis_value * -256)
        command = f"V{speed}\n"

    if command != last_command:
        ser.write(command.encode())
        print(f"Sent: {command}")
        last_command = command

    time.sleep(0.05)