import pygame
import time
import threading
import asyncio
from bleak import BleakClient, BleakScanner

# ======================
# BLE CONFIG
# ======================
DEVICE_NAME = "=CONNOR"
UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

ble_client = None
ble_loop = None
ble_ready = threading.Event()
stop_event = threading.Event()

# ======================
# GAMEPAD SETUP
# ======================
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
DEADZONE = 0.6

BUTTON_A = 1
BUTTON_X = 0
BUTTON_Y = 3
BUTTON_B = 2
# LEFT_BUMPER = 4
# RIGHT_BUMPER = 5

# 🔥 NEW: trigger axes (VERIFY THESE!)
LEFT_TRIGGER_AXIS = 4
RIGHT_TRIGGER_AXIS = 5

TRIGGER_DEADZONE = 0.05
ARM_MAX_SPEED = 255

def apply_deadzone(value):
    return 0.0 if abs(value) < DEADZONE else value

# ======================
# BLE FUNCTIONS
# ======================
def notification_handler(sender, data):
    print("[Arduino]:", data.decode(errors="ignore").strip())

async def ble_connect():
    global ble_client, ble_loop

    ble_loop = asyncio.get_event_loop()

    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()

    target = None
    for d in devices:
        print(d.name, d.address)
        if d.name and DEVICE_NAME.lower() in d.name.lower():
            target = d
            break

    if not target:
        print("Device not found")
        return

    print(f"Connecting to {target.name} ({target.address})")

    ble_client = BleakClient(target.address)
    await ble_client.connect()

    if not ble_client.is_connected:
        print("Failed to connect")
        return

    print("Connected to BLE!")
    await ble_client.start_notify(UART_CHAR_UUID, notification_handler)

    ble_ready.set()

    while not stop_event.is_set():
        await asyncio.sleep(0.1)

    await ble_client.disconnect()

def start_ble_thread():
    asyncio.run(ble_connect())

ble_thread = threading.Thread(target=start_ble_thread, daemon=True)
ble_thread.start()

# ======================
# SEND FUNCTION
# ======================
def send(cmd):
    global ble_client, ble_loop

    if not (ble_client and ble_client.is_connected and ble_loop):
        print("[!] BLE not connected")
        return

    future = asyncio.run_coroutine_threadsafe(
        ble_client.write_gatt_char(UART_CHAR_UUID, (cmd + "\n").encode()),
        ble_loop
    )

    try:
        future.result(timeout=2)
        print(f"[Sent]: {cmd}")
    except Exception as e:
        print(f"[!] Send error: {e}")

# ======================
# WAIT FOR BLE
# ======================
print("Waiting for BLE connection...")
connected = ble_ready.wait(timeout=30)
if not connected:
    print("BLE connection timed out. Exiting.")
    stop_event.set()
    exit()

print("BLE ready — starting control loop")

# ======================
# MAIN LOOP VARIABLES
# ======================
last_drive_command = ""
last_arm_command = ""

# ======================
# MAIN LOOP
# ======================
try:
    while True:
        pygame.event.pump()

        estop     = gamepad.get_button(BUTTON_A)
        auto_up   = gamepad.get_button(BUTTON_X)
        auto_down = gamepad.get_button(BUTTON_Y)
    
        lt_raw = gamepad.get_axis(LEFT_TRIGGER_AXIS)
        rt_raw = gamepad.get_axis(RIGHT_TRIGGER_AXIS)

        # normalize
        lt_val = (lt_raw + 1) / 2
        rt_val = (rt_raw + 1) / 2

        # deadzone
        lt_val = 0 if lt_val < TRIGGER_DEADZONE else lt_val
        rt_val = 0 if rt_val < TRIGGER_DEADZONE else rt_val

        # smoothing curve
        lt_val = lt_val ** 2
        rt_val = rt_val ** 2

        # convert to speed
        up_speed = int(lt_val * ARM_MAX_SPEED)
        down_speed = int(rt_val * ARM_MAX_SPEED)

        if estop:
            drive_command = "STOP"
            arm_command   = ""

        elif auto_up:
            drive_command = "DRIVE_STOP"
            arm_command   = "AUTO UP"

        elif auto_down:
            drive_command = "DRIVE_STOP"
            arm_command   = "AUTO DOWN"

        else:
            drive  = apply_deadzone(gamepad.get_axis(DRIVE_AXIS))
            steer  = apply_deadzone(gamepad.get_axis(STEER_AXIS))

            left_speed  = int((drive - steer) * -255)
            right_speed = int((drive + steer) * -255)
            left_speed  = max(-255, min(255, left_speed))
            right_speed = max(-255, min(255, right_speed))

            drive_command = f"L{left_speed},{right_speed}"

            if up_speed > 0 and down_speed == 0:
                arm_command = f"ARM_UP:{up_speed}"

            elif down_speed > 0 and up_speed == 0:
                arm_command = f"ARM_DOWN:{down_speed}"
            else:
                arm_command = "ARM_STOP"

        # send only if changed
        if drive_command != last_drive_command:
            send(drive_command)
            last_drive_command = drive_command

        if arm_command != last_arm_command:
            send(arm_command)
            last_arm_command = arm_command

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")
    stop_event.set()