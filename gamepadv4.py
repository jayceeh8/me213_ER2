import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "=CONNOR"
UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


def notification_handler(sender, data):
    print("Received:", data.decode(errors="ignore").strip())


async def main():
    print("Scanning...")

    devices = await BleakScanner.discover()

    target = None
    for d in devices:
        print(d.name, d.address)

        if d.name and DEVICE_NAME.lower() in d.name.lower():
            target = d

    if not target:
        print("Device not found")
        return

    print(f"\nConnecting to {target.name} ({target.address})")

    async with BleakClient(target.address) as client:
        print("Connected!")

        # 🚨 NO get_services() HERE

        # Optional: print services (safe way)
        for service in client.services:
            print(f"[Service] {service.uuid}")
            for char in service.characteristics:
                print(f"  - {char.uuid} | {char.properties}")

        print("\nListening...\n")

        await client.start_notify(UART_CHAR_UUID, notification_handler)

        while True:
            await asyncio.sleep(1)


asyncio.run(main())