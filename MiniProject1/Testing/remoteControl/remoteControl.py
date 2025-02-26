import asyncio
from bleak import BleakClient, BleakScanner

# UUIDs (Replace these with the actual ones from your Arduino code)
ENC_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
POSE_CHAR_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
WASD_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

async def find_device():
    devices = await BleakScanner.discover()
    for device in devices:
        if(device.name != None):
            print(device.name )
        if device.name and "Robot-BLE-DeadReckon" in device.name:
            print(f"Found device: {device.name} ({device.address})")
            return device.address
    print("Arduino Nano 33 BLE not found.")
    return None

async def connect_to_robot(address):
    async with BleakClient(address) as client:
        print("Connected to robot.")
        
        # Read initial pose
        pose = await client.read_gatt_char(POSE_CHAR_UUID)
        print(f"Initial Pose: {pose.decode()}")

        # Example: Send WASD commands
        for command in ['W', 'S', 'A', 'D', 'T']:
            temp = await client.read_gatt_char(WASD_CHAR_UUID)
            print(f"Current command: {temp}")
            await client.write_gatt_char(WASD_CHAR_UUID, bytearray(command, "utf-8"), response=True)
            print(f"Sent command: {command}")
            await asyncio.sleep(1)
        
        # Read updated pose
        pose = await client.read_gatt_char(POSE_CHAR_UUID)
        print(f"Updated Pose: {pose.decode()}")

async def main():
    address = await find_device()
    if address:
        await connect_to_robot(address)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
