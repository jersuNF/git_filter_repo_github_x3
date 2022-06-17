import asyncio
from bleak import BleakScanner, BleakClient

import time

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

async def main():
    nofence_client = None

    devices = await BleakScanner.discover(timeout=5)
    for d in devices:
        if "uuids" in d.metadata.keys():
            if UART_SERVICE_UUID.lower() in d.metadata["uuids"]:
                print(d.address)
                nofence_client = BleakClient(d.address.lower())
                break
    
    if nofence_client == None:
        return
    
    def handle_rx(_: int, data: bytearray):
        print("received:", data)

    try:
        await nofence_client.connect()
        
        await nofence_client.start_notify(UART_TX_CHAR_UUID, handle_rx)
        
        await nofence_client.write_gatt_char(UART_RX_CHAR_UUID, b"U")

        #while True:
        #    time.sleep(0.1)

    except Exception as e:
        print(e)
    finally:
        await nofence_client.disconnect()

asyncio.run(main())