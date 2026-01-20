

# Project: Telemtrix4RpiPico2w-BLE

## Description

Add a third server for [Telemtrix4RpiPico2w](https://github.com/MrYsLab/Telemetrix4RpiPico2w)
that uses BLE Nordic UART Service as a transport. 

## Code Style
*   Code in C++ for the Arduino IDE.

## Dependencies
*   [BLE Nordic UART Service.c](https://github.com/MrYsLab/Telemetrix4RpiPico2w/blob/ble/artifacts/BLE%20Nordic%20UART%20Service.c)
*   [arduino-pico](https://github.com/earlephilhower/arduino-pico)

## Functional Requirements
* The name of this new server is Telemetrix4RpiPico2w-BLE. I need to add a 
class to the Telemetrix4RpiPico2w-BLE directory of the BLE branch of 
[Telemetrix4RpiPico2w](https://github.com/MrYsLab/Telemetrix4RpiPico2w/tree/ble). 
* The class should consist of two files BLE_Transport.h and BLE_Transport.cpp.
* It should be based of a BLE NUS example located [here](https://github.com/MrYsLab/Telemetrix4RpiPico2w/blob/ble/artifacts/BLE%20Nordic%20UART%20Service.c)
* The Telemetrix BLE server is located [here](https://github.com/MrYsLab/Telemetrix4RpiPico2w/blob/ble/examples/Telemetrix4RpiPico2w-BLE/Telemetrix4RpiPico2w-BLE.ino). It was converted from the [Serial server](https://github.com/MrYsLab/Telemetrix4RpiPico2w/blob/ble/examples/Telemetrix4RpiPico2w-Serial/Telemetrix4RpiPico2w-Serial.ino). It has not yet been tested since I don't have a working BLE NUS transport.
* When the new server advertises it should be detected using the following scanner:
```python
import asyncio
from bleak import BleakScanner

async def scan_for_devices():
    print("Scanning for BLE devices...")
    # Scan for 5 seconds
    devices = await BleakScanner.discover(timeout=15.0)
    for device in devices:
        print(f"Found device: {device.name} | Address: {device.address}")

# Run the asynchronous function
if __name__ == "__main__":
    asyncio.run(scan_for_devices())
```
* The sketch should allow the user to modify the sketch so that they can set the advertising string.
* The default advertising string should be "Tmx4Pico2W".
* The arduino-pico library is required to provide the BTStack library.
* Verify that Telemetrix4RpiPico2w-BLE.ino is coded correctly.


