# Project: Telemtrix4RpiPico2w-BLE

## Description
Add a third server for [Telemtrix4RpiPico2w](https://github.com/MrYsLab/Telemetrix4RpiPico2w)
that uses BLE Nordic UART Service as a transport. The name of
this new server is Telemetrix4RpiPico2w-BLE.

## Code Style
*   Code in C++ for the Arduino IDE.
*   Reuse the code from Telemetrix4RpiPico2w-Serial

## Dependencies
*   [BLE Nordic UART Service.c](https://github.com/MrYsLab/Telemetrix4RpiPico2w/blob/ble/artifacts/BLE%20Nordic%20UART%20Service.c)
*   [arduino-pico](https://github.com/earlephilhower/arduino-pico)

## Functional Requirements
*   Create a read and write function for the BLE NUS code and replace the serial read and writes with calls to these functions.
*   Replace the initialization in Setup() of the serial port with initialization required for NUS.
*   Reuse the code from Telemetrix4RpiPico2-Serial with as little changes as possible.
*   The new server will be placed in the examples directory.


