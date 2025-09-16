# ESKF for ESP32 with BNO085, VL53Lx, PMW3901

This project implements an Extended Kalman Filter (ESKF) for the ESP32 platform, integrating the following sensors:
- BNO085 IMU
- VL53Lx time-of-flight distance sensor
- PMW3901 optical flow sensor

## Features
- Sensor fusion using ESKF
- PlatformIO-based build system
- Written in C++

## Getting Started
1. Open this project in VS Code with the PlatformIO extension installed.
2. Connect your ESP32 board.
3. Build and upload the firmware using PlatformIO.

## Directory Structure
- `src/` - Main source code
- `include/` - Header files
- `lib/` - External libraries or drivers
- `platformio.ini` - PlatformIO project configuration

## Requirements
- PlatformIO extension for VS Code
- ESP32 development board (i used the Devkit V1)
- BNO085, VL53Lx, PMW3901 sensors

## License
MIT
