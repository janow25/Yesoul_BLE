# Yesoul_BLE AI Agent Instructions

This file contains instructions for AI coding agents to work effectively in this repository.

## Project Overview
- **Type**: C++ / Arduino framework via PlatformIO.
- **Hardware**: ESP32 variants (`esp32dev`, `lolin32`, `esp32-c6-supermini`).
- **Purpose**: A BLE bridge/translator that receives proprietary data from a Yesoul indoor bike and broadcasts standard Cycling Power and Cadence service data (making it compatible with cycle computers and Apple Watch).
- **Primary Library**: `h2zero/NimBLE-Arduino` for low-energy Bluetooth (client and server operations).

## Domain Context / Device Lifecycle
- **Power Method**: The device is always powered via a USB-C Wallbrick adapter so no Battery
- **Power Lifecycle**: The ESP32 device is manually powered on *only* when the user uses the bike.
- **Shutdown**: When the user stops using the bike, they manually unplug the ESP32, completely severing power.
- **Agent Implication (Important)**: Do not write code that assumes continuous, long-term uptime. There is no need for deep sleep modes, persistent background loops across sessions, long-term data caching to flash, or handling extended idle states since the power is just manually cut when not in use.

## Build and Upload Commands
- **Build**: `platformio run` or `pio run`.
- **Target specific board**: `platformio run -e esp32-c6-supermini` (or `lolin32`, `esp32dev`).
- **Upload**: `platformio run --target upload` (or append `-e` flag for a specific environment).
- **Serial Monitor**: `platformio device monitor -b 115200`.

## Architecture & Code Structure
- **`src/main.cpp`**: Contains the core logic. It runs both a **NimBLE Client** (to connect to the Yesoul bike UUIDs) and a **NimBLE Server** (to act as a standard BLE cycling sensor to downstream devices).
- **LED states** are used as connection indicators (e.g., connecting to Yesoul, waiting for Apple Watch, etc.). See the `LEDState` enum.
