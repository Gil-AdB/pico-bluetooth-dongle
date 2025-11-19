# Pico W Bluetooth Dongle

This project aims to create a USB Bluetooth HCI dongle using the Raspberry Pi Pico W.
It utilizes the Pico SDK, TinyUSB, and BTstack to provide Bluetooth functionality
and a CDC (serial) interface for debugging.

## Build Instructions

1.  **Initialize Submodules:**
    ```bash
    git submodule update --init --recursive
    ```

2.  **Build the Project:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make -j8
    ```
    This will generate `pico_bluetooth_dongle.uf2` in the `build` directory.

## Flashing the Firmware

1.  Disconnect your Pico W from the computer.
2.  Press and hold the `BOOTSEL` button, then connect the Pico W to your computer via USB. It should appear as a mass storage device named `RPI-RP2`.
3.  Drag and drop the `pico_bluetooth_dongle.uf2` file onto the `RPI-RP2` drive. The Pico W will reboot automatically.

## Current Status

The project now compiles successfully and is configured as a composite USB device, exposing both a Bluetooth HCI interface and a CDC (serial) interface for debugging.

**Known Issue:**
On Windows, the device currently enumerates as a Bluetooth dongle but fails to start with "Code 10: An invalid parameter was passed to a service or function". The intended CDC (serial) port for `printf` output does not appear.

## Debugging

*   **Serial Output (intended):** If the CDC serial port appears (e.g., in Windows Device Manager under "Ports (COM & LPT)"), you can connect to it with a serial terminal (115200 baud) to view `printf` messages.
*   **External Debugging:** If the CDC serial port remains unavailable, external debugging methods are recommended:
    *   **Logic Analyzer:** To capture and analyze USB traffic at a low level.
    *   **TTL Adapter:** To get serial output directly from the Pico W's UART pins (refer to Pico W documentation for UART pinout and configuration).
