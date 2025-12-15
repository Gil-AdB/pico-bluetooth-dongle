# Pico W Bluetooth Dongle

A USB Bluetooth HCI dongle using the Raspberry Pi Pico W (RP2350).

## Features

- USB Bluetooth HCI transport (works with Linux BlueZ)
- A2DP audio streaming support
- SCO/HFP voice support (isochronous endpoints)
- Dynamic LED indicator (blink rate = traffic level)

## Build Instructions

```bash
# Clone with submodules
git clone --recursive https://github.com/Gil-AdB/pico-bluetooth-dongle.git
cd pico-bluetooth-dongle

# Build
mkdir build && cd build
cmake -G Ninja -DPICO_BOARD=pico2_w ..
ninja -j8
```

Output: `build/pico_bluetooth_dongle.uf2`

## Flashing

1. Hold `BOOTSEL` button and connect Pico W via USB
2. Copy `pico_bluetooth_dongle.uf2` to the `RPI-RP2` drive
3. Pico W reboots automatically

## Usage on Linux

```bash
# Check if dongle is detected
hciconfig

# Scan and pair devices
bluetoothctl
> power on
> scan on
> pair XX:XX:XX:XX:XX:XX
> connect XX:XX:XX:XX:XX:XX
```

## Serial Debugging

UART output on GPIO 0/1 (115200 baud). Use a TTL adapter to view logs.

## Architecture

- **Core 0**: CYW43 Bluetooth + statistics
- **Core 1**: TinyUSB device stack
- **Dual queues**: RX (chip→host) and TX (host→chip)
