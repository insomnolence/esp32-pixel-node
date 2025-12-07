# Docker Build Environment

This project includes a Docker container with all dependencies pre-configured for building the ESP32 LED Mesh firmware.

## Requirements

- Docker installed on your system
- ~5GB disk space for the image (ESP-IDF toolchain is large)

## Quick Start

### 1. Build the Docker Image

```bash
docker build -t esp32-led-mesh-builder .
```

This will take 10-20 minutes on first build as it downloads and installs ESP-IDF v6.0.

### 2. Build Firmware

**Build for ESP32 (default):**
```bash
docker run --rm -v $(pwd):/project esp32-led-mesh-builder esp32
```

**Build for ESP32-C3:**
```bash
docker run --rm -v $(pwd):/project esp32-led-mesh-builder esp32c3
```

**Build for all supported targets:**
```bash
docker run --rm -v $(pwd):/project esp32-led-mesh-builder all
```

### 3. Find Your Binaries

After building, binaries are in the `release/` directory:

```
release/
├── esp32/
│   ├── bootloader.bin
│   ├── partition-table.bin
│   ├── esp32_led_mesh.bin
│   └── esp32_led_mesh_merged.bin  # Single file for easy flashing
├── esp32c3/
│   └── ...
└── ...
```

## Flashing the Firmware

Flashing is done outside of Docker using tools on your host machine.

### Installing esptool.py

If you don't have `esptool.py` installed:

```bash
pip install esptool
```

### Option 1: Using the Merged Binary (Easiest)

The merged binary contains bootloader, partition table, and firmware in one file:

**ESP32:**
```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash 0x0 release/esp32/esp32_led_mesh_merged.bin
```

**ESP32-C3:**
```bash
esptool.py --chip esp32c3 --port /dev/ttyUSB0 write_flash 0x0 release/esp32c3/esp32_led_mesh_merged.bin
```

**ESP32-S3:**
```bash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash 0x0 release/esp32s3/esp32_led_mesh_merged.bin
```

> **Note:** On Windows, replace `/dev/ttyUSB0` with your COM port (e.g., `COM3`).
> On Mac, it's typically `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`.

### Option 2: Using Individual Binaries

If the merged binary doesn't work, flash each part separately:

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash \
    0x1000 release/esp32/bootloader.bin \
    0x8000 release/esp32/partition-table.bin \
    0x10000 release/esp32/esp32_led_mesh.bin
```

### Option 3: Using ESP Web Flasher (No Install Required)

For users who don't want to install any tools:

1. Open [ESP Web Tools](https://web.esphome.io/) in Chrome or Edge
2. Click "Connect" and select your ESP32's serial port
3. Click "Install" and select the merged `.bin` file
4. Wait for flashing to complete

This works directly in the browser with no software installation.

## Interactive Shell

For debugging or custom builds:

```bash
docker run --rm -it -v $(pwd):/project esp32-led-mesh-builder shell
```

Inside the container, you have full access to `idf.py`:

```bash
idf.py set-target esp32
idf.py menuconfig
idf.py build
```

## Custom Configuration

You can customize the firmware settings (LED pin, LED count, mesh parameters, etc.) using `menuconfig`:

```bash
docker run --rm -it -v $(pwd):/project esp32-led-mesh-builder shell
```

Then inside the container:

```bash
# Set your target first
idf.py set-target esp32c3

# Open the configuration menu
idf.py menuconfig
```

Navigate to **ESP32 LED Mesh Configuration** to adjust settings. When you save and exit, the `sdkconfig` file is saved to your project directory.

After configuring, build as normal:

```bash
idf.py build
```

Or exit the shell and use the standard build command:

```bash
docker run --rm -v $(pwd):/project esp32-led-mesh-builder esp32c3
```

> **Note:** If the menuconfig display looks garbled, try adding the terminal environment variable:
> ```bash
> docker run --rm -it -e TERM=xterm-256color -v $(pwd):/project esp32-led-mesh-builder shell
> ```

## Supported Targets

| Target    | Description                    | LED Pin | LED Count |
|-----------|--------------------------------|---------|-----------|
| esp32     | ESP32 (dual-core Xtensa)       | 12      | 144       |
| esp32c3   | ESP32-C3 (single-core RISC-V)  | 7       | 60        |
| esp32c2   | ESP32-C2 (single-core RISC-V)  | 7       | 60        |
| esp32s3   | ESP32-S3 (dual-core Xtensa)    | 7       | 144       |

## ESP-IDF Version

This container uses **ESP-IDF v6.0** (development branch). The firmware requires ESP-IDF v6.0+ due to ESP-NOW API changes that are incompatible with v5.x.

## Troubleshooting

### Permission Issues on Linux

If you get permission errors with the output files:

```bash
docker run --rm -v $(pwd):/project --user $(id -u):$(id -g) esp32-led-mesh-builder esp32
```

### Build Fails with Submodule Errors

Ensure your local repo has all submodules:

```bash
git submodule update --init --recursive
```

### Out of Disk Space

The ESP-IDF image is large. Free up space or use Docker's disk management:

```bash
docker system prune -a
```
