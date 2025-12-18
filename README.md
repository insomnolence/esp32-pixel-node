> Note: This is a personal project that is ongoing.
>
> *And yes, I did use AI as the tool that it is for some parts. Cast no stones*

---

# ESP32 LED Mesh Network - Configuration Guide

This project implements a sophisticated LED mesh network with autonomous root selection, BLE connectivity, and real-time LED pattern synchronization. This guide explains how to configure the system for your specific hardware and requirements.

## Quick Start

### 1. Choose Your ESP32 Platform
The system automatically detects and configures for different ESP32 variants:

| Platform | CPU Cores | Recommended LED Pin | Default LED Count | Memory |
|----------|-----------|-------------------|------------------|---------|
| ESP32 | 2 (dual-core) | Pin 12 | 144 LEDs | 520KB SRAM |
| ESP32-C3 | 1 (single-core) | Pin 7 | 60 LEDs | 400KB SRAM |  
| ESP32-S3 | 2 (dual-core) | Pin 7 | 144 LEDs | 512KB SRAM |

### 2. Set Your Target Platform
```bash
# Set your ESP32 variant
idf.py set-target esp32c3    # For ESP32-C3
idf.py set-target esp32      # For ESP32
idf.py set-target esp32s3    # For ESP32-S3
```

### 3. Configure Your Hardware
```bash
# Open configuration menu
idf.py menuconfig
```
Navigate to: "ESP32 LED Mesh Configuration"

## Project Structure

The project is organized into ESP-IDF components so each subsystem stays isolated and testable:

| Path | Purpose |
|------|---------|
| `main/` | Application entry point (`main.cpp`) that wires components together and reacts to `CONFIG_*` flags. |
| `components/mesh/` | Adaptive ESP-NOW mesh coordinator and routing stack. |
| `components/bluetooth/` | BLE GATT server, GAP handler, and the Pixel Packet / Network Health profiles. |
| `components/led/` | LED renderer, patterns, and strip driver abstraction. |
| `components/packet/` | Packet serialization/parsing helpers plus Unity tests. |
| `components/common/` | Shared lightweight headers (e.g., button feedback enums). |
| `components/system_control/` | Local device services (NVS, global object manager, stack monitor) and the optional button/feedback/root-takeover pipeline. Sources in this component are compiled only when `CONFIG_BUTTON_INTERFACE_ENABLED=y`. |
| `components/system_support/` | Shared data models such as `NetworkHealth` used by BLE and mesh code. |

When in doubt, look for the subsystem inside `components/<name>` instead of under `main/`; the entrypoint now delegates almost everything to these components.

## Hardware Configuration

### LED Strip Configuration

#### Configure LED Pin
1. Go to `menuconfig` → "ESP32 LED Mesh Configuration" → "LED Configuration"
2. Set "LED Strip GPIO Pin" to your wiring:

Safe GPIO Pins by Platform:
- ESP32: 0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
- ESP32-C3: 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 18, 19, 20, 21  
- ESP32-S3: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21

WARNING: Avoid strapping pins: ESP32C3 pins 2,8,9 / ESP32 pins 0,2,5,12,15

#### Configure LED Count
Set "Number of LEDs" based on your strip and power supply:

The WS2812B chip run at about 60mA per chip at full brightness

Power Considerations:
- USB Power (5V, 500mA): Max ~7 LEDs at full brightness
- USB Power (5V, 1A): Max ~14 LEDs at full brightness  
- External 5V Power: Current PS does not support running the board
- Official board (5V, 2A): ~28 LEDs at full power

NOTE: Brightness Levels for Power Safety

All patterns use reduced brightness levels to prevent power-related resets and ensure stable operation:
- **Idle patterns**: Brightness 35 (low power consumption)
- **Normal patterns**: Brightness 67 (balanced visibility and power)
- **Flash/Alert patterns**: Brightness 80 (attention-grabbing, short duration)

These brightness values are set directly in the pattern definitions (`sequence.cpp`) rather than using runtime clamping, which provides more predictable power consumption and allows patterns like Flash and Strobe to use soft ramp-up techniques to prevent inrush current spikes.

Memory Considerations:
- ESP32-C3: Recommended max 60 LEDs (shown in code)
- ESP32/ESP32-S3: Can handle 144+ LEDs

### Custom Hardware Examples

#### Example 1: ESP32-C3 with 30 LEDs on Pin 3
```bash
idf.py menuconfig
# LED Configuration → LED Strip GPIO Pin → 3
# LED Configuration → Number of LEDs → 30
# LED Configuration → Physical LED Strip Length → 30
```

#### Example 2: ESP32 with 200 LEDs on Pin 16  
```bash
idf.py menuconfig
# LED Configuration → LED Strip GPIO Pin → 16
# LED Configuration → Number of LEDs → 200
# LED Configuration → Physical LED Strip Length → 200
```

## Network Configuration

### Mesh Network Settings
Configure ESP-NOW mesh parameters:

```bash
# menuconfig → "ESP32 LED Mesh Configuration" → "Mesh Network Configuration"
```

| Setting | Recommended | Description |
|---------|-------------|-------------|
| ESP-NOW Channel | 6 | WiFi channel (1-14, avoid your WiFi) |
| Default Packet TTL | 4 | Max hops (1-10) |
| Maximum Payload Length | 200 | ESP-NOW limit is 250 bytes |
| Election Timeout | 30000ms | Time for root election |

### Bluetooth Configuration
Customize BLE advertising and connection:

```bash
# menuconfig → "ESP32 LED Mesh Configuration" → "BLE Configuration" 
```

| Setting | Default | Your Custom Value |
|---------|---------|-------------------|
| BLE Device Name | "ESP_LED_NODE" | "MyLEDController" |
| BLE Service UUID | Standard UUID | Your custom UUID |
| Maximum BLE Packet Size | 244 | 20-512 bytes |

## Runtime Configuration

Most runtime behavior is controlled through `idf.py menuconfig` (see the "ESP32 LED Mesh Configuration" menu). Code-level hooks now live in `components/system_control`, which centralizes the local device services (NVS, stack monitoring, button interface, and global object management).

### Accessing Global Services in Code

`components/system_control/global_objects.*` exposes the mesh coordinator, BLE server, and LED controller as heap-allocated singletons. You can safely access them anywhere after `GlobalObjects::initialize()` succeeds:

```cpp
#include "system_control/global_objects.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "led/led_controller.h"

void inspect_runtime_state() {
    auto& mesh = GlobalObjects::getMeshCoordinator();
    ESP_LOGI("Setup", "Adaptive mesh enabled: %s",
             mesh.isAdaptiveMeshEnabled() ? "yes" : "no");

    auto& strip = GlobalObjects::getLEDStrip();
    ESP_LOGI("Setup", "LED count: %u", strip.numPixels());
}
```

### Button Interface Toggle

The standalone button/feedback/root-takeover stack (now under `components/system_control`) is optional. Enable it via:

```
menuconfig → ESP32 LED Mesh Configuration → Button Interface → Enable Button Interface
```

When `CONFIG_BUTTON_INTERFACE_ENABLED=y`, the build pulls in `button_manager`, `button_logic`, `button_feedback`, `dual_button_detector`, and `root_takeover_manager`. Disabling the option keeps the binary lean for installations that are controlled purely over BLE.

### Customizing Persistence & Diagnostics

- `components/system_control/nvs_manager.*` manages device settings stored in NVS. Extend it if you need additional persisted values.
- `components/system_control/stack_monitor.*` provides the `LOG_CURRENT_STACK()` macro used throughout `main.cpp`—handy when tuning FreeRTOS tasks for different targets.
- Shared network-health structs used by BLE are defined once in `components/system_support`, so BLE and mesh code stay in sync.

## Building and Flashing

### Prerequisites
- ESP-IDF v6.0+ (tested with v6.0-dev-1489-g4e036983a7)
- Python 3.8+ (Python 3.13+ recommended for ESP-IDF v6.0)
- CMake 3.16+ (required for build system)

WARNING: Not backwards compatible with ESP-IDF v5.x due to ESP-NOW API changes.

### Complete Build Process
```bash
# 1. Set up ESP-IDF environment
source ~/esp/esp-idf/export.sh

# 2. Set your target platform
idf.py set-target esp32c3  # or esp32, esp32s3

# 3. Configure for your hardware
idf.py menuconfig

# 4. Build the project  
idf.py build

# 5. Flash to your device
idf.py flash

# 6. Monitor output
idf.py monitor
```

### Configuration Verification
After flashing, confirm that the core components report healthy startup:
```
I (1234) GlobalObjects: 🏭 Initializing global objects on heap...
I (1240) ESPNowMeshCoordinator: ✅ Adaptive mesh system enabled - ready for topology-aware networking
I (1250) BLEGattServer: 🔍 BLE advertising started successfully
I (1260) ESP_LED_MESH: 🔄 Enabling adaptive mesh system...
```
If any of these lines are missing (or show errors), check the component mentioned in the log and revisit `menuconfig`/wiring.

## Hardware Wiring Guide

### Basic LED Strip Connection
```
ESP32/ESP32C3        WS2812B LED Strip
    GND      ────────      GND
    3.3V     ────────      VCC (if ≤10 LEDs)
    5V       ────────      VCC (if >10 LEDs, external power recommended)
   GPIO_X    ────────      DIN (Data Input)
```

### Multiple ESP32 Mesh Network
Each ESP32 needs:
- LED strip connected to configured GPIO pin
- Power supply appropriate for LED count
- Same mesh channel configuration

## Mobile App Integration

### BLE Connection
Your mobile app should connect to:
- Service UUID: Configured in BLE settings (default: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`)
- Characteristic UUID: For data packets (default: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`)
- Device Name: Appears in BLE scan (configurable)

### Mesh Priority
- BLE Connected Node: Automatically becomes mesh root
- Multiple BLE Connections: Newest connection wins
- No BLE: Nodes elect autonomous root via algorithm

## Troubleshooting

### LED Issues
| Problem | Solution |
|---------|----------|
| LEDs don't light up | Check GPIO pin, power supply, wiring |
| Wrong colors | Verify WS2812B vs WS2812 timing |
| Flickering | Add capacitor, check power supply |

### Mesh Issues  
| Problem | Solution |
|---------|----------|
| Nodes don't connect | Check mesh channel, power cycle all nodes |
| Split brain (multiple roots) | Update firmware, check election timeout |
| Poor range | Check antenna, reduce interference |

### Configuration Issues
| Problem | Solution |
|---------|----------|
| Config not saving | Check NVS partition, flash memory |
| Wrong platform detected | Verify `idf.py set-target` command |
| Build errors | Run `idf.py menuconfig` to fix missing config |

### Memory Issues
| Problem | Solution |
|---------|----------|
| Crashes with many LEDs | Reduce LED count, check power supply |
| Out of memory | Use ESP32 instead of ESP32C3 for large installations |

## Performance Guidelines

### LED Count Recommendations
| Platform | Conservative | Aggressive | Maximum Tested |
|----------|--------------|------------|-----------------|
| ESP32-C3 | 30 LEDs | 60 LEDs | 100 LEDs |
| ESP32 | 60 LEDs | 144 LEDs | 300+ LEDs |
| ESP32-S3 | 100 LEDs | 200 LEDs | 500+ LEDs |

### Network Size Recommendations
- Small Network: 2-5 nodes (excellent performance)
- Medium Network: 6-15 nodes (good performance)  
- Large Network: 16-30 nodes (requires optimization)

## Security Configuration

### Enable Mesh Encryption (Future)
```bash
# menuconfig → "ESP32 LED Mesh Configuration" → "Mesh Network Configuration"
# → Enable Mesh Encryption → Yes
```

### Enable BLE Security (Future)
```bash  
# menuconfig → "ESP32 LED Mesh Configuration" → "BLE Configuration"
# → Enable BLE Security → Yes
```

## Ready to Go

Your ESP32 LED Mesh system is now configured for your specific hardware. The system will automatically:
- Detect your ESP32 platform and optimize accordingly
- Use your configured LED pin and count
- Connect to mesh network on your specified channel
- Advertise BLE with your custom device name
- Save all runtime changes to flash memory

Need help? Browse the component sources (`components/system_control`, `components/mesh`, `components/bluetooth`, etc.) for real-world integration examples and extension points.
