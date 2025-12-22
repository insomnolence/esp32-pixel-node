# ESP32 LED Mesh - Board Configuration Guide

This document covers all configurable options for the ESP32 LED Mesh firmware.

## Quick Start

```bash
# Navigate to project
cd esp32-firmware

# Source ESP-IDF environment (adjust path to your ESP-IDF installation)
source $IDF_PATH/export.sh
# Or if IDF_PATH not set: source ~/esp/esp-idf/export.sh

# Open configuration menu
idf.py menuconfig

# After making changes
idf.py build
idf.py -p PORT flash    # Replace PORT with your device (e.g., /dev/ttyUSB0, /dev/ttyACM0, COM3)
```

**Navigation:** Arrow keys to move, Enter to select, Escape to go back, ? for help.

---

## Configuration Menu Structure

```
ESP32 LED Mesh Configuration
├── LED Configuration
├── Mesh Network Configuration
├── BLE Configuration
├── System Configuration
├── Memory Management
├── Debug Configuration
├── Button Interface Configuration
├── Indicator LED Configuration
└── Battery Monitoring Configuration
```

---

## LED Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `LED Configuration`

### LED Strip GPIO Pin
| Setting | `CONFIG_LED_PIN` |
|---------|------------------|
| Range | 0-48 |
| Default (ESP32-C3) | 7 |
| Default (ESP32) | 12 |

GPIO pin connected to the LED strip data line. Avoid strapping pins:
- **ESP32-C3:** Avoid GPIO 2, 8, 9
- **ESP32:** Avoid GPIO 0, 2, 5, 12, 15

---

### Number of LEDs
| Setting | `CONFIG_LED_COUNT` |
|---------|-------------------|
| Range | 1-1000 |
| Default (ESP32-C3) | 60 |
| Default (ESP32) | 144 |

Number of LEDs to control. ESP32-C3 uses fewer LEDs due to memory/power constraints.

---

### Physical LED Strip Length
| Setting | `CONFIG_PHYSICAL_LED_STRIP_LENGTH` |
|---------|-----------------------------------|
| Range | 1-1000 |
| Default | 144 |

Total LEDs on physical strip. Used for clearing unused LEDs when `LED_COUNT < strip length`.

---

### LED Update Interval
| Setting | `CONFIG_LED_UPDATE_INTERVAL_MS` |
|---------|--------------------------------|
| Range | 1-1000 ms |
| Default | 20 ms (50 FPS) |

| Value | Frame Rate | Use Case |
|-------|------------|----------|
| 10 ms | 100 FPS | Smooth animations (high CPU) |
| 15 ms | 67 FPS | Balanced (recommended) |
| 20 ms | 50 FPS | Standard (default) |
| 33 ms | 30 FPS | Power saving |

---

### LED Task Stack Size
| Setting | `CONFIG_LED_TASK_STACK_SIZE` |
|---------|------------------------------|
| Range | 2048-8192 bytes |
| Default | 4096 bytes |

Stack size for the LED processing task. Increase if you see stack overflow errors.

---

### Maximum LED Power Budget
| Setting | `CONFIG_LED_MAX_POWER_MA` |
|---------|--------------------------|
| Range | 0-5000 mA |
| Default (ESP32-C3) | 450 mA |
| Default (ESP32) | 0 (disabled) |

Maximum current draw allowed. Brightness auto-scales if power exceeds limit.

| Value | Effect |
|-------|--------|
| 0 | **Disabled** - No power limiting (use for USB/external power) |
| 300 | Conservative - For weak/old batteries |
| 450 | Standard - For ESP32-C3 custom board (TPS61322A limit) |
| 1000+ | High power - For external power supply |

**Power Reference:** WS2812B draws ~60mA per LED at full white (20mA per R/G/B channel).

> **To disable ALL power management features:** Set this to `0`. This disables:
> - Power ceiling limiting
> - Slew rate limiting
> - Voltage-aware scaling
> - Emergency shutdown
>
> Use this when running on USB or external power supply.

---

### Maximum Current Slew Rate
| Setting | `CONFIG_LED_MAX_SLEW_MA_PER_FRAME` |
|---------|-----------------------------------|
| Range | 50-500 mA/frame |
| Default | 150 mA/frame |
| Depends on | `LED_MAX_POWER_MA != 0` |

Limits how fast current can increase per frame. Prevents voltage sag on battery power.

| Value | Ramp Time (0→450mA) | Use Case |
|-------|---------------------|----------|
| 50 | ~135 ms (9 frames) | Very conservative |
| 100 | ~68 ms (4-5 frames) | Conservative |
| 150 | ~45 ms (3 frames) | **Balanced (default)** |
| 300 | ~23 ms (1-2 frames) | Fast transitions |
| 500 | ~15 ms (1 frame) | Minimal limiting |

---

## Mesh Network Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `Mesh Network Configuration`

### ESP-NOW Channel
| Setting | `CONFIG_MESH_CHANNEL` |
|---------|----------------------|
| Range | 1-14 |
| Default | 6 |

WiFi channel for ESP-NOW mesh communication. All nodes must use the same channel.

---

### Default Packet TTL
| Setting | `CONFIG_MESH_DEFAULT_TTL` |
|---------|--------------------------|
| Range | 1-10 |
| Default | 4 |

Maximum hops a packet can travel through the mesh.

---

### Maximum Payload Length
| Setting | `CONFIG_MESH_MAX_PAYLOAD_LEN` |
|---------|------------------------------|
| Range | 1-250 |
| Default | 200 |

Maximum payload size. Limited by ESP-NOW max packet size (250 bytes).

---

### Packet Tracker Cleanup Interval
| Setting | `CONFIG_MESH_CLEANUP_INTERVAL_MS` |
|---------|----------------------------------|
| Range | 5000-300000 ms |
| Default | 30000 ms (30 sec) |

How often to clean up old packet tracking entries.

---

### Election Timeout
| Setting | `CONFIG_MESH_ELECTION_TIMEOUT_MS` |
|---------|----------------------------------|
| Range | 10000-120000 ms |
| Default | 30000 ms (30 sec) |

Timeout for mesh root election process.

---

### Root Announcement Interval
| Setting | `CONFIG_MESH_ROOT_ANNOUNCEMENT_INTERVAL_MS` |
|---------|-------------------------------------------|
| Range | 1000-30000 ms |
| Default | 5000 ms (5 sec) |

How often root node announces itself.

---

### Enable Mesh Encryption
| Setting | `CONFIG_MESH_ENCRYPTION_ENABLED` |
|---------|--------------------------------|
| Default | No |

Enable ESP-NOW encryption. Requires pre-shared key configuration.

---

### Maximum Neighbor Count
| Setting | `CONFIG_MESH_MAX_NEIGHBORS` |
|---------|----------------------------|
| Range | 8-64 |
| Default | 32 |

Maximum neighbors to track. ~14 bytes memory per neighbor. Weakest RSSI neighbor is evicted when full.

---

### Root Node Timeout
| Setting | `CONFIG_MESH_ROOT_TIMEOUT_MS` |
|---------|------------------------------|
| Range | 5000-60000 ms |
| Default | 15000 ms (15 sec) |

Time without hearing from root before considering it gone.

---

### BLE Root Timeout
| Setting | `CONFIG_MESH_BLE_ROOT_TIMEOUT_MS` |
|---------|----------------------------------|
| Range | 10000-120000 ms |
| Default | 30000 ms (30 sec) |

Time without BLE-connected root before clearing network BLE status.

---

### BLE Displacement Cooldown
| Setting | `CONFIG_MESH_DISPLACEMENT_COOLDOWN_MS` |
|---------|---------------------------------------|
| Range | 1000-30000 ms |
| Default | 5000 ms (5 sec) |

Cooldown after being displaced by another BLE root.

---

## BLE Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `BLE Configuration`

### BLE Device Name
| Setting | `CONFIG_BLE_DEVICE_NAME` |
|---------|-------------------------|
| Default | "ESP_LED_NODE" |
| Max Length | 30 characters |

Bluetooth device name advertised to mobile apps.

---

### BLE UART Service UUID
| Setting | `CONFIG_BLE_SERVICE_UUID` |
|---------|--------------------------|
| Default | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |

Nordic UART Service UUID for LED control. Must match Flutter app.

---

### BLE UART TX Characteristic UUID
| Setting | `CONFIG_BLE_CHARACTERISTIC_UUID` |
|---------|--------------------------------|
| Default | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` |

Characteristic for receiving LED commands. Must match Flutter app.

---

### BLE Health Service UUID
| Setting | `CONFIG_BLE_HEALTH_SERVICE_UUID` |
|---------|--------------------------------|
| Default | `12345678-1234-1234-1234-123456789abc` |

Service for mesh network health analytics.

---

### BLE Health Characteristic UUID
| Setting | `CONFIG_BLE_HEALTH_CHARACTERISTIC_UUID` |
|---------|---------------------------------------|
| Default | `87654321-4321-4321-4321-cba987654321` |

Characteristic for health data notifications.

---

### BLE Battery Service UUID
| Setting | `CONFIG_BLE_BATTERY_SERVICE_UUID` |
|---------|---------------------------------|
| Default | `0000180f-0000-1000-8000-00805f9b34fb` |

Standard Bluetooth SIG Battery Service (0x180F).

---

### BLE Battery Level Characteristic UUID
| Setting | `CONFIG_BLE_BATTERY_CHARACTERISTIC_UUID` |
|---------|----------------------------------------|
| Default | `00002a19-0000-1000-8000-00805f9b34fb` |

Standard Bluetooth SIG Battery Level (0x2A19).

---

### Maximum BLE Packet Size
| Setting | `CONFIG_BLE_MAX_PACKET_SIZE` |
|---------|----------------------------|
| Range | 20-512 bytes |
| Default | 244 bytes |

Maximum size for BLE packet data.

---

### Enable BLE Security
| Setting | `CONFIG_BLE_SECURITY_ENABLED` |
|---------|------------------------------|
| Default | No |

Enable BLE pairing and encryption (PIN/passkey authentication).

---

### Use Raw Advertising Data
| Setting | `CONFIG_SET_RAW_ADV_DATA` |
|---------|--------------------------|
| Default | No |

Use raw binary data for advertising packets. Lower runtime overhead.

---

## System Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `System Configuration`

### Main Loop Delay
| Setting | `CONFIG_MAIN_LOOP_DELAY_MS` |
|---------|----------------------------|
| Range | 1-1000 ms |
| Default | 15 ms |

Main application loop delay. Lower = more responsive but higher CPU.

---

### Default Mutex Timeout
| Setting | `CONFIG_MUTEX_TIMEOUT_MS` |
|---------|--------------------------|
| Range | 1-10000 ms |
| Default | 100 ms |

Timeout for mutex operations.

---

### System Task Priority
| Setting | `CONFIG_SYSTEM_TASK_PRIORITY` |
|---------|------------------------------|
| Range | 1-25 |
| Default | 5 |

Priority for system management tasks.

---

### LED Task Priority
| Setting | `CONFIG_LED_TASK_PRIORITY` |
|---------|---------------------------|
| Range | 1-25 |
| Default | 10 |

Priority for LED processing task. Higher number = higher priority.

---

## Memory Management

**Menu Path:** `ESP32 LED Mesh Configuration` → `Memory Management`

### Heap Warning Threshold
| Setting | `CONFIG_HEAP_WARNING_THRESHOLD_BYTES` |
|---------|--------------------------------------|
| Range | 4096-200000 bytes |
| Default | 32768 bytes (32 KB) |

Free heap threshold for low memory warnings.

---

### Heap Critical Threshold
| Setting | `CONFIG_HEAP_CRITICAL_THRESHOLD_BYTES` |
|---------|---------------------------------------|
| Range | 2048-100000 bytes |
| Default | 16384 bytes (16 KB) |

Free heap threshold for critical memory warnings.

---

### Memory Monitor Interval
| Setting | `CONFIG_MEMORY_MONITOR_INTERVAL_MS` |
|---------|-------------------------------------|
| Range | 1000-60000 ms |
| Default | 10000 ms (10 sec) |

Interval for memory usage monitoring.

---

## Debug Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `Debug Configuration`

### Enable Debug Logging
| Setting | `CONFIG_DEBUG_LOGGING_ENABLED` |
|---------|-------------------------------|
| Default | Yes |

Enable detailed debug logging output.

---

### Enable Network Statistics
| Setting | `CONFIG_NETWORK_STATS_ENABLED` |
|---------|-------------------------------|
| Default | Yes |

Enable mesh network statistics collection.

---

### Log Buffer Size
| Setting | `CONFIG_LOG_BUFFER_SIZE` |
|---------|-------------------------|
| Range | 256-4096 bytes |
| Default | 1024 bytes |

Size of internal log buffer.

---

## Button Interface Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `Button Interface Configuration`

### Enable Button Interface
| Setting | `CONFIG_BUTTON_INTERFACE_ENABLED` |
|---------|----------------------------------|
| Default (ESP32-C3) | Yes |
| Default (ESP32) | No |

Enable dual-button interface for standalone LED control.

---

### Button 1 GPIO Pin
| Setting | `CONFIG_BUTTON_1_GPIO` |
|---------|----------------------|
| Range | 0-48 |
| Default (ESP32-C3) | 3 |

Sequence cycling button: Idle → Warning → Exit → Idle

---

### Button 2 GPIO Pin
| Setting | `CONFIG_BUTTON_2_GPIO` |
|---------|----------------------|
| Range | 0-48 |
| Default (ESP32-C3) | 2 |

Random preset selection button.

---

### Button Debounce Time
| Setting | `CONFIG_BUTTON_DEBOUNCE_MS` |
|---------|---------------------------|
| Range | 10-500 ms |
| Default | 50 ms |

Debounce time to prevent false triggers.

---

## Indicator LED Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `Indicator LED Configuration`

### Enable Indicator LED
| Setting | `CONFIG_INDICATOR_LED_ENABLED` |
|---------|-------------------------------|
| Default (ESP32-C3) | Yes |
| Default (ESP32) | No |

Enable status indicator LED. Used for:
- Boot indication (ON during boot, OFF when ready)
- Emergency shutdown indication (BLINKING when battery critically low)

---

### Indicator LED GPIO Pin
| Setting | `CONFIG_INDICATOR_LED_GPIO` |
|---------|---------------------------|
| Range | 0-48 |
| Default (ESP32-C3) | 6 |
| Default (ESP32) | 2 |

GPIO pin for status indicator LED.

---

## Battery Monitoring Configuration

**Menu Path:** `ESP32 LED Mesh Configuration` → `Battery Monitoring Configuration`

### Enable Battery Monitoring
| Setting | `CONFIG_BATTERY_MONITOR_ENABLED` |
|---------|--------------------------------|
| Default | Yes |

Enable battery voltage monitoring. Required for:
- BLE Battery Service reporting
- Voltage-aware power limiting

---

### Battery ADC GPIO Pin
| Setting | `CONFIG_BATTERY_ADC_GPIO` |
|---------|--------------------------|
| Range | 0-39 |
| Default (ESP32-C3) | 0 (A0) |
| Default (ESP32) | 35 (A13) |

GPIO pin for battery voltage ADC reading.

---

### Voltage Divider Resistors

| Setting | Description | ESP32-C3 | ESP32 (Adafruit) |
|---------|-------------|----------|------------------|
| `CONFIG_BATTERY_DIVIDER_R_HIGH` | High-side resistor (kOhm) | 180 | 200 |
| `CONFIG_BATTERY_DIVIDER_R_LOW` | Low-side resistor (kOhm) | 51 | 200 |

Must match your hardware voltage divider circuit.

---

### Battery Voltage Range

| Setting | Description | Default |
|---------|-------------|---------|
| `CONFIG_BATTERY_FULL_MV` | 100% voltage (mV) | 4200 |
| `CONFIG_BATTERY_EMPTY_MV` | 0% voltage (mV) | 3000 |

Typical Li-ion/LiPo range.

---

### Battery Update Interval
| Setting | `CONFIG_BATTERY_UPDATE_INTERVAL_MS` |
|---------|-------------------------------------|
| Range | 1000-60000 ms |
| Default | 10000 ms (10 sec) |

How often to update battery voltage reading for BLE reporting.

---

## Voltage Thresholds (Code Constants)

These are defined in `components/led/include/led/led_strip.h` and require recompilation:

| Constant | Value | Effect |
|----------|-------|--------|
| `VOLTAGE_EMERGENCY_ENTER` | 3150 mV | LEDs turn OFF, indicator blinks |
| `VOLTAGE_EMERGENCY_EXIT` | 3300 mV | LEDs can turn back on (150mV hysteresis) |
| `VOLTAGE_VERY_LOW` | 3200 mV | 20% power ceiling |
| `VOLTAGE_LOW` | 3300 mV | 40% power ceiling |
| `VOLTAGE_MEDIUM_LOW` | 3400 mV | 60% power ceiling |
| `VOLTAGE_MEDIUM` | 3500 mV | 80% power ceiling |
| ≥ 3500 mV | - | 100% power ceiling |

---

## Quick Configuration Profiles

### ESP32-C3 Custom Board (Battery Powered)
```
LED Configuration:
  LED Strip GPIO Pin = 7
  Number of LEDs = 60
  Maximum LED Power Budget = 450
  Maximum Current Slew Rate = 150

Button Interface:
  Enable Button Interface = Yes
  Button 1 GPIO = 3
  Button 2 GPIO = 2

Indicator LED:
  Enable Indicator LED = Yes
  Indicator LED GPIO = 6

Battery Monitoring:
  Enable Battery Monitoring = Yes
  Battery ADC GPIO = 0
  Voltage Divider R High = 180
  Voltage Divider R Low = 51
```

### ESP32 Adafruit Feather (USB/External Power)
```
LED Configuration:
  LED Strip GPIO Pin = 12
  Number of LEDs = 144
  Maximum LED Power Budget = 0  (disabled)

Button Interface:
  Enable Button Interface = No

Indicator LED:
  Enable Indicator LED = No

Battery Monitoring:
  Enable Battery Monitoring = Yes (for reporting only)
  Battery ADC GPIO = 35
  Voltage Divider R High = 200
  Voltage Divider R Low = 200
```

### Testing/Development (No Power Limits)
```
LED Configuration:
  Maximum LED Power Budget = 0

Debug Configuration:
  Enable Debug Logging = Yes
  Enable Network Statistics = Yes
```

---

## Saving Configuration

After making changes in menuconfig:

```bash
# Build with new config
idf.py build

# Flash to device (replace PORT with your device)
idf.py -p PORT flash

# Optional: Save minimal config for version control
idf.py save-defconfig
```

The `save-defconfig` command creates/updates `sdkconfig.defaults` with only non-default values.

---

## Target-Specific Defaults

Create target-specific defaults in project root:

- `sdkconfig.defaults` - Common defaults (all targets)
- `sdkconfig.defaults.esp32` - ESP32-specific
- `sdkconfig.defaults.esp32c3` - ESP32-C3-specific
- `sdkconfig.defaults.esp32s3` - ESP32-S3-specific

When building, ESP-IDF merges these automatically based on target.
