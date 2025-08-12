> **Note:** This is a personal project that is ongoing.
>
> *And yes, I did use AI as the tool that it is for some parts. Cast no stones*

---

# ESP32 LED Mesh Network - Configuration Guide

This project implements a sophisticated LED mesh network with autonomous root selection, BLE connectivity, and real-time LED pattern synchronization. This guide explains how to configure the system for your specific hardware and requirements.

## 🎯 **Quick Start**

### 1. **Choose Your ESP32 Platform**
The system automatically detects and configures for different ESP32 variants:

| Platform | CPU Cores | Recommended LED Pin | Default LED Count | Memory |
|----------|-----------|-------------------|------------------|---------|
| **ESP32** | 2 (dual-core) | Pin 12 | 144 LEDs | 520KB SRAM |
| **ESP32-C3** | 1 (single-core) | Pin 7 | 60 LEDs | 400KB SRAM |  
| **ESP32-S3** | 2 (dual-core) | Pin 7 | 144 LEDs | 512KB SRAM |

### 2. **Set Your Target Platform**
```bash
# Set your ESP32 variant
idf.py set-target esp32c3    # For ESP32-C3
idf.py set-target esp32      # For ESP32
idf.py set-target esp32s3    # For ESP32-S3
```

### 3. **Configure Your Hardware**
```bash
# Open configuration menu
idf.py menuconfig
```
Navigate to: **"ESP32 LED Mesh Configuration"**

## 🔧 **Hardware Configuration**

### **LED Strip Configuration**

#### **Configure LED Pin**
1. Go to `menuconfig` → "ESP32 LED Mesh Configuration" → "LED Configuration"
2. Set "LED Strip GPIO Pin" to your wiring:

**Safe GPIO Pins by Platform:**
- **ESP32**: 0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
- **ESP32-C3**: 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 18, 19, 20, 21  
- **ESP32-S3**: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21

⚠️ **Avoid strapping pins:** ESP32C3 pins 2,8,9 / ESP32 pins 0,2,5,12,15

#### **Configure LED Count**
Set "Number of LEDs" based on your strip and power supply:

**Power Considerations:**
- **USB Power (5V, 500mA)**: Max ~25 LEDs at full brightness
- **USB Power (5V, 1A)**: Max ~50 LEDs at full brightness  
- **External 5V Power**: Scale according to supply (60mA per LED at full white)

**Memory Considerations:**
- **ESP32-C3**: Recommended max 60 LEDs (shown in code)
- **ESP32/ESP32-S3**: Can handle 144+ LEDs

### **Custom Hardware Examples**

#### **Example 1: ESP32-C3 with 30 LEDs on Pin 3**
```bash
idf.py menuconfig
# LED Configuration → LED Strip GPIO Pin → 3
# LED Configuration → Number of LEDs → 30
# LED Configuration → Physical LED Strip Length → 30
```

#### **Example 2: ESP32 with 200 LEDs on Pin 16**  
```bash
idf.py menuconfig
# LED Configuration → LED Strip GPIO Pin → 16
# LED Configuration → Number of LEDs → 200
# LED Configuration → Physical LED Strip Length → 200
```

## 📶 **Network Configuration**

### **Mesh Network Settings**
Configure ESP-NOW mesh parameters:

```bash
# menuconfig → "ESP32 LED Mesh Configuration" → "Mesh Network Configuration"
```

| Setting | Recommended | Description |
|---------|-------------|-------------|
| **ESP-NOW Channel** | 6 | WiFi channel (1-14, avoid your WiFi) |
| **Default Packet TTL** | 4 | Max hops (1-10) |
| **Maximum Payload Length** | 200 | ESP-NOW limit is 250 bytes |
| **Election Timeout** | 30000ms | Time for root election |

### **Bluetooth Configuration**
Customize BLE advertising and connection:

```bash
# menuconfig → "ESP32 LED Mesh Configuration" → "BLE Configuration" 
```

| Setting | Default | Your Custom Value |
|---------|---------|-------------------|
| **BLE Device Name** | "ESP_LED_NODE" | "MyLEDController" |
| **BLE Service UUID** | Standard UUID | Your custom UUID |
| **Maximum BLE Packet Size** | 244 | 20-512 bytes |

## ⚙️ **Runtime Configuration**

### **Programmatic Configuration**
Change settings in your code:

```cpp
#include "system/system_config.h"

void setup_my_configuration() {
    SystemConfig& config = SystemConfig::getInstance();
    config.init();
    
    // Configure for your hardware
    config.setLedPin(5);              // Your LED pin
    config.setLedCount(48);           // Your LED count
    config.setBLEDeviceName("MyNode");// Your device name
    config.setMeshChannel(11);        // Your mesh channel
    
    // Save to flash memory (persists across reboots)
    config.saveToNVS();
    
    // Verify configuration
    config.printConfiguration();
}
```

### **Check Your Platform**
```cpp
void check_platform() {
    SystemConfig& config = SystemConfig::getInstance();
    
    ESP_LOGI("Setup", "Platform: %s", config.getPlatformName());
    ESP_LOGI("Setup", "Dual-core: %s", config.isDualCoreEnabled() ? "Yes" : "No");
    ESP_LOGI("Setup", "LED Pin: %d", config.getLedPin());
    ESP_LOGI("Setup", "LED Count: %d", config.getLedCount());
}
```

## 🏗️ **Building and Flashing**

### **Complete Build Process**
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

### **Configuration Verification**
After flashing, look for this output:
```
I (1234) SystemConfig: Platform: ESP32-C3 (dual-core: no)
I (1235) SystemConfig: LED: pin=7, count=60, strip_length=60, update_interval=20ms
I (1236) SystemConfig: Mesh: channel=6, ttl=4, payload_len=200, cleanup_interval=30000ms
I (1237) SystemConfig: BLE: device='ESP_LED_NODE', packet_size=244
```

## 🔌 **Hardware Wiring Guide**

### **Basic LED Strip Connection**
```
ESP32/ESP32C3        WS2812B LED Strip
    GND      ────────      GND
    3.3V     ────────      VCC (if ≤10 LEDs)
    5V       ────────      VCC (if >10 LEDs, external power recommended)
   GPIO_X    ────────      DIN (Data Input)
```

### **Multiple ESP32 Mesh Network**
Each ESP32 needs:
- LED strip connected to configured GPIO pin
- Power supply appropriate for LED count
- Same mesh channel configuration

## 📱 **Mobile App Integration**

### **BLE Connection**
Your mobile app should connect to:
- **Service UUID**: Configured in BLE settings (default: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`)
- **Characteristic UUID**: For data packets (default: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`)
- **Device Name**: Appears in BLE scan (configurable)

### **Mesh Priority**
- **BLE Connected Node**: Automatically becomes mesh root
- **Multiple BLE Connections**: Newest connection wins
- **No BLE**: Nodes elect autonomous root via algorithm

## 🛠️ **Troubleshooting**

### **LED Issues**
| Problem | Solution |
|---------|----------|
| LEDs don't light up | Check GPIO pin, power supply, wiring |
| Wrong colors | Verify WS2812B vs WS2812 timing |
| Flickering | Add capacitor, check power supply |

### **Mesh Issues**  
| Problem | Solution |
|---------|----------|
| Nodes don't connect | Check mesh channel, power cycle all nodes |
| Split brain (multiple roots) | Update firmware, check election timeout |
| Poor range | Check antenna, reduce interference |

### **Configuration Issues**
| Problem | Solution |
|---------|----------|
| Config not saving | Check NVS partition, flash memory |
| Wrong platform detected | Verify `idf.py set-target` command |
| Build errors | Run `idf.py menuconfig` to fix missing config |

### **Memory Issues**
| Problem | Solution |
|---------|----------|
| Crashes with many LEDs | Reduce LED count, check power supply |
| Out of memory | Use ESP32 instead of ESP32C3 for large installations |

## 📊 **Performance Guidelines**

### **LED Count Recommendations**
| Platform | Conservative | Aggressive | Maximum Tested |
|----------|--------------|------------|-----------------|
| **ESP32-C3** | 30 LEDs | 60 LEDs | 100 LEDs |
| **ESP32** | 60 LEDs | 144 LEDs | 300+ LEDs |
| **ESP32-S3** | 100 LEDs | 200 LEDs | 500+ LEDs |

### **Network Size Recommendations**
- **Small Network**: 2-5 nodes (excellent performance)
- **Medium Network**: 6-15 nodes (good performance)  
- **Large Network**: 16-30 nodes (requires optimization)

## 🔒 **Security Configuration**

### **Enable Mesh Encryption** (Future)
```bash
# menuconfig → "ESP32 LED Mesh Configuration" → "Mesh Network Configuration"
# → Enable Mesh Encryption → Yes
```

### **Enable BLE Security** (Future)
```bash  
# menuconfig → "ESP32 LED Mesh Configuration" → "BLE Configuration"
# → Enable BLE Security → Yes
```

## 📚 **API Reference**

### **Configuration Functions**
```cpp
// LED Configuration
config.setLedPin(pin);                    // 0-48 (platform dependent)
config.setLedCount(count);                // 1-1000
config.setLedUpdateIntervalMs(interval);  // 1-1000ms (1000-1 FPS)

// Mesh Configuration  
config.setMeshChannel(channel);           // 1-14 (WiFi channels)
config.setMeshDefaultTTL(ttl);            // 1-10 hops

// BLE Configuration
config.setBLEDeviceName(name);            // Max 30 characters
config.setBLEServiceUUID(uuid);           // UUID format string

// System Configuration
config.setMainLoopDelayMs(delay);         // 1-1000ms
config.setDebugLoggingEnabled(enabled);   // true/false
```

### **Platform Detection**
```cpp
config.getPlatformName();                 // "ESP32", "ESP32-C3", etc.
config.isDualCoreEnabled();               // true for ESP32/ESP32S3
```

### **Convenience Macros**
```cpp
LED_PIN()           // Current LED pin
LED_COUNT()         // Current LED count  
MESH_CHANNEL()      // Current mesh channel
BLE_DEVICE_NAME()   // Current BLE device name
```

---

## 🚀 **Ready to Go!**

Your ESP32 LED Mesh system is now configured for your specific hardware. The system will automatically:
- Detect your ESP32 platform and optimize accordingly
- Use your configured LED pin and count
- Connect to mesh network on your specified channel
- Advertise BLE with your custom device name
- Save all runtime changes to flash memory

**Need help?** Check the `main/system/config_demo.cpp` file for detailed integration examples!
