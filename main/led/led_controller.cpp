#include "led_controller.h"
#include "esp_log.h"
#include <cstring>

const char* LEDController::TAG = "LEDController";

LEDController::LEDController(uint8_t pin, uint16_t count) 
    : strip(nullptr)
    , player(nullptr)
    , idleSequence(nullptr)
    , alertSequence(nullptr)
    , randomSequence(nullptr)
    , packetSequence(nullptr)
    , initialized(false)
    , ledPin(pin)
    , ledCount(count)
{
    // Initialize currentPacket to zero
    memset(&currentPacket, 0, sizeof(currentPacket));
}

LEDController::~LEDController() {
    cleanup();
}

void LEDController::cleanup() {
    delete strip;
    strip = nullptr;
    delete player;
    player = nullptr;
    delete idleSequence;
    idleSequence = nullptr;
    delete alertSequence;
    alertSequence = nullptr;
    delete randomSequence;
    randomSequence = nullptr;
    delete packetSequence;
    packetSequence = nullptr;
    initialized = false;
}

esp_err_t LEDController::begin() {
    ESP_LOGI(TAG, "Initializing LED Controller - Pin: %d, Count: %d", ledPin, ledCount);
    
    // Create LED strip
    strip = new(std::nothrow) LEDStrip(ledCount, ledPin);
    if (!strip) {
        ESP_LOGE(TAG, "Failed to create LED strip");
        return ESP_ERR_NO_MEM;
    }
    
    esp_err_t ret = strip->begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED strip: %s", esp_err_to_name(ret));
        cleanup(); // Clean up on failure
        return ret;
    }
    
    // Create player
    player = new(std::nothrow) Player();
    if (!player) {
        ESP_LOGE(TAG, "Failed to create player");
        cleanup(); // Clean up on failure
        return ESP_ERR_NO_MEM;
    }
    
    // Create sequences with proper error checking
    idleSequence = new(std::nothrow) IdleSequence();
    if (!idleSequence) {
        ESP_LOGE(TAG, "Failed to create idle sequence");
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    
    alertSequence = new(std::nothrow) AlertSequence();
    if (!alertSequence) {
        ESP_LOGE(TAG, "Failed to create alert sequence");
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    
    randomSequence = new(std::nothrow) RandomSequence();
    if (!randomSequence) {
        ESP_LOGE(TAG, "Failed to create random sequence");
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    
    packetSequence = new(std::nothrow) PacketSequence(&currentPacket);
    if (!packetSequence) {
        ESP_LOGE(TAG, "Failed to create packet sequence");
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    
    // Start with idle sequence
    player->SetSequence(idleSequence);
    
    // Clear entire physical strip to ensure no leftover LEDs are lit
    ESP_LOGI(TAG, "Clearing entire physical LED strip (%d LEDs)", PHYSICAL_LED_STRIP_LENGTH);
    strip->clearAll(PHYSICAL_LED_STRIP_LENGTH);
    
    // Show initialization pattern (green dots)
    strip->clear();
    for (uint16_t i = 0; i < 5 && i < ledCount; i++) {
        strip->setPixelColor(i, 0x00FF00); // GREEN
    }
    strip->show();
    
    initialized = true;
    ESP_LOGI(TAG, "LED Controller initialized successfully");
    return ESP_OK;
}

bool LEDController::processPacket(const GenericPacket& packet) {
    if (!initialized) {
        ESP_LOGW(TAG, "LED Controller not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Processing LED packet (%zu bytes)", packet.getLength());
    
    // Parse the packet into our Packet structure
    // Create a local copy for parsing to maintain const correctness
    Packet tempPacket;
    if (!parsePacketData(packet, tempPacket)) {
        ESP_LOGE(TAG, "Failed to parse packet data");
        return false;
    }
    
    // Copy parsed data to member variable
    currentPacket = tempPacket;
    
    logPacketInfo(currentPacket);
    
    // Switch to packet sequence to display this pattern
    player->SetSequence(packetSequence);
    
    ESP_LOGI(TAG, "✅ LED packet processed successfully");
    return true;
}

bool LEDController::parsePacketData(const GenericPacket& packet, Packet& parsedPacket) const {
    // Try exact size match first (struct alignment)
    if (packet.getPacket(parsedPacket)) {
        ESP_LOGI(TAG, "✅ Exact packet format match");
        return true;
    } 
    // Handle 19-byte packets manually (mobile app format)
    else if (packet.getLength() == 19) {
        ESP_LOGI(TAG, "📱 Manual parsing of 19-byte mobile app packet");
        const uint8_t* data = packet.getData();
        
        // Parse the packet manually (little-endian format expected)
        parsedPacket.command = data[0];
        parsedPacket.brightness = data[1];
        parsedPacket.speed = data[2];
        parsedPacket.pattern = data[3];
        
        // Parse color array (3 x uint32_t = 12 bytes)
        memcpy(&parsedPacket.color[0], &data[4], 4);
        memcpy(&parsedPacket.color[1], &data[8], 4);
        memcpy(&parsedPacket.color[2], &data[12], 4);
        
        // Parse level array (3 x uint8_t = 3 bytes)  
        parsedPacket.level[0] = data[16];
        parsedPacket.level[1] = data[17];
        parsedPacket.level[2] = data[18];
        
        ESP_LOGI(TAG, "🔧 Manual parsing complete");
        return true;
    } 
    else {
        ESP_LOGE(TAG, "❌ Unsupported packet size: %zu bytes", packet.getLength());
        return false;
    }
}

void LEDController::logPacketInfo(const Packet& pkt) const {
    ESP_LOGI(TAG, "=== Parsed LED Packet ===");
    ESP_LOGI(TAG, "Command: %u", pkt.command);
    ESP_LOGI(TAG, "Brightness: %u", pkt.brightness);
    ESP_LOGI(TAG, "Speed: %u", pkt.speed);
    ESP_LOGI(TAG, "Pattern: %u", pkt.pattern);
    ESP_LOGI(TAG, "Color: R=0x%08lX, G=0x%08lX, B=0x%08lX", pkt.color[0], pkt.color[1], pkt.color[2]);
    ESP_LOGI(TAG, "Level: L1=%u, L2=%u, L3=%u", pkt.level[0], pkt.level[1], pkt.level[2]);
    ESP_LOGI(TAG, "========================");
}

void LEDController::setIdleMode() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to idle mode");
    player->SetSequence(idleSequence);
}

void LEDController::setAlertMode() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to alert mode");
    player->SetSequence(alertSequence);
}

void LEDController::setRandomMode() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to random mode");
    player->SetSequence(randomSequence);
}

void LEDController::advanceSequence() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Advancing current sequence");
    player->AdvanceSequence();
}

void LEDController::update() {
    if (!initialized || !player || !strip) {
        return;
    }
    
    time_t now = esp_timer_get_time() / 1000; // Convert to ms
    
    // Update pattern if needed
    player->UpdatePattern(now, strip);
    
    // Update the LED strip
    player->UpdateStrip(now, strip);
}

const char* LEDController::getCurrentSequenceType() const {
    if (!initialized || !player) {
        return "Not initialized";
    }
    
    Sequence* current = player->GetSequence();
    if (current == idleSequence) {
        return "Idle";
    } else if (current == alertSequence) {
        return "Alert";
    } else if (current == randomSequence) {
        return "Random";
    } else if (current == packetSequence) {
        return "Packet/Mobile";
    } else {
        return "Unknown";
    }
}