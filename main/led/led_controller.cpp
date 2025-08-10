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
    , ledCommandQueue(nullptr)
    , ledTaskHandle(nullptr)
    , ledTaskMode(false)
{
    // Initialize currentPacket to zero
    memset(&currentPacket, 0, sizeof(currentPacket));
}

LEDController::~LEDController() {
    shutdownLEDTask();
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
    
    // Initialize LED task processing (works on both ESP32 and ESP32C3)
    esp_err_t led_task_result = initLEDTask();
    if (led_task_result == ESP_OK) {
#ifdef CONFIG_IDF_TARGET_ESP32
        ESP_LOGI(TAG, "LED Controller initialized with dedicated LED task on Core 1");
#else
        ESP_LOGI(TAG, "LED Controller initialized with high-priority LED task on Core 0");
#endif
    } else {
        ESP_LOGW(TAG, "LED task initialization failed, falling back to main loop mode");
        ledTaskMode = false;
    }
    
    // Start with idle sequence (after dual-core init if applicable)
    setSequence(idleSequence);
    
    return ESP_OK;
}

esp_err_t LEDController::processPacket(const GenericPacket& packet) {
    if (!initialized) {
        ESP_LOGW(TAG, "LED Controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Processing LED packet (%zu bytes)", packet.getLength());
    
    // Parse the packet into our Packet structure
    // Create a local copy for parsing to maintain const correctness
    Packet tempPacket;
    esp_err_t ret = parsePacketData(packet, tempPacket);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse packet data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Copy parsed data to member variable
    currentPacket = tempPacket;
    
    logPacketInfo(currentPacket);
    
    // Switch to packet sequence to display this pattern
    setSequence(packetSequence);
    
    ESP_LOGI(TAG, "✅ LED packet processed successfully");
    return ESP_OK;
}

esp_err_t LEDController::parsePacketData(const GenericPacket& packet, Packet& parsedPacket) const {
    // Bounds check: ensure packet is not empty or excessively large
    size_t packet_len = packet.getLength();
    if (packet_len == 0) {
        ESP_LOGE(TAG, "Empty packet received");
        return ESP_ERR_INVALID_ARG;
    }
    if (packet_len > 256) { // Reasonable upper bound
        ESP_LOGE(TAG, "Packet too large: %zu bytes", packet_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Try exact size match first (struct alignment)
    if (packet.getPacket(parsedPacket)) {
        ESP_LOGI(TAG, "✅ Exact packet format match");
        return ESP_OK;
    } 
    // Handle 19-byte packets manually (mobile app format)
    else if (packet.getLength() == 19) {
        ESP_LOGI(TAG, "📱 Manual parsing of 19-byte mobile app packet");
        const uint8_t* data = packet.getData();
        
        // Additional safety: verify data pointer is valid
        if (!data) {
            ESP_LOGE(TAG, "Null data pointer in packet");
            return ESP_ERR_INVALID_ARG;
        }
        
        // Parse the packet manually (little-endian format expected)
        parsedPacket.command = data[0];
        parsedPacket.brightness = data[1];
        parsedPacket.speed = data[2];
        parsedPacket.pattern = data[3];
        
        // Parse color array (3 x uint32_t = 12 bytes) - bounds are guaranteed by length check
        memcpy(&parsedPacket.color[0], &data[4], 4);
        memcpy(&parsedPacket.color[1], &data[8], 4);
        memcpy(&parsedPacket.color[2], &data[12], 4);
        
        // Parse level array (3 x uint8_t = 3 bytes) - bounds are guaranteed by length check
        parsedPacket.level[0] = data[16];
        parsedPacket.level[1] = data[17];
        parsedPacket.level[2] = data[18];
        
        ESP_LOGI(TAG, "🔧 Manual parsing complete");
        return ESP_OK;
    } 
    else {
        ESP_LOGE(TAG, "❌ Unsupported packet size: %zu bytes", packet.getLength());
        return ESP_ERR_INVALID_SIZE;
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
    setSequence(idleSequence);
}

void LEDController::setAlertMode() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to alert mode");
    setSequence(alertSequence);
}

void LEDController::setRandomMode() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to random mode");
    setSequence(randomSequence);
}

void LEDController::setSequence(Sequence* sequence) {
    if (!initialized || !sequence) return;
    
    if (ledTaskMode) {
        // Send sequence change command to LED task
        LEDCommand cmd = {
            .type = LEDCommandType::SET_SEQUENCE,
            .data = {.setSeq = {.sequence = sequence}}
        };
        
        if (!sendLEDCommand(cmd, pdMS_TO_TICKS(100))) {
            ESP_LOGW(TAG, "Failed to send sequence command to LED task, falling back to direct call");
            player->SetSequence(sequence);
        }
        return;
    }
    
    // Main loop mode: direct call
    player->SetSequence(sequence);
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
    
    // In LED task mode, LED processing happens in dedicated task
    if (ledTaskMode) {
        // LED processing is handled by dedicated high-priority task
        // Main loop can focus on BLE/WiFi/Mesh communication
        return;
    }
    
    // Main loop mode: perform LED updates on current core
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

// LED task processing implementation (ESP32 and ESP32C3)

esp_err_t LEDController::initLEDTask() {
#ifdef CONFIG_IDF_TARGET_ESP32
    ESP_LOGI(TAG, "Initializing LED task for ESP32 (Core 1)");
#else
    ESP_LOGI(TAG, "Initializing high-priority LED task for ESP32C3 (Core 0)");
#endif
    
    // Create command queue for main loop to LED task communication
    ledCommandQueue = xQueueCreate(10, sizeof(LEDCommand));
    if (!ledCommandQueue) {
        ESP_LOGE(TAG, "Failed to create LED command queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Create LED processing task with platform-specific configuration
#ifdef CONFIG_IDF_TARGET_ESP32
    // ESP32: Pin to Core 1, standard priority
    BaseType_t result = xTaskCreatePinnedToCore(
        ledProcessingTaskWrapper,              // Task function
        "LED_Task",                            // Task name
        4096,                                  // Stack size (4KB)
        this,                                  // Task parameter (this instance)
        10,                                    // Priority (high for LED processing)
        &ledTaskHandle,                        // Task handle
        1                                      // Pin to Core 1
    );
#else
    // ESP32C3: Core 0 with high priority (Priority 8 > mesh/BLE at Priority 1-5)
    BaseType_t result = xTaskCreate(
        ledProcessingTaskWrapper,              // Task function
        "LED_Task",                            // Task name
        4096,                                  // Stack size (4KB)
        this,                                  // Task parameter (this instance)
        8,                                     // High priority (8 > networking tasks)
        &ledTaskHandle                         // Task handle
    );
#endif
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED processing task");
        vQueueDelete(ledCommandQueue);
        ledCommandQueue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    
    ledTaskMode = true;
    ESP_LOGI(TAG, "✅ LED task processing initialized successfully");
    return ESP_OK;
}

void LEDController::shutdownLEDTask() {
    if (!ledTaskMode) {
        return;
    }
    
    ESP_LOGI(TAG, "Shutting down LED task processing");
    
    // Send shutdown command to LED task
    LEDCommand shutdownCmd;
    shutdownCmd.type = LEDCommandType::SHUTDOWN;
    sendLEDCommand(shutdownCmd, pdMS_TO_TICKS(1000));
    
    // Wait for task to complete
    if (ledTaskHandle) {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = nullptr;
    }
    
    // Clean up queue
    if (ledCommandQueue) {
        vQueueDelete(ledCommandQueue);
        ledCommandQueue = nullptr;
    }
    
    ledTaskMode = false;
    ESP_LOGI(TAG, "LED task processing shut down");
}

void LEDController::ledProcessingTaskWrapper(void* parameter) {
    LEDController* controller = static_cast<LEDController*>(parameter);
    controller->ledProcessingTask();
}

void LEDController::ledProcessingTask() {
#ifdef CONFIG_IDF_TARGET_ESP32
    ESP_LOGI(TAG, "LED processing task started on ESP32 Core 1");
#else
    ESP_LOGI(TAG, "LED processing task started on ESP32C3 Core 0 (Priority 8)");
#endif
    
    LEDCommand command;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(15); // 15ms = 67 FPS for smooth LED performance
    
    while (true) {
        // Process any pending commands with non-blocking receive
        while (xQueueReceive(ledCommandQueue, &command, 0) == pdTRUE) {
            switch (command.type) {
                case LEDCommandType::SHUTDOWN:
                    ESP_LOGI(TAG, "LED task received shutdown command");
                    vTaskDelete(nullptr); // Delete self
                    return;
                    
                case LEDCommandType::SET_SEQUENCE:
                    if (player && command.data.setSeq.sequence) {
                        player->SetSequence(command.data.setSeq.sequence);
                        ESP_LOGD(TAG, "Sequence updated in LED task");
                    }
                    break;
                    
                case LEDCommandType::UPDATE_PATTERN:
                    // This is handled in the main update loop below
                    break;
            }
        }
        
        // Perform regular LED updates with precise timing
        if (initialized && player && strip) {
            led_time_t now = esp_timer_get_time() / 1000; // Convert to ms
            
            // Update pattern if needed
            player->UpdatePattern(now, strip);
            
            // Update the LED strip
            player->UpdateStrip(now, strip);
        }
        
        // Maintain precise FPS timing (guaranteed CPU time due to high priority)
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

bool LEDController::sendLEDCommand(const LEDCommand& command, TickType_t timeout) {
    if (!ledTaskMode || !ledCommandQueue) {
        return false;
    }
    
    return xQueueSend(ledCommandQueue, &command, timeout) == pdTRUE;
}

