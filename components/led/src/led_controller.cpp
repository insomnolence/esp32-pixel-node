#include "led/player.h"
#include "led/led_controller.h"
#include "led/led_strip.h"
#include "esp_log.h"
#include "common/button_feedback_types.h"
#include <cstring>

const char* LEDController::TAG = "LEDController";

LEDController::LEDController(uint8_t pin, uint16_t count) 
    : strip(nullptr)
    , player(nullptr)
    , idleSequence(nullptr)
    , alertSequence(nullptr)
    , randomSequence(nullptr)
    , singleRandomSequence(nullptr)
    , packetSequence(nullptr)
    , warningSequence(nullptr)
    , exitSequence(nullptr)
    , initialized(false)
    , ledPin(pin)
    , ledCount(count)
    , singleRandomActive(false)
    , singleRandomStartTime(0)
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
    delete singleRandomSequence;
    singleRandomSequence = nullptr;
    delete packetSequence;
    packetSequence = nullptr;
    delete warningSequence;
    warningSequence = nullptr;
    delete exitSequence;
    exitSequence = nullptr;
    initialized = false;
}

esp_err_t LEDController::begin() {
    ESP_LOGI(TAG, "Initializing LED Controller - Pin: %d, Count: %d", ledPin, ledCount);

    // Track number of successful allocations for better error reporting
    int allocations_completed = 0;

    // Create LED strip (allocation 1 of 9)
    strip = new(std::nothrow) LEDStrip(ledCount, ledPin);
    if (!strip) {
        ESP_LOGE(TAG, "Failed to create LED strip (allocation 1/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    esp_err_t ret = strip->begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED strip: %s - cleaning up %d allocations", esp_err_to_name(ret), allocations_completed);
        cleanup();
        return ret;
    }

    // Create player (allocation 2 of 9)
    player = new(std::nothrow) Player();
    if (!player) {
        ESP_LOGE(TAG, "Failed to create player (allocation 2/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    // Create sequences with proper error checking
    // Allocation 3 of 9
    idleSequence = new(std::nothrow) IdleSequence();
    if (!idleSequence) {
        ESP_LOGE(TAG, "Failed to create idle sequence (allocation 3/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    // Allocation 4 of 9
    alertSequence = new(std::nothrow) AlertSequence();
    if (!alertSequence) {
        ESP_LOGE(TAG, "Failed to create alert sequence (allocation 4/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    // Allocation 5 of 9
    randomSequence = new(std::nothrow) RandomSequence();
    if (!randomSequence) {
        ESP_LOGE(TAG, "Failed to create random sequence (allocation 5/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    // Allocation 6 of 9
    singleRandomSequence = new(std::nothrow) SingleRandomSequence();
    if (!singleRandomSequence) {
        ESP_LOGE(TAG, "Failed to create single random sequence (allocation 6/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    // Allocation 7 of 9
    packetSequence = new(std::nothrow) PacketSequence(&currentPacket);
    if (!packetSequence) {
        ESP_LOGE(TAG, "Failed to create packet sequence (allocation 7/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    // Create Warning sequence (Yellow colors)
    // Primary: YELLOW (#FFFF00), Secondary: Light Yellow (#FFFC40)
    // Allocation 8 of 9
    warningSequence = new(std::nothrow) ParameterizedSequence(YELLOW, 0xFFFC40, YELLOW);
    if (!warningSequence) {
        ESP_LOGE(TAG, "Failed to create warning sequence (allocation 8/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    // Create Exit sequence (Red colors)
    // Primary: RED (#FF0000), Secondary: Light Red (#FF4040)
    // Allocation 9 of 9
    exitSequence = new(std::nothrow) ParameterizedSequence(RED, 0xFF4040, RED);
    if (!exitSequence) {
        ESP_LOGE(TAG, "Failed to create exit sequence (allocation 9/9) - cleaning up %d prior allocations", allocations_completed);
        cleanup();
        return ESP_ERR_NO_MEM;
    }
    allocations_completed++;

    ESP_LOGI(TAG, "All %d allocations completed successfully", allocations_completed);
    
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
    
    // Reset single random tracking - BLE patterns take priority over button patterns
    singleRandomActive = false;
    
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
    // Reset single random tracking
    singleRandomActive = false;
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

void LEDController::setWarningMode() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to warning mode (yellow pattern)");
    setSequence(warningSequence);
    // Reset single random tracking
    singleRandomActive = false;
}

void LEDController::setExitMode() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to exit mode (red pattern)");
    setSequence(exitSequence);
    // Reset single random tracking
    singleRandomActive = false;
}

void LEDController::setRandomModeWithNewPattern() {
    if (!initialized) return;
    
    ESP_LOGI(TAG, "Switching to single random pattern mode with new pattern selection");
    // Pick a new random pattern
    singleRandomSequence->pickNewRandomPattern();
    // Switch to the single random sequence (will run for 30 seconds then we'll switch back to idle)
    setSequence(singleRandomSequence);
    
    // Track that we're in single random mode and when it started
    singleRandomActive = true;
    singleRandomStartTime = esp_timer_get_time() / 1000; // Current time in ms
    ESP_LOGI(TAG, "Single random pattern started - will return to idle after 30 seconds");
}

// Note: Visual feedback methods moved to ButtonFeedbackController for local-only feedback
// This avoids creating temporary packet sequences and broadcasting over mesh

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

    if (ledTaskMode) {
        // Route through LED task queue to prevent race conditions
        LEDCommand cmd;
        cmd.type = LEDCommandType::ADVANCE_SEQUENCE;

        if (!sendLEDCommand(cmd, pdMS_TO_TICKS(100))) {
            ESP_LOGW(TAG, "Failed to send advance sequence command to LED task");
        }
        return;
    }

    // Main loop mode: direct call
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
    
    // Check if single random pattern should return to idle (30 seconds elapsed)
    if (singleRandomActive) {
        if (now - singleRandomStartTime >= 30000) { // 30 seconds = 30000ms
            ESP_LOGI(TAG, "Single random pattern completed - returning to idle mode");
            setIdleMode();
            singleRandomActive = false;
        }
    }
    
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
    } else if (current == singleRandomSequence) {
        return "SingleRandom";
    } else if (current == warningSequence) {
        return "Warning";
    } else if (current == exitSequence) {
        return "Exit";
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

    // Wait for task to self-delete by giving it time to process the shutdown command
    // The task deletes itself when receiving SHUTDOWN, so we must not call vTaskDelete
    vTaskDelay(pdMS_TO_TICKS(100));

    // Clear the task handle since the task has self-deleted
    ledTaskHandle = nullptr;

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
                    
                case LEDCommandType::BUTTON_FEEDBACK:
                    // Handle button feedback within LED task to prevent RMT conflicts
                    handleButtonFeedbackInLEDTask(command.data.buttonFeedback.feedback_type,
                                                  command.data.buttonFeedback.duration_ms);
                    break;

                case LEDCommandType::ADVANCE_SEQUENCE:
                    // Handle sequence advancement in LED task to prevent race conditions
                    if (player) {
                        player->AdvanceSequence();
                        ESP_LOGD(TAG, "Sequence advanced in LED task");
                    }
                    break;
            }
        }
        
        // Perform regular LED updates with precise timing
        if (initialized && player && strip) {
            led_time_t now = esp_timer_get_time() / 1000; // Convert to ms
            
            // Check if single random pattern should return to idle (30 seconds elapsed)
            if (singleRandomActive) {
                if (now - singleRandomStartTime >= 30000) { // 30 seconds = 30000ms
                    ESP_LOGI(TAG, "Single random pattern completed - returning to idle mode");
                    // Switch directly to idle sequence (we're in LED task, safe to call)
                    if (player && idleSequence) {
                        player->SetSequence(idleSequence);
                        singleRandomActive = false;
                        ESP_LOGI(TAG, "Switched to idle mode from LED task");
                    }
                }
            }
            
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

esp_err_t LEDController::showButtonFeedback(FeedbackType type, uint32_t duration_ms) {
    if (!initialized) {
        ESP_LOGW(TAG, "LED Controller not initialized - cannot show button feedback");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!ledTaskMode) {
        ESP_LOGW(TAG, "LED task mode not active - button feedback requires LED task");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send button feedback command to LED task
    LEDCommand cmd = {
        .type = LEDCommandType::BUTTON_FEEDBACK,
        .data = {.buttonFeedback = {.feedback_type = type, .duration_ms = duration_ms}}
    };
    
    if (!sendLEDCommand(cmd, pdMS_TO_TICKS(10))) {
        ESP_LOGW(TAG, "Failed to send button feedback command to LED task");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGD(TAG, "Button feedback command sent to LED task (type=%d, duration=%ums)", 
             (int)type, duration_ms);
    return ESP_OK;
}

void LEDController::handleButtonFeedbackInLEDTask(FeedbackType type, uint32_t duration_ms) {
    if (!strip) {
        ESP_LOGW(TAG, "LED strip not available for button feedback");
        return;
    }
    
    uint16_t led_count = strip->numPixels();
    if (led_count == 0) {
        ESP_LOGW(TAG, "No LEDs configured for button feedback");
        return;
    }
    
    ESP_LOGD(TAG, "Handling button feedback in LED task: type=%d, duration=%ums", 
             (int)type, duration_ms);
    
    // Apply immediate feedback pattern based on type
    switch (type) {
        case BUTTON_PRESS_FEEDBACK: {
            // Quick blue flash for button press
            for (uint16_t i = 0; i < led_count; i++) {
                strip->setPixelColor(i, 0x0000FF); // BLUE
            }
            strip->show();
            // Brief flash, then clear
            vTaskDelay(pdMS_TO_TICKS(100));
            strip->clear();
            strip->show();
            break;
        }
        
        case DUAL_PRESS_DETECTED: {
            // Magenta pattern for dual press detection
            strip->clear();
            for (uint16_t i = 0; i < led_count && i < 10; i++) {
                strip->setPixelColor(i, 0xFF00FF); // MAGENTA
            }
            strip->show();
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
            strip->clear();
            strip->show();
            break;
        }
        
        case TAKEOVER_IN_PROGRESS: {
            // Animated purple marching pattern
            uint32_t end_time = esp_timer_get_time() / 1000 + duration_ms;
            while (esp_timer_get_time() / 1000 < end_time) {
                uint32_t elapsed = (esp_timer_get_time() / 1000) - (end_time - duration_ms);
                uint32_t phase = (elapsed / 150) % 4;
                
                strip->clear();
                for (uint16_t i = phase; i < led_count; i += 4) {
                    strip->setPixelColor(i, 0xFF00FF); // MAGENTA
                }
                strip->show();
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            strip->clear();
            strip->show();
            break;
        }
        
        case TAKEOVER_SUCCESS: {
            // Green success pattern
            for (uint16_t i = 0; i < led_count; i++) {
                strip->setPixelColor(i, 0x00FF00); // GREEN
            }
            strip->show();
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
            strip->clear();
            strip->show();
            break;
        }
        
        case TAKEOVER_FAILED: {
            // Red strobe for failure
            uint32_t end_time = esp_timer_get_time() / 1000 + duration_ms;
            while (esp_timer_get_time() / 1000 < end_time) {
                // Red on
                for (uint16_t i = 0; i < led_count; i++) {
                    strip->setPixelColor(i, 0xFF0000); // RED
                }
                strip->show();
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // Off
                strip->clear();
                strip->show();
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            break;
        }
        
        case BUTTON_BLOCKED_BLE:
        case BUTTON_BLOCKED_CLIENT: {
            // Silent blocking - no visual feedback (deprecated, handled in ButtonLogic)
            ESP_LOGD(TAG, "Button blocked feedback type %d (silent mode)", (int)type);
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown button feedback type: %d", (int)type);
            break;
    }
    
    ESP_LOGD(TAG, "Button feedback completed in LED task");
}

