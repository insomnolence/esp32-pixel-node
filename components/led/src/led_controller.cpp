#include "led/player.h"
#include "led/led_controller.h"
#include "led/led_strip.h"
#include "packet/packet.h"
#include "esp_log.h"
#include "common/button_feedback_types.h"
#include <cstring>

const char* LEDController::TAG = "LEDController";

LEDController::LEDController(ILedStrip* s)
    : strip(s)
    , player(nullptr)
    , idleSequence(nullptr)
    , alertSequence(nullptr)
    , randomSequence(nullptr)
    , singleRandomSequence(nullptr)
    , packetSequence(nullptr)
    , warningSequence(nullptr)
    , exitSequence(nullptr)
    , initialized(false)
    , ledCount(s ? s->numPixels() : 0)
    , singleRandomActive(false)
    , singleRandomStartTime(0)
    , ledCommandQueue(nullptr)
    , ledTaskHandle(nullptr)
    , ledTaskExitSemaphore(nullptr)
    , ledTaskMode(false)
    , ledTaskShutdownRequested(false)
    , lastPatternReceivedTime(0)
    , lastRootStatusCheck(0)
{
    // Initialize currentPacket to zero
    memset(&currentPacket, 0, sizeof(currentPacket));
}

LEDController::~LEDController() {
    shutdownLEDTask();
    cleanup();
}

void LEDController::cleanup() {
    // Do NOT delete strip, we don't own it
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

esp_err_t LEDController::begin(bool useTaskMode) {
    ESP_LOGI(TAG, "Initializing LED Controller - Count: %d, TaskMode: %s", ledCount, useTaskMode ? "true" : "false");

    if (!strip) {
        ESP_LOGE(TAG, "No LED strip provided!");
        return ESP_ERR_INVALID_ARG;
    }

    // Track number of successful allocations for better error reporting
    int allocations_completed = 0;

    esp_err_t ret = strip->begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED strip: %s", esp_err_to_name(ret));
        cleanup();
        return ret;
    }

    // Create player (allocation 1 of 8)
    player = new(std::nothrow) Player();
    if (!player) {
        ESP_LOGE(TAG, "Failed to create player (allocation 1/8)");
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
    // Skip task mode for unit tests to allow synchronous testing
    if (useTaskMode) {
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
    } else {
        ESP_LOGI(TAG, "LED Controller initialized in synchronous mode (no task)");
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
    
    // Handle 19-byte packets - always use manual parsing for correct color byte order
    // The Flutter app sends colors as little-endian {R, G, B, 0} which needs conversion
    if (packet.getLength() == 19) {
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
        // Flutter sends 32-bit ARGB colors in little-endian, so bytes are {B, G, R, A} in memory
        // Convert to standard RGB format: 0xRRGGBB
        for (int i = 0; i < 3; i++) {
            const uint8_t* colorBytes = &data[4 + i * 4];
            parsedPacket.color[i] = ((uint32_t)colorBytes[2] << 16) |  // R (byte 2) -> high byte
                                    ((uint32_t)colorBytes[1] << 8) |   // G (byte 1) -> middle byte
                                    ((uint32_t)colorBytes[0]);          // B (byte 0) -> low byte
        }
        
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

void LEDController::setPatternBroadcastCallback(PatternBroadcastCallback callback) {
    patternBroadcastCallback = callback;

    // Set up player's step change callback to broadcast step changes to mesh
    if (player && callback) {
        player->setStepChangeCallback([this](const Packet& packet) {
            ESP_LOGI(TAG, "Player step changed - broadcasting to mesh");
            broadcastPattern(packet);
        });
    }
}

void LEDController::setRootStatusCallback(RootStatusCallback callback) {
    rootStatusCallback = callback;
}

void LEDController::onPatternReceived() {
    lastPatternReceivedTime = esp_timer_get_time() / 1000; // Convert to ms
}

void LEDController::broadcastPattern(const Packet& packet) {
    if (!patternBroadcastCallback) {
        // No callback set - this is fine, not all modes need broadcasting
        return;
    }

    // Serialize the packet to wire format (19 bytes)
    uint8_t wireBuffer[LED_PACKET_WIRE_SIZE];
    size_t wireLen = serializePacketToWire(packet, wireBuffer, sizeof(wireBuffer));

    if (wireLen == 0) {
        ESP_LOGE(TAG, "Failed to serialize pattern for broadcast");
        return;
    }

    // Create GenericPacket and invoke callback
    GenericPacket genericPacket(wireBuffer, wireLen);
    patternBroadcastCallback(genericPacket);
}

void LEDController::setIdleMode() {
    if (!initialized) return;

    ESP_LOGI(TAG, "Switching to idle mode");
    setSequence(idleSequence);
    // Reset single random tracking
    singleRandomActive = false;

    // Broadcast idle pattern to mesh if callback is set
    if (idleSequence && idleSequence->GetStepCount() > 0) {
        Packet packet;
        packet.command = idleSequence->GetCommand(0);
        packet.brightness = idleSequence->GetBrightness(0);
        packet.speed = idleSequence->GetSpeed(0);
        packet.pattern = idleSequence->GetPatternId(0);
        packet.color[0] = idleSequence->GetColors(0, 0);
        packet.color[1] = idleSequence->GetColors(0, 1);
        packet.color[2] = idleSequence->GetColors(0, 2);
        packet.level[0] = idleSequence->GetLevels(0, 0);
        packet.level[1] = idleSequence->GetLevels(0, 1);
        packet.level[2] = idleSequence->GetLevels(0, 2);
        broadcastPattern(packet);
    }
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
    // Reset pattern received time - autonomous root generates patterns, doesn't receive them
    // This prevents the "root disappeared" 15s timeout from triggering
    lastPatternReceivedTime = 0;

    // Broadcast warning pattern to mesh if callback is set
    if (warningSequence && warningSequence->GetStepCount() > 0) {
        Packet packet;
        packet.command = warningSequence->GetCommand(0);
        packet.brightness = warningSequence->GetBrightness(0);
        packet.speed = warningSequence->GetSpeed(0);
        packet.pattern = warningSequence->GetPatternId(0);
        packet.color[0] = warningSequence->GetColors(0, 0);
        packet.color[1] = warningSequence->GetColors(0, 1);
        packet.color[2] = warningSequence->GetColors(0, 2);
        packet.level[0] = warningSequence->GetLevels(0, 0);
        packet.level[1] = warningSequence->GetLevels(0, 1);
        packet.level[2] = warningSequence->GetLevels(0, 2);
        broadcastPattern(packet);
    }
}

void LEDController::setExitMode() {
    if (!initialized) return;

    ESP_LOGI(TAG, "Switching to exit mode (red pattern)");
    setSequence(exitSequence);
    // Reset single random tracking
    singleRandomActive = false;
    // Reset pattern received time - autonomous root generates patterns, doesn't receive them
    // This prevents the "root disappeared" 15s timeout from triggering
    lastPatternReceivedTime = 0;

    // Broadcast exit pattern to mesh if callback is set
    if (exitSequence && exitSequence->GetStepCount() > 0) {
        Packet packet;
        packet.command = exitSequence->GetCommand(0);
        packet.brightness = exitSequence->GetBrightness(0);
        packet.speed = exitSequence->GetSpeed(0);
        packet.pattern = exitSequence->GetPatternId(0);
        packet.color[0] = exitSequence->GetColors(0, 0);
        packet.color[1] = exitSequence->GetColors(0, 1);
        packet.color[2] = exitSequence->GetColors(0, 2);
        packet.level[0] = exitSequence->GetLevels(0, 0);
        packet.level[1] = exitSequence->GetLevels(0, 1);
        packet.level[2] = exitSequence->GetLevels(0, 2);
        broadcastPattern(packet);
    }
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
    singleRandomStartTime = strip->getMillis(); // Current time in ms
    // Reset pattern received time - autonomous root generates patterns, doesn't receive them
    // This prevents the "root disappeared" 15s timeout from triggering
    lastPatternReceivedTime = 0;
    ESP_LOGI(TAG, "Single random pattern started - will return to idle after 30 seconds");

    // Broadcast random pattern to mesh if callback is set
    if (singleRandomSequence && singleRandomSequence->GetStepCount() > 0) {
        Packet packet;
        packet.command = singleRandomSequence->GetCommand(0);
        packet.brightness = singleRandomSequence->GetBrightness(0);
        packet.speed = singleRandomSequence->GetSpeed(0);
        packet.pattern = singleRandomSequence->GetPatternId(0);
        packet.color[0] = singleRandomSequence->GetColors(0, 0);
        packet.color[1] = singleRandomSequence->GetColors(0, 1);
        packet.color[2] = singleRandomSequence->GetColors(0, 2);
        packet.level[0] = singleRandomSequence->GetLevels(0, 0);
        packet.level[1] = singleRandomSequence->GetLevels(0, 1);
        packet.level[2] = singleRandomSequence->GetLevels(0, 2);
        broadcastPattern(packet);
    }
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
    time_t now = strip->getMillis(); // Convert to ms
    
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

    // Reset shutdown flag before creating task
    ledTaskShutdownRequested = false;

    // Create semaphore to signal task exit completion
    ledTaskExitSemaphore = xSemaphoreCreateBinary();
    if (!ledTaskExitSemaphore) {
        ESP_LOGE(TAG, "Failed to create LED task exit semaphore");
        return ESP_ERR_NO_MEM;
    }

    // Create command queue for main loop to LED task communication
    ledCommandQueue = xQueueCreate(10, sizeof(LEDCommand));
    if (!ledCommandQueue) {
        ESP_LOGE(TAG, "Failed to create LED command queue");
        vSemaphoreDelete(ledTaskExitSemaphore);
        ledTaskExitSemaphore = nullptr;
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
        vSemaphoreDelete(ledTaskExitSemaphore);
        ledTaskExitSemaphore = nullptr;
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

    // Signal the task to stop accessing the strip pointer FIRST
    // This is critical to prevent use-after-free when MockLedStrip is destroyed
    ledTaskShutdownRequested = true;

    // Send shutdown command to LED task (may wake it from delay)
    LEDCommand shutdownCmd;
    shutdownCmd.type = LEDCommandType::SHUTDOWN;
    sendLEDCommand(shutdownCmd, pdMS_TO_TICKS(100));

    // Wait for the task to signal it has fully exited using semaphore
    // This is more reliable than polling eTaskGetState() on a potentially deleted handle
    if (ledTaskExitSemaphore) {
        if (xSemaphoreTake(ledTaskExitSemaphore, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "LED task did not signal exit in time, forcing deletion");
            if (ledTaskHandle) {
                vTaskDelete(ledTaskHandle);
            }
        }
    }

    // Clear the task handle since the task has exited
    ledTaskHandle = nullptr;

    // Now safe to clean up queue and semaphore - task is guaranteed to be gone
    if (ledCommandQueue) {
        vQueueDelete(ledCommandQueue);
        ledCommandQueue = nullptr;
    }

    if (ledTaskExitSemaphore) {
        vSemaphoreDelete(ledTaskExitSemaphore);
        ledTaskExitSemaphore = nullptr;
    }

    ledTaskMode = false;
    ledTaskShutdownRequested = false;  // Reset for potential re-init
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
        // Check shutdown flag BEFORE accessing queue to prevent race condition
        // where queue is deleted while we're trying to use it
        if (ledTaskShutdownRequested) {
            ESP_LOGI(TAG, "LED task shutdown requested via flag");
            // Signal that we're exiting before deleting ourselves
            if (ledTaskExitSemaphore) {
                xSemaphoreGive(ledTaskExitSemaphore);
            }
            vTaskDelete(nullptr);
            return;
        }

        // Process any pending commands with non-blocking receive
        // Queue access is safe here because shutdown flag check above ensures
        // the queue won't be deleted while we're using it
        while (ledCommandQueue && xQueueReceive(ledCommandQueue, &command, 0) == pdTRUE) {
            switch (command.type) {
                case LEDCommandType::SHUTDOWN:
                    ESP_LOGI(TAG, "LED task received shutdown command");
                    // Signal that we're exiting before deleting ourselves
                    if (ledTaskExitSemaphore) {
                        xSemaphoreGive(ledTaskExitSemaphore);
                    }
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
        // Capture strip pointer locally and double-check shutdown flag to prevent
        // accessing strip after cleanup() nullifies it (race condition with destructor)
        ILedStrip* localStrip = strip;  // Capture atomically
        if (initialized && player && localStrip && !ledTaskShutdownRequested) {
            led_time_t now = localStrip->getMillis(); // Use local copy
            
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

            // NOTE: Removed "return to idle when root disappears" logic
            // Clients will stay on their current pattern until root sends a new one
            // If root truly disappears, clients will naturally go to idle when they
            // lose mesh connectivity and restart, or when a new root emerges

            // Update pattern if needed
            player->UpdatePattern(now, localStrip);

            // Update the LED strip
            player->UpdateStrip(now, localStrip);
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
            uint32_t end_time = strip->getMillis() + duration_ms;
            while (strip->getMillis() < end_time) {
                uint32_t elapsed = (strip->getMillis()) - (end_time - duration_ms);
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
            uint32_t end_time = strip->getMillis() + duration_ms;
            while (strip->getMillis() < end_time) {
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

