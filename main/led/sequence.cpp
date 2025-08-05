#include "sequence.h"
#include "esp_random.h"
#include "led_strip.h" // For color constants
#include "esp_log.h"

static const char* TAG = "Sequence";

// IdleSequence implementation - matches Arduino: Red-White-Green gradient with slow speed
static const Step idle_steps[] = {
    {0, 20, 35, PATTERN_GRADIENT, {RED, WHITE, GREEN}, 17},      // Arduino idle: RWG gradient, brightness 20, speed 35, level 17
};

IdleSequence::IdleSequence() 
    : StepSequence(idle_steps, sizeof(idle_steps) / sizeof(Step)) {
    ESP_LOGI(TAG, "IdleSequence created with %d steps (Arduino-compatible)", stepCount);
}

// AlertSequence implementation  
static const Step alert_steps[] = {
    // High intensity alert patterns
    {500,  255, 100, PATTERN_FLASH,   {RED, WHITE, BLACK}, 255},         // 0.5s red flash
    {300,  255, 150, PATTERN_STROBE,  {RED, BLACK, BLACK}, 255},         // 0.3s red strobe
    {500,  255, 100, PATTERN_FLASH,   {RED, WHITE, BLACK}, 255},         // 0.5s red flash
    {1000, 200, 80,  PATTERN_FIXED,   {RED, BLACK, BLACK}, 200},         // 1s dim red hold
    {500,  255, 120, PATTERN_WIPE,    {WHITE, RED, BLACK}, 255},         // 0.5s white wipe
    {300,  255, 150, PATTERN_STROBE,  {WHITE, BLACK, BLACK}, 255},       // 0.3s white strobe
    {2000, 100, 50,  PATTERN_MINI_TWINKLE, {RED, YELLOW, BLACK}, 100},   // 2s warning twinkle
};

AlertSequence::AlertSequence() 
    : StepSequence(alert_steps, sizeof(alert_steps) / sizeof(Step)) {
    ESP_LOGI(TAG, "AlertSequence created with %d steps", stepCount);
}

// RandomSequence implementation - matches Arduino patterns exactly
static const Step random_steps[] = {
    // From Arduino randomSteps[] - exact timing, colors, and parameters
    {30000, 127, 160, PATTERN_MINI_TWINKLE, {RED, WHITE, YELLOW}, 160},    // rwy twinkle
    {30000, 127, 160, PATTERN_MINI_TWINKLE, {RED, WHITE, GREEN}, 160},     // rwg twinkle  
    {30000, 127, 35,  PATTERN_GRADIENT,    {RED, WHITE, RED}, 17},         // rwr subtle ⭐
    {30000, 127, 75,  PATTERN_GRADIENT,    {BLUE, 0x8080FF, BLUE}, 75},    // blue smooth ⭐
    {30000, 127, 160, PATTERN_MINI_TWINKLE, {RED, WHITE, BLUE}, 160},      // rwb twinkle
    {30000, 63,  65,  PATTERN_CANDY_CANE,  {RED, WHITE, GREEN}, 255},      // rwg candy
    {30000, 63,  100, PATTERN_CANDY_CANE,  {RED, WHITE, RED}, 255},        // rwr candy
    {30000, 127, 100, PATTERN_FIXED,       {RED, WHITE, GREEN}, 255},      // rwg tree
    {30000, 127, 127, PATTERN_MARCH,       {RED, WHITE, GREEN}, 8},        // rwg march
    {30000, 127, 127, PATTERN_WIPE,        {RED, WHITE, GREEN}, 8},        // rwg wipe
    {30000, 127, 255, PATTERN_MINI_SPARKLE, {RED, WHITE, GREEN}, 9},       // rwg flicker
    // Idle step as last step (like Arduino)
    {0,     20,  35,  PATTERN_GRADIENT,    {RED, WHITE, GREEN}, 17},       // idle step
};

RandomSequence::RandomSequence() 
    : StepSequence(random_steps, sizeof(random_steps) / sizeof(Step)) {
    ESP_LOGI(TAG, "RandomSequence created with %d steps", stepCount);
}

int RandomSequence::Reset() {
    ESP_LOGI(TAG, "RandomSequence reset - selecting random starting step");
    return esp_random() % stepCount;
}

int RandomSequence::Advance(int step, bool timed) {
    if (timed) {
        // For timed advances, pick completely random next step
        int nextStep = esp_random() % stepCount;
        ESP_LOGI(TAG, "RandomSequence timed advance: %d -> %d", step, nextStep);
        return nextStep;
    } else {
        // For manual advances (button press), go to next step in sequence
        int nextStep = (step + 1) % stepCount;
        ESP_LOGI(TAG, "RandomSequence manual advance: %d -> %d", step, nextStep);
        return nextStep;
    }
}