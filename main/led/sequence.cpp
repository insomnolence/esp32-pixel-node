#include "sequence.h"
#include "esp_random.h"
#include "led_strip.h" // For color constants
#include "esp_log.h"
#include <new> // For std::nothrow

static const char* TAG = "Sequence";

// IdleSequence implementation - matches Arduino: Red-White-Green gradient with slow speed
static const Step idle_steps[] = {
    {0, 127, 35, PATTERN_GRADIENT, {RED, WHITE, GREEN}, 17},      // Test: Higher brightness to check color saturation
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

// ParameterizedSequence implementation - configurable Warning/Exit patterns
ParameterizedSequence::ParameterizedSequence(uint32_t primaryColor, uint32_t secondaryColor, uint32_t tertiaryColor)
    : StepSequence(nullptr, 4), dynamicSteps(nullptr) {
    
    createSteps(primaryColor, secondaryColor, tertiaryColor);
    
    // Set the base class members to point to our dynamic steps
    steps = dynamicSteps;
    stepCount = 4;
    
    ESP_LOGI(TAG, "ParameterizedSequence created with colors: 0x%06lX, 0x%06lX, 0x%06lX", 
             primaryColor, secondaryColor, tertiaryColor);
}

ParameterizedSequence::~ParameterizedSequence() {
    delete[] dynamicSteps;
    dynamicSteps = nullptr;
}

void ParameterizedSequence::createSteps(uint32_t primaryColor, uint32_t secondaryColor, uint32_t tertiaryColor) {
    // Allocate memory for 4 steps (Warning/Exit pattern structure)
    dynamicSteps = new(std::nothrow) Step[4];
    if (!dynamicSteps) {
        ESP_LOGE(TAG, "Failed to allocate memory for ParameterizedSequence steps");
        return;
    }
    
    // Step 1: Flash - 4 seconds duration, primary color, Flash pattern (ESP32C3 power-safe brightness)
    dynamicSteps[0] = {4000, 200, 100, PATTERN_FLASH, {primaryColor, primaryColor, primaryColor}, 200};
    
    // Step 2: March - 60 seconds duration, primary color, March pattern
    dynamicSteps[1] = {60000, 200, 40, PATTERN_MARCH, {primaryColor, primaryColor, primaryColor}, 34};
    
    // Step 3: MiniTwinkle - 60 seconds duration, primary + secondary colors, MiniTwinkle pattern
    dynamicSteps[2] = {60000, 200, 100, PATTERN_MINI_TWINKLE, {primaryColor, secondaryColor, primaryColor}, 75};
    
    // Step 4: Gradient - permanent (0ms), primary + secondary colors, Gradient pattern
    dynamicSteps[3] = {0, 127, 75, PATTERN_GRADIENT, {primaryColor, secondaryColor, primaryColor}, 75};
    
    ESP_LOGI(TAG, "ParameterizedSequence: 4 steps created (Flash->March->MiniTwinkle->Gradient)");
}

// SingleRandomSequence implementation - picks one random pattern, then returns to idle
SingleRandomSequence::SingleRandomSequence() 
    : StepSequence(nullptr, 1), dynamicSteps(nullptr) {
    createRandomStep();
    steps = dynamicSteps;
    ESP_LOGI(TAG, "SingleRandomSequence created with 1 random pattern");
}

SingleRandomSequence::~SingleRandomSequence() {
    delete[] dynamicSteps;
    dynamicSteps = nullptr;
}

void SingleRandomSequence::createRandomStep() {
    // Clean up existing step if any
    delete[] dynamicSteps;
    
    // Allocate memory for 1 step
    dynamicSteps = new(std::nothrow) Step[1];
    if (!dynamicSteps) {
        ESP_LOGE(TAG, "Failed to allocate memory for SingleRandomSequence step");
        return;
    }
    
    // Available random patterns (excluding idle step)
    static const Step available_patterns[] = {
        {30000, 127, 160, PATTERN_MINI_TWINKLE, {RED, WHITE, YELLOW}, 160},    // rwy twinkle
        {30000, 127, 160, PATTERN_MINI_TWINKLE, {RED, WHITE, GREEN}, 160},     // rwg twinkle  
        {30000, 127, 35,  PATTERN_GRADIENT,    {RED, WHITE, RED}, 17},         // rwr subtle
        {30000, 127, 75,  PATTERN_GRADIENT,    {BLUE, 0x8080FF, BLUE}, 75},    // blue smooth
        {30000, 127, 160, PATTERN_MINI_TWINKLE, {RED, WHITE, BLUE}, 160},      // rwb twinkle
        {30000, 63,  65,  PATTERN_CANDY_CANE,  {RED, WHITE, GREEN}, 255},      // rwg candy
        {30000, 63,  100, PATTERN_CANDY_CANE,  {RED, WHITE, RED}, 255},        // rwr candy
        {30000, 127, 100, PATTERN_FIXED,       {RED, WHITE, GREEN}, 255},      // rwg tree
        {30000, 127, 127, PATTERN_MARCH,       {RED, WHITE, GREEN}, 8},        // rwg march
        {30000, 127, 127, PATTERN_WIPE,        {RED, WHITE, GREEN}, 8},        // rwg wipe
        {30000, 127, 255, PATTERN_MINI_SPARKLE, {RED, WHITE, GREEN}, 9},       // rwg flicker
    };
    
    // Pick random pattern (11 patterns available) and modify duration to be permanent
    int randomIndex = esp_random() % 11;
    dynamicSteps[0] = available_patterns[randomIndex];
    
    // Override duration to 0 (permanent) - LEDController will handle 30-second timeout
    dynamicSteps[0].duration = 0;
    
    ESP_LOGI(TAG, "Selected random pattern #%d (permanent - controller will timeout after 30s)", randomIndex);
}

void SingleRandomSequence::pickNewRandomPattern() {
    createRandomStep();
    ESP_LOGI(TAG, "New random pattern selected");
}

int SingleRandomSequence::Advance(int step, bool timed) {
    // When the single pattern completes (30 seconds), don't advance - stay at step 0
    // The LED controller will handle returning to idle mode
    ESP_LOGI(TAG, "SingleRandomSequence pattern completed - ready to return to idle");
    return 0; // Stay at step 0, don't advance
}