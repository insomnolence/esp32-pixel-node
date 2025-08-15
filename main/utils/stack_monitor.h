#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <cstdint>

/**
 * Comprehensive stack monitoring utility for ESP32-C3
 * Tracks stack usage across multiple tasks with configurable thresholds
 */
class StackMonitor {
private:
    static const char* TAG;
    static constexpr uint32_t WARNING_THRESHOLD_BYTES = 512;  // Warn when < 512 bytes free
    static constexpr uint32_t CRITICAL_THRESHOLD_BYTES = 256; // Critical when < 256 bytes free
    
public:
    struct TaskStackInfo {
        const char* task_name;
        TaskHandle_t handle;
        uint32_t stack_size;
        uint32_t free_bytes;
        uint32_t used_bytes;
        float usage_percent;
        bool is_warning;
        bool is_critical;
        
        TaskStackInfo() : task_name(nullptr), handle(nullptr), stack_size(0), 
                         free_bytes(0), used_bytes(0), usage_percent(0.0f),
                         is_warning(false), is_critical(false) {}
    };
    
    // Monitor current task (usually main task when called from app_main)
    static TaskStackInfo monitorCurrentTask();
    
    // Monitor specific task by handle
    static TaskStackInfo monitorTask(TaskHandle_t task_handle, const char* task_name);
    
    // Monitor multiple critical tasks
    static void monitorAllCriticalTasks();
    
    // Log stack status with appropriate warning levels
    static void logTaskStack(const TaskStackInfo& info);
    
    // Check if any task is in critical state
    static bool hasAnyCriticalTasks();
    
    // Get stack info for main task specifically
    static TaskStackInfo getMainTaskInfo();
    
    // Periodic monitoring (call from main loop)
    static void periodicCheck(uint32_t interval_ms = 30000);
    
private:
    static uint32_t last_check_time_;
    static TaskStackInfo fillTaskInfo(TaskHandle_t handle, const char* name, 
                                     uint32_t configured_size = 0);
};

// Inline helper for easy main task monitoring
inline StackMonitor::TaskStackInfo StackMonitor::monitorCurrentTask() {
    return monitorTask(xTaskGetCurrentTaskHandle(), "Current");
}

// Convenience macro for logging current task stack
#define LOG_CURRENT_STACK() \
    do { \
        auto stack_info = StackMonitor::monitorCurrentTask(); \
        StackMonitor::logTaskStack(stack_info); \
    } while(0)
