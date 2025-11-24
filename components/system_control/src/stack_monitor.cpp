#include "system_control/stack_monitor.h"
#include "esp_timer.h"
#include "sdkconfig.h"

const char* StackMonitor::TAG = "StackMonitor";
uint32_t StackMonitor::last_check_time_ = 0;

StackMonitor::TaskStackInfo StackMonitor::monitorTask(TaskHandle_t task_handle, const char* task_name) {
    if (!task_handle) {
        ESP_LOGW(TAG, "Cannot monitor task: invalid handle for %s", task_name ? task_name : "unknown");
        return TaskStackInfo();
    }
    
    // For main task, we know the configured size
    uint32_t configured_size = 0;
    if (task_handle == xTaskGetCurrentTaskHandle()) {
        configured_size = CONFIG_ESP_MAIN_TASK_STACK_SIZE;
    }
    
    return fillTaskInfo(task_handle, task_name, configured_size);
}

StackMonitor::TaskStackInfo StackMonitor::fillTaskInfo(TaskHandle_t handle, const char* name, 
                                                      uint32_t configured_size) {
    TaskStackInfo info;
    info.task_name = name;
    info.handle = handle;
    info.stack_size = configured_size;
    
    // Get high water mark (minimum free stack ever seen)
    UBaseType_t high_water_mark = uxTaskGetStackHighWaterMark(handle);
    info.free_bytes = high_water_mark * sizeof(StackType_t);
    
    if (configured_size > 0) {
        info.used_bytes = configured_size - info.free_bytes;
        info.usage_percent = (float(info.used_bytes) / float(configured_size)) * 100.0f;
    }
    
    // Determine warning states
    info.is_warning = info.free_bytes <= WARNING_THRESHOLD_BYTES;
    info.is_critical = info.free_bytes <= CRITICAL_THRESHOLD_BYTES;
    
    return info;
}

void StackMonitor::logTaskStack(const TaskStackInfo& info) {
    if (!info.handle) {
        return;
    }
    
    const char* status_icon = "📊";
    esp_log_level_t log_level = ESP_LOG_INFO;
    
    if (info.is_critical) {
        status_icon = "🚨";
        log_level = ESP_LOG_ERROR;
    } else if (info.is_warning) {
        status_icon = "⚠️";
        log_level = ESP_LOG_WARN;
    }
    
    if (info.stack_size > 0) {
        ESP_LOG_LEVEL(log_level, TAG, "%s STACK [%s]: %lu/%lu bytes used (%.1f%%) - %lu bytes free", 
                      status_icon, info.task_name, info.used_bytes, info.stack_size, 
                      info.usage_percent, info.free_bytes);
    } else {
        ESP_LOG_LEVEL(log_level, TAG, "%s STACK [%s]: %lu bytes free (size unknown)", 
                      status_icon, info.task_name, info.free_bytes);
    }
    
    // Additional warnings for critical states
    if (info.is_critical) {
        ESP_LOGE(TAG, "🚨 STACK CRITICAL: %s task has only %lu bytes free!", 
                 info.task_name, info.free_bytes);
        ESP_LOGE(TAG, "🚨 Risk of stack overflow - immediate action required!");
    } else if (info.is_warning) {
        ESP_LOGW(TAG, "⚠️ STACK WARNING: %s task approaching limits (%lu bytes free)", 
                 info.task_name, info.free_bytes);
    }
}

StackMonitor::TaskStackInfo StackMonitor::getMainTaskInfo() {
    return fillTaskInfo(xTaskGetCurrentTaskHandle(), "Main", CONFIG_ESP_MAIN_TASK_STACK_SIZE);
}

void StackMonitor::monitorAllCriticalTasks() {
    ESP_LOGI(TAG, "🔍 Monitoring critical task stacks...");
    
    // Monitor main task
    TaskStackInfo main_info = getMainTaskInfo();
    logTaskStack(main_info);
    
    // TODO: Add monitoring for other critical tasks when we have their handles
    // For now, we'll focus on the main task which was causing the issues
    
    // Summary
    if (main_info.is_critical) {
        ESP_LOGE(TAG, "🚨 CRITICAL: One or more tasks have dangerously low stack space!");
    } else if (main_info.is_warning) {
        ESP_LOGW(TAG, "⚠️ WARNING: One or more tasks approaching stack limits");
    } else {
        ESP_LOGI(TAG, "✅ All monitored tasks have healthy stack usage");
    }
}

bool StackMonitor::hasAnyCriticalTasks() {
    TaskStackInfo main_info = getMainTaskInfo();
    return main_info.is_critical;
    // TODO: Check other tasks when we have their handles
}

void StackMonitor::periodicCheck(uint32_t interval_ms) {
    uint32_t now = esp_timer_get_time() / 1000; // Convert to ms
    
    if (last_check_time_ == 0 || (now - last_check_time_) >= interval_ms) {
        monitorAllCriticalTasks();
        last_check_time_ = now;
    }
}
