#pragma once

#include <cstdlib>
#include <utility>
#include "esp_heap_caps.h"
#include "esp_log.h"

/**
 * Safe heap allocation patterns for ESP32-C3 embedded development
 * Provides RAII without exceptions, graceful failure handling
 */

namespace HeapUtils {

// Custom allocator that never throws, returns nullptr on failure
template<typename T>
class NoThrowAllocator {
public:
    static T* allocate(size_t count = 1) noexcept {
        void* ptr = heap_caps_malloc(sizeof(T) * count, MALLOC_CAP_8BIT);
        return static_cast<T*>(ptr);
    }
    
    static void deallocate(T* ptr) noexcept {
        heap_caps_free(ptr);
    }
    
    template<typename... Args>
    static T* construct(Args&&... args) noexcept {
        T* ptr = allocate();
        if (ptr) {
            new(ptr) T(std::forward<Args>(args)...);
        }
        return ptr;
    }
};

// Safe unique_ptr equivalent for ESP32 without exception throwing
template<typename T>
class esp_unique_ptr {
private:
    T* ptr_;
    
public:
    esp_unique_ptr() noexcept : ptr_(nullptr) {}
    
    explicit esp_unique_ptr(T* p) noexcept : ptr_(p) {}
    
    // Move constructor
    esp_unique_ptr(esp_unique_ptr&& other) noexcept : ptr_(other.release()) {}
    
    // Move assignment
    esp_unique_ptr& operator=(esp_unique_ptr&& other) noexcept {
        reset(other.release());
        return *this;
    }
    
    // No copy construction/assignment (RAII principle)
    esp_unique_ptr(const esp_unique_ptr&) = delete;
    esp_unique_ptr& operator=(const esp_unique_ptr&) = delete;
    
    ~esp_unique_ptr() { reset(); }
    
    T* get() const noexcept { return ptr_; }
    T& operator*() const noexcept { return *ptr_; }
    T* operator->() const noexcept { return ptr_; }
    
    explicit operator bool() const noexcept { return ptr_ != nullptr; }
    
    T* release() noexcept {
        T* result = ptr_;
        ptr_ = nullptr;
        return result;
    }
    
    void reset(T* p = nullptr) noexcept {
        if (ptr_) {
            ptr_->~T();
            NoThrowAllocator<T>::deallocate(ptr_);
        }
        ptr_ = p;
    }
};

// Factory pattern with error handling for graceful allocation failures
template<typename T>
class ObjectFactory {
public:
    enum class CreateResult {
        SUCCESS,
        OUT_OF_MEMORY,
        INITIALIZATION_FAILED
    };
    
    struct FactoryReturn {
        esp_unique_ptr<T> object;
        CreateResult result;
        
        bool success() const { return result == CreateResult::SUCCESS; }
        explicit operator bool() const { return success(); }
    };
    
    template<typename... Args>
    static FactoryReturn create(Args&&... args) noexcept {
        T* raw_ptr = NoThrowAllocator<T>::construct(std::forward<Args>(args)...);
        
        if (!raw_ptr) {
            return {esp_unique_ptr<T>(), CreateResult::OUT_OF_MEMORY};
        }
        
        return {esp_unique_ptr<T>(raw_ptr), CreateResult::SUCCESS};
    }
    
    template<typename... Args>
    static FactoryReturn createWithValidation(Args&&... args) noexcept {
        T* raw_ptr = NoThrowAllocator<T>::construct(std::forward<Args>(args)...);
        
        if (!raw_ptr) {
            return {esp_unique_ptr<T>(), CreateResult::OUT_OF_MEMORY};
        }
        
        // Post-construction initialization check (if T has isInitialized method)
        if constexpr (requires { raw_ptr->isInitialized(); }) {
            if (!raw_ptr->isInitialized()) {
                raw_ptr->~T();
                NoThrowAllocator<T>::deallocate(raw_ptr);
                return {esp_unique_ptr<T>(), CreateResult::INITIALIZATION_FAILED};
            }
        }
        
        return {esp_unique_ptr<T>(raw_ptr), CreateResult::SUCCESS};
    }
};

// System initialization status tracking
struct InitStatus {
    enum class Result {
        SUCCESS,
        PARTIAL_SUCCESS,
        CRITICAL_FAILURE
    };
    
    Result result = Result::SUCCESS;
    uint32_t free_heap_before = 0;
    uint32_t free_heap_after = 0;
    uint8_t components_initialized = 0;
    uint8_t components_failed = 0;
    
    bool canContinue() const { 
        return result != Result::CRITICAL_FAILURE; 
    }
    
    void logResults(const char* tag) const {
        ESP_LOGI(tag, "📊 Initialization Results:");
        ESP_LOGI(tag, "  • Result: %s", 
                 result == Result::SUCCESS ? "SUCCESS" :
                 result == Result::PARTIAL_SUCCESS ? "PARTIAL_SUCCESS" : "CRITICAL_FAILURE");
        ESP_LOGI(tag, "  • Heap usage: %lu → %lu bytes (used: %lu bytes)",
                 free_heap_before, free_heap_after, free_heap_before - free_heap_after);
        ESP_LOGI(tag, "  • Components: %d initialized, %d failed", 
                 components_initialized, components_failed);
    }
};

// Helper macro for allocation logging
#define LOG_HEAP_ALLOCATION(tag, name, ptr) \
    do { \
        if (ptr) { \
            ESP_LOGI(tag, "✅ %s allocated on heap (size: %zu bytes)", name, sizeof(*ptr)); \
        } else { \
            ESP_LOGE(tag, "❌ %s allocation failed - insufficient heap memory", name); \
        } \
    } while(0)

} // namespace HeapUtils