set(srcs
    "src/nvs_manager.cpp"
    "src/global_objects.cpp"
    "src/stack_monitor.cpp"
    "src/esp_idf_system_hardware.cpp"
)

if(CONFIG_BUTTON_INTERFACE_ENABLED)
    list(APPEND srcs
        "src/button_feedback.cpp"
        "src/button_logic.cpp"
        "src/button_manager.cpp"
        "src/dual_button_detector.cpp"
        "src/root_takeover_manager.cpp"
    )
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES
        bluetooth
        common
        driver
        esp_system
        esp_timer
        freertos
        led
        mesh
        nvs_flash
)
