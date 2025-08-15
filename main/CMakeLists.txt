# Core source files (always compiled)
set(COMPONENT_SRCS 
    "main.cpp"
    "bluetooth/ble_gatt_server.cpp" 
    "bluetooth/ble_gap_handler.cpp"
    "bluetooth/gatt_profile.cpp" 
    "bluetooth/pixel_packet_profile.cpp"
    "bluetooth/network_health_profile.cpp"
    "mesh/espnow_mesh_coordinator.cpp"
    "mesh/adaptive/neighbor_manager.cpp"
    "mesh/adaptive/topology_manager.cpp"
    "mesh/adaptive/router.cpp"
    "packet/generic_packet.cpp"
    "packet/led_packet_processor.cpp"
    "system/nvs_manager.cpp"
    "system/network_health.cpp"
    "led/led_strip.cpp"
    "led/led_strip_encoder.cpp"
    "led/pattern.cpp"
    "led/gradient.cpp"
    "led/player.cpp"
    "led/sequence.cpp"
    "led/led_controller.cpp"
    "utils/global_objects.cpp"
    "utils/stack_monitor.cpp"
)

# Conditionally add button interface source files
if(CONFIG_BUTTON_INTERFACE_ENABLED)
    list(APPEND COMPONENT_SRCS 
        "system/button_manager.cpp"
        "system/button_logic.cpp"
        "system/button_feedback.cpp"
        "system/dual_button_detector.cpp"
        "system/root_takeover_manager.cpp"
    )
endif()

idf_component_register(SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS "." "bluetooth" "mesh" "mesh/adaptive" "packet" "led" "system" "utils")
