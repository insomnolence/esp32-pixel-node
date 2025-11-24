idf_component_register(
    SRCS
        "src/ble_gap_handler.cpp"
        "src/ble_gatt_server.cpp"
        "src/gatt_profile.cpp"
        "src/network_health_profile.cpp"
        "src/pixel_packet_profile.cpp"
    INCLUDE_DIRS "include"
    REQUIRES packet system_support bt freertos log
)
