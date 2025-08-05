idf_component_register(SRCS 
    "main.cpp"
    "bluetooth/ble_gatt_server.cpp" 
    "bluetooth/ble_gap_handler.cpp"
    "bluetooth/gatt_profile.cpp" 
    "bluetooth/pixel_packet_profile.cpp"
    "mesh/espnow_mesh_coordinator.cpp"
    "packet/generic_packet.cpp"
    "packet/led_packet_processor.cpp"
    "packet/packet.cpp"
    "system/nvs_manager.cpp"
    INCLUDE_DIRS "." "bluetooth" "mesh" "packet" "led" "system")
