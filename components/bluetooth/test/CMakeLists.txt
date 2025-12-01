idf_component_register(
    SRCS "test_ble_gatt_server.cpp" "test_ble_gap_handler.cpp"
    INCLUDE_DIRS "." "../include"
    REQUIRES bluetooth unity
)
