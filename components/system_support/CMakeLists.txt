idf_component_register(
    SRCS "src/network_health.cpp"
         "src/battery_monitor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer log esp_adc
)
