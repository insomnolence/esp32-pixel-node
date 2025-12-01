idf_component_register(SRCS "src/espnow_mesh_coordinator.cpp"
                        "src/neighbor_tracker.cpp"
                        "src/esp_idf_mesh_hardware.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES "packet" "common" "bluetooth" "esp_wifi" "nvs_flash" "esp_event" "log" "freertos")
