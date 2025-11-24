idf_component_register(SRCS "src/espnow_mesh_coordinator.cpp" 
                        "src/adaptive/neighbor_manager.cpp"
                        "src/adaptive/router.cpp"
                        "src/adaptive/topology_manager.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES "packet" "common" "bluetooth" "esp_wifi" "nvs_flash" "esp_event" "log" "freertos")
