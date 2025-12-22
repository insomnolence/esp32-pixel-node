idf_component_register(SRCS "src/gradient.cpp" "src/led_controller.cpp" "src/led_strip.cpp" "src/led_strip_encoder.cpp" "src/pattern.cpp" "src/player.cpp" "src/sequence.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES "packet" "common" "esp_driver_rmt" "esp_driver_gpio" "esp_timer" "freertos" "log" "system_support")
