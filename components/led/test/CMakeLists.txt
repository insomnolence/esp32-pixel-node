idf_component_register(
    SRCS
        "test_led_patterns.cpp"
        "test_led_controller.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES led unity
)
