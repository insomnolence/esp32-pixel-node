idf_component_register(
    SRCS
        "test_packet_parsing.cpp"
        "test_led_packet_processor.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES packet unity
)
