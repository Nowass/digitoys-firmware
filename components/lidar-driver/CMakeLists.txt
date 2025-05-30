idf_component_register(
    SRCS "streaming-parser.cpp" "frame-parser.cpp" "uart-hal.cpp" "motor-hal.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)
