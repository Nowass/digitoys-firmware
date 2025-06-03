idf_component_register(
    SRCS "frame-parser.cpp" "near-distance-filter.cpp" "uart-hal.cpp" "motor-hal.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)
