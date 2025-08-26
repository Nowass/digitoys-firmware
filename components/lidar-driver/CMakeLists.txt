idf_component_register(
    SRCS "frame-parser.cpp" "near-distance-filter.cpp" "uart-hal.cpp" "motor-hal.cpp" "lidar.cpp" "src/LiDARConfigImpl.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "digitoys-core"
    PRIV_REQUIRES driver
)
