idf_component_register(
    SRCS "src/DataLogger.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "digitoys-core" "freertos" "esp_timer"
)
