idf_component_register(
    SRCS "src/DataLogger.cpp" "src/IDataSource.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "digitoys-core" "freertos" "esp_timer"
)
