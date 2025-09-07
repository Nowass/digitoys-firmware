idf_component_register(
    SRCS "src/DataLogger.cpp" "src/IDataSource.cpp" "src/ControlTaskDataSource.cpp" "src/DataLoggerService.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "digitoys-core" "freertos" "esp_timer"
    PRIV_REQUIRES "control-task"
)
