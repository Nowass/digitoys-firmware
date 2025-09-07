idf_component_register(
    SRCS "src/DataLogger.cpp" "src/IDataSource.cpp" "src/ControlTaskDataSource.cpp" "src/DataLoggerService.cpp" "src/PhysicsAnalyzer.cpp" "src/PhysicsAnalyzerAPI.cpp" "src/MultiLoggerAPI.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "digitoys-core" "freertos" "esp_timer" "esp_http_server"
    PRIV_REQUIRES "control-task"
)
