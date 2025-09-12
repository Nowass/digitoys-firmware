idf_component_register(
    SRCS "src/DataModeling.cpp" "src/TestSession.cpp" "src/PhysicsAnalyzer.cpp" "src/BehaviorModel.cpp" "src/ModelingDataSource.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "digitoys-core" "freertos" "esp_timer" "lidar-driver" "adas-pwm-driver"
    PRIV_REQUIRES "data-logger"
)
