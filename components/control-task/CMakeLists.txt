idf_component_register(
    SRCS "src/ControlTask.cpp"
         "src/ControlState.cpp" 
         "src/ObstacleDetector.cpp"
         "src/RCInputProcessor.cpp"
         "src/SpeedController.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "freertos" "esp_timer" "log" "adas-pwm-driver" "lidar-driver" "digitoys-core"
    PRIV_REQUIRES "driver" "wifi-monitor"
)
