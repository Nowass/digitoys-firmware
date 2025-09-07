idf_component_register(
    SRCS "adas_pwm_driver.cpp" "src/PwmChannelConfigImpl.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "digitoys-core" "esp_driver_rmt"
    PRIV_REQUIRES driver
)
