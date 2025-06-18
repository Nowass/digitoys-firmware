idf_component_register(
    SRCS " adas_pwm_driver.cpp" "adas_logic.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)
