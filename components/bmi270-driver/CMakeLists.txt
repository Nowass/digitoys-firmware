idf_component_register(
    SRCS "i2c-hal.cpp" "bmi270.cpp" "bmi270-config.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)
