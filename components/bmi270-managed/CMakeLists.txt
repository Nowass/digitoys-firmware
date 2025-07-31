idf_component_register(
    SRCS "bmi270_managed.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver espressif2022__bmi270 espressif__i2c_bus
)
