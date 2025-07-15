idf_component_register(
    SRCS "BMI270.cpp" "I2C.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)
