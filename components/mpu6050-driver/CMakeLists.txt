idf_component_register(
    SRCS "MPU6050.cpp" "I2C.cpp" "mpu6050_example.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)
