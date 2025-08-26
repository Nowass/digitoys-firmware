idf_component_register(
    SRCS "src/ComponentBase.cpp"
         "src/ComponentError.cpp"
         "src/ConfigValidator.cpp"
         "src/ComponentConfigFactory.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "esp_common" "log" "esp_driver_gpio" "esp_driver_uart" "esp_driver_ledc" "freertos"
    PRIV_REQUIRES "driver"
)
