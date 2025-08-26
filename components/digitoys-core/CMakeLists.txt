idf_component_register(
    SRCS "src/ComponentBase.cpp"
         "src/ComponentError.cpp"
         "src/ConfigValidator.cpp"
         "src/ComponentConfigFactory.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "esp_common" "log"
    PRIV_REQUIRES "driver"
)
