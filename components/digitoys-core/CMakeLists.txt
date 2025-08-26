idf_component_register(
    SRCS "src/ComponentBase.cpp"
         "src/ComponentError.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "esp_common" "log"
    PRIV_REQUIRES "driver"
)
