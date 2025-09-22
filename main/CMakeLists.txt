idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES spi_flash lidar-driver driver esp_timer adas-pwm-driver control-task digitoys-core wifi-monitor
                       INCLUDE_DIRS "")
