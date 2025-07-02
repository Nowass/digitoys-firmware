idf_component_register(SRCS "Monitor.cpp" "SystemMonitor.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server
                       PRIV_REQUIRES esp_wifi esp_netif nvs_flash)
