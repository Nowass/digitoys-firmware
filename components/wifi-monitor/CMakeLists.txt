idf_component_register(SRCS "WifiMonitor.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server json digitoys-core esp_netif esp_wifi monitor
                       PRIV_REQUIRES nvs_flash)
