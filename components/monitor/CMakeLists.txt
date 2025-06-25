idf_component_register(SRCS "Monitor.cpp"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_netif nvs_flash esp_http_server)
