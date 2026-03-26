idf_component_register(SRCS "main.c" "ota.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_wifi esp_http_server mdns esp_timer app_update)
