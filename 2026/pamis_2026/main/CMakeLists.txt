idf_build_get_property(target IDF_TARGET)

idf_component_register(SRCS "websocket_server.cpp" "wifi_setup.cpp" "telelogs.cpp" "locomotion.cpp" "main.cpp"
                       PRIV_REQUIRES spi_flash nvs_flash esp_event esp_netif lwip esp_wifi esp_http_server spiffs
                       REQUIRES driver stepper
                       INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)
