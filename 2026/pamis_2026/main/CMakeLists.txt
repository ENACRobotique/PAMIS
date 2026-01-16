idf_build_get_property(target IDF_TARGET)

idf_component_register(SRCS "telelogs.c" "wifi_setup.c" "main.c"
                       PRIV_REQUIRES spi_flash nvs_flash esp_event esp_netif lwip esp_wifi
                       REQUIRES driver
                       INCLUDE_DIRS ".")
