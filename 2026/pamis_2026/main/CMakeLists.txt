idf_build_get_property(target IDF_TARGET)

idf_component_register(SRCS "wifi_setup.cpp" "telelogs.cpp" "locomotion.cpp" "main.cpp"
                       PRIV_REQUIRES spi_flash nvs_flash esp_event esp_netif lwip esp_wifi
                       REQUIRES driver stepper
                       INCLUDE_DIRS ".")
