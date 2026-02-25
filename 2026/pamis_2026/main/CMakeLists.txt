idf_build_get_property(target IDF_TARGET)

idf_component_register(
    SRCS
        "radar_vl53.cpp"
        "utils.cpp"
        "imu.cpp"
        "locomotion.cpp"
        "main.cpp"
        "wifi/websocket_server.cpp"
        "wifi/wifi_setup.cpp"
        "wifi/telelogs.cpp"
    PRIV_REQUIRES
        spi_flash
        nvs_flash
        esp_event
        esp_netif
        lwip
        esp_wifi
        esp_http_server
        spiffs
        esp_driver_i2c
    REQUIRES
        driver
        stepper
        vl53l1x
    INCLUDE_DIRS "." "wifi"
)

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)
