/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "wifi_setup.h"
#include "telelogs.h"
#include "locomotion.h"
#include "websocket_server.h"
#include "pins_config.h"


#define TAG ""


Locomotion locomotion;

static void blinker1(void* arg) {
    while(true) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 0);
    }
}

static void blinker2(void* arg) {
    // locomotion.moveBlocking(0, 2*M_PI * 10);
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    while(true) {
        // locomotion.moveBlocking(300, 0);
        // locomotion.moveBlocking(0, M_PI_2);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}




extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting tutorial...");
    ESP_ERROR_CHECK(wifi_init());

    char* ssid = read_string_from_nvs("sta_ssid");
    char* password = read_string_from_nvs("sta_password");

    if(ssid == NULL || password == NULL) {
        ESP_LOGE("NVS", "Failed to fetch WiFi credentials, using default credentials");
        ssid = CONFIG_WIFI_STA_SSID;
        password = CONFIG_WIFI_STA_PASSWORD;
    }

    esp_err_t ret = wifi_connect(ssid, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi network %s", ssid);
        wifi_create_ap(CONFIG_WIFI_AP_SSID, CONFIG_WIFI_AP_PASSWORD);
    }

    // wifi_ap_record_t ap_info;
    // ret = esp_wifi_sta_get_ap_info(&ap_info);
    // if (ret == ESP_ERR_WIFI_CONN) {
    //     ESP_LOGE(TAG, "Wi-Fi station interface not initialized");
    // }
    // else if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
    //     ESP_LOGE(TAG, "Wi-Fi station is not connected");
    // } else {
    //     ESP_LOGI(TAG, "--- Access Point Information ---");
    //     ESP_LOG_BUFFER_HEX("MAC Address", ap_info.bssid, sizeof(ap_info.bssid));
    //     ESP_LOG_BUFFER_CHAR("SSID", ap_info.ssid, sizeof(ap_info.ssid));
    //     ESP_LOGI(TAG, "Primary Channel: %d", ap_info.primary);
    //     ESP_LOGI(TAG, "RSSI: %d", ap_info.rssi);
    // }

    telelogs_init();

    locomotion.init();


    printf("Hello world!\n");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1 << LED1) | (1 << LED2),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);


    start_web_server();



    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());







    xTaskCreate( blinker1, "Blinker", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate( blinker2, "Blinker2", configMINIMAL_STACK_SIZE+2048, NULL, 1, NULL);

    float val = 0;
    while(1) {
        ws_async_send_robot_pos();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        printf("Free heap size: %" PRIu32 " bytes\n", esp_get_free_heap_size());

        
        // val+= 2.3;
        // telelogs_send_float("test", val);
        // vTaskDelay(100 / portTICK_PERIOD_MS);

        // if(val > (float)rand()*50.0/RAND_MAX) {
        //     val = 0;
        // }
    }

    //esp_restart();
}
