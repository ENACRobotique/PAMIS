#pragma once

#include "esp_err.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"

esp_err_t wifi_init(void);
esp_err_t wifi_connect(const char* wifi_ssid, const char* wifi_password);
esp_err_t wifi_disconnect(void);
esp_err_t wifi_dede(void);

esp_err_t wifi_create_ap(const char* ap_ssid, const char* ap_password);


char* read_string_from_nvs(const char* key);
esp_err_t write_string_to_nvs(const char* key, const char* value);

