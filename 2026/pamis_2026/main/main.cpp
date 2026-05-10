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
#include "config.h"
#include "imu.h"
#include "radar_vl53.h"
#include "sts3032.h"
#include "SAP_controller.h"
#include "math.h"
#include "strat.h"
#include "evitement.h"
#include "Astar.h"
#include "stratWallid.h"

#define NINJA_ID 9

const LocomParam PARAM_NINJA = {97.8, (360.0 / 1.8) / (M_PI * 71.0)};
const LocomParam PARAM_ECUREUIL_1 = {91.0, (360.0 / 1.8) / (M_PI * 76.2)};
const LocomParam PARAM_ECUREUIL_2 = {91.0, (360.0 / 1.8) / (M_PI * 76.2)};
const LocomParam PARAM_ECUREUIL_3 = {88.5, (360.0 / 1.8) / (M_PI * 76.2)};
const LocomParam PARAM_ECUREUIL_4 = {91.0, (360.0 / 1.8) / (M_PI * 76.2)};
const LocomParam PARAM_ECUREUIL_5 = {91.0, (360.0 / 1.8) / (M_PI * 76.2)};
const LocomParam PARAM_DEFAUT = {91.0, (360.0 / 1.8) / (M_PI * 76.2)};

i2c_master_bus_handle_t bus_handle;
i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = SDA,
    .scl_io_num = SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = {.enable_internal_pullup = false},
};

static void blinker(void *arg)
{
    while (true)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 0);
    }
}

extern "C" void app_main(void)
{
    // setup LEDS and buttons
    gpio_config_t leds_config = {
        .pin_bit_mask = (1 << LED1) | (1 << LED2),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&leds_config);

    gpio_config_t fdc_config = {
        .pin_bit_mask = (1 << FDC1) | (1 << FDC2) | (1 << FDC3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&fdc_config);

    esp_err_t nvs_err = nvs_init();
    if (nvs_err != ESP_OK)
    {
        ESP_LOGE("NVS", "Failed to init NVS!");
    }

    // setup sensors
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    imu_init(&bus_handle);
    radar_vl53_start(&bus_handle);

    if (sap_init(500000) != ESP_OK)
    {
        ESP_LOGE("SAP", "Failed to init SAP");
    }
    else
    {
        ESP_LOGI("SAP", "SAP initialized !");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // create blinker task
    xTaskCreate(blinker, "Blinker", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    // on lit notre nom
    uint16_t pami_id;
    esp_err_t ret = read_u16_from_nvs("pami_id_u16", &pami_id);
    if (ret != ESP_OK)
    {
        pami_id = 100;
        printf("Failed to read pami id from NVS: %d\n", ret);
    }

    printf("Je suis le n° %d \n", pami_id);

    LocomParam mes_parametres;

    switch (pami_id)
    {
    case NINJA_ID:
        mes_parametres = PARAM_NINJA;
        break;
    case 1:
        mes_parametres = PARAM_ECUREUIL_1;
        break;
    case 2:
        mes_parametres = PARAM_ECUREUIL_2;
        break;
    case 3:
        mes_parametres = PARAM_ECUREUIL_3;
        break;
    case 4:
        mes_parametres = PARAM_ECUREUIL_4;
        break;
    case 5:
        mes_parametres = PARAM_ECUREUIL_5;
        break;
    default:
        mes_parametres = PARAM_DEFAUT;
        break;
    }

    // setup locomotion
    locomotion.init(mes_parametres);
    locomotion.enableSteppers(false);

    if (pami_id == NINJA_ID)
    {
        xTaskCreate(strat_grenier, "pami strat", 8192, NULL, 1, NULL);
    }
    else if (pami_id == 100)
    {
        xTaskCreate(strat_marchepas, "pami strat", 4096, NULL, 1, NULL);
    }
    else
    {
        init_evitement();
        init_map();
        xTaskCreate(strat_pami2026, "pami strat", 20000, (void *)pami_id, 1, NULL);
    }

    ESP_LOGI("Wifi", "Starting WiFi...");
    ESP_ERROR_CHECK(wifi_init());

    char *ssid = read_string_from_nvs("sta_ssid");
    char *password = read_string_from_nvs("sta_password");

    if (ssid == NULL || password == NULL)
    {
        ESP_LOGE("NVS", "Failed to fetch WiFi credentials, using default credentials");
        ssid = CONFIG_WIFI_STA_SSID;
        password = CONFIG_WIFI_STA_PASSWORD;
        write_string_to_nvs("sta_ssid", CONFIG_WIFI_STA_SSID);
        write_string_to_nvs("sta_password", CONFIG_WIFI_STA_PASSWORD);
    }

    char *teleplot_ip = read_string_from_nvs("teleplot_ip");
    uint16_t teleplot_port;
    ret = read_u16_from_nvs("teleplot_port", &teleplot_port);
    if (teleplot_ip == NULL || ret != ESP_OK)
    {
        write_string_to_nvs("teleplot_ip", "192.168.42.201");
        write_u16_to_nvs("teleplot_port", 47269);
    }

    ret = wifi_connect(ssid, password);
    if (ret == ESP_OK)
    {
        // Blink 3 times on WiFi connect
        for (int i = 0; i < 3; i++)
        {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(LED2, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(LED2, 0);
        }
    }
    else
    {
        ESP_LOGE("Wifi", "Failed to connect to Wi-Fi network %s", ssid);
        ret = wifi_create_ap(CONFIG_WIFI_AP_SSID, CONFIG_WIFI_AP_PASSWORD);
        if (ret == ESP_OK)
        {
            // Blink 10 times on WiFi AP creation after failure to connect to WiFi
            for (int i = 0; i < 10; i++)
            {
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED2, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED2, 0);
            }
        }
        else
        {
            // Fail to setup WiFi, no need to go further here.
            vTaskDelay(portMAX_DELAY);
        }
    }

    telelogs_init();

    start_web_server();

    // strat_grenier((void*) NULL);

    while (1)
    {
        ws_async_send_robot_pos();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // printf("Free heap size: %" PRIu32 " bytes\n", esp_get_free_heap_size());
    }
}
