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


Locomotion locomotion;
i2c_master_bus_handle_t bus_handle;
i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = SDA,
    .scl_io_num = SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags={.enable_internal_pullup = false},
};


static void blinker(void* arg) {
    while(true) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 0);
    }
}


static void pami_strat(void* arg) {
    
    Position test[2] = {
        {.x=0, .y=500, .theta=0},
        {.x=500, .y=500, .theta=M_PI},
    };

    
    // wait to plug tirette
    while (gpio_get_level(FDC1)) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    locomotion.enableSteppers(true);

    // wait to unplug tirette
    while (!gpio_get_level(FDC1)) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    Position a = locomotion.getPos();
    printf("%f, %f, %f\n", a.x, a.y, a.theta);
    locomotion.trajectory(test, 2);
    vTaskDelay(500 / portTICK_PERIOD_MS);


    while(true) {
        auto pos = locomotion.getPos();
        char pos_str[30];
        snprintf(pos_str, sizeof(pos_str), "x=%d,y=%d,theta=%.2f",(int)pos.x,(int)pos.y,pos.theta);
        telelogs_send_string("Odom", pos_str);

        vTaskDelay(100 / portTICK_PERIOD_MS);
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


    // setup sensors
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    imu_init(&bus_handle);
    radar_vl53_start(&bus_handle);

    // setup locomotion
    locomotion.init();
    locomotion.enableSteppers(false);
    


    // create blinker task
    xTaskCreate( blinker, "Blinker", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // create strat task
    xTaskCreate( pami_strat, "pami strat", configMINIMAL_STACK_SIZE+4096, NULL, 1, NULL);



    ESP_LOGI("Wifi", "Starting WiFi...");
    ESP_ERROR_CHECK(wifi_init());

    char* ssid = read_string_from_nvs("sta_ssid");
    char* password = read_string_from_nvs("sta_password");

    if(ssid == NULL || password == NULL) {
        ESP_LOGE("NVS", "Failed to fetch WiFi credentials, using default credentials");
        ssid = CONFIG_WIFI_STA_SSID;
        password = CONFIG_WIFI_STA_PASSWORD;
    }

    esp_err_t ret = wifi_connect(ssid, password);
    if(ret == ESP_OK) {
        // Blink 3 times on WiFi connect
        for(int i=0; i<3; i++) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(LED2, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(LED2, 0);
        }
    }
    else {
        ESP_LOGE("Wifi", "Failed to connect to Wi-Fi network %s", ssid);
        ret = wifi_create_ap(CONFIG_WIFI_AP_SSID, CONFIG_WIFI_AP_PASSWORD);
        if(ret == ESP_OK) {
            // Blink 10 times on WiFi AP creation after failure to connect to WiFi
            for(int i=0; i<10; i++) {
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED2, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED2, 0);
            }
        }
        else {
            // Fail to setup WiFi, no need to go further here.
            vTaskDelay(portMAX_DELAY);
        }
    }

    telelogs_init();

    start_web_server();


    while(1) {
        ws_async_send_robot_pos();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        //printf("Free heap size: %" PRIu32 " bytes\n", esp_get_free_heap_size());

    }

}
