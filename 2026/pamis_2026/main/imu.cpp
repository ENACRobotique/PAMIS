#include "imu.h"
#include "soc/clk_tree_defs.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lms6dsl_reg.h"
#include "esp_log.h"

#define I2C_MASTER_NUM              I2C_NUM_0   

constexpr uint32_t CHECK_IMU_MAX_FAIL = 50;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t imu_dev_handle;

i2c_device_config_t imu_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0b1101010,
    .scl_speed_hz = 400000,
};

i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_MASTER_NUM,
    .sda_io_num = SDA,
    .scl_io_num = SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags={.enable_internal_pullup = false},
};



static void read_imu(void* arg) {
    //reboot IMU
    //writeRegister(LSM6DSL_CTRL3_C, 0x80);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // Read WHO_AM_I register until we get the expected value
    while(true) {
        uint8_t who_am_i = 0;
        register_read(LSM6DSL_WHO_AM_I_REG, &who_am_i, 1);
        if(who_am_i == LSM6DSL_WHO_AM_I) {
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // configure IMU
    


    while(true) {
        uint8_t who_am_i = 0;
        vTaskDelay(200 / portTICK_PERIOD_MS);
        register_read(LSM6DSL_WHO_AM_I_REG, &who_am_i, 1);
        ESP_LOGI("IMU", "Who am I ? 0x%x", who_am_i);
    }
}



void imu_init(){
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &imu_dev_config, &imu_dev_handle));

     xTaskCreate( read_imu, "Read_imu", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL);
}


esp_err_t register_read(uint8_t reg, uint8_t* buff, size_t length){
    return i2c_master_transmit_receive(imu_dev_handle, &reg, 1, buff, length, 100);
}


static bool check_imu() {

}