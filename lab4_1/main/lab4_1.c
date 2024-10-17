#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           10
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_FREQ_HZ          400000

#define ICM42670_ADDR               0x68
#define GYRO_CONFIG_REG             0x14

// ICM-42670-P Gyroscope registers
#define GYRO_XOUT_H 0x0B

static const char *TAG = "ICM42670-P";

static esp_err_t i2c_master_init() {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t write_register(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read a register from the ICM-42670-P
static esp_err_t read_register(uint8_t reg, uint8_t *data, size_t len) {
    if(len == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_READ, true);
    if(len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read gyroscope data
void gyroscope_task(void *arg) {
    uint8_t data[6];
    while(1) {
        if(read_register(GYRO_XOUT_H, data, 6) == ESP_OK) {
            int16_t gyro_x = (int16_t)((data[0] << 8) | data[1]);
            int16_t gyro_y = (int16_t)((data[2] << 8) | data[3]);
            int16_t gyro_z = (int16_t)((data[4] << 8) | data[5]);
            if(gyro_y > 300) {
                if(gyro_x > 300) {
                    ESP_LOGI(TAG, "UP LEFT");
                }
                else if(gyro_x < -300) {
                    ESP_LOGI(TAG, "UP RIGHT");
                }
                else {
                    ESP_LOGI(TAG, "UP");
                }
            }
            else if(gyro_y < -300) {
                if(gyro_x > 300) {
                    ESP_LOGI(TAG, "DOWN LEFT");
                }
                else if(gyro_x < -300) {
                    ESP_LOGI(TAG, "DOWN RIGHT");
                }
                else {
                    ESP_LOGI(TAG, "DOWN");
                }
            }
            else {
                if(gyro_x > 300) {
                    ESP_LOGI(TAG, "LEFT");
                }
                else if(gyro_x < -300) {
                    ESP_LOGI(TAG, "RIGHT");
                }
                else {
                    ESP_LOGI(TAG, "FLAT");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    // Initialize I2C and sensor here
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    write_register(0x1F, 0x0F);
    write_register(0x20, 0x66);
    xTaskCreate(gyroscope_task, "gyroscope_task", 2048, NULL, 10, NULL);
}