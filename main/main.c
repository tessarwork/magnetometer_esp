#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define QMC6309_SENSOR_ADDR 0x7C

static const char *TAG = "DATA";

void i2c_master_init(){ 
    i2c_config_t conf = { 
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "i2c initialized");
}
void configure_sensor(){ 
    uint8_t mode_reg_value = 0x63;
    ESP_LOGI(TAG, "Configuring Sensor");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC6309_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0A, true);
    i2c_master_write_byte(cmd, mode_reg_value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor configured successfully");
    } else {
        ESP_LOGE(TAG, "Sensor configuration failed");
    }
}

void read_data(){ 
    uint8_t sensor_data[6];
    ESP_LOGI(TAG, "READING SENSOR DATA....");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC6309_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x01, true); // starting address of data registers
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC6309_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, sensor_data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        int16_t x = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
        int16_t y = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
        int16_t z = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);
        ESP_LOGI(TAG, "X: %d, Y: %d, Z: %d", x, y, z);

    } else{ 
        ESP_LOGE(TAG, "FAILED TO READ");
    }
    

    
    // vTaskDelay(pdMS_TO_TICKS(100));
    
    

    

}

void i2c_scanner(){ 
    int i;
    esp_err_t espRc;
    for (i = 1; i < 128; i++){ 
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
        if (espRc == 0) {
            ESP_LOGI(TAG, "Found I2C device at address 0x%02x", i);
        }
        i2c_cmd_link_delete(cmd);
    }
}
void app_main(void)
{
    i2c_master_init();
    while(1){ 
        read_data();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Scan every 2000 ms
    }
    // read_data();

    // return 0;

}