// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "mpu6050.h"

extern "C" {
	void app_main(void);
}

mpu6050_data_t mpu6050_data;

void mpu6050(void *pvParameters);
void printer(void *pvParameters);
void controller(void *pvParameters);

void app_main(void)
{
    // starting I2C condig
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
    conf.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	vTaskDelay(500/portTICK_PERIOD_MS);
	xTaskCreate(&mpu6050, "IMU", 1024*8, &mpu6050_data, 5, NULL);
    xTaskCreate(&printer, "printer", 1024*8, &mpu6050_data, 5, NULL);
    xTaskCreate(&controller, "controller", 1024*8, &mpu6050_data, 5, NULL);
}