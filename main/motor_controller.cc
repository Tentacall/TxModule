#include "iot_servo.h"
#include "mpu6050.h"
#include "pid.h"

servo_config_t servo_config = {
    .max_angle = 180,               // Maximum angle of the servo
    .min_width_us = 500,            // Pulse width corresponding to minimum angle
    .max_width_us = 2500,           // Pulse width corresponding to maximum angle
    .freq = 50,                     // PWM frequency
    .timer_number = LEDC_TIMER_0,   // Timer number of ledc
    .channels = {
        .servo_pin = {GPIO_NUM_18}, // GPIO pin for the servo
        .ch = { LEDC_CHANNEL_0,     /*!< LEDC channel 0 */
                LEDC_CHANNEL_1,     /*!< LEDC channel 1 */
                LEDC_CHANNEL_2,     /*!< LEDC channel 2 */
                LEDC_CHANNEL_3,     /*!< LEDC channel 3 */
                LEDC_CHANNEL_4,     /*!< LEDC channel 4 */
                LEDC_CHANNEL_5,
            },     // LEDC channel used
    },
    .channel_number = 1,            // Total channel number
};

servo_config_t servo2_config = {
    .max_angle = 180,               // Maximum angle of the servo
    .min_width_us = 500,            // Pulse width corresponding to minimum angle
    .max_width_us = 2500,           // Pulse width corresponding to maximum angle
    .freq = 50,                     // PWM frequency
    .timer_number = LEDC_TIMER_0,   // Timer number of ledc
    .channels = {
        .servo_pin = {GPIO_NUM_19}, // GPIO pin for the servo
        .ch = { LEDC_CHANNEL_0,     /*!< LEDC channel 0 */
                LEDC_CHANNEL_1,     /*!< LEDC channel 1 */
                LEDC_CHANNEL_2,     /*!< LEDC channel 2 */
                LEDC_CHANNEL_3,     /*!< LEDC channel 3 */
                LEDC_CHANNEL_4,     /*!< LEDC channel 4 */
                LEDC_CHANNEL_5,
            },     // LEDC channel used
    },
    .channel_number = 1,            // Total channel number
};

void controller(void *pvParameters)
{
    esp_err_t ret = iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_config);
    if (ret != ESP_OK) {
        printf("Servo init failed\n");
        vTaskDelete(NULL);
    }

    esp_err_t ret2 = iot_servo_init(LEDC_LOW_SPEED_MODE, &servo2_config);
    if (ret2 != ESP_OK) {
        printf("Servo 2 init failed\n");
        vTaskDelete(NULL);
    }
    printf("Both servo initialized\n");

    mpu6050_data_t *data = (mpu6050_data_t *)pvParameters;
    int32_t rot_angle = 0;
    int32_t increment = 1;
    while (true) {
        rot_angle += increment;
        if (rot_angle >= 180) {
            rot_angle = 180;
            increment = -increment;
        } else if (rot_angle <= 0) {
            rot_angle = 0;
            increment = -increment;
        }
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, rot_angle);
        // if (ret != ESP_OK) {
        //     printf("Servo write failed\n");
        // }
        // else {
        //     printf("Servo write success, rotangle : %ld\n", rot_angle);
        // }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}