#include "mpu6050.h"

void printer(void *pvParameters)
{
    mpu6050_data_t *data = (mpu6050_data_t *)pvParameters;
    while (true)
    {
        printf("%6.2f,%6.2f,%6.2f,%6.2f,", data->q.x, data->q.y, data->q.z, data->q.w);
        printf("%d,%d,%d\n", data->aaWorld.x, data->aaWorld.y, data->aaWorld.z);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}