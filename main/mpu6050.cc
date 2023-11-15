#include "mpu6050.h"

static const char *TAG = "IMU";

MPU6050 mpu;
// mpu control variables 
bool dmpReady = false;  // True if dmp initialization is successful
// mpu6050_data_t mpu6050_data;

void Initialize() {
    printf("[?] Initializing I2C Device ...\n");
    mpu.initialize();

    printf("[?] Testing device connection ...");
    printf(mpu.testConnection() ? "MPU6050 Connection Successful\n" : "MPU6050 Connection Failed");
}

// display quaternion values in easy matrix form: w x y z
void getQuaternion(mpu6050_data_t* data) {
	mpu.dmpGetQuaternion(&(data->q), data->fifoBuffer);
	// x, y , z , w
	// printf("%6.2f,%6.2f,%6.2f,%6.2f,", data->q.x, data->q.y, data->q.z, data->q.w);
}

// display Euler angles in degrees
// void getEuler() {
// 	mpu.dmpGetQuaternion(&mpu6050_data.q, mpu6050_data.fifoBuffer);
// 	mpu.dmpGetEuler(mpu6050_data.euler, &mpu6050_data.q);
// 	printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", mpu6050_data.euler[0] * RAD_TO_DEG, mpu6050_data.euler[1] * RAD_TO_DEG, mpu6050_data.euler[2] * RAD_TO_DEG);
// }

// display Euler angles in degrees
// void getYawPitchRoll() {
// 	mpu.dmpGetQuaternion(&mpu6050_data.q, mpu6050_data.fifoBuffer);
// 	mpu.dmpGetGravity(&mpu6050_data.gravity, &mpu6050_data.q);
// 	mpu.dmpGetYawPitchRoll(mpu6050_data.ypr, &mpu6050_data.q, &mpu6050_data.gravity);
// #if 0
// 	float _roll = mpu6050_data.ypr[2] * RAD_TO_DEG;
// 	float _pitch = mpu6050_data.ypr[1] * RAD_TO_DEG;
// 	float _yaw = mpu6050_data.ypr[0] * RAD_TO_DEG;
// 	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",_roll, _pitch, _yaw);
// #endif
// 	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
// 	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",mpu6050_data.ypr[2] * RAD_TO_DEG, mpu6050_data.ypr[1] * RAD_TO_DEG, mpu6050_data.ypr[0] * RAD_TO_DEG);
// }

// display real acceleration, adjusted to remove gravity
// void getRealAccel() {
// 	mpu.dmpGetQuaternion(&mpu6050_data.q, mpu6050_data.fifoBuffer);
// 	mpu.dmpGetAccel(&mpu6050_data.aa, mpu6050_data.fifoBuffer);
// 	mpu.dmpGetGravity(&mpu6050_data.gravity, &mpu6050_data.q);
// 	mpu.dmpGetLinearAccel(&mpu6050_data.aaReal, &mpu6050_data.aa, &mpu6050_data.gravity);
// 	printf("areal x=%d y:%d z:%d\n", mpu6050_data.aaReal.x, mpu6050_data.aaReal.y, mpu6050_data.aaReal.z);
// }

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
void getWorldAccel(mpu6050_data_t* data) {
	mpu.dmpGetQuaternion(&(data->q), data->fifoBuffer);
	mpu.dmpGetAccel(&(data->aa), data->fifoBuffer);
	mpu.dmpGetGravity(&(data->gravity), &(data->q));
	mpu.dmpGetLinearAccel(&(data->aaReal), &(data->aa), &(data->gravity));
	mpu.dmpGetLinearAccelInWorld(&(data->aaWorld), &(data->aaReal), &(data->q));
	//x,y,z
	// printf("%d,%d,%d\n", data->aaWorld.x, data->aaWorld.y, data->aaWorld.z);
}


void mpu6050(void *pvParameters) {
    Initialize();
    mpu6050_data_t* data = (mpu6050_data_t*) pvParameters;
    // get device id
    uint8_t buffer[1];
    I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
    ESP_LOGI(TAG, "getDeviceID=0x%x", buffer[0]);

    data->devStatus = mpu.dmpInitialize();
    ESP_LOGI(TAG, "mpu6050_data.devStatus=%d", data->devStatus );
    if (data->devStatus != 0){
       ESP_LOGE(TAG, "DMP Initialization failed [%d]", data->devStatus);
       while (true){
            vTaskDelay(1);
       } 
    }

   	mpu.setXAccelOffset(-1450);
	mpu.setYAccelOffset(24);
	mpu.setZAccelOffset(3116);
	mpu.setXGyroOffset(112);
	mpu.setYGyroOffset(9);
	mpu.setZGyroOffset(-36);

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	mpu.setDMPEnabled(true); 
    ESP_LOGE(TAG, "Calibration Complete.");

    // mainloop
    while (true) {
        if (mpu.dmpGetCurrentFIFOPacket(data->fifoBuffer)) {
            // getYawPitchRoll();
			getQuaternion(data);
			//getEuler();
			// getRealAccel();
			getWorldAccel(data);
        }

        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}