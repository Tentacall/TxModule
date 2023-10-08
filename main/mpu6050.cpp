#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c.h>

// I2C and MPU6050 drivers
#include "I2Cdev.h"
// #include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

static const char *TAG = "IMU";
#define RAD_TO_DEG (180.0/M_PI)
#define DEF_TO_RAD 0.0174533


MPU6050 mpu;
// mpu control variables 
bool dmpReady = false;  // True if dmp initialization is successful
uint8_t mpuIntStatus;   // Holds actuall interupt status
uint8_t devStatus;      // return status after each device operation
uint16_t packetSize;    // expected DMP packet size ( default to 42 bytes )
uint16_t fifoCount;     // counts all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO Storage buffer

// orientation/motion variables
Quaternion q;           // [w,x,y,z]        quaternion container
VectorInt16 aa;         // [x,y,z]          accel sensor measurement
VectorInt16 aaReal;     // [x,y,z]          gravity free measurement
VectorInt16 aaWorld;    // [x,y,z]          world-frame accell sensor
VectorFloat gravity;    // [x,y,z]          gravity Vector
float euler[3];         // [psi,theta,phi]  Euler Angle container
float ypr[3];           // [yaw,pitch,roll] yaw/pitch/roll container


void Initialize() {
    printf("[?] Initializing I2C Device ...\n");
    mpu.initialize();

    printf("[?] Testing device connection ...");
    printf(mpu.testConnection() ? "MPU6050 Connection Successful\n" : "MPU6050 Connection Failed");
}

// display quaternion values in easy matrix form: w x y z
void getQuaternion() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	// x, y , z , w
	printf("%6.2f,%6.2f,%6.2f,%6.2f,", q.x, q.y, q.z, q.w);
}

// display Euler angles in degrees
void getEuler() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
}

// display Euler angles in degrees
void getYawPitchRoll() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if 0
	float _roll = ypr[2] * RAD_TO_DEG;
	float _pitch = ypr[1] * RAD_TO_DEG;
	float _yaw = ypr[0] * RAD_TO_DEG;
	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",_roll, _pitch, _yaw);
#endif
	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
}

// display real acceleration, adjusted to remove gravity
void getRealAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
void getWorldAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	//x,y,z
	printf("%d,%d,%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}


void mpu6050(void *pvParameters) {
    Initialize();
    
    // get device id
    uint8_t buffer[1];
    I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
    ESP_LOGI(TAG, "getDeviceID=0x%x", buffer[0]);

    devStatus = mpu.dmpInitialize();
    ESP_LOGI(TAG, "devStatus=%d", devStatus );
    if (devStatus != 0){
       ESP_LOGE(TAG, "DMP Initialization failed [%d]", devStatus);
       while (true){
            vTaskDelay(1);
       } 
    }
   	mpu.setXAccelOffset(-1450);
	mpu.setYAccelOffset(24);
	mpu.setZAccelOffset(1116);
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
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            // getYawPitchRoll();
			getQuaternion();
			//getEuler();
			// getRealAccel();
			getWorldAccel();
        }

        vTaskDelay(100/portTICK_PERIOD_MS);
    }


    vTaskDelete(NULL);

}