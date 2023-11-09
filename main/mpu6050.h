#ifndef MPU6050_H
#define MPU6050_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c.h>
// I2C and MPU6050 drivers
#include "I2Cdev.h"
// #include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEF_TO_RAD 0.0174533

struct mpu6050_data_t {
	int8_t mpuIntStatus;   // Holds actuall interupt status
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
	float ypr[3];
};

#endif // MPU6050_H