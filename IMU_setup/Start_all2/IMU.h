#ifndef IMU_H
#define IMU_H

#include "ICM_20948.h"
extern ICM_20948_I2C myICM;

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

void IMU_setup();
void readIMU(float &x_acc, float &y_acc, float &z_acc, float &x_gyr, float &y_gyr, float &z_gyr, float &x_mag, float &y_mag, float &z_mag);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals, const char* label);
void printScaledAGMT(float x_acc, float y_acc, float z_acc, float x_gyr, float y_gyr, float z_gyr, float x_mag, float y_mag, float z_mag);
// void printScaledAGMT1(ICM_20948_I2C *sensor);

#endif