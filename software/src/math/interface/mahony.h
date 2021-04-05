#ifndef __MAHONY_H_
#define __MAHONY_H_

#include "quaternion.h"
#include "accelerometer.h"
#include "usart.h"
#include "magnetometer.h"

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
Vector3f_t GetCopterAngle(void);

#endif


