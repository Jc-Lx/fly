#ifndef __GYROSCOPE_H
#define __GYROSCOPE_H

#include "vector3.h"
#include "lowpassfilter.h"
#include "sensordata.h"
#include "parameter.h"
#include "usart.h"
#include "flightStatus.h"
#include "accelerometer.h"

typedef struct {
    Vector3f_t data;
    Vector3f_t dataLpf;
    //float temperature;
    LPF2ndData_t lpf_2nd;
    SENSOR_CALI_t cali;
} GYROSCOPE_t;



void GyroPreTreatInit(void);
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData, Vector3f_t* gyroLpfData);
void GyroCalibration(Vector3f_t gyroRaw);
Vector3f_t GyroGetData(void);

#endif


