#ifndef __MAGNETOMETER_H
#define	__MAGNETOMETER_H

#include "vector3.h"
#include "sensordata.h"
#include "parameter.h"
#include "delay.h"
#include "lowpassfilter.h"
#include "usart.h"
#include "LevenbergMarquardt.h"
#include "gyroscope.h"
#include "mpu9250.h"



typedef struct {
    Vector3f_t data;
    float mag;
    SENSOR_CALI_t cali;
    float earthMag;

} MAGNETOMETER_t;

enum {
    MaxX,
    MinX,
    MaxY,
    MinY,
    MaxZ,
    MinZ
};


void MagCaliDataInit(void);
void MagCalibration(void);
void MagDataPreTreat(void);
Vector3f_t MagGetData(void);
float GetMagMag(void);


#endif

