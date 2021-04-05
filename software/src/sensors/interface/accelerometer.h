#ifndef __ACCELEROMETER_H
#define	__ACCELEROMETER_H

#include "vector3.h"
#include "sensordata.h"
#include "lowpassfilter.h"

#include "parameter.h"
#include "flightStatus.h"
#include "LevenbergMarquardt.h"

#include "parameter.h"

#include "usart.h"


typedef struct {
    Vector3f_t data;     //加速度数据校准后数据存储结构体
    Vector3f_t dataLpf;  //加速度数据滤波后数据存储结构体，用于计算加速度向量的模
    float mag;           //加速向量的模
    float vibraCoef;     //加速度振动系数
    LPF2ndData_t lpf_2nd;//加速低通滤波系数
    SENSOR_CALI_t cali;  //传感器校准结构体
    SENSOR_CALI_t levelCali;
} ACCELEROMETER_t;

void AccPreTreatInit(void);
void AccCalibration(Vector3f_t accRaw);
void ImuLevelCalibration(void);
Vector3f_t AccGetData(void);
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData);
float GetAccMag(void);
Vector3f_t GetLevelCalibraData(void);

#endif

