#ifndef __SENSORDATA_H
#define __SENSORDATA_H

#include "mathtool.h"
#include "FreeRTOS.h"

#include "usart.h"

//默认设置传感器自带低通滤波频率为42Hz

//陀螺仪低通滤波截止频率
//二次低通滤波的陀螺仪数据用于角速度环控制
//频率过低，信号延迟大，会导致控制发散
//频率过高，信号噪声大，会增加高频抖动
//要达到最佳效果，需经调试选取最佳的截止频率
//飞行器越小，系统惯性越小，所需的控制频率越高，对信号实时性的要求也就越高
//450轴距左右的飞机，截止频率不低于50就好，要根据实际情况选取，比如有时过软的硬件减震措施也会增加实际信号延迟
#define GYRO_LPF_CUT 88

//加速度低通滤波截止频率
//二次低通滤波的加速度数据主要用于计算模值
#define ACC_LPF_CUT 30

//定义解算数据结构体sensordata_t
typedef struct 
{
    Vector3f_t acc;
    Vector3f_t gyro;
    Vector3f_t mag;
}sensordata_t;

//飞控放置方向
enum ORIENTATION_STATUS
{
    ORIENTATION_UP,
    ORIENTATION_DOWN,
    ORIENTATION_LEFT,
    ORIENTATION_RIGHT,
    ORIENTATION_FRONT,
    ORIENTATION_BACK,
};


typedef struct
{
    Vector3f_t offset;	    //零偏误差
    Vector3f_t scale;		//比例误差
    //bool should_cali;		//传感器校准标志位
    bool success;           //校准成功标志位
    uint8_t step;           //步骤标志位
} SENSOR_CALI_t;



void ImuOrientationDetect(Vector3f_t accraw);
enum ORIENTATION_STATUS GetImuOrientation(void);

#endif


