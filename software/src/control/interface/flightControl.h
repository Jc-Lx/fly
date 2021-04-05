#ifndef _FLIGHTCONTROL_H_
#define _FLIGHTCONTROL_H_

#include "mathtool.h"
#include "pid.h"
#include "motor.h"
#include "mahony.h"

#define MAXANGLE  60                  //最大飞行角度：60°
#define MAXYAWANGLE  40               //最大转弯角度：30°

#define MAXRCDATA 400


enum
{
    ROLL_INNER,
    PITCH_INNER,
    YAW_INNER,
    ROLL_OUTER,
    PITCH_OUTER,
    YAW_OUTER,
    // VEL_X,
    // VEL_Y,
    // VEL_Z,
    // POS_X,
    // POS_Y,
    // POS_Z,
    PIDNUM
};
typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float throttle;
} RCTARGET_t;

typedef struct
{
    PID_t      pid[PIDNUM];         //PID参数结构体

    RCTARGET_t rcTarget;            //摇杆控制量
    Vector3f_t angleLpf;

    Vector3f_t attInnerCtlValue;    //姿态内环控制量
    float      altInnerCtlValue;    //高度内环控制量

    Vector3f_t attInnerTarget;      //姿态内环（角速度）控制目标
    Vector3f_t attOuterTarget;      //姿态外环（角度）控制目标

    Vector3f_t attInnerError;       //姿态内环（角速度）控制误差
    Vector3f_t attOuterError;       //姿态外环（角度）控制误差

} FLIGHTCONTROL_t;

void FlightControlInit(void);
void FlightControlInnerLoop(Vector3f_t gyro);
void AttitudeOuterControl(void);
void SetRcTarget(RCTARGET_t rcTarget);
RCTARGET_t GetRcTarget(void);
void FcSetPID(uint8_t id, PID_t pid);

#endif

