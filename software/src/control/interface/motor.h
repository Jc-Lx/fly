#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "pwm.h"
#include "flightStatus.h"
#include "parameter.h"
#include "system.h"

typedef struct
{
    float throttle;     //油门
    float pitch;        //俯仰
    float roll;         //横滚
    float yaw;          //偏航
} MOTOR_MIXER_t;

typedef struct
{
    int8_t        motorNum;
    MOTOR_MIXER_t motorMixer[8];
} MOTOR_TYPE_t;

void MotorInit(void);
void MotorControl(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle);
void MotorStop(void);

#endif

