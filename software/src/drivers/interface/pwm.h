#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx.h"

void MOTOR_Init(void);
void MotorPWMSet(uint8_t motor, uint16_t pwmValue);


#endif

