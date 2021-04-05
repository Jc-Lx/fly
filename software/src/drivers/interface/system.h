#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos �����ļ� */
#include "FreeRTOSConfig.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "nvic.h"


/*�ײ�Ӳ������*/
#include "taskconfig.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "ws2812.h"
#include "soft_iic_gpio.h"
#include "mpu9250.h"
#include "bmp280.h"
#include "sensordata.h"
#include "messageQueue.h"
#include "accelerometer.h"
#include "flightStatus.h"
#include "cppm.h"
#include "pwm.h"
#include "beeper.h"


static uint8_t GY_91_Init(void);
void systemInit(void);

uint32_t GetSysTimeMs(void);
uint64_t GetSysTimeUs(void);

#endif /* __SYSTEM_H */














