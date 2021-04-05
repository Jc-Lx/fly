#ifndef __MESSAGE_QUEUE_H__
#define __MESSAGE_QUEUE_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "vector3.h"


enum {
    GYRO_SENSOR_READ,
    ACC_SENSOR_READ,
    GYRO_DATA_PRETREAT,
    ACC_DATA_PRETREAT,
    GYRO_FOR_CONTROL,
    QUEUE_NUM
};

extern QueueHandle_t messageQueue[QUEUE_NUM];
void MessageQueueCreate(void);

#endif


