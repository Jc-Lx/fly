#include "messageQueue.h"

//声明消息队列句柄
QueueHandle_t messageQueue[QUEUE_NUM];

/**********************************************************************************************************
*函 数 名: MessageQueueCreate
*功能说明: 消息队列创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageQueueCreate(void)
{
    messageQueue[ACC_SENSOR_READ]    =  xQueueCreate(2, sizeof(Vector3f_t *));
    messageQueue[GYRO_SENSOR_READ]   =  xQueueCreate(2, sizeof(Vector3f_t *));

    messageQueue[GYRO_DATA_PRETREAT] =  xQueueCreate(2, sizeof(Vector3f_t *));
    messageQueue[ACC_DATA_PRETREAT]  =  xQueueCreate(2, sizeof(Vector3f_t *));
    messageQueue[GYRO_FOR_CONTROL]   =  xQueueCreate(2, sizeof(Vector3f_t *));
}


