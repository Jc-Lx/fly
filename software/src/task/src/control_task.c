#include "TaskConfig.h"

xTaskHandle flightControlTask;

/**********************************************************************************************************
*函 数 名: vFlightControlTask
*功能说明: 飞行控制相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vFlightControlTask, pvParameters)
{
    static uint32_t count = 0;
    Vector3f_t* gyro;

    //遥控相关功能初始化
    RcInit();

    //控制参数初始化
    FlightControlInit();

    for(;;)
    {
        //从消息队列中获取数据
        xQueueReceive(messageQueue[GYRO_FOR_CONTROL], &gyro, (3 / portTICK_RATE_MS));

        //200Hz
        if(count%5 == 0)
        {
            RcCheck();
        }

        //500Hz
        if(count % 2 == 0)
        {
            //用户控制模式下的操控逻辑处理
            UserControl();

            //姿态外环控制
            AttitudeOuterControl();

        }
        
        //飞行内环控制，包括姿态内环和高度内环
        FlightControlInnerLoop(*gyro);

        count++;
        vTaskDelay(1);
    }

}

/**********************************************************************************************************
*函 数 名: ControlTaskCreate
*功能说明: 控制相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ControlTaskCreate(void)
{
    xTaskCreate(vFlightControlTask, "flightControl", 256, NULL, 10, &flightControlTask);
}

