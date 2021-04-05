#include "taskconfig.h"

#include "navigation.h"

xTaskHandle flightStatusTask;
xTaskHandle navigationTask;

/**********************************************************************************************************
*函 数 名: vFlightStatusTask
*功能说明: 飞行状态检测相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vFlightStatusTask, pvParameters)
{
    Vector3f_t *accraw;
    Vector3f_t *gyroraw;
    for(;;)
    {
        xQueueReceive(messageQueue[ACC_SENSOR_READ],&accraw,10);
        xQueueReceive(messageQueue[GYRO_SENSOR_READ],&gyroraw,10);

        //传感器方向检测(用于校准时的判断)
        // ImuOrientationDetect(*accraw);

        //飞行器放置姿态检测
        PlaceStausCheck(*gyroraw);

        vTaskDelay(50);
    }

}

/**********************************************************************************************************
*函 数 名: vNavigationTask
*功能说明: 导航计算相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vNavigationTask, pvParameters)
{
    Vector3f_t* gyro;
    Vector3f_t* acc;
    Vector3f_t mag;

    for(;;)
    {  
        xQueueReceive(messageQueue[ACC_DATA_PRETREAT],&acc,3);
        xQueueReceive(messageQueue[GYRO_FOR_CONTROL],&gyro,3);

        mag = MagGetData();

        //互补滤波算法更新四元数
        MahonyAHRSupdate(gyro->x,gyro->y,gyro->z,acc->x,acc->y,acc->z,mag.x,mag.y,mag.z);

       vTaskDelay(5);     
    }

}

/**********************************************************************************************************
*函 数 名: NavigationTaskCreate
*功能说明: 导航相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationTaskCreate(void)
{
    xTaskCreate(vNavigationTask, "navigation", 512, NULL, 11, &navigationTask);
    xTaskCreate(vFlightStatusTask, "flightStatus", 256, NULL, 5, &flightStatusTask);
}
