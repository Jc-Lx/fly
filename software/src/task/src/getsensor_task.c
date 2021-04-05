#include "taskconfig.h"

#include "getsensor.h"


//声明任务句柄
xTaskHandle imuSensorReadTask;
xTaskHandle sensorUpdateTask; 

/**********************************************************************************************************
*函 数 名: vImuSensorReadTask
*功能说明: IMU传感器数据读取任务，此任务具有最高优先级，运行频率为1KHz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vImuSensorReadTask, pvParameters)
{
    Vector3f_t* gyroRawData = pvPortMalloc(sizeof(Vector3f_t));
    Vector3f_t* accRawData = pvPortMalloc(sizeof(Vector3f_t));

    for(;;)
    {

        //获取IMU原始数据
       MPU_Get_Accelerometer(accRawData);
       MPU_Get_Gyroscope(gyroRawData);


       //发送IMU原始数据到消息队列
       xQueueSendToBack(messageQueue[ACC_SENSOR_READ],(void *)&accRawData,0);
       xQueueSendToBack(messageQueue[GYRO_SENSOR_READ],(void *)&gyroRawData,0);

      vTaskDelay(1);
    }
}

/**********************************************************************************************************
*函 数 名: vSensorUpdateTask
*功能说明: IMU之外的传感器数据更新任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vSensorUpdateTask, pvParameters)
{
    static uint16_t count =0 ;

    for(;;)
    {
        //地磁传感器数据更新 100Hz
        if(count % 2 == 0)
        {
            vTaskSuspendAll();
            MPU_Get_Magnetometer();
            xTaskResumeAll();
            //printf("\r\n  mag x: %f y: %f z: %f  \r\n",magRawData->x,magRawData->y,magRawData->z);
        }

        //飞控参数保存(参数有更新才会执行）20Hz
        if(count % 10 == 0)
        {
            ParamSaveToFlash();
        }
        count ++;
        vTaskDelay(5);
    }


}

/**********************************************************************************************************
*函 数 名: ModuleTaskCreate
*功能说明: 传感器组件相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GetSensorTaskCreate(void)
{
    xTaskCreate(vImuSensorReadTask, "imuSensorRead", 256, NULL, 13, &imuSensorReadTask);
    xTaskCreate(vSensorUpdateTask, "sensorUpdate", 256, NULL, 8, &sensorUpdateTask);
    
}

