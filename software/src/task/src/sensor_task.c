#include "taskconfig.h"

#include "sensor.h"

extern ACCELEROMETER_t acc;

xTaskHandle imuDataPreTreatTask;
xTaskHandle otherSensorTask;

//定义传感器校准标志宏，用来开启或者关闭校准程序
#define CalibrationON_OFF       1

/**********************************************************************************************************
*函 数 名: vImuDataPreTreatTask
*功能说明: IMU传感器数据预处理任务，任务优先级仅次于IMU传感器读取
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vImuDataPreTreatTask, pvParameters)
{
    Vector3f_t* gyroRawData;
    Vector3f_t* accRawData;
    Vector3f_t* accData  = pvPortMalloc(sizeof(Vector3f_t));
    Vector3f_t* gyroData = pvPortMalloc(sizeof(Vector3f_t));
    Vector3f_t* gyroLpfData = pvPortMalloc(sizeof(Vector3f_t));

    //挂起调度器
    vTaskSuspendAll();

    //加速度预处理初始化
    AccPreTreatInit();

    //陀螺仪预处理初始化
    GyroPreTreatInit();

    //唤醒调度器
    xTaskResumeAll();
    
  while (1)
  {
    xQueueReceive(messageQueue[ACC_SENSOR_READ],&accRawData,3);
    xQueueReceive(messageQueue[GYRO_SENSOR_READ],&gyroRawData,3);
    // printf("\r\n accraw x: %f y: %f z: %f \r\n",accRawData->x,accRawData->y,accRawData->z);
    // printf("gyroraw x: %f y: %f z: %f \r\n",gyroRawData->x,gyroRawData->y,gyroRawData->z);
    if(CalibrationON_OFF)
    {
        //加速度校准
        //AccCalibration(*accRawData);

        //陀螺仪校准
        // GyroCalibration(*gyroRawData);

        //IMU安装误差校准
        // ImuLevelCalibration();
    }
        //陀螺仪数据预处理
        GyroDataPreTreat(*gyroRawData, gyroData, gyroLpfData);

        //加速度数据预处理
        AccDataPreTreat(*accRawData, accData);

        //往下一级消息队列中填充数据
        xQueueSendToBack(messageQueue[ACC_DATA_PRETREAT], (void *)&accData, 0);
        xQueueSendToBack(messageQueue[GYRO_DATA_PRETREAT], (void *)&gyroData, 0);
        xQueueSendToBack(messageQueue[GYRO_FOR_CONTROL], (void *)&gyroLpfData, 0);
   
    vTaskDelay(5);
  }
}
/**********************************************************************************************************
*函 数 名: vOtherSensorTask
*功能说明: 其它传感器数据预处理任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vOtherSensorTask, pvParameters)
{
    static uint16_t count = 0;

    //挂起调度器
    vTaskSuspendAll();

    //磁力计校准参数初始化
    MagCaliDataInit();

    //唤醒调度器
    xTaskResumeAll();
    for(;;)
    {
      //100Hz
        if(count % 2 == 0)
        {
            //磁力计校准
            // MagCalibration();

            //磁力计数据预处理
            MagDataPreTreat();
        }
        count ++;
        vTaskDelay(5);
    }
}

/**********************************************************************************************************
*函 数 名: SensorTaskCreate
*功能说明: 传感器数据预处理相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SensorTaskCreate(void)
{
    xTaskCreate(vImuDataPreTreatTask, "imuDataPreTreat", 256, NULL, 12, &imuDataPreTreatTask);
    xTaskCreate(vOtherSensorTask, "otherSensor", 256, NULL, 7, &otherSensorTask);
}



