#include "gyroscope.h"

GYROSCOPE_t    gyro;

/**********************************************************************************************************
*函 数 名: GyroPreTreatInit
*功能说明: 陀螺仪预处理初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GyroPreTreatInit(void)
{
    ParamGetData(PARAM_GYRO_OFFSET_X, &gyro.cali.offset.x, 4);
    ParamGetData(PARAM_GYRO_OFFSET_Y, &gyro.cali.offset.y, 4);
    ParamGetData(PARAM_GYRO_OFFSET_Z, &gyro.cali.offset.z, 4);
    ParamGetData(PARAM_GYRO_SCALE_X, &gyro.cali.scale.x, 4);
    ParamGetData(PARAM_GYRO_SCALE_Y, &gyro.cali.scale.y, 4);
    ParamGetData(PARAM_GYRO_SCALE_Z, &gyro.cali.scale.z, 4);

    printf("\r\n    gyro.cali.offet:  x:%f    y:%f    z:%f    \r\n",gyro.cali.offset.x,gyro.cali.offset.y,gyro.cali.offset.z);

    if(isnan(gyro.cali.offset.x) || isnan(gyro.cali.offset.y) || isnan(gyro.cali.offset.z))
    {
        gyro.cali.offset.x = 0;
        gyro.cali.offset.y = 0;
        gyro.cali.offset.z = 0;
    }

    if(abs(gyro.cali.scale.x - 1) > 0.1f || abs(gyro.cali.scale.y - 1) > 0.1f || abs(gyro.cali.scale.z - 1) > 0.1f)
    {
        gyro.cali.scale.x = 1;
        gyro.cali.scale.y = 1;
        gyro.cali.scale.z = 1;
    }

    //陀螺仪低通滤波系数计算
    LowPassFilter2ndFactorCal(0.001, GYRO_LPF_CUT, &gyro.lpf_2nd);
}

/**********************************************************************************************************
*函 数 名: GyroDataPreTreat
*功能说明: 陀螺仪数据预处理
*形    参: 陀螺仪原始数据 陀螺仪预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData, Vector3f_t* gyroLpfData)
{
    gyro.data = gyroRaw;

    //检测陀螺仪是否工作正常
    // GyroDetectCheck(gyroRaw);


    //零偏误差校准
    gyro.data.x = (gyro.data.x - gyro.cali.offset.x) * gyro.cali.scale.x;
    gyro.data.y = (gyro.data.y - gyro.cali.offset.y) * gyro.cali.scale.y;
    gyro.data.z = (gyro.data.z - gyro.cali.offset.z) * gyro.cali.scale.z;

    //安装误差校准
    // gyro.data = VectorRotateToBodyFrame(gyro.data, GetLevelCalibraData());

    //低通滤波
    gyro.dataLpf = LowPassFilter2nd(&gyro.lpf_2nd, gyro.data);

    *gyroData = gyro.data;
    *gyroLpfData = gyro.dataLpf;
}

/**********************************************************************************************************
*函 数 名: GyroCalibration
*功能说明: 陀螺仪校准
*形    参: 陀螺仪原始数据
*返 回 值: 无
**********************************************************************************************************/
void GyroCalibration(Vector3f_t gyroRaw)
{
    const int16_t CALIBRATING_GYRO_CYCLES = 1000;
    static float gyro_sum[3] = {0, 0, 0};
    Vector3f_t gyro_cali_temp, gyro_raw_temp;
    static int16_t count = 0;
    static uint8_t staticFlag = 0;
    gyro_raw_temp = gyroRaw;
    printf("\r\n    gyro_sum:  x:%f    y:%f    z:%f    \r\n",gyro_sum[0],gyro_sum[1],gyro_sum[2]);
    printf("\r\n    count:     %d    \r\n",count);
    printf("\r\n    gyro.cali.offet:  x:%f    y:%f    z:%f    \r\n",gyro.cali.offset.x,gyro.cali.offset.y,gyro.cali.offset.z);

    //陀螺仪校准过程中如果检测到飞机不是静止状态则不会进行校准
    if(GetPlaceStatus() == STATIC)
    {
        staticFlag = 0;
        gyro_sum[0] += gyro_raw_temp.x;
        gyro_sum[1] += gyro_raw_temp.y;
        gyro_sum[2] += gyro_raw_temp.z;
        count++;
        if(count == CALIBRATING_GYRO_CYCLES)
        {
            gyro.cali.step = 1;

            gyro_cali_temp.x = gyro_sum[0] / CALIBRATING_GYRO_CYCLES;
            gyro_cali_temp.y = gyro_sum[1] / CALIBRATING_GYRO_CYCLES;
            gyro_cali_temp.z = gyro_sum[2] / CALIBRATING_GYRO_CYCLES;
            gyro_sum[0] = 0;
            gyro_sum[1] = 0;
            gyro_sum[2] = 0;

            //检测校准数据是否有效
            if((abs(gyro_raw_temp.x - gyro_cali_temp.x) + abs(gyro_raw_temp.x - gyro_cali_temp.x)
                    + abs(gyro_raw_temp.x - gyro_cali_temp.x)) < 0.6f && !staticFlag)
            {
                gyro.cali.success = 1;
                printf("\r\n  GyroCalibration  down!  \r\n");
            }
            else
            {
                gyro.cali.success = 0;
                printf("\r\n  GyroCalibration  false!  \r\n");
            }

            if(gyro.cali.success)
            {
                gyro.cali.step = 2;
                gyro.cali.offset.x = gyro_cali_temp.x;
                gyro.cali.offset.y = gyro_cali_temp.y;
                gyro.cali.offset.z = gyro_cali_temp.z;
                gyro.cali.scale.x = 1;
                gyro.cali.scale.y = 1;
                gyro.cali.scale.z = 1;

                //保存陀螺仪校准参数
                ParamUpdateData(PARAM_GYRO_OFFSET_X, &gyro.cali.offset.x);
                ParamUpdateData(PARAM_GYRO_OFFSET_Y, &gyro.cali.offset.y);
                ParamUpdateData(PARAM_GYRO_OFFSET_Z, &gyro.cali.offset.z);
                ParamUpdateData(PARAM_GYRO_SCALE_X, &gyro.cali.scale.x);
                ParamUpdateData(PARAM_GYRO_SCALE_Y, &gyro.cali.scale.y);
                ParamUpdateData(PARAM_GYRO_SCALE_Z, &gyro.cali.scale.z);
                count = 0;

        
                gyro.cali.step = 0;
            }
        }
        //防止程序跑飞
        if(count > (CALIBRATING_GYRO_CYCLES + 10))   
        {
            count = 0;
            gyro_sum[0] = 0;
            gyro_sum[1] = 0;
            gyro_sum[2] = 0;
        }
    }else staticFlag = 1;
    
}

/**********************************************************************************************************
*函 数 名: GyroGetData
*功能说明: 获取经过处理后的陀螺仪数据
*形    参: 无
*返 回 值: 角速度
**********************************************************************************************************/
Vector3f_t GyroGetData(void)
{
    return gyro.data;
}

