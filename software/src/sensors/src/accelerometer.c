#include "accelerometer.h "


ACCELEROMETER_t acc;
/**********************************************************************************************************
*函 数 名: AccPreTreatInit
*功能说明: 加速度预处理初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AccPreTreatInit(void)
{  
    ParamGetData(PARAM_ACC_OFFSET_X, &acc.cali.offset.x, 4);
    ParamGetData(PARAM_ACC_OFFSET_Y, &acc.cali.offset.y, 4);
    ParamGetData(PARAM_ACC_OFFSET_Z, &acc.cali.offset.z, 4);
    ParamGetData(PARAM_ACC_SCALE_X, &acc.cali.scale.x, 4);
    ParamGetData(PARAM_ACC_SCALE_Y, &acc.cali.scale.y, 4);
    ParamGetData(PARAM_ACC_SCALE_Z, &acc.cali.scale.z, 4);

    ParamGetData(PARAM_IMU_LEVEL_X, &acc.levelCali.scale.x, 4);
    ParamGetData(PARAM_IMU_LEVEL_Y, &acc.levelCali.scale.y, 4);
    ParamGetData(PARAM_IMU_LEVEL_Z, &acc.levelCali.scale.z, 4);

    printf("\r\n  acc.cali.offset :  x:%f    y:%f    z:%f    \r\n",acc.cali.offset.x,acc.cali.offset.y,acc.cali.offset.z);
    printf("\r\n  acc.cali.scale :  x:%f    y:%f    z:%f    \r\n",acc.cali.scale.x,acc.cali.scale.y,acc.cali.scale.z); 

    //isnan检查参数是否是非数字值
    if(isnan(acc.cali.offset.x) || isnan(acc.cali.offset.y) || isnan(acc.cali.offset.z) || \
            isnan(acc.cali.scale.x) || isnan(acc.cali.scale.y) || isnan(acc.cali.scale.z) ||    \
            acc.cali.scale.x == 0 || acc.cali.scale.y == 0 || acc.cali.scale.z == 0)
    {
        acc.cali.offset.x = 0;
        acc.cali.offset.y = 0;
        acc.cali.offset.z = 0;
        acc.cali.scale.x = 1;
        acc.cali.scale.y = 1;
        acc.cali.scale.z = 1;
    }

    //abs获取整数类型绝对值
    if(abs(acc.cali.offset.x) > 1 || abs(acc.cali.offset.y) > 1 || abs(acc.cali.offset.z) > 1 ||
            abs(acc.cali.scale.x) > 2 || abs(acc.cali.scale.y) > 2 || abs(acc.cali.scale.z) > 2 ||
            abs(acc.cali.scale.x) < 0.3f || abs(acc.cali.scale.y) < 0.3f || abs(acc.cali.scale.z) < 0.3f)
    {
        acc.cali.offset.x = 0;
        acc.cali.offset.y = 0;
        acc.cali.offset.z = 0;
        acc.cali.scale.x = 1;
        acc.cali.scale.y = 1;
        acc.cali.scale.z = 1;
    }

    if(isnan(acc.levelCali.scale.x) || isnan(acc.levelCali.scale.y) || isnan(acc.levelCali.scale.z) ||
            abs(acc.levelCali.scale.x) > 0.2f || abs(acc.levelCali.scale.y) > 0.2f || abs(acc.levelCali.scale.z) > 0.2f)
    {
        acc.levelCali.scale.x = 0;
        acc.levelCali.scale.y = 0;
        acc.levelCali.scale.z = 0;
    } 

    //加速度低通滤波系数计算
    LowPassFilter2ndFactorCal(0.001, ACC_LPF_CUT, &acc.lpf_2nd);

}

/**********************************************************************************************************
*函 数 名: AccDataPreTreat
*功能说明: 加速度数据预处理
*形    参: 加速度原始数据 加速度预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData)
{
    static float lastAccMag, accMagderi;
    const float deltaT = 0.001f;

    acc.data = accRaw;

    //加速度数据校准
    acc.data.x = (acc.data.x - acc.cali.offset.x) * acc.cali.scale.x;
    acc.data.y = (acc.data.y - acc.cali.offset.y) * acc.cali.scale.y;
    acc.data.z = (acc.data.z - acc.cali.offset.z) * acc.cali.scale.z;

    //水平误差校准,进行水平安装误差校准前关闭此函数
    // acc.data = VectorRotateToBodyFrame(acc.data, acc.levelCali.scale);

    //低通滤波
    acc.dataLpf = LowPassFilter2nd(&acc.lpf_2nd, acc.data);

    //计算加速度模值
    acc.mag = Pythagorous3(acc.dataLpf.x, acc.dataLpf.y, acc.dataLpf.z);

    //震动系数计算
    accMagderi = (acc.mag - lastAccMag) / deltaT;
    lastAccMag = acc.mag;
    acc.vibraCoef = acc.vibraCoef * 0.9995f + abs(accMagderi) * 0.0005f;

    *accData = acc.data;
}

/**********************************************************************************************************
*函 数 名: AccCalibration
*功能说明: 加速度校准
*形    参: 加速度原始数据
*返 回 值: 无
**********************************************************************************************************/
void AccCalibration(Vector3f_t accRaw)
{
    static uint16_t samples_count = 0;
    static uint8_t orientationCaliFlag[6];
    static Vector3f_t new_offset;
    static Vector3f_t new_scale;
    static Vector3f_t samples[6];
    static uint8_t caliFlag = 0;
    static uint32_t caliCnt = 0;

    //调试打印窗口数据
        printf("samples[0]:  x:%f    y:%f    z:%f    \r\n",samples[0].x,samples[0].y,samples[0].z);
        printf("samples[1]:  x:%f    y:%f    z:%f    \r\n",samples[1].x,samples[1].y,samples[1].z);
        printf("samples[2]:  x:%f    y:%f    z:%f    \r\n",samples[2].x,samples[2].y,samples[2].z);
        printf("samples[3]:  x:%f    y:%f    z:%f    \r\n",samples[3].x,samples[3].y,samples[3].z);
        printf("samples[4]:  x:%f    y:%f    z:%f    \r\n",samples[4].x,samples[4].y,samples[4].z);
        printf("samples[5]:  x:%f    y:%f    z:%f    \r\n",samples[5].x,samples[5].y,samples[5].z);
        printf("\r\n  %d \r\n",acc.cali.step);
        printf("\r\n  acc.cali.offset :  x:%f    y:%f    z:%f    \r\n",acc.cali.offset.x,acc.cali.offset.y,acc.cali.offset.z);
        printf("\r\n  acc.cali.scale :  x:%f    y:%f    z:%f    \r\n",acc.cali.scale.x,acc.cali.scale.y,acc.cali.scale.z);    

/*********************************检测IMU放置方向************************************/
    if (GetPlaceStatus() == STATIC)
    {
        caliCnt ++;
        if (caliCnt > 10)
        {
            switch (GetImuOrientation())
            {
                case ORIENTATION_UP:  if (!orientationCaliFlag[ORIENTATION_UP])
                    {
                        caliFlag = 1;
                        orientationCaliFlag[ORIENTATION_UP] = 1;
                        samples_count = 0;
                        acc.cali.step ++;
                        break;
                    }else 
                    {
                        break;
                    }
                case ORIENTATION_DOWN:  if (!orientationCaliFlag[ORIENTATION_DOWN])
                    {
                        caliFlag = 1;
                        orientationCaliFlag[ORIENTATION_DOWN] = 1;
                        samples_count = 0;
                        acc.cali.step ++;
                        break;
                    }else 
                    {
                        break;
                    }
                case ORIENTATION_FRONT:  if (!orientationCaliFlag[ORIENTATION_FRONT])
                    {
                        caliFlag = 1;
                        orientationCaliFlag[ORIENTATION_FRONT] = 1;
                        samples_count = 0;
                        acc.cali.step ++;
                        break;
                    }else 
                    {
                        break;
                    }
                case ORIENTATION_BACK:  if (!orientationCaliFlag[ORIENTATION_BACK])
                    {
                        caliFlag = 1;
                        orientationCaliFlag[ORIENTATION_BACK] = 1;
                        samples_count = 0;
                        acc.cali.step ++;
                        break;
                    }else 
                    {
                        break;
                    }
                case ORIENTATION_LEFT:  if (!orientationCaliFlag[ORIENTATION_LEFT])
                    {
                        caliFlag = 1;
                        orientationCaliFlag[ORIENTATION_LEFT] = 1;
                        samples_count = 0;
                        acc.cali.step ++;
                        break;
                    }else 
                    {
                        break;
                    }
                case ORIENTATION_RIGHT:  if (!orientationCaliFlag[ORIENTATION_RIGHT])
                    {
                        caliFlag = 1;
                        orientationCaliFlag[ORIENTATION_RIGHT] = 1;
                        samples_count = 0;
                        acc.cali.step ++;
                        break;
                    }else 
                    {
                        break;
                    }
                default: break;
                           
            }   
        }
        
    }
    
    
    /****************************************************************************************/
    //分别采集加速度计6个方向的数据，采样顺序随意，每个方向取100个样本
    if(caliFlag && (GetPlaceStatus() == STATIC))
    {
        if (samples_count<150)
        {
            samples[acc.cali.step - 1].x += accRaw.x;
            samples[acc.cali.step - 1].y += accRaw.y;
            samples[acc.cali.step - 1].z += accRaw.z;

            samples_count ++;
        }
        else if(samples_count == 150)
        {
            samples[acc.cali.step - 1].x /= 150;
            samples[acc.cali.step - 1].y /= 150;
            samples[acc.cali.step - 1].z /= 150;

            samples_count ++;
            caliFlag = 0;
            caliCnt = 0;
        }
    }
    if(acc.cali.step == 6 && samples_count == 151)
    {
        //给定LM法的初始解
        float initBeat[6];
        initBeat[0] = 0;
        initBeat[1] = 0;
        initBeat[2] = 0;
        initBeat[3] = 1;
        initBeat[4] = 1;
        initBeat[5] = 1;

        //LM法求解传感器误差方程最优解
        LevenbergMarquardt(samples, &new_offset, &new_scale, initBeat, 1);
        // printf("\r\n  new_offest :  x:%f    y:%f    z:%f    \r\n",new_offset.x,new_offset.y,new_offset.z);
        // printf("\r\n  new_scale :  x:%f    y:%f    z:%f    \r\n",new_scale.x,new_scale.y,new_scale.z);

        //判断校准参数是否正常
        if(fabsf(new_scale.x-1.0f) > 0.1f || fabsf(new_scale.y-1.0f) > 0.1f || fabsf(new_scale.z-1.0f) > 0.1f)
        {
            acc.cali.success = false;
            printf("\r\n AccCalibration false1 \r\n");
        }
        else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f))
        {
            acc.cali.success = false;
            printf("\r\n AccCalibration false2 \r\n");
        }
        else
        {
            acc.cali.success = true;
        }
        if(acc.cali.success)
        {
            acc.cali.offset = new_offset;
            acc.cali.scale = new_scale;

            //保存加速度校准参数到FLASH中
            ParamUpdateData(PARAM_ACC_OFFSET_X, &acc.cali.offset.x);
            ParamUpdateData(PARAM_ACC_OFFSET_Y, &acc.cali.offset.y);
            ParamUpdateData(PARAM_ACC_OFFSET_Z, &acc.cali.offset.z);
            ParamUpdateData(PARAM_ACC_SCALE_X, &acc.cali.scale.x);
            ParamUpdateData(PARAM_ACC_SCALE_Y, &acc.cali.scale.y);
            ParamUpdateData(PARAM_ACC_SCALE_Z, &acc.cali.scale.z);
            printf("\r\n AccCalibration DOWM \r\n");
            for (uint8_t i = 0; i < 6; i++)
            {
                orientationCaliFlag[i] = 0;
                samples[i].x = 0;
                samples[i].y = 0;
                samples[i].z = 0;
            }
                acc.cali.step = 0;
                samples_count = 0;
        }
    }


}

/**********************************************************************************************************
*函 数 名: ImuLevelCalibration
*功能说明: IMU传感器的水平校准（安装误差），主要读取静止时的加速度数据并求平均值，得到校准角度值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuLevelCalibration(void)
{
    const int16_t CALIBRATING_ACC_LEVEL_CYCLES = 300;
    static float acc_sum[3] = {0, 0, 0};
    Vector3f_t accAverage;
    Vector3f_t caliTemp;
    static int16_t count = 0;

    printf("\r\n    count:    %d    \r\n",count);
    printf("\r\n    acc_sum[0] =  %f  acc_sum[0] =  %f  acc_sum[0] =  %f    \r\n",acc_sum[0],acc_sum[1],acc_sum[2]);
    printf("\r\n    accAverage.x = %f  accAverage.x = %f  accAverage.x = %f  \r\n",accAverage.x,accAverage.y,accAverage.z);

    if(count == 0)
    {
        acc.levelCali.scale.x = 0;
        acc.levelCali.scale.y = 0;
        acc.levelCali.scale.z = 0;
    }
    else //if(GetPlaceStatus() == STATIC)
    {
        acc_sum[0] += acc.data.x;
        acc_sum[1] += acc.data.y;
        acc_sum[2] += acc.data.z;
    }
    count++;

    acc.levelCali.step = 1;

    if(count == CALIBRATING_ACC_LEVEL_CYCLES)
    {
        accAverage.x = acc_sum[0] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
        accAverage.y = acc_sum[1] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
        accAverage.z = acc_sum[2] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
        acc_sum[0] = 0;
        acc_sum[1] = 0;
        acc_sum[2] = 0;
        count = 0;
        acc.levelCali.step = 2;

        if(abs(Degrees(caliTemp.x)) < 10 && abs(Degrees(caliTemp.y)) < 10)
        {
            acc.levelCali.success = 1;

            acc.levelCali.scale.x = -caliTemp.x;
            acc.levelCali.scale.y = -caliTemp.y;
            acc.levelCali.scale.z = 0;

            //保存IMU安装误差校准参数
            ParamUpdateData(PARAM_IMU_LEVEL_X, &acc.levelCali.scale.x);
            ParamUpdateData(PARAM_IMU_LEVEL_Y, &acc.levelCali.scale.y);
            ParamUpdateData(PARAM_IMU_LEVEL_Z, &acc.levelCali.scale.z);
            printf("\r\n    ImuLevelCalibration work down    \r\n");
        }
        else
        {
            acc.levelCali.success = 0;
            printf("\r\n    ImuLevelCalibration work false    \r\n");
        
        }
        acc.levelCali.step = 0;
    }
}

/**********************************************************************************************************
*函 数 名: AccGetData
*功能说明: 获取经过处理后的加速度数据
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t AccGetData(void)
{
    return acc.data;
}

/**********************************************************************************************************
*函 数 名: GetAccMag
*功能说明: 获取加速度数据模值
*形    参: 无
*返 回 值: 模值
**********************************************************************************************************/
float GetAccMag(void)
{
    return acc.mag;
}

/**********************************************************************************************************
*函 数 名: GetLevelCalibraData
*功能说明: 获取IMU安装误差校准参数
*形    参: 无
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetLevelCalibraData(void)
{
    return acc.levelCali.scale;
}
