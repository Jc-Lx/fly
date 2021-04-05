#include "magnetometer.h"



//地球表面赤道上的磁场强度在0.29～0.40高斯之间,两极处的强度略大,地磁北极约0.61高斯,南极约0.68高斯
//不同地方磁场强度有所区别，所以每次校准磁力计时要把当地磁场强度的大概值保存下来

MAGNETOMETER_t mag;
extern enum ORIENTATION_STATUS orientationStatus;

/**********************************************************************************************************
*函 数 名: MagCaliDataInit
*功能说明: 磁力计校准参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MagCaliDataInit(void)
{
    ParamGetData(PARAM_MAG_OFFSET_X, &mag.cali.offset.x, 4);
    ParamGetData(PARAM_MAG_OFFSET_Y, &mag.cali.offset.y, 4);
    ParamGetData(PARAM_MAG_OFFSET_Z, &mag.cali.offset.z, 4);
    ParamGetData(PARAM_MAG_SCALE_X, &mag.cali.scale.x, 4);
    ParamGetData(PARAM_MAG_SCALE_Y, &mag.cali.scale.y, 4);
    ParamGetData(PARAM_MAG_SCALE_Z, &mag.cali.scale.z, 4);
    ParamGetData(PARAM_MAG_EARTH_MAG, &mag.earthMag, 4);

    if(isnan(mag.cali.offset.x) || isnan(mag.cali.offset.y) || isnan(mag.cali.offset.z) || \
            isnan(mag.cali.scale.x) || isnan(mag.cali.scale.y) || isnan(mag.cali.scale.z) ||    \
            abs(mag.cali.scale.x) < 0.3f || abs(mag.cali.scale.x) > 2 || \
            abs(mag.cali.scale.y) < 0.3f || abs(mag.cali.scale.y) > 2 || \
            abs(mag.cali.scale.z) < 0.3f || abs(mag.cali.scale.z) > 2 || \
            mag.cali.scale.x == 0 || mag.cali.scale.y == 0 || mag.cali.scale.z == 0)
    {
        mag.cali.offset.x = 0;
        mag.cali.offset.y = 0;
        mag.cali.offset.z = 0;
        mag.cali.scale.x = 1;
        mag.cali.scale.y = 1;
        mag.cali.scale.z = 1;
        mag.earthMag = 0.4;
    }
    printf("\r\n  mag.cali.offset :  x:%f    y:%f    z:%f    \r\n",mag.cali.offset.x,mag.cali.offset.y,mag.cali.offset.z);
    printf("\r\n  mag.cali.scale :  x:%f    y:%f    z:%f    \r\n",mag.cali.scale.x,mag.cali.scale.y,mag.cali.scale.z);
    printf("\r\n  mag.earthMag:   %f    \r\n",mag.earthMag); 
}

/**********************************************************************************************************
*函 数 名: MagDataPreTreat
*功能说明: 磁力计数据预处理
*形    参: 磁力计原始数据 磁力计预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void MagDataPreTreat(void)
{
    Vector3f_t magRaw;

    //获取磁力计传感器采样值
    MagRaw_Read(&magRaw);
    
    //磁力计数据校准
    mag.data.x = (magRaw.x - mag.cali.offset.x) * mag.cali.scale.x;
    mag.data.y = (magRaw.y - mag.cali.offset.y) * mag.cali.scale.y;
    mag.data.z = (magRaw.z - mag.cali.offset.z) * mag.cali.scale.z;

    //计算磁场强度模值，用于判断周边是否存在磁场干扰（正常值为1）
    mag.mag = mag.mag * 0.99f + Pythagorous3(mag.data.x, mag.data.y, mag.data.z) / mag.earthMag * 0.01f;
}

/**********************************************************************************************************
*函 数 名: MagCalibration
*功能说明: 磁力计校准
*形    参: 磁力计原始数据
*返 回 值: 无
**********************************************************************************************************/
void MagCalibration(void)
{
    static Vector3f_t samples[6];
    static uint32_t cnt_m=0;
    static float cali_rotate_angle = 0;
    static Vector3f_t new_offset;
    static Vector3f_t new_scale;
    static float earthMag = 0;
    Vector3f_t magRaw;

    //计算时间间隔，用于积分
    static uint64_t previousT;
    float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
    previousT = GetSysTimeUs();

    printf("\r\n    mag.cali.step:    %d    \r\n",mag.cali.step);
    printf("\r\n    cali_rotate_angle:    %f    \r\n",cali_rotate_angle);
    printf("\r\n    samples[MaxX]:    %f    %f    %f    \r\n",samples[MaxX].x,samples[MaxX].y,samples[MaxX].z);
    printf("\r\n    samples[MinX]:    %f    %f    %f    \r\n",samples[MinX].x,samples[MinX].y,samples[MinX].z);
    printf("\r\n    samples[MaxY]:    %f    %f    %f    \r\n",samples[MaxY].x,samples[MaxY].y,samples[MaxY].z);
    printf("\r\n    samples[MinY]:    %f    %f    %f    \r\n",samples[MinY].x,samples[MinY].y,samples[MinY].z);
    printf("\r\n    samples[MaxZ]:    %f    %f    %f    \r\n",samples[MaxZ].x,samples[MaxZ].y,samples[MaxZ].z);
    printf("\r\n    samples[MinZ]:    %f    %f    %f    \r\n",samples[MinZ].x,samples[MinZ].y,samples[MinZ].z);

    printf("\r\n  mag.cali.offset :  x:%f    y:%f    z:%f    \r\n",mag.cali.offset.x,mag.cali.offset.y,mag.cali.offset.z);
    printf("\r\n  mag.cali.scale :  x:%f    y:%f    z:%f    \r\n",mag.cali.scale.x,mag.cali.scale.y,mag.cali.scale.z); 
     
    //读取罗盘数据
    MagRaw_Read(&magRaw);
    
    //校准分两个阶段：1.水平旋转 2.机头朝上或朝下然后水平旋转
    //两个阶段分别对飞机的z轴和x轴陀螺仪数据进行积分，记录旋转过的角度
    if((GetImuOrientation() == ORIENTATION_UP) && (mag.cali.step ==1))
    {
        cali_rotate_angle += GyroGetData().z * deltaT;
    }
    if((GetImuOrientation() == ORIENTATION_BACK) && (mag.cali.step == 2))
    {
        cali_rotate_angle += GyroGetData().y * deltaT;
    }
    if(cnt_m == 0)
    {
        mag.cali.step = 1;
        cali_rotate_angle = 0;
        //初始化磁场强度模值
        earthMag = Pythagorous3(magRaw.x, magRaw.y, magRaw.z);
        //初始化采样点
        samples[MaxX] = samples[MinX] = magRaw;
        samples[MaxY] = samples[MinY] = magRaw;
        samples[MaxZ] = samples[MinZ] = magRaw;
        cnt_m++;       
    }
    else 
    {
            //实时计算磁场强度模值
            earthMag = earthMag * 0.998f + Pythagorous3(magRaw.x, magRaw.y, magRaw.z) * 0.002f;

            //找到每个轴的最大最小值，并对采样值进行一阶低通滤波
            if(Pythagorous3(magRaw.x, magRaw.y, magRaw.z) < earthMag * 1.5f)
            {
                //找到每个轴的最大最小值，并对采样值进行一阶低通滤波
                if(magRaw.x > samples[MaxX].x)
                {
                    LowPassFilter1st(&samples[MaxX], magRaw, 0.3);
                }
                if(magRaw.x < samples[MinX].x)
                {
                    LowPassFilter1st(&samples[MinX], magRaw, 0.3);
                }
                if(magRaw.y > samples[MaxY].y)
                {
                    LowPassFilter1st(&samples[MaxY], magRaw, 0.3);
                }
                if(magRaw.y < samples[MinY].y)
                {
                    LowPassFilter1st(&samples[MinY], magRaw, 0.3);
                }
                if(magRaw.z > samples[MaxZ].z)
                {
                    LowPassFilter1st(&samples[MaxZ], magRaw, 0.3);
                }
                if(magRaw.z < samples[MinZ].z)
                {
                    LowPassFilter1st(&samples[MinZ], magRaw, 0.3);
                }
            }
            else
            {
                earthMag = earthMag;
            }
            //水平旋转一圈
            if(mag.cali.step == 1 && abs(cali_rotate_angle) > 360)
            {
                mag.cali.step = 2;
                cali_rotate_angle  = 0;
            }
            //竖直旋转一圈
            if(mag.cali.step == 2 && abs(cali_rotate_angle ) > 360)
            {
                mag.cali.step = 3;
                cali_rotate_angle  = 0;
                earthMag = 0;

                //计算当地地磁场强度模值均值
                for(u8 i=0; i<3; i++)
                {
                    earthMag += Pythagorous3((samples[i*2].x - samples[i*2+1].x) * 0.5f,
                                             (samples[i*2].y - samples[i*2+1].y) * 0.5f,
                                             (samples[i*2].z - samples[i*2+1].z) * 0.5f);
                }
                earthMag /= 3;

                //计算方程解初值
                float initBeta[6];
                initBeta[0] = (samples[MaxX].x + samples[MinX].x) * 0.5f;
                initBeta[1] = (samples[MaxY].y + samples[MinY].y) * 0.5f;
                initBeta[2] = (samples[MaxZ].z + samples[MinZ].z) * 0.5f;
                initBeta[3] = 1 / earthMag;
                initBeta[4] = 1 / earthMag;
                initBeta[5] = 1 / earthMag;

                //LM法求解传感器误差方程最优解
                LevenbergMarquardt(samples, &new_offset, &new_scale, initBeta, earthMag);

                //判断校准参数是否正常
                if(isnan(new_scale.x) || isnan(new_scale.y) || isnan(new_scale.z))
                {
                    mag.cali.success = false;
                }
                else if(fabsf(new_scale.x-1.0f) > 0.8f || fabsf(new_scale.y-1.0f) > 0.8f || fabsf(new_scale.z-1.0f) > 0.8f)
                {
                    mag.cali.success = false;
                }
                else if(fabsf(new_offset.x) > (earthMag * 2) || fabsf(new_offset.y) > (earthMag * 2) || fabsf(new_offset.z) > (earthMag * 2))
                {
                    mag.cali.success = false;
                }
                else
                {
                    mag.cali.success = true;
                }

                if(mag.cali.success&&(mag.cali.step == 3))
                {
                    mag.cali.offset = new_offset;
                    mag.cali.scale = new_scale;
                    mag.earthMag = earthMag;

                    //保存校准参数
                    ParamUpdateData(PARAM_MAG_OFFSET_X, &mag.cali.offset.x);
                    ParamUpdateData(PARAM_MAG_OFFSET_Y, &mag.cali.offset.y);
                    ParamUpdateData(PARAM_MAG_OFFSET_Z, &mag.cali.offset.z);
                    ParamUpdateData(PARAM_MAG_SCALE_X, &mag.cali.scale.x);
                    ParamUpdateData(PARAM_MAG_SCALE_Y, &mag.cali.scale.y);
                    ParamUpdateData(PARAM_MAG_SCALE_Z, &mag.cali.scale.z);
                    ParamUpdateData(PARAM_MAG_EARTH_MAG, &mag.earthMag);
                    printf("\r\n    MagCalibration work!   \r\n");
                    mag.cali.step = 0;
                    earthMag = 0;
                    cnt_m = 0;

                }
                else
                {
                    printf("\r\n    MagCalibration false!   \r\n");
                }
        }
    }
}

/**********************************************************************************************************
*函 数 名: MagGetData
*功能说明: 获取经过处理后的磁力计数据
*形    参: 无
*返 回 值: 磁力计数据
**********************************************************************************************************/
Vector3f_t MagGetData(void)
{
    return mag.data;
}

/**********************************************************************************************************
*函 数 名: GetAccMag
*功能说明: 获取磁力计数据模值
*形    参: 无
*返 回 值: 模值
**********************************************************************************************************/
float GetMagMag(void)
{
    return mag.mag;
}
