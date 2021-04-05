#include "sensordata.h"
#include "mpu9250.h"
#include "accelerometer.h"

enum ORIENTATION_STATUS orientationStatus;

/**********************************************************************************************************
*函 数 名: ImuOrientationDetect
*功能说明: 检测传感器放置方向
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuOrientationDetect(Vector3f_t accrawdata)
{
    const float CONSTANTS_ONE_G = 1;
    const float accel_err_thr = 0.40;

    Vector3f_t  acc;
    //读取加速度数据
    //acc = AccGetData();
    acc = accrawdata;

    // [ g, 0, 0 ]
    if (fabsf(acc.x - CONSTANTS_ONE_G) < accel_err_thr && 
            fabsf(acc.y) < accel_err_thr && 
            fabsf(acc.z) < accel_err_thr )
    {
        orientationStatus = ORIENTATION_RIGHT;
        printf("\r\n  RIGHT  \r\n");
    }
    // [ -g, 0, 0 ]
    if (fabsf(acc.x + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_LEFT;
        printf("\r\n  LEFT  \r\n");
    }
    // [ 0, g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_BACK;
        printf("\r\n  BACK  \r\n");
    }
    // [ 0, -g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_FRONT;
        printf("\r\n  FRONT  \r\n");
    }
    // [ 0, 0, g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z - CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_UP;
        printf("\r\n  UP  \r\n");
    }
    // [ 0, 0, -g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z + CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_DOWN;
        printf("\r\n  DOWN \r\n");
    }

}

/**********************************************************************************************************
*函 数 名: GetImuOrientation
*功能说明: 获取传感器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
enum ORIENTATION_STATUS GetImuOrientation(void)
{
    return orientationStatus;
}
