#include "flightStatus.h"

FLIGHT_STATUS_t flyStatus;

/**********************************************************************************************************
*函 数 名: SetArmedStatus
*功能说明: 设置飞控锁定状态
*形    参: 状态
*返 回 值: 无
**********************************************************************************************************/
bool SetArmedStatus(uint8_t status)
{
    if(status == DISARMED)	//上锁
    {

        flyStatus.armed = DISARMED;

        return true;
    }
    else if(status == ARMED)	//解锁
    {
        if(flyStatus.armed == ARMED)
            return true;

        // 解锁检查
        // if(!ArmedCheck())
        //     return false;

        flyStatus.armed = ARMED;

        return true;
    }
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: GetArmedStatus
*功能说明: 获取飞控锁定状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetArmedStatus(void)
{
    return flyStatus.armed;
}

/**********************************************************************************************************
*函 数 名: PlaceStausCheck
*功能说明: 飞行器放置状态检测：静止或运动
*形    参: 角速度
*返 回 值: 无
**********************************************************************************************************/
void PlaceStausCheck(Vector3f_t gyro)
{
    Vector3f_t gyroDiff;
    static Vector3f_t lastGyro;
    static float threshold = 3.0f;
    static uint16_t checkNum = 0;
    static int16_t count = 0;

    gyroDiff.x = gyro.x - lastGyro.x;
    gyroDiff.y = gyro.y - lastGyro.y;
    gyroDiff.z = gyro.z - lastGyro.z;
    lastGyro = gyro;

    if(count < 30)
    {
        count++;
        //陀螺仪数值变化大于阈值
        if(abs(gyroDiff.x) > threshold || abs(gyroDiff.y) > threshold || abs(gyroDiff.z) > threshold)
        {
            checkNum++;
        }
    }
    else
    {
        //陀螺仪数据抖动次数大于一定值时认为飞机不处于静止状态
        if(checkNum > 10)
            flyStatus.placement = MOTIONAL;    
        else
            flyStatus.placement = STATIC;
        checkNum = 0;
        count = 0;
    }
}

/**********************************************************************************************************
*函 数 名: GetPlaceStatus
*功能说明: 获取飞行器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetPlaceStatus(void)
{
    return flyStatus.placement;
}

/**********************************************************************************************************
*函 数 名: SetFlightMode
*功能说明: 设置飞行模式
*形    参: 模式
*返 回 值: 无
**********************************************************************************************************/
void SetFlightMode(uint8_t mode)
{
    switch(mode)
    {
    case    MANUAL:
        flyStatus.mode = MANUAL;			//手动模式（自稳）
        break;
    case    SEMIAUTO:
        flyStatus.mode = SEMIAUTO;			//半自动模式（定高）
        break;
    case    AUTO:
        flyStatus.mode = AUTO;				//自动模式（定点）
        break;
    case    AUTOTAKEOFF:
        //flyStatus.mode = AUTOTAKEOFF;		//自动起飞
        break;
    case    AUTOLAND:
        if(flyStatus.armed == ARMED)
            flyStatus.mode = AUTOLAND;		//自动降落
        break;
    case    RETURNTOHOME:
        if(flyStatus.armed == ARMED)
            flyStatus.mode = RETURNTOHOME;	//自动返航
        break;
    case    AUTOCIRCLE:
        //flyStatus.mode = AUTOCIRCLE;		//自动环绕
        break;
    case    AUTOPILOT:
        flyStatus.mode = AUTOPILOT;		//自动航线
        break;
    case    FOLLOWME:
        //flyStatus.mode = FOLLOWME;		//自动跟随
        break;
    case    0xFF:
        break;
    default:
        flyStatus.mode = AUTO;
        break;
    }
}

/**********************************************************************************************************
*函 数 名: GetFlightMode
*功能说明: 获取当前的飞行模式
*形    参: 无
*返 回 值: 模式
**********************************************************************************************************/
uint8_t GetFlightMode(void)
{
    return flyStatus.mode;
}
