#include "rc.h"
 

#define MINCHECK        1100
#define MIDCHECK        1500
#define MAXCHECK        1900

RCCOMMAND_t rcCommand;
static uint8_t  armedCheckFlag = 0;
static uint8_t RCSignal = 0;

/**********************************************************************************************************
*函 数 名: RcInit
*功能说明: 遥控相关功能初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RcInit(void)
{
    //遥控通道数据初始化
    rcCommand.pitch    = 1500;
    rcCommand.roll     = 1500;
    rcCommand.yaw      = 1500;
    rcCommand.throttle = 1100;

    //初始化飞行模式为自动模式
    SetFlightMode(MANUAL);
}

/**********************************************************************************************************
*函 数 名: RcCheck
*功能说明: 遥控器各项数据以及失控检测
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RcCheck(void)
{
    //遥控器连接信号检测
    RcCheckFailsafe();

    //遥控器摇杆命令检测,内八字解锁,上锁
    RcCheckSticks();

    //摇杆原始数据更新
    PPM_Decode();

    //遥控器摇杆命令更新
    RcCommandUpdate();

}

/**********************************************************************************************************
*函 数 名: static int16_t RCDataLimit(int16_t rcdata)
*功能说明: 摇杆控制命令限幅(1100-1900)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static int16_t RCDataLimit(int16_t rcdata)
{
    if((rcdata>1100) && (rcdata<1900))
    {
        rcdata = rcdata;
    }else if(rcdata<1100)
    {
        rcdata = 1100;
    }else if(rcdata>1900)
    {
        rcdata = 1900;
    }
    return rcdata;
}

/**********************************************************************************************************
*函 数 名: RcCommandUpdate
*功能说明: 摇杆控制命令更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcCommandUpdate(void)
{
    RCDATA_t  rcrawData;
    // 获取摇杆原始数据
    rcrawData = GetRawRcData();

    if((RCSignal == 1)&&GetArmedStatus())
    {
        rcrawData.pitch = RCDataLimit(rcrawData.pitch);
        rcCommand.pitch = rcrawData.pitch - 1500;
        

        rcrawData.roll  = RCDataLimit(rcrawData.roll);
        rcCommand.roll  = rcrawData.roll  - 1500;

        rcrawData.yaw   = RCDataLimit(rcrawData.yaw);
        rcCommand.yaw   = rcrawData.yaw   - 1500;

        rcCommand.throttle = RCDataLimit(rcrawData.throttle);
    }else
    {
        rcCommand.pitch = 0;
        rcCommand.roll  = 0;
        rcCommand.yaw   = 0;
        rcCommand.throttle = 0;
    }
    // printf("\r\n    rcCommand:    pitch:%d    roll:%d    yaw:%d    throttle:%d    \r\n",rcCommand.pitch,rcCommand.roll,rcCommand.yaw,rcCommand.throttle);
}

/**********************************************************************************************************
*函 数 名: GetRcCommad
*功能说明: 获取摇杆控制命令
*形    参: 无
*返 回 值: 摇杆命令
**********************************************************************************************************/
RCCOMMAND_t GetRcCommad(void)
{
    return rcCommand;
}

/**********************************************************************************************************
*函 数 名: RcCheckSticks
*功能说明: 检查摇杆位置，判断解锁动作等
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcCheckSticks(void)
{
    static uint32_t armedCheckTime = 0;
    static uint32_t armedDisarmedTime = 0;

    RCDATA_t  rcData;
    // 获取摇杆原始数据
    rcData = GetRawRcData();
    // printf("\r\n    rcrawData:    pitch:%d    roll:%d    yaw:%d    throttle:%d    \r\n",rcData.pitch,rcData.roll,rcData.yaw,rcData.throttle);

    //摇杆内八字解锁,同时也可上锁，即使在飞行中，也可通过内八强制上锁
    if((rcData.pitch < MINCHECK ) && (rcData.roll > MAXCHECK ) && ( rcData.yaw < MINCHECK ) && (rcData.throttle < MINCHECK ))
    {
        //上锁3秒后才可再次解锁
        if(GetArmedStatus() == DISARMED && (GetSysTimeMs() - armedDisarmedTime) > 3000)
        {
            if(GetSysTimeMs() - armedCheckTime > 1500)
            {
                SetArmedStatus(ARMED);
                //解锁检查标志置1，用于判断摇杆是否已回中，防止解锁后因为摇杆位置没变化导致自动上锁
                armedCheckFlag = 1;
            }
        }
        else if((GetArmedStatus() == ARMED) && (armedCheckFlag == 0))
        {
            //持续3秒后强制上锁
            if(GetSysTimeMs() - armedCheckTime > 3000)
            {
                SetArmedStatus(DISARMED);
                //记录上锁时间
                armedDisarmedTime = GetSysTimeMs();
            }
        }
    }
    else
    {
        armedCheckTime = GetSysTimeMs();
    }
    //摇杆若回中，则重置解锁标志位，此时可以再次通过外八操作将飞机上锁
    if(rcData.pitch > MINCHECK + 100)
    {
        armedCheckFlag = 0;
    }
}

/**********************************************************************************************************
*函 数 名: RcCheckFailsafe
*功能说明: 失控保护检测，主要分两种方式，一是接收不到遥控数据，二是遥控数据出现特定数值（遥控器上一般可设的失控保护）
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void RcCheckFailsafe(void)
{
    RCDATA_t rcrawdata;
    rcrawdata = GetRawRcData();

    // 判断各通道数据是否是无信号输出值
    if( (rcrawdata.pitch > 3500) || (rcrawdata.roll > 3500) || (rcrawdata.yaw > 3500) || (rcrawdata.throttle > 3500))
    {
        RCSignal = 0;
    }else 
    {
        RCSignal = 1;
    }
    
}
