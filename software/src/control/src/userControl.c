#include "userControl.h"

static void ManualControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget);
/**********************************************************************************************************
*函 数 名: UserControl
*功能说明: 用户控制模式下的操控逻辑处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UserControl(void)
{
    uint8_t flightMode;
    RCCOMMAND_t rcCommand;
    RCTARGET_t rcTarget;
    static float rollRate  = (float)MAXANGLE / MAXRCDATA;
    static float pitchRate = (float)MAXANGLE / MAXRCDATA;

    //获取当前飞行模式
    flightMode = GetFlightMode();

    //获取摇杆数据
    rcCommand = GetRcCommad();

    //通用控制部分，将摇杆量转换为横滚俯仰的目标控制角度
    rcTarget.pitch = rcCommand.pitch * pitchRate;
    rcTarget.roll  = rcCommand.roll  * rollRate;

    if(flightMode == MANUAL)
    {
        //手动档（自稳）
        ManualControl(rcCommand, &rcTarget);
    }

    //设置目标控制量
    SetRcTarget(rcTarget);
}

/**********************************************************************************************************
*函 数 名: ManualControl
*功能说明: 手动档，飞机姿态与油门直接由摇杆量控制
*形    参: 摇杆量 控制目标量
*返 回 值: 无
**********************************************************************************************************/
static void ManualControl(RCCOMMAND_t rcCommand, RCTARGET_t* rcTarget)
{
    // static float yawRate  = (float)MAXYAWANGLE / MAXRCDATA;

    //航向控制
    //YawControl(rcCommand, rcTarget);
    rcTarget->yaw = rcCommand.yaw ;

    //摇杆量直接转换为油门值
    rcTarget->throttle = rcCommand.throttle;
}
