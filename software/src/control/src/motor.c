#include "motor.h"

//油门行程为[0:2000]
#define MINTHROTTLE	    1100                     //最小油门值           
#define MAXTHROTTLE 	1900                    //最大油门值

//机型选择
#define motorType quadX

static int16_t motorPWM[8];
uint8_t escCaliFlag = 0;

//四轴X型
const MOTOR_TYPE_t quadX =
{
    .motorNum   = 4,                            //电机数量
    .motorMixer =
    {
        { 1.0f,  1.0f, -1.0f, -1.0f },          //后右
        { 1.0f, -1.0f, -1.0f,  1.0f },          //前右
        { 1.0f,  1.0f,  1.0f,  1.0f },          //后左
        { 1.0f, -1.0f,  1.0f, -1.0f },          //前左
    }
};

/**********************************************************************************************************
*函 数 名: MotorInit
*功能说明: 电机控制初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MotorInit(void)
{
    ParamGetData(PARAM_ESC_CALI_FLAG, &escCaliFlag, 1);
}

/**********************************************************************************************************
*函 数 名: MotorControl
*功能说明: 电机控制
*形    参: 横滚控制量 俯仰控制量 偏航控制量 油门控制量
*返 回 值: 无
**********************************************************************************************************/
void MotorControl(int16_t pitch, int16_t roll, int16_t yaw, int16_t throttle)
{
    int16_t maxMotorValue;
    static int16_t motorResetPWM[8];

    //电机动力分配
    for(u8 i=0; i<motorType.motorNum; i++)
    {
        motorPWM[i] = throttle * motorType.motorMixer[i].throttle  +
                      pitch    * motorType.motorMixer[i].pitch     +
                      roll     * motorType.motorMixer[i].roll      +
                      yaw      * motorType.motorMixer[i].yaw;

        motorPWM[i] = ConstrainInt16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
    }

    //防止电机输出饱和
    maxMotorValue = motorPWM[0];
    // for (uint8_t i=1; i<motorType.motorNum; i++)
    // {
    //     if(motorPWM[i] > maxMotorValue)
    //         maxMotorValue = motorPWM[i];
    // }
    // for (uint8_t i=0; i<motorType.motorNum; i++)
    // {
    //     if (maxMotorValue > MAXTHROTTLE)
    //         motorPWM[i] -= maxMotorValue - MAXTHROTTLE;
    //     // 限制电机输出的最大最小值
    //     motorPWM[i] = ConstrainInt16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
    // }

    //判断飞控锁定状态，并输出电机控制量
    if(GetArmedStatus() == ARMED )
    {
        for (uint8_t i=0; i<motorType.motorNum; i++)
        {
            //输出PWM
            MotorPWMSet(i+1, motorPWM[i]);
            //记录当前PWM值
            motorResetPWM[i] = motorPWM[i];
        }
    }
    else
    {
        for (uint8_t i=0; i<motorType.motorNum; i++)
        {
            //电机逐步减速，防止电机刹车引发射桨
            motorResetPWM[i] -= motorResetPWM[i] * 0.003f;
            MotorPWMSet(i+1, motorResetPWM[i]);
        }
    }

    printf("\r\n  motorPWM    1:%d    2:%d    3:%d    4:%d  \r\n",motorPWM[0],motorPWM[1],motorPWM[2],motorPWM[3]);
}
/**********************************************************************************************************
*函 数 名: MotorStop
*功能说明: 所有电机停转
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MotorStop(void)
{
    for (uint8_t i=0; i<motorType.motorNum; i++)
    {
        MotorPWMSet(i+1, 0);
    }
}
