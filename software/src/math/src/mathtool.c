#include "mathTool.h"

/**********************************************************************************************************
*函 数 名: SafeArcsin
*功能说明: 带有安全区域的反正弦函数
*形    参: 输入值
*返 回 值: 反正弦值
**********************************************************************************************************/
float SafeArcsin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}

/**********************************************************************************************************
*函 数 名: constrainFloat
*功能说明: 浮点型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
float ConstrainFloat(float amt, float low, float high)
{
    if (isnan(amt))
    {
        return (low+high)*0.5f;
    }
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ConstrainInt16
*功能说明: 16位有符号整型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
int16_t ConstrainInt16(int16_t amt, int16_t low, int16_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ConstrainUint16
*功能说明: 16位无符号整型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
uint16_t ConstrainUint16(uint16_t amt, uint16_t low, uint16_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ConstrainInt32
*功能说明: 32位整型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
int32_t ConstrainInt32(int32_t amt, int32_t low, int32_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ApplyDeadbandInt
*功能说明: 应用死区
*形    参: 输入值 死区大小
*返 回 值: 处理后的值
**********************************************************************************************************/
int32_t ApplyDeadbandInt(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband)
    {
        value = 0;
    }
    else if (value > 0)
    {
        value -= deadband;
    }
    else if (value < 0)
    {
        value += deadband;
    }
    return value;
}

/**********************************************************************************************************
*函 数 名: ApplyDeadbandFloat
*功能说明: 应用死区
*形    参: 输入值 死区大小
*返 回 值: 处理后的值
**********************************************************************************************************/
float ApplyDeadbandFloat(float value, float deadband)
{
    if (abs(value) < deadband)
    {
        value = 0;
    }
    else if (value > 0)
    {
        value -= deadband;
    }
    else if (value < 0)
    {
        value += deadband;
    }
    return value;
}

/**********************************************************************************************************
*函 数 名: Radians
*功能说明: 角度转弧度
*形    参: 角度值
*返 回 值: 弧度值
**********************************************************************************************************/
float Radians(float deg)
{
    return deg * DEG_TO_RAD;
}

/**********************************************************************************************************
*函 数 名: Radians
*功能说明: 弧度转角度
*形    参: 弧度值
*返 回 值: 角度值
**********************************************************************************************************/
float Degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

/**********************************************************************************************************
*函 数 名: Sq
*功能说明: 求平方
*形    参: 输入值
*返 回 值: 输入值的平方
**********************************************************************************************************/
float Sq(float v)
{
    return v*v;
}

/**********************************************************************************************************
*函 数 名: Pythagorous2
*功能说明: 2维向量长度
*形    参: 变量a 变量b
*返 回 值: 模值大小
**********************************************************************************************************/
float Pythagorous2(float a, float b)
{
    return sqrtf(Sq(a)+Sq(b));
}

/**********************************************************************************************************
*函 数 名: Pythagorous3
*功能说明: 3维向量长度
*形    参: 变量a 变量b 变量c
*返 回 值: 模值大小
**********************************************************************************************************/
float Pythagorous3(float a, float b, float c)
{
    return sqrtf(Sq(a)+Sq(b)+Sq(c));
}

/**********************************************************************************************************
*函 数 名: Pythagorous4
*功能说明: 4维向量长度
*形    参: 变量a 变量b 变量c 变量d
*返 回 值: 模值大小
**********************************************************************************************************/
float Pythagorous4(float a, float b, float c, float d)
{
    return sqrtf(Sq(a)+Sq(b)+Sq(c)+Sq(d));
}

/**********************************************************************************************************
*函 数 名: WrapDegree360
*功能说明: 角度限幅为0-360
*形    参: 角度值
*返 回 值: 限幅后的角度值
**********************************************************************************************************/
float WrapDegree360(float angle)
{
    if(angle >= 0)
        angle -= 180;
    if(angle < 179.5f)
        angle += 180;
    return angle;
}

/**********************************************************************************************************
*函 数 名: GetRandom
*功能说明: 获取一个随机数
*形    参: 无
*返 回 值: 随机数
**********************************************************************************************************/
// int32_t GetRandom(void)
// {
//     srand(GetSysTimeUs());
//     return rand();
// }
