#include "flightControl.h"


FLIGHTCONTROL_t fc;

static void PIDWriteToFlash(void);
/**********************************************************************************************************
*函 数 名: PIDReset
*功能说明: PID参数重置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void PIDReset(void)
{
    //参数大小和电调型号有较大关系（电机电调的综合响应速度影响了PID参数）
    PID_SetParam(&fc.pid[ROLL_INNER],  3.0, 8.0, 0.2, 250, 35);
    PID_SetParam(&fc.pid[PITCH_INNER], 3.0, 8.0, 0.2, 250, 35);
    PID_SetParam(&fc.pid[YAW_INNER],   5.0, 6.0, 0, 250, 35);
    PID_SetParam(&fc.pid[ROLL_OUTER],  5.0, 0, 0, 0, 0);
    PID_SetParam(&fc.pid[PITCH_OUTER], 4.0, 0, 0, 0, 0);
    PID_SetParam(&fc.pid[YAW_OUTER],   3.0, 0, 0, 0, 0);

    PIDWriteToFlash();
    printf("\r\n    ROLL_INNER:    KP:%f    KI:%f    KD:%f    \r\n",fc.pid[ROLL_INNER].kP,fc.pid[ROLL_INNER].kI,fc.pid[ROLL_INNER].kD);
    printf("\r\n    PITCH_INNER:   KP:%f    KI:%f    KD:%f    \r\n",fc.pid[PITCH_INNER].kP,fc.pid[PITCH_INNER].kI,fc.pid[PITCH_INNER].kD);
    printf("\r\n    YAW_INNER:     KP:%f    KI:%f    KD:%f    \r\n",fc.pid[YAW_INNER].kP,fc.pid[YAW_INNER].kI,fc.pid[YAW_INNER].kD);
    printf("\r\n    ROLL_OUTERKP:%f    PITCH_OUTERKP:%f    YAW_OUTERKP:%f    \r\n",fc.pid[ROLL_OUTER].kP,fc.pid[PITCH_OUTER].kP,fc.pid[YAW_OUTER].kP);

}

/**********************************************************************************************************
*函 数 名: PIDReadFromFlash
*功能说明: 从Flash中读取PID参数
*形    参: 无
*返 回 值: 成功标志位
**********************************************************************************************************/
bool PIDReadFromFlash(void)
{
    bool flag = true;

    //读取姿态PID参数
    ParamGetData(PARAM_PID_ATT_INNER_X_KP, &fc.pid[ROLL_INNER].kP, 4);
    ParamGetData(PARAM_PID_ATT_INNER_X_KI, &fc.pid[ROLL_INNER].kI, 4);
    ParamGetData(PARAM_PID_ATT_INNER_X_KD, &fc.pid[ROLL_INNER].kD, 4);
    ParamGetData(PARAM_PID_ATT_INNER_Y_KP, &fc.pid[PITCH_INNER].kP, 4);
    ParamGetData(PARAM_PID_ATT_INNER_Y_KI, &fc.pid[PITCH_INNER].kI, 4);
    ParamGetData(PARAM_PID_ATT_INNER_Y_KD, &fc.pid[PITCH_INNER].kD, 4);
    ParamGetData(PARAM_PID_ATT_INNER_Z_KP, &fc.pid[YAW_INNER].kP, 4);
    ParamGetData(PARAM_PID_ATT_INNER_Z_KI, &fc.pid[YAW_INNER].kI, 4);
    ParamGetData(PARAM_PID_ATT_INNER_Z_KD, &fc.pid[YAW_INNER].kD, 4);
    ParamGetData(PARAM_PID_ATT_OUTER_X_KP, &fc.pid[ROLL_OUTER].kP, 4);
    ParamGetData(PARAM_PID_ATT_OUTER_Y_KP, &fc.pid[PITCH_OUTER].kP, 4);
    ParamGetData(PARAM_PID_ATT_OUTER_Z_KP, &fc.pid[YAW_OUTER].kP, 4);

    //判断读出的PID参数是否正常
    for(uint8_t i=0; i<PIDNUM; i++)
    {
        if(isnan(fc.pid[i].kP) || fc.pid[i].kP <= 0 || fc.pid[i].kP > 1000)
        {
            flag = false;
        }
    }

    return flag;
}
/**********************************************************************************************************
*函 数 名: PIDWriteToFlash
*功能说明: 将PID参数写入Flash
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void PIDWriteToFlash(void)
{
    //写入姿态PID参数
    ParamUpdateData(PARAM_PID_ATT_INNER_X_KP, &fc.pid[ROLL_INNER].kP);
    ParamUpdateData(PARAM_PID_ATT_INNER_X_KI, &fc.pid[ROLL_INNER].kI);
    ParamUpdateData(PARAM_PID_ATT_INNER_X_KD, &fc.pid[ROLL_INNER].kD);
    ParamUpdateData(PARAM_PID_ATT_INNER_Y_KP, &fc.pid[PITCH_INNER].kP);
    ParamUpdateData(PARAM_PID_ATT_INNER_Y_KI, &fc.pid[PITCH_INNER].kI);
    ParamUpdateData(PARAM_PID_ATT_INNER_Y_KD, &fc.pid[PITCH_INNER].kD);
    ParamUpdateData(PARAM_PID_ATT_INNER_Z_KP, &fc.pid[YAW_INNER].kP);
    ParamUpdateData(PARAM_PID_ATT_INNER_Z_KI, &fc.pid[YAW_INNER].kI);
    ParamUpdateData(PARAM_PID_ATT_INNER_Z_KD, &fc.pid[YAW_INNER].kD);
    ParamUpdateData(PARAM_PID_ATT_OUTER_X_KP, &fc.pid[ROLL_OUTER].kP);
    ParamUpdateData(PARAM_PID_ATT_OUTER_Y_KP, &fc.pid[PITCH_OUTER].kP);
    ParamUpdateData(PARAM_PID_ATT_OUTER_Z_KP, &fc.pid[YAW_OUTER].kP);

}

/**********************************************************************************************************
*函 数 名: FcGetPID
*功能说明: 获取飞控PID参数
*形    参: PID的ID号
*返 回 值: PID结构体
**********************************************************************************************************/
PID_t FcGetPID(uint8_t id)
{
    return fc.pid[id];
}

/**********************************************************************************************************
*函 数 名: FcSetPID
*功能说明: 设置飞控PID参数
*形    参: PID的ID号 PID结构体
*返 回 值: 无
**********************************************************************************************************/
void FcSetPID(uint8_t id, PID_t pid)
{
    fc.pid[id].kP = pid.kP;
    fc.pid[id].kI = pid.kI;
    fc.pid[id].kD = pid.kD;
    
    PIDWriteToFlash();
}

/**********************************************************************************************************
*函 数 名: FlightControlInit
*功能说明: 控制相关参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void FlightControlInit(void)
{
    //电机控制初始化
    MotorInit();
    
    //PID参数初始化
    PIDReset();

    // 从Flash中读取PID参数
    if(!PIDReadFromFlash())
    {
        //若读出的PID参数为非正常值则重置参数并写入Flash中
        PIDReset();
        PIDWriteToFlash();
    }

}
/**********************************************************************************************************
*函 数 名: SetRcTarget
*功能说明: 设置摇杆控制量
*形    参: 摇杆控制量
*返 回 值: 无
**********************************************************************************************************/
void SetRcTarget(RCTARGET_t rcTarget)
{
    fc.rcTarget.roll  = fc.rcTarget.roll * 0.95f + rcTarget.roll * 0.05f;
    fc.rcTarget.pitch = fc.rcTarget.pitch * 0.95f + (-rcTarget.pitch) * 0.05f;
    fc.rcTarget.yaw   = fc.rcTarget.yaw * 0.95f + rcTarget.yaw * 0.05f;
    fc.rcTarget.throttle  = rcTarget.throttle;
    // printf("\r\n fc.rcTarget:  pitch:%f   roll:%f   yaw:%f   throttle:%f    \r\n",fc.rcTarget.pitch,fc.rcTarget.roll,fc.rcTarget.yaw,fc.rcTarget.throttle);
}
/**********************************************************************************************************
*函 数 名: AttitudeInnerControl
*功能说明: 姿态内环控制
*形    参: 角速度测量值 运行时间间隔
*返 回 值: 姿态内环控制量
**********************************************************************************************************/
static Vector3f_t AttitudeInnerControl(Vector3f_t gyro, float deltaT)
{
    static Vector3f_t rateControlOutput;
    static Vector3f_t attInnerTargetLpf;

    attInnerTargetLpf.z = attInnerTargetLpf.z * 0.9f + fc.attInnerTarget.z * 0.1f;

    //保留小数点后两位，减小数据误差对控制器的干扰（貌似没什么用）
    gyro.x = (float)((int32_t)(gyro.x * 100)) * 0.01f;
    gyro.y = (float)((int32_t)(gyro.y * 100)) * 0.01f;
    gyro.z = (float)((int32_t)(gyro.z * 100)) * 0.01f;

    //计算角速度环控制误差：目标角速度 - 实际角速度（低通滤波后的陀螺仪测量值）
    fc.attInnerError.x = fc.attInnerTarget.x - gyro.x;
    fc.attInnerError.y = fc.attInnerTarget.y - gyro.y;
    fc.attInnerError.z = attInnerTargetLpf.z - gyro.z;

    //PID算法，计算出角速度环的控制量
    rateControlOutput.x = PID_GetPID(&fc.pid[PITCH_INNER],  fc.attInnerError.x, deltaT);
    rateControlOutput.y = PID_GetPID(&fc.pid[ROLL_INNER], fc.attInnerError.y, deltaT);
    rateControlOutput.z = PID_GetPID(&fc.pid[YAW_INNER],   fc.attInnerError.z, deltaT);

    //限制俯仰和横滚轴的控制输出量
    rateControlOutput.x = -ConstrainInt32(rateControlOutput.x, -500, +500);
    rateControlOutput.y = ConstrainInt32(rateControlOutput.y, -500, +500);
    //限制偏航轴控制输出量
    rateControlOutput.z = ConstrainInt32(rateControlOutput.z, -300, +300);

    printf("\r\n  fc.attInnerError    x:%f    y:%f    z:%f    \r\n",fc.attInnerError.x,fc.attInnerError.y,fc.attInnerError.z);

    return rateControlOutput;
}
/**********************************************************************************************************
*函 数 名: SetAttInnerCtlTarget
*功能说明: 设置姿态内环控制目标量
*形    参: 控制目标值
*返 回 值: 无
**********************************************************************************************************/
void SetAttInnerCtlTarget(Vector3f_t target)
{
    fc.attInnerTarget = target;
}
/**********************************************************************************************************
*函 数 名: FlightControlInnerLoop
*功能说明: 飞行内环控制，包括姿态内环和高度内环控制
*形    参: 角速度测量值
*返 回 值: 无
**********************************************************************************************************/
void FlightControlInnerLoop(Vector3f_t gyro)
{
    //计算函数运行时间间隔
    static uint64_t previousT;
    float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
    previousT = GetSysTimeUs();

    //姿态内环控制
    fc.attInnerCtlValue = AttitudeInnerControl(gyro, deltaT);

    //高度内环控制
    //在手动模式下（MANUAL），油门直接由摇杆数据控制
    if(GetFlightMode() == MANUAL)   
    fc.altInnerCtlValue = fc.rcTarget.throttle;

    //飞机没有起飞前不计算除油门外其他控制量
    if(fc.altInnerCtlValue < 1300)
    {
        fc.attInnerCtlValue.x = 0;
        fc.attInnerCtlValue.y = 0;
        fc.attInnerCtlValue.z = 0;
    }
    printf("\r\n  fc.altInnerCtlValue   pitch:%f    roll:%f    yaw:%f    throttle:%f    \r\n",fc.attInnerCtlValue.x,fc.attInnerCtlValue.y,fc.attInnerCtlValue.z,fc.altInnerCtlValue);
    //将内环控制量转换为动力电机输出
    MotorControl(fc.attInnerCtlValue.x, fc.attInnerCtlValue.y, fc.attInnerCtlValue.z, fc.altInnerCtlValue);
    
}
/**********************************************************************************************************
*函 数 名: AttitudeOuterControl
*功能说明: 姿态外环控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AttitudeOuterControl(void)
{
    Vector3f_t angle;
    Vector3f_t attOuterCtlValue;
    float 	   yawRate = 0.35f;

    //获取当前飞机的姿态角
    angle = GetCopterAngle();
    // printf("\r\n    angle:    pith:%f    roll:%f    yaw:%f    \r\n",angle.x,angle.y,angle.z);
    // printf("\r\n    fc.rcTarget    x:%f    y:%f    z:%f   \r\n",fc.rcTarget.pitch,fc.rcTarget.roll,fc.rcTarget.yaw);

    //对姿态测量值进行低通滤波，减少数据噪声对控制器的影响
    fc.angleLpf.x = fc.angleLpf.x * 0.92f + angle.x * 0.08f;
    fc.angleLpf.y = fc.angleLpf.y * 0.92f + angle.y * 0.08f;
    fc.angleLpf.z = fc.angleLpf.z * 0.92f + angle.z * 0.08f;

    //保留小数点后两位，减小数据误差对控制器的干扰（貌似没什么用）
    fc.angleLpf.x = (float)((int32_t)(fc.angleLpf.x * 100)) * 0.01f;
    fc.angleLpf.y = (float)((int32_t)(fc.angleLpf.y * 100)) * 0.01f;
    fc.angleLpf.z = (float)((int32_t)(fc.angleLpf.z * 100)) * 0.01f;

    //计算姿态外环控制误差：目标角度 - 实际角度
    fc.attOuterError.x = fc.rcTarget.pitch  - fc.angleLpf.y;
    fc.attOuterError.y = fc.rcTarget.roll  - fc.angleLpf.x;

    //PID算法，计算出姿态外环的控制量，并以一定比例缩放来控制PID参数的数值范围
    attOuterCtlValue.x = PID_GetP(&fc.pid[PITCH_OUTER], fc.attOuterError.x) * 1.0f;
    attOuterCtlValue.y = PID_GetP(&fc.pid[ROLL_OUTER], fc.attOuterError.y) * 1.0f;

    //PID控制输出限幅，目的是限制飞行中最大的运动角速度，单位为°/s
    //同时限制各种位置控制状态下的角速度，提升飞行过程中的控制感观
    attOuterCtlValue.x = ConstrainFloat(attOuterCtlValue.x, -200, 200);
    attOuterCtlValue.y = ConstrainFloat(attOuterCtlValue.y, -200, 200);

    attOuterCtlValue.z = fc.rcTarget.yaw * yawRate;

    // printf("\r\n    fc.attOuterError    x:%f    y:%f    z:%f   \r\n",fc.attOuterError.x,fc.attOuterError.y,fc.attOuterError.z);
    printf("\r\n    attOuterCtlValue    x:%f    y:%f    z:%f   \r\n",attOuterCtlValue.x,attOuterCtlValue.y,attOuterCtlValue.z);
    //将姿态外环控制量作为姿态内环的控制目标
    SetAttInnerCtlTarget(attOuterCtlValue);
}

/**********************************************************************************************************
*函 数 名: SetRcTarget
*功能说明: 获取摇杆控制量
*形    参: 摇杆控制量
*返 回 值: 无
**********************************************************************************************************/
RCTARGET_t GetRcTarget(void)
{
    return fc.rcTarget;
}
