#include "pid.h"
#include "maths.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * PID��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

void pidInit(PidObject* pid, float kp, float ki, float kd, float iLimit, float outputLimit, float dt, bool enableDFilter, float cutoffFreq)
{
	pid->desired   = 0;
	pid->error     = 0;
	pid->prevError = 0;	
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->kp 	   = kp;
	pid->ki        = ki;
	pid->kd        = kd;
	pid->outP      = 0;
	pid->outI      = 0;
	pid->outD      = 0;
	pid->iLimit    = iLimit;
	pid->outputLimit = outputLimit;
	pid->dt        = dt;
	pid->enableDFilter = enableDFilter;
	if (pid->enableDFilter)
	{
		biquadFilterInitLPF(&pid->dFilter, (1.0f/dt), cutoffFreq);
	}
}

float pidUpdate(PidObject* pid, float error)//error = desired - measured 
{
	float output = 0.0f;
	
	pid->error = error;

	pid->integ += pid->error * pid->dt;
	
	//�����޷�
	if (pid->iLimit != 0)
	{
		pid->integ = constrainf(pid->integ, -pid->iLimit, pid->iLimit);
	}
	
	pid->deriv = (pid->error - pid->prevError) / pid->dt;
	if (pid->enableDFilter)
	{
		pid->deriv = biquadFilterApply(&pid->dFilter, pid->deriv);
	}
	
	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP + pid->outI + pid->outD;
	
	//����޷�
	if (pid->outputLimit != 0)
	{
		output = constrainf(output, -pid->outputLimit, pid->outputLimit);
	}
	
	pid->prevError = pid->error;

	return output;
}

void pidReset(PidObject* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}

void pidResetIntegral(PidObject* pid)
{
	pid->integ     = 0;
}

void pidSetIntegral(PidObject* pid, float integ)
{
	pid->integ     = integ;
}
