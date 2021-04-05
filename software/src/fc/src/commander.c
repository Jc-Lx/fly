#include <math.h>
#include "commander.h"
#include "rx.h"
#include "rc_controls.h"
#include "runtime_config.h"
#include "config.h"
#include "pos_estimator.h"
#include "state_control.h"
#include "beeper.h"
#include "maths.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ��ȡң��������������
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

/*commonder���ò���*/
#define DEAD_BAND			5			//����roll\pitch�е�����ֵ
#define DEAD_BAND_YAW	   	10			//yaw�е�����ֵ

#define MAX_ANGLE_ROLL		(30.0f)		//�������Ƕ�
#define MAX_ANGLE_PITCH  	(30.0f)

#define MAX_RATE_ROLL		(200.0f)	//�ֶ�ģʽ�������
#define MAX_RATE_PITCH		(200.0f)
#define MAX_RATE_YAW     	(200.0f)

#define ALT_HOLD_DEADBAND	50			//��������ң������ֵ
#define POS_HOLD_DEADBAND	30			//���㷽��ң������ֵ

#define CLIMB_RATE_UP		150.0f		//��������������� cm/s
#define CLIMB_RATE_DOWN		110.0f		//��������½����� cm/s

#define LAND_THROTTLE		1200		//ʧ�ؽ�������ֵ

uint16_t altHoldThrottleBase = 1500;	//�������Ż���ֵ
bool setupAltitudeHoldFlag = false;		//��־�����趨���
int16_t rcCommand[4];//[1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

autoLandState_t autoLandState = //�Զ�����
{
	.autoLandActive = 0,
	.autoLandTime = 0,
};	


//ң������ת��Ϊ����
static float rcCommandToRate(int16_t stick, float rate)
{
    return scaleRangef((float) stick, -500, 500, -rate, rate);
}

//ң������ת��Ϊ�Ƕ�
static float rcCommandToAngle(int16_t stick, float maxInclination)
{
    stick = constrain(stick, -500, 500);
    return scaleRangef((float) stick, -500, 500, -maxInclination, maxInclination);
}

//ң�������������̺�ȥ������
static int16_t getAxisRcCommand(int16_t rawData, int16_t deadband)
{
    int16_t stickDeflection;

    stickDeflection = constrain(rawData - RC_MID, -500, 500);
    stickDeflection = applyDeadband(stickDeflection, deadband);
    return stickDeflection;
}

//��λ����ģʽ
static void resetSetpointMode(setpoint_t *setpoint)
{
	setpoint->mode.roll = modeDisable;
	setpoint->mode.pitch = modeDisable;
	setpoint->mode.yaw = modeDisable;
	
	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;
	setpoint->mode.z = modeDisable;
}

//���ö���ģʽ
static void setupAltitudeHold(const state_t *state, setpoint_t *setpoint)
{
	throttleStatus_e throttleStatus = calculateThrottleStatus();//��ȡ��ǰ����ң��λ��

	if (throttleStatus == THROTTLE_LOW) //������ŵ�λʱ�л��Ķ��ߣ���������Ϊ1500
	{
		altHoldThrottleBase = 1500;
		stateControlSetVelocityZPIDIntegration(-400);//����Z���ٶȿ���������Ϊ-400��ֹ������Ծ
	}
	else
	{
		altHoldThrottleBase = rcCommand[THROTTLE];
		stateControlSetVelocityZPIDIntegration(0);//���Z���ٶȿ���������
	}

	posEstimatorReset();//��λλ��Ԥ������Ԥ��λ�ú��ٶȣ���ֹײ�������ٶȺ�λ���쳣��

	setpoint->position.z = state->position.z;//�趨Ŀ��߶�Ϊ��ǰ����ĸ߶�
}

//�趨��������ͷ���ģʽ
void commanderGetSetpoint(const state_t *state, setpoint_t *setpoint)
{
	if (autoLandState.autoLandActive != true)//��������
	{
		rcCommand[ROLL] = getAxisRcCommand(rcData[ROLL], DEAD_BAND);
		rcCommand[PITCH] = getAxisRcCommand(rcData[PITCH], DEAD_BAND);
		rcCommand[YAW] = -getAxisRcCommand(rcData[YAW], DEAD_BAND_YAW);//������Z�����ֶ�����ʱ��Ϊ��������Ϊ����
		rcCommand[THROTTLE] = constrain(rcData[THROTTLE], RC_MIN, RC_MAX);
	}
	else//ʧ�ر����Զ�����
	{
		ENABLE_FLIGHT_MODE(ANGLE_MODE);//��������ģʽ
		rcCommand[ROLL] = 0;
		rcCommand[PITCH] = 0;
		rcCommand[YAW] = 0;
		if ((state->acc.z > 800.f) || (getSysTickCnt() - autoLandState.autoLandTime > 20000))//Z����ٶȴ���800cm/ss˵�����䵽�����˻򳬹�20����رյ��
		{
			rcCommand[THROTTLE] = 0;
			mwDisarm();
		}
		else
		{
			rcCommand[THROTTLE] = LAND_THROTTLE;//��������
		}
		beeper(BEEPER_RX_LOST_LANDING);//����������
	}
	
	//�������ֵС�ڻ����MINCHECKʱ��yaw���Ʋ�������
	//Ŀ�����ڼ����ͽ�������ʱ��ֹ������ת
	if(rcCommand[THROTTLE] <= RC_COMMANDER_MINCHECK)
	{
		rcCommand[YAW] = 0;
	}
	
	if(FLIGHT_MODE(HEADFREE_MODE))//��ͷģʽ
	{
		float yawRad = degreesToRadians(-state->attitude.yaw);
		float cosy = cosf(yawRad);
		float siny = sinf(yawRad);
		int16_t rcCommand_PITCH = rcCommand[PITCH]*cosy + rcCommand[ROLL]*siny;
		rcCommand[ROLL] = rcCommand[ROLL]*cosy - rcCommand[PITCH]*siny;
		rcCommand[PITCH] = rcCommand_PITCH;
	}

	resetSetpointMode(setpoint);//��λ����ģʽ
	
	//�ֶ�ģʽ������ģʽ
	if (FLIGHT_MODE(ACRO_MODE))
	{
		setpoint->mode.roll = modeVelocity;
		setpoint->mode.pitch = modeVelocity;
		setpoint->mode.yaw = modeVelocity;
		
		setpoint->attitudeRate.roll = rcCommandToRate(rcCommand[ROLL], MAX_RATE_ROLL);
		setpoint->attitudeRate.pitch = rcCommandToRate(rcCommand[PITCH], MAX_RATE_PITCH);
		setpoint->attitudeRate.yaw = rcCommandToRate(rcCommand[YAW], MAX_RATE_YAW);
	}
	else if (FLIGHT_MODE(ANGLE_MODE))
	{
		setpoint->mode.yaw = modeVelocity;
		
		setpoint->attitude.roll = rcCommandToAngle(rcCommand[ROLL], MAX_ANGLE_ROLL);
		setpoint->attitude.pitch = rcCommandToAngle(rcCommand[PITCH], MAX_ANGLE_PITCH);
		setpoint->attitudeRate.yaw = rcCommandToRate(rcCommand[YAW], MAX_RATE_YAW);
	}

	//����ģʽ
	if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
	{
		static bool isAdjustAlttitude = false;
		
		//��ʼ������ģʽ
		if (setupAltitudeHoldFlag == false)
		{
			setupAltitudeHold(state, setpoint);
			setupAltitudeHoldFlag = true;
		}
		
		//�����趨�߶�
		int16_t rcThrottleAdjustment = applyDeadband(rcCommand[THROTTLE] - altHoldThrottleBase, ALT_HOLD_DEADBAND);
		if (rcThrottleAdjustment == 0 && isAdjustAlttitude == true)
		{
			setpoint->mode.z = modeAbs;
			setpoint->position.z = state->position.z;
			isAdjustAlttitude = false;
		}
		else if (rcThrottleAdjustment > 0)
		{
			setpoint->mode.z = modeVelocity;
			setpoint->velocity.z = rcThrottleAdjustment * CLIMB_RATE_UP / (RC_MAX - altHoldThrottleBase - ALT_HOLD_DEADBAND);
			isAdjustAlttitude = true;
		}
		else
		{
			setpoint->mode.z = modeVelocity;
			setpoint->velocity.z = rcThrottleAdjustment * CLIMB_RATE_DOWN / (altHoldThrottleBase - RC_MIN - ALT_HOLD_DEADBAND);
			isAdjustAlttitude = true;
		}
	}
	
	setpoint->thrust = rcCommand[THROTTLE];
}

//���ö���ģʽ
void commanderSetupAltitudeHoldMode(void)
{
	setupAltitudeHoldFlag = false;
}

//��ȡ���߻�������ֵ
uint16_t commanderGetALtHoldThrottle(void)
{
	return altHoldThrottleBase;
}

//����ʧ�ر���ģʽ
void commanderActiveFailsafe(void)
{
	autoLandState.autoLandActive = true;
	autoLandState.autoLandTime = getSysTickCnt();
}
