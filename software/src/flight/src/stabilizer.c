#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "commander.h"
#include "state_control.h"
#include "power_control.h"
#include "pos_estimator.h"
#include "gyro.h"
#include "led.h"
#include "runtime_config.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * �������ȿ��ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit;
setpoint_t		setpoint;	/*����Ŀ��״̬*/
sensorData_t 	sensorData;	/*����������*/
state_t 		state;		/*������̬*/
control_t 		control;	/*������Ʋ���*/


void stabilizerInit(void)
{
	if(isInit) return;
	
	stateControlInit();		/*��̬PID��ʼ��*/
	powerControlInit();		/*�����ʼ��*/
	imuInit();				/*��̬�����ʼ��*/
	isInit = true;
}

void stabilizerTask(void* param)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	
	//�ȴ�������У׼���
	while(!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
	}
	
	while(1) 
	{
		//1KHz����Ƶ��
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));	
		
		//��ȡ����������
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			sensorsAcquire(&sensorData, tick);				
		}
		
		//��Ԫ����ŷ���Ǽ���
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdateAttitude(&sensorData, &state, ATTITUDE_ESTIMAT_DT);				
		}
		
		//λ��Ԥ������
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{  	
			updatePositionEstimator(&sensorData, &state, POSITION_ESTIMAT_DT);
		}
		
		//Ŀ����̬�ͷ���ģʽ�趨	
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			commanderGetSetpoint(&state, &setpoint);
			updateArmingStatus();
		}
		
		//PID����������������
		stateControl(&sensorData, &state, &setpoint, &control, tick);
		
		//���Ƶ�������500Hz��
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			powerControl(&control);
		}
		
		tick++;
	}
}

