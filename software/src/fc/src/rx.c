#include "rx.h"
#include "config.h"
#include "system.h"
#include "commander.h"
#include "cppm.h"
#include "rc_controls.h"
#include "runtime_config.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ң�ؽ��մ������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


static uint8_t currChannel = 0;
uint16_t rcData[CH_NUM];//����PPM��ͨ���ź�ֵ
rcLinkState_t rcLinkState;
failsafeState_t  failsafeState;

void rxInit(void)
{
	cppmInit();
}

void ppmTask(void *param)
{
	uint16_t ppm;
	uint32_t currentTick;
	while(1)
	{
		currentTick = getSysTickCnt();
		
		if(cppmGetTimestamp(&ppm) == pdTRUE)//20ms����ʽ��ȡPPM����ֵ
		{
			if (cppmIsAvailible() && ppm < 2100)//�ж�PPM֡����
			{
				if(currChannel < CH_NUM)
				{
					rcData[currChannel] = ppm;
				}
				currChannel++;
			}
			else//������һ֡����
			{
				currChannel = 0;
				rcLinkState.linkState = true;
				if (rcData[THROTTLE] < 950 || rcData[THROTTLE] > 2100)//��Ч���壬˵�����ջ������ʧ�ر�����ֵ
					rcLinkState.invalidPulse = true;
				else
					rcLinkState.invalidPulse = false;
				rcLinkState.realLinkTime = currentTick;
			}
		}
		
		if (currentTick - rcLinkState.realLinkTime > 1000)//1Sû���յ��ź�˵��ң��������ʧ��
		{
			rcLinkState.linkState = false;
		}	
	}
}

void rxTask(void *param)
{	
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	uint32_t currentTick;
	
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz����Ƶ��
		
		if (RATE_DO_EXECUTE(RATE_50_HZ, tick))
		{
			currentTick = getSysTickCnt();
			
			//����ң������͸���ͨ��ģʽ�л�
			if (rcLinkState.linkState == true)
			{
				processRcStickPositions();
				processRcAUXPositions();
			}
			
			//�������״̬��ң��ʧȥ����
			if (ARMING_FLAG(ARMED))
			{
				if (rcLinkState.linkState == false || rcLinkState.invalidPulse == true)//ң��ʧȥ���ӻ���Ч����
				{
					if (failsafeState.failsafeActive == false)
					{
						if (currentTick > failsafeState.throttleLowPeriod )//��һ�ε�����ʱ�䣨��5�룩��˵���ɻ��ڵ��Ͽ�ֱ�ӹرյ��
						{
							mwDisarm();
						}
						else 
						{
							failsafeState.failsafeActive = true;
							commanderActiveFailsafe();//����ʧ�ر����Զ�����
						}
					}
				}
				else//ң����������
				{
					throttleStatus_e throttleStatus = calculateThrottleStatus();
					if (throttleStatus == THROTTLE_HIGH)
					{
						failsafeState.throttleLowPeriod = currentTick + 5000;//5000��ʾ��Ҫ�����ŵ�ʱ�䣨5�룩
					}
				}
			}
			else
			{
				failsafeState.throttleLowPeriod = 0;
			}
			
		}
		tick++;
	}
}



