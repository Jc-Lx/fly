#include "rc_controls.h"
#include "rx.h"
#include "sensorsalignment.h"
#include "beeper.h"
#include "commander.h"
#include "runtime_config.h"
#include "ledstrip.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ң�ؿ��ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define RC_PERIOD_MS     20

stickPositions_e rcStickPositions = (stickPositions_e) 0; //ң��λ��״̬
auxPositions_e   channelPos[CH_NUM];//����ͨ��λ��״̬

stickPositions_e getRcStickPositions(void)
{
    return rcStickPositions;
}

//����ң��λ��
static void updateRcStickPositions(void)
{
	stickPositions_e tmp = (stickPositions_e)0;
	
	tmp |= ((rcData[ROLL] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (ROLL * 2);
	tmp |= ((rcData[ROLL] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (ROLL * 2);

	tmp |= ((rcData[PITCH] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (PITCH * 2);
	tmp |= ((rcData[PITCH] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (PITCH * 2);

	tmp |= ((rcData[THROTTLE] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (THROTTLE * 2);
	tmp |= ((rcData[THROTTLE] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (THROTTLE * 2);

	tmp |= ((rcData[YAW] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (YAW * 2);
	tmp |= ((rcData[YAW] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (YAW * 2);
	
    rcStickPositions = tmp;
}

//����ң��λ�úͶ�Ӧ����
void processRcStickPositions(void)
{
	static u32 lastTickTimeMs;
	static uint8_t rcDelayCommand;
	static uint32_t rcSticks;
	u32 currentTimeMs = getSysTickCnt();
	
    updateRcStickPositions();
	uint32_t stTmp = getRcStickPositions();
	if(stTmp == rcSticks)
	{
		if(rcDelayCommand<250 && (currentTimeMs - lastTickTimeMs) >= RC_PERIOD_MS)
		{	
			lastTickTimeMs = currentTimeMs;
			rcDelayCommand++;
		}
	}
	else
		rcDelayCommand = 0;
	
	rcSticks = stTmp;
	
	if (rcDelayCommand != 20) return;
	
	//����
	if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)
	{
		mwDisarm();
	}
	//����
	if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
	{
		if (channelPos[AUX1] == AUX_LO)
		{
			mwArm();
		}
	}
	
	//���ٶ�΢��
    if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) 
	{
        applyAndSaveBoardAlignmentDelta(0, -1);
        rcDelayCommand = 10;
        return;
    } 
	else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) 
	{
        applyAndSaveBoardAlignmentDelta(0, 1);
        rcDelayCommand = 10;
        return;
    } 
	else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) 
	{
        applyAndSaveBoardAlignmentDelta(-1, 0);
        rcDelayCommand = 10;
        return;
    } 
	else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) 
	{
        applyAndSaveBoardAlignmentDelta(1, 0);
        rcDelayCommand = 10;
        return;
    }
	
	//�򿪺͹ر�RGB��
    if (rcSticks == THR_LO + YAW_CE + PIT_LO + ROL_HI) 
	{
		setLedStripAllwaysIsON(true);
	}
	else if (rcSticks == THR_LO + YAW_CE + PIT_LO + ROL_LO) 
	{
		setLedStripAllwaysIsON(false);
	}
}

//������ͨ��λ�úͶ�Ӧ����
void processRcAUXPositions(void)
{
	for (int i=AUX1; i<CH_NUM; i++)
	{
		if (rcData[i] < (RC_MID-200))
			channelPos[i] = AUX_LO;
		else if (rcData[i] > (RC_MID+200))
			channelPos[i] = AUX_HI;
		else
			channelPos[i] = AUX_CE;
	}

	//AUX3ͨ�������ο��أ���Ӧ����ģʽ������ģʽ������ģʽ������ģʽ��
	if(channelPos[AUX3] == AUX_LO)//����ģʽ
	{
		if (!FLIGHT_MODE(ANGLE_MODE))
			ENABLE_FLIGHT_MODE(ANGLE_MODE);
		
		if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
	
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	else if(channelPos[AUX3] == AUX_CE)//����ģʽ
	{
		if (!FLIGHT_MODE(NAV_ALTHOLD_MODE))
		{
			ENABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
			commanderSetupAltitudeHoldMode();
		}
		
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	else if(channelPos[AUX3] == AUX_HI)//����ģʽ
	{
		if (!FLIGHT_MODE(NAV_POSHOLD_MODE))
			ENABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	
	//AUX4ͨ�������ο��أ���Ӧ����ģʽ����ͷģʽ����ͷģʽ��
	if(channelPos[AUX4] == AUX_LO)
	{
		if (FLIGHT_MODE(HEADFREE_MODE))//��ͷģʽ
			DISABLE_FLIGHT_MODE(HEADFREE_MODE);
	}
	else if(channelPos[AUX4] == AUX_HI)
	{
		if (!FLIGHT_MODE(HEADFREE_MODE))//��ͷģʽ
			ENABLE_FLIGHT_MODE(HEADFREE_MODE);
	}
	
	//AUX2ͨ�������ο��أ��򿪺͹رշ�������ը��ʱѰ���ã�
	if(channelPos[AUX2] == AUX_LO)
	{
		if (FLIGHT_MODE(BEEPER_ON_MODE))//�رշ�����
			DISABLE_FLIGHT_MODE(BEEPER_ON_MODE);
	}
	else if(channelPos[AUX2] == AUX_HI)
	{
		if (!FLIGHT_MODE(BEEPER_ON_MODE))//�򿪷�����
			ENABLE_FLIGHT_MODE(BEEPER_ON_MODE);
	}
	
	//AUX1ͨ�������ο��أ�����ͣ��
	if(channelPos[AUX1] == AUX_LO)
	{
		
	}
	else if(channelPos[AUX1] == AUX_HI)
	{
		if(ARMING_FLAG(ARMED))
		{
			mwDisarm();//����
		}
	}	
}

//��ȡ����ң��״̬
throttleStatus_e calculateThrottleStatus(void)
{
    if (rcData[THROTTLE] < RC_COMMANDER_MINCHECK)
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}






