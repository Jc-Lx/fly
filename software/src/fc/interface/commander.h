#ifndef __COMMANDER_H
#define __COMMANDER_H
#include "atkp.h"
#include "config.h"
#include "stabilizer_types.h"
#include "rx.h"

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

typedef struct
{
	bool autoLandActive;
	uint32_t autoLandTime;
}autoLandState_t;

extern int16_t rcCommand[4];

void commanderGetSetpoint(const state_t *state, setpoint_t *setpoint);
void commanderSetupAltitudeHoldMode(void);
uint16_t commanderGetALtHoldThrottle(void);
void commanderActiveFailsafe(void);







#endif /* __COMMANDER_H */
