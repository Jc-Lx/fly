#ifndef __POS_ESTIMATOR_H
#define __POS_ESTIMATOR_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * λ��Ԥ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


void updatePositionEstimator(const sensorData_t *sensorData, state_t *state, float dt);
bool posEstimatorIsCalibrationComplete(void);
void posEstimatorReset(void);


#endif
