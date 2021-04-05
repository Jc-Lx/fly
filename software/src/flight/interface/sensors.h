#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"
#include "config.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ���������ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

/*��������ȡ����Ƶ��*/
#define GYRO_UPDATE_RATE		RATE_500_HZ
#define ACC_UPDATE_RATE			RATE_500_HZ
#define MAG_UPDATE_RATE			RATE_10_HZ
#define BARO_UPDATE_RATE		RATE_50_HZ

extern sensorData_t sensors;

void sensorsTask(void *param);
void sensorsInit(void);			/*��������ʼ��*/
bool sensorsTest(void);			/*����������*/
bool sensorsAreCalibrated(void);	/*����������У׼*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick);/*��ȡ����������*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag);
bool sensorsIsMagPresent(void);

/* ������������������ */
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
bool sensorsReadBaro(baro_t *baro);

#endif //__SENSORS_H
