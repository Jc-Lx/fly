#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ģ��IIC��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


void busIIC_Init(void);
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data);
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, const uint8_t * data);


#endif




