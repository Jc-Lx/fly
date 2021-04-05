#ifndef __MOTORS_H
#define __MOTORS_H
#include "sys.h"
#include "config.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * �����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


#define TIM_CLOCK_HZ 				84000000

#if defined(USE_ESC_PROTOCOL_STANDARD)//STANDARD_PWM
#define STANDARD_PWM_PERIOD      0.0025// 2.5ms = 400Hz
#define MOTOR_TIM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * STANDARD_PWM_PERIOD + 1) //+1��ʾ����Ҫ��Ƶ��0xFFFF��ʾ����ֵ
#define MOTOR_TIM_PERIOD 		 (uint32_t)(TIM_CLOCK_HZ * STANDARD_PWM_PERIOD / MOTOR_TIM_PRESCALE_RAW)//��ʱ����װ��ֵ
#define MOTOR_TIM_PRESCALE       (uint16_t)(MOTOR_TIM_PRESCALE_RAW - 1)//��ʱ��Ԥ��Ƶֵ
#define MOTOR_TIM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * 0.001 / MOTOR_TIM_PRESCALE_RAW)//1000us�ߵ�ƽ�������ֵ

#elif defined(USE_ESC_PROTOCOL_ONESHOT125)//ONESHOT125
#define ONESHOT125_PERIOD        0.0005 // 0.5ms = 2000Hz
#define MOTOR_TIM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * ONESHOT125_PERIOD + 1) //+1��ʾ����Ҫ��Ƶ��0xFFFF��ʾ����ֵ
#define MOTOR_TIM_PERIOD 		 (uint32_t)(TIM_CLOCK_HZ * ONESHOT125_PERIOD / MOTOR_TIM_PRESCALE_RAW)//��ʱ����װ��ֵ
#define MOTOR_TIM_PRESCALE       (uint16_t)(MOTOR_TIM_PRESCALE_RAW - 1)
#define MOTOR_TIM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * 0.000125 / MOTOR_TIM_PRESCALE_RAW)//125us�ߵ�ƽ�������ֵ

#endif


#define NBR_OF_MOTORS 	4
#define MOTOR_M1  		0
#define MOTOR_M2  		1
#define MOTOR_M3  		2
#define MOTOR_M4  		3

void motorsInit(void);
void motorsSetRatio(u32 id, u16 ithrust);

#endif /* __MOTORS_H */

