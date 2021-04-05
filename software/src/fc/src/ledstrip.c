#include "ledstrip.h"
#include "ws2812.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "timers.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * WS2812�ƴ���������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define NBR_LEDS  8	//ws2812 RGB�Ƹ���

enum ledStripColor
{
	RED = 0,
	GREEN,
	BLUE,
	WHITE,
	BLACK,//��ɫ��������
	COLOR_NUM,
};

const uint8_t colorTable[COLOR_NUM][3] =
{
	{0xff, 0x00, 0x00},//��
	{0x00, 0xFF, 0x00},//��
	{0x00, 0x00, 0xFF},//��
	{0xff, 0xff, 0xff},//��
	{0x00, 0x00, 0x00},//�ڣ�������
};

typedef enum
{
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
} ledStripState_e;

static uint8_t colorBuffer[NBR_LEDS][3];


//�����ɫ
static void ledStripFillBufferWitchColor(enum ledStripColor color)
{
	for (int i = 0; i < NBR_LEDS; i++)
	{
		colorBuffer[i][0] = colorTable[color][0];
		colorBuffer[i][1] = colorTable[color][1];
		colorBuffer[i][2] = colorTable[color][2];
	}
}


//�ƴ���ʼ��
void ledStripInit(void)
{
	ws2812Init();
	ledStripOFF();
}

void ledStripON(void)
{
	ledStripFillBufferWitchColor(RED);
	ws2812Send(colorBuffer, NBR_LEDS);
}

void ledStripOFF(void)
{
	ledStripFillBufferWitchColor(BLACK);
	ws2812Send(colorBuffer, NBR_LEDS);
}



