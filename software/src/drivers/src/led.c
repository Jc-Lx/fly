#include "led.h"


#define NBR_LEDS  6	//ws2812 RGB�Ƹ���

enum ledStripColor
{
	RED = 0,
	GREEN,
	BLUE,
	WHITE,
	BLACK,//��ɫ��������
	COLOR_NUM,
};

typedef enum
{
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
} ledStripState_e;

static uint8_t colorBuFFer[NBR_LEDS][3];

const uint8_t colorTable[COLOR_NUM][3] =
{
	{0xFF, 0x00, 0x00},//��
	{0x00, 0xFF, 0x00},//��
	{0x00, 0x00, 0xFF},//��
	{0xFF, 0xFF, 0xFF},//��
	{0x00, 0x00, 0x00},//�ڣ�������
};

/**********************************************************************************************************
*�� �� ��: MagSensorRead
*����˵��: �������ݶ�ȡ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ledInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	
}

/**********************************************************************************************************
*�� �� ��: static void ledStripFillBuFFerWitchColor(enum ledStripColor color)
*����˵��: �����ɫ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void ledStripFillBuFFerWitchColor(enum ledStripColor color)
{
	for (int i = 0; i < NBR_LEDS; i++)
	{
		colorBuFFer[i][0] = colorTable[color][0];
		colorBuFFer[i][1] = colorTable[color][1];
		colorBuFFer[i][2] = colorTable[color][2];
	}
}
/**********************************************************************************************************
*�� �� ��: void ledStripInit(void)
*����˵��: �ƴ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ledStripInit(void)
{
	ws2812Init();
	ledStripON();
}

/**********************************************************************************************************
*�� �� ��: void ledStripON(void)
*����˵��: �ƴ�������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ledStripON(void)
{
	ledStripFillBuFFerWitchColor(GREEN);
	ws2812Send(colorBuFFer, NBR_LEDS);
}

/**********************************************************************************************************
*�� �� ��: void ledStripOFF(void)
*����˵��: �ƴ��غ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ledStripOFF(void)
{
	ledStripFillBuFFerWitchColor(BLACK);
	ws2812Send(colorBuFFer, NBR_LEDS);
}




