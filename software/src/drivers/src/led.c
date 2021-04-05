#include "led.h"


#define NBR_LEDS  6	//ws2812 RGB灯个数

enum ledStripColor
{
	RED = 0,
	GREEN,
	BLUE,
	WHITE,
	BLACK,//黑色（不亮）
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
	{0xFF, 0x00, 0x00},//红
	{0x00, 0xFF, 0x00},//绿
	{0x00, 0x00, 0xFF},//蓝
	{0xFF, 0xFF, 0xFF},//白
	{0x00, 0x00, 0x00},//黑（不亮）
};

/**********************************************************************************************************
*函 数 名: MagSensorRead
*功能说明: 罗盘数据读取
*形    参: 无
*返 回 值: 无
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
*函 数 名: static void ledStripFillBuFFerWitchColor(enum ledStripColor color)
*功能说明: 填充颜色
*形    参: 无
*返 回 值: 无
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
*函 数 名: void ledStripInit(void)
*功能说明: 灯带初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ledStripInit(void)
{
	ws2812Init();
	ledStripON();
}

/**********************************************************************************************************
*函 数 名: void ledStripON(void)
*功能说明: 灯带开函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ledStripON(void)
{
	ledStripFillBuFFerWitchColor(GREEN);
	ws2812Send(colorBuFFer, NBR_LEDS);
}

/**********************************************************************************************************
*函 数 名: void ledStripOFF(void)
*功能说明: 灯带关函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ledStripOFF(void)
{
	ledStripFillBuFFerWitchColor(BLACK);
	ws2812Send(colorBuFFer, NBR_LEDS);
}




