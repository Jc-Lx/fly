#include "cppm.h"

/**********************************************************************************************************
*函 数 名: void CPPM_Init(void)
*功能说明: PPM捕获引脚配置，TIM5时基配置，CH4捕获配置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CPPM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//配置PPM信号输入引脚（PA3）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	//配置定时器1us tick
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	//配置输入捕获
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM5,TIM_IT_CC4, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}

uint16_t capureVal = 0;
uint16_t capureValDiff = 0;
uint16_t prevCapureVal = 0;

uint16_t RCdata[5];
uint16_t tempIndex = 0;//PPM数据缓存数组
uint8_t  PPM_flag = 0;//PPM开始解码标志位
/**********************************************************************************************************
*函 数 名: TIMx_IRQHandler
*功能说明: 定时器中断函数,PPM捕获解码
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TIM5_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM5,TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
		capureVal = TIM_GetCapture4(TIM5);
		if (prevCapureVal > capureVal)
		{
			capureValDiff = 0xFFFF - prevCapureVal +capureVal;
		}
		else
		{
			capureValDiff = capureVal - prevCapureVal;
		}
		prevCapureVal = capureVal;
		if(capureValDiff >= 3500)
	    {
			PPM_flag = 1;
			//tempIndex = 0;
	    }
		if(PPM_flag == 1)
		{
			RCdata[tempIndex] = capureValDiff;
			tempIndex ++;
			if(tempIndex >= 5) 
			{
				tempIndex = 0;
				PPM_flag = 0;
			}	
		}
	}	
}

RCDATA_t rcRawData;//遥控器数据结构体
/**********************************************************************************************************
*函 数 名: void PPM_Decode(void)
*功能说明: 遥控器数据从数组中赋值给遥控器数据结构体存储
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PPM_Decode(void)
{
	rcRawData.pitch    = RCdata[1];
	rcRawData.roll     = RCdata[2];
	rcRawData.yaw      = RCdata[3];
	rcRawData.throttle = RCdata[4];
	// printf("\r\n    sync:%d    pitch:%d    roll:%d    yaw:%d    throttle:%d    \r\n",RCdata[0],RCdata[1],RCdata[2],RCdata[3],RCdata[4]);
}

/**********************************************************************************************************
*函 数 名: RCDATA_t GetRawRcData(void)
*功能说明: 获取遥控原始数据
*形    参: 无
*返 回 值: 遥控器数据结构体
**********************************************************************************************************/
RCDATA_t GetRawRcData(void)
{
	return rcRawData;
}

