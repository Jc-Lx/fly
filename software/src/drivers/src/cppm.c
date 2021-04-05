#include "cppm.h"

/**********************************************************************************************************
*�� �� ��: void CPPM_Init(void)
*����˵��: PPM�����������ã�TIM5ʱ�����ã�CH4��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void CPPM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//����PPM�ź��������ţ�PA3��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	//���ö�ʱ��1us tick
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	//�������벶��
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
uint16_t tempIndex = 0;//PPM���ݻ�������
uint8_t  PPM_flag = 0;//PPM��ʼ�����־λ
/**********************************************************************************************************
*�� �� ��: TIMx_IRQHandler
*����˵��: ��ʱ���жϺ���,PPM�������
*��    ��: ��
*�� �� ֵ: ��
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

RCDATA_t rcRawData;//ң�������ݽṹ��
/**********************************************************************************************************
*�� �� ��: void PPM_Decode(void)
*����˵��: ң�������ݴ������и�ֵ��ң�������ݽṹ��洢
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: RCDATA_t GetRawRcData(void)
*����˵��: ��ȡң��ԭʼ����
*��    ��: ��
*�� �� ֵ: ң�������ݽṹ��
**********************************************************************************************************/
RCDATA_t GetRawRcData(void)
{
	return rcRawData;
}

