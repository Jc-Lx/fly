#include "ws2812.h"

#define TIMING_ONE  76	// �ߵ�ƽʱ�����ֵ
#define TIMING_ZERO 28	// �͵�ƽʱ�����ֵ

uint32_t dmaBuffer0[24];
uint32_t dmaBuffer1[24];

static xSemaphoreHandle allLedDone = NULL;
/**********************************************************************************************************
*�� �� ��: static void ws2812_GPIO_Config(void)
*����˵��: ws2812GPIO��ʼ��
*��    ��: ���ٶ�ԭʼ���� ���ٶ�Ԥ��������ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void ws2812_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//��ʼ��LED_STRIP pin PA15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

}

/**********************************************************************************************************
*�� �� ��: static void ws2812_TIMOC_Config(void)
*����˵��: ws2812TIMOC��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void ws2812_TIMOC_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = (105 - 1); //800kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);	

}

/**********************************************************************************************************
*�� �� ��: static void ws2812_DMA_Config(void)
*����˵��: ws2812DMA��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void ws2812_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM2->CCR1;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dmaBuffer0;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_BufferSize = 24;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);//ʹ��˫����
	DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)dmaBuffer1, DMA_Memory_0);
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/**********************************************************************************************************
*�� �� ��: void ws2812Init(void)
*����˵��: ws2812��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ws2812Init(void)
{
	ws2812_GPIO_Config();
	ws2812_TIMOC_Config();
	ws2812_DMA_Config();
	vSemaphoreCreateBinary(allLedDone);
}

/**********************************************************************************************************
*�� �� ��: static void fillLed(uint32_t *buffer, uint8_t *color)
*����˵��: ws2812��ɫ���
*��    ��: ��������ָ�� ��ɫ����ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void fillLed(uint32_t *buffer, uint8_t *color)
{
    int i;

    for(i=0; i<8; i++) // GREEN
	{
	    buffer[i] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // RED
	{
	    buffer[8+i] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // BLUE
	{
	    buffer[16+i] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
}

static int current_led = 0;
static int total_led = 0;
static uint8_t(*color_led)[3] = NULL;
/**********************************************************************************************************
*�� �� ��: void ws2812Send(uint8_t (*color)[3], uint16_t len)
*����˵��: ws2812��ɫ������DMA
*��    ��: ��ɫ����ָ�� ����
*�� �� ֵ: ��
**********************************************************************************************************/
void ws2812Send(uint8_t (*color)[3], uint16_t len)
{
	if(len<1) return;

	xSemaphoreTake(allLedDone, portMAX_DELAY);//�ȴ���һ�η������

	current_led = 0;
	total_led = len;
	color_led = color;
	
	fillLed(dmaBuffer0, color_led[current_led]);
	current_led++;
	fillLed(dmaBuffer1, color_led[current_led]);
	current_led++;
	
	DMA_Cmd(DMA1_Stream5, ENABLE);	//ʹ��DMA
	TIM_Cmd(TIM2, ENABLE);			//ʹ�ܶ�ʱ��
}

/**********************************************************************************************************
*�� �� ��: void DMA1_Stream5_IRQHandler(void)
*����˵��: DMA�жϴ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void DMA1_Stream5_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	if (total_led == 0)
	{
		TIM_Cmd(TIM2, DISABLE);
		DMA_Cmd(DMA1_Stream5, DISABLE);
	}
	
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
	{
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == DMA_Memory_0)//DMA��ǰʹ���ڴ�0
		{
			if (current_led<total_led)
				fillLed(dmaBuffer1, color_led[current_led]);
			else
				memset(dmaBuffer1, 0, sizeof(dmaBuffer1));
		}
		else//DMA��ǰʹ���ڴ�1
		{
			if (current_led<total_led)
				fillLed(dmaBuffer0, color_led[current_led]);
			else		
				memset(dmaBuffer0, 0, sizeof(dmaBuffer0));
		}
		current_led++;
	}

	if (current_led >= total_led+2) //�ഫ��2��LED����60us�ĵ͵�ƽ
	{
		xSemaphoreGiveFromISR(allLedDone, &xHigherPriorityTaskWoken);
		TIM_Cmd(TIM2, DISABLE); 					
		DMA_Cmd(DMA1_Stream5, DISABLE); 
		total_led = 0;
	}
}


