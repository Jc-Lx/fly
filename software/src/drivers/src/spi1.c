#include "spi1.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * SPI1��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/
static bool isInit = false;

static const uint16_t divisorMap[] = { 
    SPI_BaudRatePrescaler_256,    // SPI_CLOCK_INITIALIZATON      328.125 KBits/s
    SPI_BaudRatePrescaler_128,    // SPI_CLOCK_SLOW               656.25 KBits/s
    SPI_BaudRatePrescaler_8,      // SPI_CLOCK_STANDARD           10.5 MBits/s
    SPI_BaudRatePrescaler_4,      // SPI_CLOCK_FAST               21.0 MBits/s
    SPI_BaudRatePrescaler_2       // SPI_CLOCK_ULTRAFAST          42.0 MBits/s
};

void spi1Init(void)
{
	if (isInit) return;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	//����SPI1��SCK(PA5),MISO(PA6),MOSI(PA7)����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	//��ʼ��SPI1
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE);
	
	isInit = true;
}

u8 spi1TransferByte(u8 data)
{
    uint16_t spiTimeout = 1000;

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		if((spiTimeout--)==0)
			return false;
	}  
    SPI_SendData(SPI1, data);

	spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		if ((spiTimeout--) == 0)
            return false;
	}
    return ((u8)SPI_ReceiveData(SPI1));
}

bool spi1Transfer(uint8_t *out, const uint8_t *in, int len)
{
	uint16_t spiTimeout = 1000;

	SPI1->DR;
	while (len--) 
	{
		uint8_t b = in ? *(in++) : 0xFF;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) 
		{
			if ((spiTimeout--) == 0)
				return false;;
		}
		SPI_SendData(SPI1, b);

		spiTimeout = 1000;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) 
		{
			if ((spiTimeout--) == 0)
				return false;
		}
		b = SPI_ReceiveData(SPI1);
		if (out)
			*(out++) = b;
	}
	return true;
}

void spi1SetSpeed(SPIClockSpeed_e speed)
{
#define BR_CLEAR_MASK 0xFFC7
    SPI_Cmd(SPI1, DISABLE);

    uint16_t tempRegister = SPI1->CR1;
    tempRegister &= BR_CLEAR_MASK;
    tempRegister |= divisorMap[speed];
    SPI1->CR1 = tempRegister;
    SPI_Cmd(SPI1, ENABLE);
}


