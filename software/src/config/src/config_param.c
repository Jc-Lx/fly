#include <string.h>
#include "config_param.h"
#include "stmflash.h"
#include "beeper.h"
#include "runtime_config.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ���ò�����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


#define VERSION 11	/*11 ��ʾV1.1*/

configParam_t configParam;

static configParam_t configParamDefault=
{
	.version = VERSION,		/*����汾��*/

	.pid = 
	{
		[RATE_ROLL]   = {85, 900, 18},
		[RATE_PITCH]  = {90, 1000, 18},
		[RATE_YAW]    = {120, 800, 0},
		[ANGLE_ROLL]  = {600, 0, 0},
		[ANGLE_PITCH] = {600, 0, 0},
		[ANGLE_YAW]   = {600, 0, 0},
		[VELOCITY_Z]  = {150, 200, 50},
		[POSHOLD_Z]   = {45, 0, 0},
		[VELOCITY_XY] = {0, 0, 0},
		[POSHOLD_XY]  = {0, 0, 0},
	},
	.accBias= 	/*���ٶ�У׼ֵ*/
	{
		.accZero = 
		{
			0,
			0,
			0,
		},
		.accGain = 
		{
			4096,
			4096,
			4096,
		}
	},
	.magBias=
	{
		.magZero = 
		{
			0,
			0,
			0,
		},
	},
	.boardAlign=
	{
		.rollDeciDegrees = 0,
		.pitchDeciDegrees = 0,
		.yawDeciDegrees = 0,
	},
};

static u32 lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;
static SemaphoreHandle_t  xSemaphore = NULL;
static u32 semaphoreGiveTime;

static u8 configParamCksum(configParam_t* data)
{
	int i;
	u8 cksum=0;	
	u8* c = (u8*)data;  	
	size_t len=sizeof(configParam_t);

	for (i=0; i<len; i++)
		cksum += *(c++);
	cksum-=data->cksum;
	
	return cksum;
}

void configParamInit(void)	/*�������ó�ʼ��*/
{
	if(isInit) return;
	
	lenth=sizeof(configParam);
	lenth=lenth/4+(lenth%4 ? 1:0);

	STMFLASH_Read(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth);
	
	if (configParam.version == VERSION)	/*�汾��ȷ*/
	{
		if(configParamCksum(&configParam) == configParam.cksum)	/*У����ȷ*/
		{
			printf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
			isConfigParamOK = true;
		} else
		{
			printf("Version check [FAIL]\r\n");
			isConfigParamOK = false;
		}
	}
	else	/*�汾����*/
	{
		isConfigParamOK = false;
	}
	
	if(isConfigParamOK == false)	/*���ò�������д��Ĭ�ϲ���*/
	{
		memcpy((u8 *)&configParam, (u8 *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);				/*����У��ֵ*/
		STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);	/*д��stm32 flash*/
		isConfigParamOK=true;
	}	
	
	xSemaphore = xSemaphoreCreateBinary();
	
	isInit=true;
}

void configParamTask(void* param)
{
	u8 cksum = 0;
	
	while(1) 
	{	
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		
		for (;;)
		{
			if ((getSysTickCnt() - semaphoreGiveTime > 2000) && (!ARMING_FLAG(ARMED)))//�ϴ��ͷ��ź���ʱ�����ٴ���2���дFlash����ֹƵ��д��
			{
				cksum = configParamCksum(&configParam);
				if((configParam.cksum != cksum))
				{
					configParam.cksum = cksum;					
					STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);
				}
				DISABLE_STATE(FLASH_WRITING);
			}
			else
			{
				xSemaphoreTake(xSemaphore, 100);
			}
		}
	}
}

void saveConfigAndNotify(void)
{
	if (!ARMING_FLAG(ARMED)) //����״̬�ſ��Բ���Flash
	{
		beeperConfirmationBeeps(1);	
		xSemaphoreGive(xSemaphore);
		semaphoreGiveTime = getSysTickCnt();
		ENABLE_STATE(FLASH_WRITING);
	}
}

//�ָ�ΪĬ�ϲ���
void resetConfigParam(void)
{
	configParam = configParamDefault;
	saveConfigAndNotify();
}

