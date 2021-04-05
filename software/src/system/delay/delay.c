#include "delay.h"

//���ʹ��OS,����������ͷ�ļ�����ucosΪ��������.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��	 
#include "task.h"	  
#endif


static uint32_t fac_us=0;							//us��ʱ������

#if SYSTEM_SUPPORT_OS		
    static uint16_t fac_ms=0;				        //ms��ʱ������,��os��,����ÿ�����ĵ�ms��
#endif

/**********************************************************************************************************
*�� �� ��: void delay_init(u8 SYSCLK))
*����˵��: ��ʼ���ӳٺ���
*��    ��: ��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
		   SYSTICK��ʱ�ӹ̶�ΪAHBʱ�ӵ�1/8  SYSCLK:ϵͳʱ��Ƶ��
*�� �� ֵ: ��
**********************************************************************************************************/  			   
void delay_init(u8 SYSCLK)
{
#if SYSTEM_SUPPORT_OS 						//�����Ҫ֧��OS.
	uint32_t reload;
#endif
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 
	fac_us=SYSCLK;							//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
#if SYSTEM_SUPPORT_OS 						//�����Ҫ֧��OS.
	reload=SYSCLK;							//ÿ���ӵļ������� ��λΪM	   
	reload*=1000000/configTICK_RATE_HZ;		//��configTICK_RATE_HZ�����ʱ��
											//reloadΪ24λ�Ĵ���,���ֵ:16777216,��96M��,Լ��0.17476s����	
	fac_ms=1000/configTICK_RATE_HZ;			//����OS������ʱ�����ٵ�λ	   
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//����SYSTICK�ж�
	SysTick->LOAD=reload; 					//ÿ1/delay_ostickspersec���ж�һ��	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;	//����SYSTICK    
#else
	fac_ms=(uint16_t)fac_us*1000;				//��OS��,����ÿ��ms��Ҫ��systickʱ����   
#endif
}								    

#if SYSTEM_SUPPORT_OS 						//�����Ҫ֧��OS.
/**********************************************************************************************************
*�� �� ��: void delay_us(uint32_t nus)
*����˵��: ��ʱnus
*��    ��: nus:Ҫ��ʱ��us��.nus:0~44739242(���ֵ��2^32/fac_us@fac_us=96)
*�� �� ֵ: ��
**********************************************************************************************************/   								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};											    
}

/**********************************************************************************************************
*�� �� ��: void delay_ms(uint16_t nms)
*����˵��: ��ʱnms
*��    ��: nms:Ҫ��ʱ��ms�� nms:0~65535
*�� �� ֵ: ��
**********************************************************************************************************/ 
void delay_ms(uint16_t nms)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����	    
	{		 
		if(nms>=fac_ms)						//��ʱ��ʱ�����OS������ʱ������ 
		{ 
   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS��ʱ
		}
		nms%=fac_ms;						//OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
	}
	delay_us((uint32_t)(nms*1000));				//��ͨ��ʽ��ʱ
}
#else  //����ucosʱ
//��ʱnus
//nusΪҪ��ʱ��us��.	
//ע��:nus��ֵ,��Ҫ����174762us(���ֵ��2^24/fac_us@fac_us=96)
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//ʱ�����	  		 
	SysTick->VAL=0x00;        				//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //�رռ�����
	SysTick->VAL =0X00;       				//��ռ����� 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��168M������,nms<=798ms 
//void delay_xms(uint16_t nms)
//{	 		  	  
//	uint32_t temp;		   
//	SysTick->LOAD=(uint32_t)nms*fac_ms;			//ʱ�����(SysTick->LOADΪ24bit)
//	SysTick->VAL =0x00;           			//��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ���� 
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));	//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
//	SysTick->VAL =0X00;     		  		//��ռ�����	  	    
//} 
//��ʱnms 
//nms:0~65535
void delay_ms(uint16_t nms)
{	 	 
	u8 repeat=nms/540;						//������540,�ǿ��ǵ�ĳЩ�ͻ����ܳ�Ƶʹ��,
											//���糬Ƶ��248M��ʱ��,delay_xms���ֻ����ʱ541ms������
	uint16_t remain=nms%540;
	while(repeat)
	{
		delay_xms(540);
		repeat--;
	}
	if(remain)delay_xms(remain);
} 
#endif

/**********************************************************************************************************
*�� �� ��: void delay_xms(uint32_t nms)
*����˵��: ��ʱnms,���������������
*��    ��: nms:Ҫ��ʱ��ms��
*�� �� ֵ: ��
**********************************************************************************************************/
void delay_xms(uint32_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}	

/**********************************************************************************************************
*�� �� ��: void  delay_ns(uint16_t nns)
*����˵��: �����ʱns
*��    ��: nns
*�� �� ֵ: ��
**********************************************************************************************************/
void  delay_ns(uint32_t ns)
{
    uint32_t ns_cnt;
    for(; ns!= 0; ns--)
    {
        ns_cnt = 3;//��Լ200ns
        while(ns_cnt)
        {
            ns_cnt--;
        }
    }	
}

/**********************************************************************************************************
*�� �� ��: GetSysTimeUs
*����˵��: ��ȡ��ǰϵͳ����ʱ�䣬��λΪ΢��
*��    ��: ��
*�� �� ֵ: ϵͳʱ��
**********************************************************************************************************/
uint64_t GetSysTimeUs(void)
{
    uint64_t ms;
    uint64_t value;
    ms = xTaskGetTickCount();
    value = ms * 1000 + (SysTick->LOAD - SysTick->VAL) * 1000 / SysTick->LOAD;
    return value;
}

/**********************************************************************************************************
*�� �� ��: GetSysTimeMs
*����˵��: ��ȡ��ǰϵͳ����ʱ�䣬��λΪ����
*��    ��: ��
*�� �� ֵ: ϵͳʱ��
**********************************************************************************************************/
uint32_t GetSysTimeMs(void)
{
    return xTaskGetTickCount();
}












