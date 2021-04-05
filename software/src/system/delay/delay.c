#include "delay.h"

//如果使用OS,则包括下面的头文件（以ucos为例）即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	 
#include "task.h"	  
#endif


static uint32_t fac_us=0;							//us延时倍乘数

#if SYSTEM_SUPPORT_OS		
    static uint16_t fac_ms=0;				        //ms延时倍乘数,在os下,代表每个节拍的ms数
#endif

/**********************************************************************************************************
*函 数 名: void delay_init(u8 SYSCLK))
*功能说明: 初始化延迟函数
*形    参: 当使用OS的时候,此函数会初始化OS的时钟节拍
		   SYSTICK的时钟固定为AHB时钟的1/8  SYSCLK:系统时钟频率
*返 回 值: 无
**********************************************************************************************************/  			   
void delay_init(u8 SYSCLK)
{
#if SYSTEM_SUPPORT_OS 						//如果需要支持OS.
	uint32_t reload;
#endif
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 
	fac_us=SYSCLK;							//不论是否使用OS,fac_us都需要使用
#if SYSTEM_SUPPORT_OS 						//如果需要支持OS.
	reload=SYSCLK;							//每秒钟的计数次数 单位为M	   
	reload*=1000000/configTICK_RATE_HZ;		//根configTICK_RATE_HZ定溢出时间
											//reload为24位寄存器,最大值:16777216,在96M下,约合0.17476s左右	
	fac_ms=1000/configTICK_RATE_HZ;			//代表OS可以延时的最少单位	   
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD=reload; 					//每1/delay_ostickspersec秒中断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;	//开启SYSTICK    
#else
	fac_ms=(uint16_t)fac_us*1000;				//非OS下,代表每个ms需要的systick时钟数   
#endif
}								    

#if SYSTEM_SUPPORT_OS 						//如果需要支持OS.
/**********************************************************************************************************
*函 数 名: void delay_us(uint32_t nus)
*功能说明: 延时nus
*形    参: nus:要延时的us数.nus:0~44739242(最大值即2^32/fac_us@fac_us=96)
*返 回 值: 无
**********************************************************************************************************/   								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};											    
}

/**********************************************************************************************************
*函 数 名: void delay_ms(uint16_t nms)
*功能说明: 延时nms
*形    参: nms:要延时的ms数 nms:0~65535
*返 回 值: 无
**********************************************************************************************************/ 
void delay_ms(uint16_t nms)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行	    
	{		 
		if(nms>=fac_ms)						//延时的时间大于OS的最少时间周期 
		{ 
   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS延时
		}
		nms%=fac_ms;						//OS已经无法提供这么小的延时了,采用普通方式延时    
	}
	delay_us((uint32_t)(nms*1000));				//普通方式延时
}
#else  //不用ucos时
//延时nus
//nus为要延时的us数.	
//注意:nus的值,不要大于174762us(最大值即2^24/fac_us@fac_us=96)
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //关闭计数器
	SysTick->VAL =0X00;       				//清空计数器 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对168M条件下,nms<=798ms 
//void delay_xms(uint16_t nms)
//{	 		  	  
//	uint32_t temp;		   
//	SysTick->LOAD=(uint32_t)nms*fac_ms;			//时间加载(SysTick->LOAD为24bit)
//	SysTick->VAL =0x00;           			//清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数 
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//	SysTick->VAL =0X00;     		  		//清空计数器	  	    
//} 
//延时nms 
//nms:0~65535
void delay_ms(uint16_t nms)
{	 	 
	u8 repeat=nms/540;						//这里用540,是考虑到某些客户可能超频使用,
											//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
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
*函 数 名: void delay_xms(uint32_t nms)
*功能说明: 延时nms,不会引起任务调度
*形    参: nms:要延时的ms数
*返 回 值: 无
**********************************************************************************************************/
void delay_xms(uint32_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}	

/**********************************************************************************************************
*函 数 名: void  delay_ns(uint16_t nns)
*功能说明: 软件延时ns
*形    参: nns
*返 回 值: 无
**********************************************************************************************************/
void  delay_ns(uint32_t ns)
{
    uint32_t ns_cnt;
    for(; ns!= 0; ns--)
    {
        ns_cnt = 3;//大约200ns
        while(ns_cnt)
        {
            ns_cnt--;
        }
    }	
}

/**********************************************************************************************************
*函 数 名: GetSysTimeUs
*功能说明: 获取当前系统运行时间，单位为微秒
*形    参: 无
*返 回 值: 系统时间
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
*函 数 名: GetSysTimeMs
*功能说明: 获取当前系统运行时间，单位为毫秒
*形    参: 无
*返 回 值: 系统时间
**********************************************************************************************************/
uint32_t GetSysTimeMs(void)
{
    return xTaskGetTickCount();
}












