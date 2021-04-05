#include "system.h"   /* 头文件集合 */

static TaskHandle_t AppTaskCreate_Handle = NULL;/* 创建任务句柄 */
static TaskHandle_t systemtip_Handle = NULL;/* LED任务句柄 */


static void AppTaskCreate(void);/* 用于创建任务 */
static void systemtip(void* pvParameters);/* systemtip任务实现 */


/**********************************************************************************************************
*函 数 名: int main(void)
*功能说明: main函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  /* 开发板硬件初始化 */
  systemInit();

  //创建消息队列
  MessageQueueCreate();
  
  //从内部flash中读取数据
  ParamInit();

   /* 创建AppTaskCreate任务 */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
  /* 启动任务调度 */           
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* 启动任务，开启调度 */
  else
    return -1;  
  
  while(1);   /* 正常不会执行到这里 */    
}

/**********************************************************************************************************
*函 数 名: static void AppTaskCreate(void)
*功能说明: 用户任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */

  taskENTER_CRITICAL();           //进入临界区
  
  /* 创建systemtip任务 */
  xReturn = xTaskCreate((TaskFunction_t )systemtip, /* 任务入口函数 */
                        (const char*    )"systemtip",/* 任务名字 */
                        (uint16_t       )512,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )2,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&systemtip_Handle);/* 任务控制块指针 */
  if(pdPASS == xReturn)
    printf("创建systemtip任务成功!\r\n");


  //创建传感器数据读取相关任务
  GetSensorTaskCreate();

  //创建传感器数据处理相关任务
  SensorTaskCreate();

  //创建飞行器姿态检测任务
  NavigationTaskCreate();
  
  //飞行控制任务创建
  ControlTaskCreate();
  
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}
 
/**********************************************************************************************************
*函 数 名: static void systemtip(void* parameter)
*功能说明: 系统提示任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void systemtip(void* parameter)
{	

  while (1)
  {
    if (GetArmedStatus() == ARMED)
    {
      BEEP_TOGGLE;
      printf("\r\n    yes    \r\n"); 
    }
    else
    {
      BEEP_OFF;
      printf("\r\n    no     \r\n"); 
    }
	  LED0_TOGGLE;
    vTaskDelay(100);   
  }
}

/********************************END OF FILE****************************/













