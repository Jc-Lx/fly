#include "system.h"   /* ͷ�ļ����� */

static TaskHandle_t AppTaskCreate_Handle = NULL;/* ���������� */
static TaskHandle_t systemtip_Handle = NULL;/* LED������ */


static void AppTaskCreate(void);/* ���ڴ������� */
static void systemtip(void* pvParameters);/* systemtip����ʵ�� */


/**********************************************************************************************************
*�� �� ��: int main(void)
*����˵��: main����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  
  /* ������Ӳ����ʼ�� */
  systemInit();

  //������Ϣ����
  MessageQueueCreate();
  
  //���ڲ�flash�ж�ȡ����
  ParamInit();

   /* ����AppTaskCreate���� */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* ������ں��� */
                        (const char*    )"AppTaskCreate",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* ������ƿ�ָ�� */ 
  /* ����������� */           
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* �������񣬿������� */
  else
    return -1;  
  
  while(1);   /* ��������ִ�е����� */    
}

/**********************************************************************************************************
*�� �� ��: static void AppTaskCreate(void)
*����˵��: �û����񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */

  taskENTER_CRITICAL();           //�����ٽ���
  
  /* ����systemtip���� */
  xReturn = xTaskCreate((TaskFunction_t )systemtip, /* ������ں��� */
                        (const char*    )"systemtip",/* �������� */
                        (uint16_t       )512,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )2,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&systemtip_Handle);/* ������ƿ�ָ�� */
  if(pdPASS == xReturn)
    printf("����systemtip����ɹ�!\r\n");


  //�������������ݶ�ȡ�������
  GetSensorTaskCreate();

  //�������������ݴ����������
  SensorTaskCreate();

  //������������̬�������
  NavigationTaskCreate();
  
  //���п������񴴽�
  ControlTaskCreate();
  
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}
 
/**********************************************************************************************************
*�� �� ��: static void systemtip(void* parameter)
*����˵��: ϵͳ��ʾ����
*��    ��: ��
*�� �� ֵ: ��
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













