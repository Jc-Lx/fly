#include "pwm.h"

/**********************************************************************************************************
*函 数 名: static void MOTOR_GPIOConfig(void)
*功能说明: 电机PWM输出GPIO设置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MOTOR_GPIOConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    //MOTOR 1 右下
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);

    //MOTOR 2 右上
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);

    //MOTOR 3 左下
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);

    //MOTOR 4 左上
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);

}

/**********************************************************************************************************
*函 数 名: static void MOTOR_TIMConfig(void)
*功能说明: 电机PWM输出TIM设置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MOTOR_TIMConfig(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    
    TIM_TimeBaseStructure.TIM_Period = 2000 - 1;   //输出PWM周期2ms 500Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;//100KHz   配置定时器时基1us计数一次
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    //MOTOR 1 
    TIM_OC3Init(TIM3,&TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);

    //MOTOR 2 
    TIM_OC4Init(TIM3,&TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);

    //MOTOR 3 
    TIM_OC2Init(TIM3,&TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);

    //MOTOR 4 
    TIM_OC1Init(TIM3,&TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);

    TIM_Cmd(TIM3,ENABLE);
    TIM_ARRPreloadConfig(TIM3,ENABLE);

}

/**********************************************************************************************************
*函 数 名: void MOTOR_Init(void)
*功能说明: 电机PWM输出初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MOTOR_Init(void)
{
    MOTOR_GPIOConfig();
    MOTOR_TIMConfig();
}

/**********************************************************************************************************
*函 数 名: MotorPWMSet
*功能说明: 电机PWM输出值设置
*形    参: 电机号 PWM值（1100-1900）
*返 回 值: 无
**********************************************************************************************************/
void MotorPWMSet(uint8_t motor, uint16_t pwmValue)
{
    if(motor == 1)
    {
        TIM3->CCR3 = pwmValue;
    }
    else if(motor == 2)
    {
        TIM3->CCR4 = pwmValue;
    }
    else if(motor == 3)
    {
        TIM3->CCR2 = pwmValue;
    }
    else if (motor == 4)
    {
        TIM3->CCR1 = pwmValue;
    }
    
}

