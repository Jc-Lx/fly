#ifndef __SOFT_IIC_GPIO_H
#define __SOFT_IIC_GPIO_H

#include "stm32f4xx.h"
#include "inttypes.h"

#define IIC_WR	0		/* 写控制bit */
#define IIC_RD	1		/* 读控制bit */


/* 定义IIC总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define IIC_GPIO_PORT				GPIOB			            /* GPIO端口 */
#define IIC_GPIO_CLK			 	RCC_AHB1Periph_GPIOB		/* GPIO端口时钟 */
#define IIC_SCL_PIN					GPIO_Pin_6			        /* 连接到SCL时钟线的GPIO */
#define IIC_SDA_PIN					GPIO_Pin_7			        /* 连接到SDA数据线的GPIO */


/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
#if 1	/* 条件编译： 1 选择GPIO的库函数实现IO读写 */
	#define IIC_SCL_1()  GPIO_SetBits(IIC_GPIO_PORT, IIC_SCL_PIN)		/* SCL = 1 */
	#define IIC_SCL_0()  GPIO_ResetBits(IIC_GPIO_PORT, IIC_SCL_PIN)		/* SCL = 0 */
	
	#define IIC_SDA_1()  GPIO_SetBits(IIC_GPIO_PORT, IIC_SDA_PIN)		/* SDA = 1 */
	#define IIC_SDA_0()  GPIO_ResetBits(IIC_GPIO_PORT, IIC_SDA_PIN)		/* SDA = 0 */
	
	#define IIC_SDA_READ()  GPIO_ReadInputDataBit(IIC_GPIO_PORT, IIC_SDA_PIN)	/* 读SDA口线状态 */
#else	/* 这个分支选择直接寄存器操作实现IO读写 */
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
	#define IIC_SCL_1()  IIC_GPIO_PORT->BSRRL = IIC_SCL_PIN				/* SCL = 1 */
	#define IIC_SCL_0()  IIC_GPIO_PORT->BSRRH = IIC_SCL_PIN				/* SCL = 0 */
	
	#define IIC_SDA_1()  IIC_GPIO_PORT->BSRRL = IIC_SDA_PIN				/* SDA = 1 */
	#define IIC_SDA_0()  IIC_GPIO_PORT->BSRRH = IIC_SDA_PIN				/* SDA = 0 */
	
	#define IIC_SDA_READ()  ((IIC_GPIO_PORT->IDR & IIC_SDA_PIN) != 0)	/* 读SDA口线状态 */
#endif


void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(uint8_t _ucByte);
uint8_t IIC_ReadByte(uint8_t ack);
uint8_t IIC_WaitAck(void);
void IIC_Ack(void);
void IIC_NAck(void);
uint8_t IIC_Init(uint8_t _Address);
//void IIC_CfgGpio(void);


#endif




