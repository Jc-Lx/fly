#ifndef __LED_H
#define __LED_H

#include "sys.h"
#include "delay.h"
#include "nvic.h"
#include "ws2812.h"

#define LED0_ON 		GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define LED0_OFF 		GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define LED0_TOGGLE 	GPIO_ToggleBits(GPIOB, GPIO_Pin_9)


void ledInit(void);

void ledStripInit(void);
void ledStripON(void);
void ledStripOFF(void);

#endif
