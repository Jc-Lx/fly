#ifndef __BEEPER_H__
#define __BEEPER_H__

#include "sys.h"
#include "led.h"

#define BEEP_OFF	    GPIO_ResetBits(GPIOC, GPIO_Pin_13);          
#define BEEP_ON		    GPIO_SetBits(GPIOC, GPIO_Pin_13); 
#define BEEP_TOGGLE 	GPIO_ToggleBits(GPIOC, GPIO_Pin_13);     
	

void beeperInit(void);


#endif
