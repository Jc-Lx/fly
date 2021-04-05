#ifndef __DELAY_H
#define __DELAY_H 			   
	  
#include "sys.h"

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay_xms(u32 nms);

void  delay_ns(uint32_t ns);
uint64_t GetSysTimeUs(void);
uint32_t GetSysTimeMs(void);

#endif




























