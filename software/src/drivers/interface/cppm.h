#ifndef __CPPM_H
#define __CPPM_H

#include "sys.h"
#include "usart.h"

typedef struct
{
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t throttle;
} RCDATA_t;


void CPPM_Init(void);
void PPM_Decode(void);
RCDATA_t GetRawRcData(void);

#endif


