#ifndef _RC_H_
#define _RC_H_

#include "delay.h"
#include "mathtool.h"
#include "cppm.h"
#include "flightControl.h"
#include "flightStatus.h"


typedef struct
{
    int16_t roll;       //横滚
    int16_t pitch;      //俯仰
    int16_t yaw;        //偏航
    int16_t throttle;   //油门
} RCCOMMAND_t;

static void RcCheckSticks(void);
static void RcCommandUpdate(void);
static void RcCheckFailsafe(void);
void RcInit(void);
void RcCheck(void);
RCDATA_t GetRcData(void);
RCCOMMAND_t GetRcCommad(void);

#endif

