#include "runtime_config.h"
#include "beeper.h"
#include "led.h"
#include "ledstrip.h"
#include "sensors.h"
#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "barometer.h"

#include "pos_estimator.h"
#include "state_control.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ���м�����	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

uint32_t armingFlags = 0;
uint32_t stateFlags = 0;
uint32_t flightModeFlags = 0;


//ʹ�ܸ�������ģʽ
uint32_t enableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

//���ܸ�������ģʽ
uint32_t disableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

//У׼������
bool isCalibrating(void)
{
	if (!baroIsCalibrationComplete()) 
	{
		return true;
	}

	if (!posEstimatorIsCalibrationComplete()) 
	{
		return true;
	}

	if (!accIsCalibrationComplete()) 
	{
		return true;
	}

	if (!gyroIsCalibrationComplete()) 
	{
		return true;
	}

    return false;
}

//���½�����־λ״̬
void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
        LED0_ON;
    } 
	else 
	{
		//������У׼��
		static bool calibratingFinishedBeep = false;
		if (isCalibrating()) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);
			calibratingFinishedBeep = false;
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);

			if (!calibratingFinishedBeep) 
			{
				calibratingFinishedBeep = true;
				beeper(BEEPER_RUNTIME_CALIBRATION_DONE);
			}
		}

		//���ˮƽ��С��
		if (!STATE(SMALL_ANGLE)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}

		//�����ٶȼ��Ƿ��Ѿ�У׼���
		if (!STATE(ACCELEROMETER_CALIBRATED)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}

		//���������Ƿ�У׼
		if (sensorsIsMagPresent() && !STATE(COMPASS_CALIBRATED)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_COMPASS_NOT_CALIBRATED);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_COMPASS_NOT_CALIBRATED);
		}
		
		//����Ƿ���дFlash
		if (STATE(FLASH_WRITING))
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		
		//ˢ��LED��״̬
        if (!isArmingDisabled()) 
		{
            warningLedFlash();
        } 
		else 
		{
            warningLedON();
        }
		
        warningLedUpdate();
    }
}

//�����ɿ�
void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
        DISABLE_ARMING_FLAG(ARMED);
        beeper(BEEPER_DISARMING); 
    }
}

//�����ɿ�
void mwArm(void)
{
    updateArmingStatus();

    if (!isArmingDisabled()) 
	{
        if (ARMING_FLAG(ARMED)) 
		{
            return;
        }
		ENABLE_ARMING_FLAG(ARMED);
		ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
		
		stateControlResetYawHolding();//��λ���������ֵ
        beeper(BEEPER_ARMING);
        return;
    }

    if (!ARMING_FLAG(ARMED)) 
	{
        beeperConfirmationBeeps(1);
    }
}

