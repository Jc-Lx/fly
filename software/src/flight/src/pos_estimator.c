#include "pos_estimator.h"
#include "maths.h"
#include "axis.h"
#include "runtime_config.h"
#include "accelerometer.h"
#include "imu.h"
#include "nvic.h"
#include "barometer.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * λ��Ԥ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define  W_Z_BARO_P   0.28f		//��ѹ����Ȩ��
#define  W_ACC_BIAS   0.01f		//���ٶȼ�����Ȩ��

typedef struct 
{
    float       alt; //(cm)
    float       epv;
} navPositionEstimatorBARO_t;

typedef struct 
{
    Axis3f 		pos;
    Axis3f 		vel;
    float       eph;
    float       epv;
} posEstimatorEst_t;

typedef struct 
{
    Axis3f     accelNEU;
    Axis3f     accelBias;
    bool       gravityCalibrationComplete;
} navPosisitonEstimatorIMU_t;

typedef struct 
{
    // Data sources
    navPositionEstimatorBARO_t  baro;
    navPosisitonEstimatorIMU_t  imu;

    // Estimate
    posEstimatorEst_t  est;
	
} posEstimator_t;

posEstimator_t posEstimator;


//���ٶ�Ԥ��λ�ƺ��ٶ�
static void posAndVelocityPredict(int axis, float dt, float acc)
{
    posEstimator.est.pos.axis[axis] += posEstimator.est.vel.axis[axis] * dt + acc * dt * dt / 2.0f;
    posEstimator.est.vel.axis[axis] += acc * dt;
}

//�������Ԥ����λ�ƺ��ٶ�
static void posAndVelocityCorrect(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    posEstimator.est.pos.axis[axis] += ewdt;
    posEstimator.est.vel.axis[axis] += w * ewdt;
}

//�������Ԥ�����ٶ�
static void inavFilterCorrectVel(int axis, float dt, float e, float w)
{
    posEstimator.est.vel.axis[axis] += e * w * dt;
}

//������ѹ������
static void updateBaroTopic(float newBaroAlt)
{
	if (baroIsCalibrationComplete()) 
	{
		posEstimator.baro.alt = newBaroAlt;
	}
	else
	{
		posEstimator.baro.alt = 0.0f;
	}
}

//����NEU����ϵ�ļ��ٶ�
//accBF��Ϊ��������ϵ���ٶ�
static void updateIMUTopic(const Axis3f accBF)
{
    static float calibratedGravityCMSS = GRAVITY_CMSS;
    static u32 gravityCalibrationTimeout = 0;
	Axis3f accelCMSS;

	//��������ϵ�ļ��ٶ�תΪ��cm/ss��
	accelCMSS.x = accBF.x * GRAVITY_CMSS;
	accelCMSS.y = accBF.y * GRAVITY_CMSS;
	accelCMSS.z = accBF.z * GRAVITY_CMSS;

	//ȥ�����ٶ�ƫ��
	accelCMSS.x -= posEstimator.imu.accelBias.x;
	accelCMSS.y -= posEstimator.imu.accelBias.y;
	accelCMSS.z -= posEstimator.imu.accelBias.z;

	//��ת��������ϵ���ٶȵ�NEU����ϵ
	imuTransformVectorBodyToEarth(&accelCMSS);
	
	//��ˮƽ״̬У׼��������ļ��ٶ���ƫ
	if (!posEstimator.imu.gravityCalibrationComplete && STATE(SMALL_ANGLE)) 
	{
		//��������У׼������ƫ
		const float gravityOffsetError = accelCMSS.z - calibratedGravityCMSS;
		calibratedGravityCMSS += gravityOffsetError * 0.0025f;

		if (ABS(gravityOffsetError) < 5)//���ҪС��5cm/ss
		{
			if ((getSysTickCnt() - gravityCalibrationTimeout) > 250) 
			{
				posEstimator.imu.gravityCalibrationComplete = true;
			}
		}
		else 
		{
			gravityCalibrationTimeout = getSysTickCnt();
		}
	}
	
	//NEU����ϵ���ٶȴ���
	if (posEstimator.imu.gravityCalibrationComplete) 
	{
		accelCMSS.z -= calibratedGravityCMSS;//ȥ������
		for (int axis = 0; axis < 3; axis++)
		{
			applyDeadband(accelCMSS.axis[axis], 4);//ȥ��4(cm/ss)����
			posEstimator.imu.accelNEU.axis[axis] += (accelCMSS.axis[axis] - posEstimator.imu.accelNEU.axis[axis]) * 0.3f;//һ�׵�ͨ
		}
	}
	else 
	{
		posEstimator.imu.accelNEU.x = 0;
		posEstimator.imu.accelNEU.y = 0;
		posEstimator.imu.accelNEU.z = 0;
	}
}

//�ٶ�Ԥ����λ��Ԥ��
static void updateEstimatedTopic(float dt)
{
	//ʹ�ü��ٶ�Ԥ��λ�ƺ��ٶ�
	posAndVelocityPredict(Z, dt, posEstimator.imu.accelNEU.z);
	posAndVelocityPredict(Y, dt, posEstimator.imu.accelNEU.y);
	posAndVelocityPredict(X, dt, posEstimator.imu.accelNEU.x);
	
    //���ٶ�ƫ��ֵ
    const bool updateAccBias = (W_ACC_BIAS > 0);
    Axis3f accelBiasCorr = { { 0, 0, 0} };

	//ʹ����ѹ�Ƹ߶��������Ԥ����λ�ƺ��ٶȣ�Z�ᣩ
	const float baroResidual =   posEstimator.baro.alt - posEstimator.est.pos.z;
	posAndVelocityCorrect(Z, dt, baroResidual, W_Z_BARO_P);
	if (updateAccBias) 
	{
		accelBiasCorr.z -= baroResidual * sq(W_Z_BARO_P);
	}
	
	//�������ٶ�ƫ��ֵ
    if (updateAccBias) 
	{
        const float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
        if (accelBiasCorrMagnitudeSq < sq(GRAVITY_CMSS * 0.25f))//ƫ��С��0.25Gʱ�������� 
		{
			//�����ٶ�ƫ��ֵ����������ϵת��Ϊ��������ϵ
            imuTransformVectorEarthToBody(&accelBiasCorr);
			
            posEstimator.imu.accelBias.x += accelBiasCorr.x * W_ACC_BIAS * dt;
            posEstimator.imu.accelBias.y += accelBiasCorr.y * W_ACC_BIAS * dt;
            posEstimator.imu.accelBias.z += accelBiasCorr.z * W_ACC_BIAS * dt;
        }
    }
}

//Ԥ������ʼ��
static void initializePositionEstimator(void)
{
    posEstimator.imu.gravityCalibrationComplete = false;
	
    for (int axis = 0; axis < 3; axis++) 
	{
        posEstimator.imu.accelBias.axis[axis] = 0;
        posEstimator.est.pos.axis[axis] = 0;
        posEstimator.est.vel.axis[axis] = 0;
    }
}

//����Ԥ��λ�ú��ٶ�
static void publishEstimatedTopic(state_t *state)
{
	static u32 publishTime;
	
	//������������ϵ�ļ��ٶ�
	state->acc.x = posEstimator.imu.accelNEU.x;
	state->acc.y = posEstimator.imu.accelNEU.y;
	state->acc.z = posEstimator.imu.accelNEU.z;
	
	//���¹��Ƶ�λ�ú��ٶȣ�10ms->100Hz��
	if ((getSysTickCnt() - publishTime) >= 10)
	{
		state->position.x = posEstimator.est.pos.x;
		state->position.y = posEstimator.est.pos.y;
		state->position.z = posEstimator.est.pos.z;
		
		state->velocity.x = posEstimator.est.vel.x;
		state->velocity.y = posEstimator.est.vel.y;
		state->velocity.z = constrainf(posEstimator.est.vel.z, -150.0f, 150.0f);//����Z����ٶ�Ϊ150cm/s
		
		publishTime = getSysTickCnt();
	}
}

//����Ԥ����������ѭ������
void updatePositionEstimator(const sensorData_t *sensorData, state_t *state, float dt)
{
    static bool isInitialized = false;
	
	//��ʼ��Ԥ����
	if (!isInitialized) 
	{
		initializePositionEstimator();
		isInitialized = true;
	}
	
	//����Ԥ��������ѹ�߶�
	updateBaroTopic(sensorData->baro.asl);
	
    //����Ԥ�����ļ��ٶȣ���������ϵ��
    updateIMUTopic(sensorData->acc);
	
    //Ԥ���ٶȺ�λ��
    updateEstimatedTopic(dt);

    //����Ԥ�����ٶȺ�λ��
    publishEstimatedTopic(state);
}

bool posEstimatorIsCalibrationComplete(void)
{
    return posEstimator.imu.gravityCalibrationComplete;
}

void posEstimatorReset(void)
{
	for (int axis = 0; axis < 3; axis++) 
	{
		posEstimator.imu.accelBias.axis[axis] = 0;
		posEstimator.est.pos.axis[axis] = 0;
		posEstimator.est.vel.axis[axis] = 0;
	}
	posEstimator.est.pos.z = posEstimator.baro.alt;
}

