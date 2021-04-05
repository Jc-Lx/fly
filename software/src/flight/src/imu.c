#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "axis.h"
#include "maths.h"
#include "imu.h"
#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "runtime_config.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ��̬������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

/**	
 * ��̬����������£�
 *     ROLL  = ��X����ת�����ֶ�����ʱ��Ϊ��˳ʱ��Ϊ����
 *     PITCH = ��Y����ת�����ֶ�����ʱ��Ϊ��˳ʱ��Ϊ����
 *     YAW   = ��Z����ת�����ֶ�����ʱ��Ϊ��˳ʱ��Ϊ����
 */

#define DCM_KP_ACC			0.600f		//���ٶȲ���������PI����
#define DCM_KI_ACC			0.005f

#define DCM_KP_MAG			1.000f		//�����Ʋ���������PI����
#define DCM_KI_MAG			0.000f

#define IMU_SMALL_ANGLE		15.0f		//����ˮƽ״̬����С�Ƕȣ���λdeg��

#define SPIN_RATE_LIMIT     20			//��ת����

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//��Ԫ��
static float rMat[3][3];//��Ԫ������ת����		
static float smallAngleCosZ;//ˮƽ��С������ֵ

static void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuInit(void)
{
	smallAngleCosZ = cos_approx(degreesToRadians(IMU_SMALL_ANGLE));//��С�������ֵ
    imuComputeRotationMatrix();
}

void imuTransformVectorBodyToEarth(Axis3f *v)
{
    const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
    const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
    const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;

    v->x = x;
    v->y = -y;//
    v->z = z;
}

void imuTransformVectorEarthToBody(Axis3f *v)
{
    v->y = -v->y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
    const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
    const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

    v->x = x;
    v->y = y;
    v->z = z;
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

//�����ƿ�������
static float imuMagFastPGainSaleFactor(void)
{
	//�����ϵ��������ں���Ҫһ��ʱ��
	//Ϊ�˿����ںϣ�ǰ100��ʹ�ÿ�������
	static u32 magFastPGainCount = 100;
	
	if (!ARMING_FLAG(ARMED) && (magFastPGainCount--))
		return 10.0f;
	else
		return 1.0f;
}

static void imuMahonyAHRSupdate(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz,
								bool useMag,float dt)
{
	static float integralAccX = 0.0f,  integralAccY = 0.0f, integralAccZ = 0.0f;    //���ٶȻ������
	static float integralMagX = 0.0f,  integralMagY = 0.0f, integralMagZ = 0.0f;    //�����ƻ������
	float ex, ey, ez;

    //������ת����(rad/s)
    const float spin_rate_sq = sq(gx) + sq(gy) + sq(gz);

    //Step 1: Yaw correction
    if (useMag) 
	{
		const float magMagnitudeSq = mx * mx + my * my + mz * mz;
		float kpMag = DCM_KP_MAG * imuMagFastPGainSaleFactor();
		
		if (magMagnitudeSq > 0.01f) 
		{
			//��λ�������Ʋ���ֵ
			const float magRecipNorm = invSqrt(magMagnitudeSq);
			mx *= magRecipNorm;
			my *= magRecipNorm;
			mz *= magRecipNorm;
		
			//����X\Y����Ĵ�ͨ���ű������ͨ
			const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
			const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
			const float bx = sqrtf(hx * hx + hy * hy);

			//����������ǹ��ƴű��Ͳ����ű�֮��Ľ���˻�
			const float ez_ef = -(hy * bx);

			//��ת����������ϵ
			ex = rMat[2][0] * ez_ef;
			ey = rMat[2][1] * ez_ef;
			ez = rMat[2][2] * ez_ef;
		}
		else 
		{
			ex = 0;
			ey = 0;
			ez = 0;
		}

		//�ۼ�����
		if (DCM_KI_MAG > 0.0f) 
		{
			//�����ת���ʴ�������ֵ��ֹͣ����
			if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) 
			{
				integralMagX += DCM_KI_MAG * ex * dt;
				integralMagY += DCM_KI_MAG * ey * dt;
				integralMagZ += DCM_KI_MAG * ez * dt;

				gx += integralMagX;
				gy += integralMagY;
				gz += integralMagZ;
			}
		}
		
		//����
		gx += kpMag * ex;
		gy += kpMag * ey;
		gz += kpMag * ez;
	}

	
    //Step 2: Roll and pitch correction
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		//��λ�����ټƲ���ֵ
		const float accRecipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= accRecipNorm;
		ay *= accRecipNorm;
		az *= accRecipNorm;

		//���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���
		ex = (ay * rMat[2][2] - az * rMat[2][1]);
		ey = (az * rMat[2][0] - ax * rMat[2][2]);
		ez = (ax * rMat[2][1] - ay * rMat[2][0]);

		//�ۼ�����
		if (DCM_KI_ACC > 0.0f) 
		{
			//�����ת���ʴ�������ֵ��ֹͣ����
			if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)))
			{
				integralAccX += DCM_KI_ACC * ex * dt;
				integralAccY += DCM_KI_ACC * ey * dt;
				integralAccZ += DCM_KI_ACC * ez * dt;

				gx += integralAccX;
				gy += integralAccY;
				gz += integralAccZ;
			}
		}

		//����
		gx += DCM_KP_ACC * ex;
		gy += DCM_KP_ACC * ey;
		gz += DCM_KP_ACC * ez;
	}
	
	//һ�׽����㷨����Ԫ���˶�ѧ���̵���ɢ����ʽ�ͻ���
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

	//��λ����Ԫ��
    const float quatRecipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;

    //������Ԫ������ת����
    imuComputeRotationMatrix();
}


float imuAttitudeYaw;//��Χ��-180~180�������ϴ���������λ����֧�ַ�Χ-180~180��

//����ŷ����
static void imuUpdateEulerAngles(attitude_t *attitude)
{
	attitude->roll = RADIANS_TO_DEGREES(atan2_approx(rMat[2][1], rMat[2][2]));
	attitude->pitch = RADIANS_TO_DEGREES((0.5f * M_PIf) - acos_approx(-rMat[2][0]));//arcsin = 0.5PI - arccos
	attitude->yaw = RADIANS_TO_DEGREES(atan2_approx(rMat[1][0], rMat[0][0]));

	imuAttitudeYaw = attitude->yaw;

	if (attitude->yaw < 0.0f)//ת��λ0~360
		attitude->yaw += 360.0f;

	//������С���״̬
	if (rMat[2][2] > smallAngleCosZ) 
		ENABLE_STATE(SMALL_ANGLE);
	else 
		DISABLE_STATE(SMALL_ANGLE);
}

void imuUpdateAttitude(const sensorData_t *sensorData, state_t *state, float dt)
{
	bool useMag = compassIsHealthy();
	
	Axis3f gyro = sensorData->gyro;
	Axis3f acc  = sensorData->acc;
	Axis3f mag  = sensorData->mag;
	
	//���ٶȵ�λ�ɶ�תΪ����
	gyro.x = gyro.x * DEG2RAD;
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;
	
	//������Ԫ������ת����
    imuMahonyAHRSupdate(gyro.x, gyro.y, gyro.z,
                        acc.x, acc.y, acc.z,
                        mag.x, mag.y, mag.z,
						useMag,dt);
	
    //����ŷ����               
    imuUpdateEulerAngles(&state->attitude);
}





