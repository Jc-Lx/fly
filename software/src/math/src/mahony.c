#include "mahony.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	50.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 15.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.01f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
Vector3f_t angle;

// static float_t invSqrt(float_t temp);

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// 定义一些辅助变量用于转换矩阵
    float q0q0 = q0*q0;  
    float q0q1 = q0*q1;  
    float q0q2 = q0*q2;  
    float q0q3 = q0*q3;  
    float q1q1 = q1*q1;  
    float q1q2 = q1*q2;  
    float q1q3 = q1*q3;  
    float q2q2 = q2*q2;  
    float q2q3 = q2*q3;  
    float q3q3 = q3*q3; 

    //转换角速度单位为弧度
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{

		// Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
		recipNorm = sqrtf(ax * ax + ay * ay + az * az);
		ax /= recipNorm;
		ay /= recipNorm;
		az /= recipNorm;  
 
		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

        // Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
	
		// Compute and apply integral feedback if enabled
        // 在PI补偿器中积分项使能情况下计算并应用积分项
		if(twoKi > 0.0f) 
		{
            // integral error scaled by Ki
            // 积分过程
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            
            // apply integral feedback
            // 应用误差补偿中的积分项
			gx += integralFBx;	
			gy += integralFBy;
			gz += integralFBz;
		}
		else 
		{
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
			integralFBx = 0.0f;	
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
        // 应用误差补偿中的比例项
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
    // 微分方程迭代求解
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz); 
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
	recipNorm = sqrtf(q0q0 + q1q1 + q2q2 + q3q3);
	q0 /= recipNorm;
	q1 /= recipNorm;
	q2 /= recipNorm;
	q3 /= recipNorm;
    
    //Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
    //姿态解算预计算

    //9.四元数转欧拉角输出
   angle.x = asinf(2 * (q0q1 + q2q3))*57.3f;
   angle.y = atan2(-2 * (q1q3 - q0q2),q0q0 - q1q1 - q2q2 + q3q3)*57.3f;
   angle.z = atan2(2 * (q1q2 - q0q3), q0q0 - q1q1 + q2q2 - q3q3)*57.3f;
   
}

/**********************************************************************************************************
*函 数 名: GetCopterAngle
*功能说明: 获取表示飞行器姿态的欧拉角
*形    参: 无
*返 回 值: 角度值
**********************************************************************************************************/
Vector3f_t GetCopterAngle(void)
{
    return angle;
}


void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float recipNorm;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

    // 定义一些辅助变量用于转换矩阵
    float q0q0 = q0*q0;  
    float q0q1 = q0*q1;  
    float q0q2 = q0*q2;  
    float q0q3 = q0*q3;  
    float q1q1 = q1*q1;  
    float q1q2 = q1*q2;  
    float q1q3 = q1*q3;  
    float q2q2 = q2*q2;  
    float q2q3 = q2*q3;  
    float q3q3 = q3*q3; 
 
    //转换角速度单位为弧度
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    //因为系统中磁力计数据读取滞后,所以在无磁力计数据时不做融合
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}
 
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
 
		// Normalise accelerometer measurement
		recipNorm = sqrtf(ax * ax + ay * ay + az * az);
		ax /= recipNorm;
		ay /= recipNorm;
		az /= recipNorm;     
 
		// Normalise magnetometer measurement
		recipNorm = sqrtf(mx * mx + my * my + mz * mz);
		mx /= recipNorm;
		my /= recipNorm;
		mz /= recipNorm;   
 
        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   
 
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
 
		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
        if(GetMagMag() > 2.0f)//磁干扰过大时不融合磁力计数据
        {
            halfex = (ay * halfvz - az * halfvy);
		    halfey = (az * halfvx - ax * halfvz);
		    halfez = (ax * halfvy - ay * halfvx);
        }
        else
        {
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);    
        }
 
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
 
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = sqrtf(q0q0 + q1q1 + q2q2 + q3q3);
	q0 /= recipNorm;
	q1 /= recipNorm;
	q2 /= recipNorm;
	q3 /= recipNorm;

    //9.四元数转欧拉角输出
    angle.x = asinf(2 * (q0q1 + q2q3))*57.3f;
    angle.y = atan2(-2 * (q1q3 - q0q2),q0q0 - q1q1 - q2q2 + q3q3)*57.3f;
    angle.z = atan2(2 * (q1q2 - q0q3), q0q0 - q1q1 + q2q2 - q3q3)*57.3f;
   //angle.z = WrapDegree360(angle.z);
    //    printf("\r\n    angle:    pith:%f    roll:%f    yaw:%f    \r\n",angle.x,angle.y,angle.z);
}
