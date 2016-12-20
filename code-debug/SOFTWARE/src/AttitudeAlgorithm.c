//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files
#include "System.h"
#include <math.h>
#include "AttitudeAlgorithm.h"
#include "arm_math.h"
IMU imu;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  *yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 180 / M_PI;
  *pitch = atan(gx / sqrt(gy*gy + gz*gz)) * 180 / M_PI;
  *roll = atan(gy / sqrt(gx*gx + gz*gz)) * 180 / M_PI;
}
  // ?????????
void EulerToQuaternion(const float *roll,const float *pitch,const float *yaw,float q[4])
{
   //z-yaw,x-roll,y-pitch
   float cx = cosf(*roll/2);
   float sx = sinf(*roll/2);
   float cy = cosf(*pitch/2);
   float sy = sinf(*pitch/2);
   float cz = cosf(*yaw/2);
   float sz = sinf(*yaw/2);
 
   q[0] = cx*cy*cz + sx*sy*sz;
   q[1] = sx*cy*cz - cx*sy*sz;
   q[2] = cx*sy*cz + sx*cy*sz;
   q[3] = cx*cy*sz - sx*sy*cz;
};
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
    // normal       better
    //0x5f3759df 0x5f375a86 
	i = 0x5f375a86 - (i>>1); 
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files


//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	250.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.025f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void MahonyAHRSInit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2f(ay, az);
    initialPitch = -atan2f(ax, az);
    

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);  
    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

}
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

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
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    static char init_flag = 0;
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	if(init_flag == 0) {
		MahonyAHRSInit(ax,ay,az,mx,my,mz);
		init_flag = 1;
	}    
    
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

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
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

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
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}



//====================================================================================================
// END OF CODE
//====================================================================================================
void q4ToDCM(float DCMgb[9])
{
    int i = 0;
    float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; 
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
		// Convert q->R, This R converts inertial frame to body frame.
		Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
		Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
		Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
		Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
		Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
		Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
		Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
		Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
		Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

		//1-2-3 Representation.
		//Equation (290) 
		//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
		// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.

		
		//DCM . ground to body
		for(i=0;i<9;i++)
		{
				*(&(DCMgb[0]) + i)=Rot_matrix[i];
		}
}




//卡尔曼滤波参数与函数
float dt_z=SYSTEM_CYCLE;//注意：dt的取值为kalman滤波器采样时间
float Q_angle_z=1.0f, Q_gyro_z=0.0; //角度数据置信度,角速度数据置信度
float R_angle_z=0.0f ,C_0_z = 1.0f; 
void Kalman_Filter_Z(float angle_m, float gyro_m,float *angle,float *gyro)//angleAx 和 gyroGy
{
    static float P[2][2] = {{ 1.0f, 0.0f },
                            { 0.0f, 1.0f }};
    static float Pdot[4] ={ 0.0f,0.0f,0.0f,0.0f};
    static float angle_kalmen, angle_dot;//角度和角速度
    static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    
    angle_kalmen+=(gyro_m-q_bias) * dt_z;
    angle_err = angle_m - angle_kalmen;
    Pdot[0]=Q_angle_z - P[0][1] - P[1][0];
    Pdot[1]=-P[1][1];
    Pdot[2]=-P[1][1];
    Pdot[3]=Q_gyro_z;
    P[0][0] += Pdot[0] * dt_z;
    P[0][1] += Pdot[1] * dt_z;
    P[1][0] += Pdot[2] * dt_z;
    P[1][1] += Pdot[3] * dt_z;
    PCt_0 = C_0_z * P[0][0];
    PCt_1 = C_0_z * P[1][0];
    E = R_angle_z + C_0_z * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0_z * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle_kalmen += K_0 * angle_err; //最优角度
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;//最优角速度
    //*angle = angle_kalmen;
    *gyro  = angle_dot;
}
//卡尔曼滤波参数与函数
float dt_a=0.25;//注意：dt的取值为kalman滤波器采样时间
float Q_angle_a=1.0f, Q_gyro_a=0.0; //角度数据置信度,角速度数据置信度
float R_angle_a=0.0f ,C_0_a = 1.0f; 
void Kalman_Filter_A(float angle_m, float gyro_m,float *angle)//angleAx 和 gyroGy
{
    static float P[2][2] = {{ 1.0f, 0.0f },
                            { 0.0f, 1.0f }};
    static float Pdot[4] ={ 0.0f,0.0f,0.0f,0.0f};
    static float angle_kalmen, angle_dot;//角度和角速度
    static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    
    angle_kalmen+=(gyro_m-q_bias) * dt_z;
    angle_err = angle_m - angle_kalmen;
    Pdot[0]=Q_angle_z - P[0][1] - P[1][0];
    Pdot[1]=-P[1][1];
    Pdot[2]=-P[1][1];
    Pdot[3]=Q_gyro_z;
    P[0][0] += Pdot[0] * dt_z;
    P[0][1] += Pdot[1] * dt_z;
    P[1][0] += Pdot[2] * dt_z;
    P[1][1] += Pdot[3] * dt_z;
    PCt_0 = C_0_z * P[0][0];
    PCt_1 = C_0_z * P[1][0];
    E = R_angle_z + C_0_z * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0_z * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle_kalmen += K_0 * angle_err; //最优角度
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;//最优角速度
    *angle = angle_kalmen;
    //*gyro  = angle_dot;
}





#define INTEGRAL_MAX 200
#define DIFFERENTIAL_MAX 200
#define PID_OUT_MAX  1000
#define dt         SYSTEM_CYCLE
void PID_Normal(PID_Typedef *PID)
{

	float Error = 0.0f;

	Error = PID->Target - PID->Measure;
    PID->Integral += Error*dt;
    if ( PID->Integral > INTEGRAL_MAX )
    {
        PID->Integral = INTEGRAL_MAX;
    }
    if ( PID->Integral < -INTEGRAL_MAX )
    {
        PID->Integral = -INTEGRAL_MAX;
    } 
    
    PID->Output = PID->Kp*Error + PID->Ki*PID->Integral + PID->Kd*PID->Diff;
    if ( PID->Output > PID_OUT_MAX )
    {
        PID->Output = PID_OUT_MAX;
    }
    if ( PID->Output < -PID_OUT_MAX )
    {
        PID->Output = -PID_OUT_MAX;
    }
}
void PID_Postion(PID_Typedef *PID,unsigned char Rata)
{
	float Error = 0.0f;
	Error = PID->Target - PID->Measure;
    if ( Rata )
    {
            PID->Integral += Error*dt;
    }
    else
    {
           PID->Integral += Error*0.02;
    }
     
    if ( PID->Integral > PID->IntegralLimit )
    {
        PID->Integral = PID->IntegralLimit;
    }
    if ( PID->Integral < -PID->IntegralLimit )
    {
        PID->Integral = -PID->IntegralLimit;
    } 
    if ( Rata )
    {
        PID->Diff = (Error - PID->LastError)*SYSTEM_FREQ;
    }


    PID->Output = PID->Kp*Error + PID->Ki*PID->Integral + PID->Kd*PID->Diff;
    PID->LastError = Error;
    if ( PID->Output > PID_OUT_MAX )
    {
        PID->Output = PID_OUT_MAX;
    }
    if ( PID->Output < -PID_OUT_MAX )
    {
        PID->Output = -PID_OUT_MAX;
    }
}