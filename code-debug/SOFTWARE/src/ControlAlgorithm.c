#include "System.h"
#include "ControlAlgorithm.h"
#include "AttitudeAlgorithm.h"
#include "math.h"
#include "MS5611.h"
//
nav_t nav;		//NED frame in earth
//
float z_est[3];	// estimate z Vz  Az
static float w_z_baro=0.5f;
static float w_z_acc=20.0f;
static float w_acc_bias=0.05f;

/* acceleration in NED frame */
float accel_NED[3] = { 0.0f, 0.0f, -CONSTANTS_ONE_G };

/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
float corr_acc[] = { 0.0f, 0.0f, 0.0f };	// N E D ,  m/s2
float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame ,  
float corr_baro = 0.0f;					//m 
//float accb[3]={0,0,0};


//Combine Filter to correct err
void inertial_filter_predict(float dt, float x[3])
{
	x[0] += x[1] * dt + x[2] * dt * dt / 2.0f;
	x[1] += x[2] * dt;
}

void inertial_filter_correct(float e, float dt, float x[3], int i, float w)
{
	float ewdt = e * w * dt;
	x[i] += ewdt;

	if (i == 0) {
		x[1] += w * ewdt;
		x[2] += w * w * ewdt / 3.0;

	} else if (i == 1) {
		x[2] += w * ewdt;
	}
}
 
 


//timeStamp in us. Thread should be executed every 2~20ms
//MS5611_Altitude  , should be in m. (can be fixed to abs, not relative). positive above ground
//accFilted  ,should be filted .
uint8_t accUpdated = 0;
void AltitudeCombineThread(void)
{
	static uint32_t tPre=0;
	uint32_t t;
	float dt;
	static float posZPrev=0;
	
	/* accelerometer bias correction */
	float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
	uint8_t i,j;
    dt = 0.024;
    if(!paOffsetInited)	//wait baro to init its offset
        return;

	//store err when sensor update 
	if(Baro_ALT_Updated)	//?????sensor??????timeStamp,??????
	{
			corr_baro = 0 - MS5611_Altitude - z_est[0];		// MS5611_Altitude baro alt, is postive above offset level. not in NED. z_est is in NED frame. 
			Baro_ALT_Updated=0;
	}
 
	if(accUpdated)
	{			
			 imu.acc_x -= acc_bias[0];
			 imu.acc_y -= acc_bias[1];
			 imu.acc_z -= acc_bias[2];
		
	#ifndef ASSUME_LEVEL	
			for(i=0;i<3;i++)
			{
				accel_NED[i]=0.0f;
				for(j=0;j<3;j++)
				{
						accel_NED[i]+=imu.DCM[j][i] * (*((&imu.acc_x)+j));
				}
			} 
			accel_NED[2]=-accel_NED[2];
		#else
			accel_NED[2]=-accb[2];
		#endif
			
			corr_acc[2] = accel_NED[2] + CONSTANTS_ONE_G - z_est[2];
			
			accUpdated=0;
	}
	
	//correct accelerometer bias every time step 
	accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;

	#ifndef ASSUME_LEVEL
	//transform error vector from NED frame to body frame
		for (i = 0; i < 3; i++) 
	 {
			float c = 0.0f;

			for (j = 0; j < 3; j++) {
				c += imu.DCM[i][j] * accel_bias_corr[j];
			}

			acc_bias[i] += c * w_acc_bias * dt;		//accumulate bias
		} 
		
		acc_bias[2]=-acc_bias[2];

	#else
	 	{
	 	float cz= -accel_bias_corr[2];
		acc_bias[2]+=cz* w_acc_bias * dt;
	 	}
	#endif
		
		/* inertial filter prediction for altitude */
		inertial_filter_predict(dt, z_est);
		/* inertial filter correction for altitude */
		inertial_filter_correct(corr_baro, dt, z_est, 0, w_z_baro);	//0.5f
		inertial_filter_correct(corr_acc[2], dt, z_est, 2, w_z_acc);		//20.0f
		
		nav.z=z_est[0];
		nav.vz=z_est[1];
		nav.az=z_est[2];	
}



#define MotorLU(pwm) TIM_SetCompare4(TIM8,(pwm));
#define MotorRU(pwm) TIM_SetCompare3(TIM8,(pwm));
#define MotorLD(pwm) TIM_SetCompare1(TIM8,(pwm));
#define MotorRD(pwm) TIM_SetCompare2(TIM8,(pwm));
#define MotorLM(pwm) TIM_SetCompare3(TIM2,(pwm));
#define MotorRM(pwm) TIM_SetCompare4(TIM2,(pwm));

#define PWM_MAX 1999
//油门抑制 保证向上，向下加速度接近0
//Yaw校正 保证水平偏航角速度为 0
//  PWM输出            基准油门         油门抑制         Pitch校正      Roll校正      Yaw校正   
void PWM_Out( int16_t Throttle,int16_t Restrain,int16_t dPitch,int16_t dRoll,int16_t dYaw)
{
    int PWM_LU = 0;
    int PWM_RU = 0;
    int PWM_LD = 0;
    int PWM_RD = 0;
    if ( Throttle <= 0 )
    {
        //PWM输出0
        MotorLU(0);	
        MotorRU(0);	
        MotorLD(0);	
        MotorRD(0);	
        return;
    }
//  PWM输出  基准油门   油门抑制  Pitch校正 Roll校正b Yaw校正    
	PWM_LU = Throttle - Restrain + dPitch + dRoll + dYaw;
	PWM_RU = Throttle - Restrain + dPitch - dRoll - dYaw;	
	PWM_LD = Throttle - Restrain - dPitch + dRoll - dYaw;
	PWM_RD = Throttle - Restrain - dPitch - dRoll + dYaw;
    if ( PWM_LU < 0 )PWM_LU = 0;
    if ( PWM_RU < 0 )PWM_RU = 0;
    if ( PWM_LD < 0 )PWM_LD = 0;
    if ( PWM_RD < 0 )PWM_RD = 0;
    if ( PWM_LU > PWM_MAX )PWM_LU = PWM_MAX;
    if ( PWM_RU > PWM_MAX )PWM_RU = PWM_MAX;
    if ( PWM_LD > PWM_MAX )PWM_LD = PWM_MAX;
    if ( PWM_RD > PWM_MAX )PWM_RD = PWM_MAX;    
    MotorLU(PWM_LU);	
    MotorRU(PWM_RU);	
    MotorLD(PWM_LD);	
    MotorRD(PWM_RD);	
}