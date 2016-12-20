#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "pwm.h"

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define ALT_THREAD_PRD  5000	//us . store error when sensor updates, but correct on each time step to avoid jumps in estimated value 
typedef struct NAV_tt
{
float x;
float y;
float z;
float vx;
float vy;
float vz;
float ax;
float ay;
float az;
}nav_t;
extern nav_t nav;
extern float z_est[3];	// estimate z Vz  Az
void inertial_filter_predict(float dt, float x[3]);
void inertial_filter_correct(float e, float dt, float x[3], int i, float w);
void AltitudeCombineThread(void);
void PWM_Out( int16_t Throttle,int16_t Restrain,int16_t dPitch,int16_t dRoll,int16_t dYaw);
#endif//__CONTROL_H__
