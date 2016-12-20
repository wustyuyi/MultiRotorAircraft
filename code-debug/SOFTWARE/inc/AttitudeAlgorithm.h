#ifndef __ATTITUDEALGORITHM_H__
#define __ATTITUDEALGORITHM_H__
#define M_PI 3.1415926f
const char* get_lib_info();
typedef struct{
    short acc_adc_x;
    short acc_adc_y;
    short acc_adc_z;
    
    short gyro_adc_x;
    short gyro_adc_y;
    short gyro_adc_z;

    float acc_raw_x;
    float acc_raw_y;
    float acc_raw_z;
    
    float gyro_raw_x;
    float gyro_raw_y;
    float gyro_raw_z;

    float acc_offset_x;
    float acc_offset_y;
    float acc_offset_z;
    
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;

    float acc_x;
    float acc_y;
    float acc_z;
    
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float mag_x;
    float mag_y;
    float mag_z;
    
    float   DCM[3][3];
    float   q[4];
    float   roll;				//deg
    float   pitch;
    float 	yaw;
    float   rollRate;				//rad
    float   pitchRate;
    float 	yawRate; 
}IMU;
extern IMU imu;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
    
	float Target;
    float Measure;
    
    float Error;
    float LastError;
    
    float Integral;
    float IntegralLimit;
    
    float Diff;
    
    float Output;
}PID_Typedef;

//void PID_ParaInit(PID_Typedef *pid);
void PID_Postion(PID_Typedef *PID,unsigned char DiffFlag);
void PID_Normal(PID_Typedef *pid);


void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
void q4ToDCM(float DCMgb[9]);
void Kalman_Filter_Z(float angle_m, float gyro_m,float *angle,float *gyro);
void Kalman_Filter_A(float angle_m, float gyro_m,float *angle);//angleAx ºÍ gyroGy
#endif//__ATTITUDEALGORITHM_H__
