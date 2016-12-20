#include "mpu6050.h"
#include "usart.h"
#include "bsp_i2c.h"
#include <math.h>
void PMU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	i2c_SendByte(reg_dat);
	i2c_WaitAck();
	i2c_Stop();
}

void PMU6050_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
	unsigned char i;
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1);
	i2c_WaitAck();
	
	for(i=0;i<(num-1);i++){
		*Read=i2c_ReadByte(1);
		Read++;
	}
	*Read=i2c_ReadByte(0);
	i2c_Stop();
}

void MPU6050_Init(void)
{
  int i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //解除休眠状态
  
	PMU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x03);	    //陀螺仪采样率250hz
	PMU6050_WriteReg(MPU6050_RA_CONFIG , MPU6050_DLPF_BW_98);	
	PMU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACCEL_FS_8<<3);	  //配置加速度传感器工作在16G模式
	PMU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_1000<<3);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
}
unsigned char MPU6050ReadID(void)
{
	unsigned char Re = 0;
    PMU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
    return Re;
    
}
void MPU6050ReadAcc(short *accData)
{
    u8 buf[6];
    PMU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}
void MPU6050ReadGyro(short *gyroData)
{
    u8 buf[6];
    PMU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReturnTemp(short*Temperature)
{
	short temp3;
	u8 buf[2];
	
	PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}





//---------------------------------------------------------------------------------------------------
// IMU algorithm update
//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq	250.0f			// sample frequency in Hz
#define dsampleFreq	0.004f			// sample frequency in Hz
//#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKpDef	1.0f	        // 2 * proportional gain
//#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
#define twoKiDef	0	            // 2 * integral gain
//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//input a(m/s*s) g(deg/s) hudu
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
//			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
//			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
//			integralFBz += twoKi * halfez * (1.0f / sampleFreq);

			integralFBx += twoKi * halfex * dsampleFreq;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dsampleFreq;
			integralFBz += twoKi * halfez * dsampleFreq;            
            
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
//	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
//	gy *= (0.5f * (1.0f / sampleFreq));
//	gz *= (0.5f * (1.0f / sampleFreq));
    
	gx *= (0.5f * dsampleFreq);		// pre-multiply common factors
	gy *= (0.5f * dsampleFreq);
	gz *= (0.5f * dsampleFreq);  
    
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
//#define PI 3.141592f
#define DEG 57.2958f
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

//  *yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 180 / PI;
//  *pitch = atan(gx / sqrt(gy*gy + gz*gz)) * 180 / PI;
//  *roll = atan(gy / sqrt(gx*gx + gz*gz)) * 180 / PI;
  *yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * DEG;
  *pitch = atan(gx / sqrt(gy*gy + gz*gz)) * DEG;
  *roll = atan(gy / sqrt(gx*gx + gz*gz)) * DEG;    
    
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

