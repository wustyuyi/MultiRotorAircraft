#include "mpu6050.h"
#include "usart.h"
#include "myiic.h"
#include "delay.h"
#include <math.h>
void PMU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	IIC_Start();
	IIC_Send_Byte(MPU6050_SLAVE_ADDRESS);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg_add);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg_dat);
	IIC_Wait_Ack();
	IIC_Stop();
}

void PMU6050_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
	unsigned char i;
	
	IIC_Start();
	IIC_Send_Byte(MPU6050_SLAVE_ADDRESS);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg_add);
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte(MPU6050_SLAVE_ADDRESS+1);
	IIC_Wait_Ack();
	
	for(i=0;i<(num-1);i++){
		*Read=IIC_Read_Byte(1);
		Read++;
	}
	*Read=IIC_Read_Byte(0);
	IIC_Stop();
}

void MPU6050_Init(void)
{
    IIC_Init();
    PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x80);
    delay_ms(50);    
	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //解除休眠状态

	PMU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x01);	    //陀螺仪采样率250hz
    PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1 , 0x03);	    //
	PMU6050_WriteReg(MPU6050_RA_CONFIG , MPU6050_DLPF_BW_42);	
	PMU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG,MPU6050_ACCEL_FS_8<<3);	  //配置加速度传感器工作在16G模式
	PMU6050_WriteReg(MPU6050_RA_GYRO_CONFIG ,MPU6050_GYRO_FS_1000<<3);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    delay_ms(10);
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


