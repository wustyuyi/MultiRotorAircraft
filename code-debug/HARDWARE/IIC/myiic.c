#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//IIC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
        delay_us(1);
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(1);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}


/*
*********************************************************************************************************
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参：_Address：设备的I2C总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
uint8_t IIC_Check_Device(uint8_t _Address)
{
	uint8_t ucAck;

	IIC_Init();		/* 配置GPIO */
	
	IIC_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	IIC_Send_Byte(_Address|I2C_WR);
	ucAck = IIC_Wait_Ack();	/* 检测设备的ACK应答 */

	IIC_Stop();			/* 发送停止信号 */

	return ucAck;
}

/**************************????********************************************
*????:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*?  ?:	    ?????? ?????????
??	I2C_Addr  ??????
		addr	   ?????
??   ?????
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
  
	IIC_Send_Byte(I2C_Addr);	   //?????
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //????
	IIC_Wait_Ack();	  
	//IIC_Stop();//????????	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //??????			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//????????

	return res;
}
/**************************????********************************************
*????:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*?  ?:	    ?????? ?????? length??
??	dev  ??????
		reg	  ?????
		length ??????
		*data  ????????????
??   ????????
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	u8 temp;
	IIC_Start();
	IIC_Send_Byte(dev);	   //?????
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //????
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //??????	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=(length-1))
		 	temp = IIC_Read_Byte(1);  //?ACK????
		 	else  
			temp = IIC_Read_Byte(0);	 //??????NACK

		data[count] = temp;
	}
    IIC_Stop();//????????
    return count;
}

/**************************????********************************************
*????:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*?  ?:	    ??????????? ?????
??	dev  ??????
		reg	  ?????
		length ??????
		*data  ??????????
??   ??????
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //?????
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //????
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//????????

    return 1; //status == 0;
}
/**************************????********************************************
*????:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*?  ?:	    ?????? ?????????
??	dev  ??????
		reg	   ?????
		data  ???????
??   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}




















