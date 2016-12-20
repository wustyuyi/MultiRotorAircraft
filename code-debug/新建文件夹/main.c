#include "delay.h"
#include "sys.h"

#include "usart.h"
#include "spi.h"
#include "NRF2401.h"

#include "adc.h"
#include "dac.h"
#include "key.h"
#include "timer.h"
#include "led.h"
#include "DataScope_DP.h"
#include "mpu6050.h"
#include "bsp_i2c.h"
#define BYTE1(x) ((unsigned char)((x)>>8))
#define BYTE0(x) ((unsigned char)(x))    
#define FULL_SPEED 1
#define MID_SPEED  2
#define LOW_SPEED  3
#define SLEEP_MODE 4

#define FORWARD    1
#define BACK       2
typedef struct motor
{
    uint8_t brake;
    uint8_t mode;
    uint8_t dir;
    uint16_t speed;
} MOTOR_TYPEDEF;

int otudata[4]={0};
unsigned char data_to_send[32]={0};

void Data_Send_Senser(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=BYTE1(Acc.X);  //?8?
//	data_to_send[_cnt++]=BYTE0(Acc.X);  //?8?
//	data_to_send[_cnt++]=BYTE1(Acc.Y);
//	data_to_send[_cnt++]=BYTE0(Acc.Y);
//	data_to_send[_cnt++]=BYTE1(Acc.Z);
//	data_to_send[_cnt++]=BYTE0(Acc.Z);
//	data_to_send[_cnt++]=BYTE1(Gyr.X);
//	data_to_send[_cnt++]=BYTE0(Gyr.X);
//	data_to_send[_cnt++]=BYTE1(Gyr.Y);
//	data_to_send[_cnt++]=BYTE0(Gyr.Y);
//	data_to_send[_cnt++]=BYTE1(Gyr.Z);
//	data_to_send[_cnt++]=BYTE0(Gyr.Z);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
//	u8 sum = 0;
//	for(u8 i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	data_to_send[_cnt++] = sum;
//    for(u8 i=0;i<_cnt;i++)
//        Uart_Sent_Char(data_to_send[i]);
}
//**************************************************************************
//   Kalman filter
//**************************************************************************
//-------------------------------------------------------
//const float Q_angle=0.05, Q_gyro=0.0005, R_angle=0.5, dt=0.004;
//const float Q_angle=0.05, Q_gyro=0.0005, R_angle=0.5, dt=0.004;
const float Q_angle=0.5, Q_gyro=0.5, R_angle=0.5, dt=0.004;
//-------------------------------------------------------
float x_angle=0, x_angle_dot=0;         //
void x_Kalman_Filter(float angle_m,float gyro_m)          //gyro_m:gyro_measure
{
    static float P[2][2] = {
                            { 1, 0 },
                            { 0, 1 }
                            };
    static float Pdot[4] ={0,0,0,0};
    static const char C_0 = 1;
    static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;    
    
    x_angle+=(gyro_m-q_bias) * dt;
    
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]= 0.0f - P[1][1];
    Pdot[2]= 0.0f - P[1][1];
    Pdot[3]=Q_gyro;
    
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
       
    angle_err = angle_m - x_angle;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
       
    x_angle   += K_0 * angle_err;
    q_bias  += K_1 * angle_err;
    x_angle_dot = gyro_m-q_bias;
}
float y_angle=0, y_angle_dot=0;         //
void y_Kalman_Filter(float angle_m,float gyro_m)          //gyro_m:gyro_measure
{
    static float P[2][2] = {
                            { 1, 0 },
                            { 0, 1 }
                            };
    static float Pdot[4] ={0,0,0,0};
    static const char C_0 = 1;
    static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;    
    
    y_angle+=(gyro_m-q_bias) * dt;
    
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]= 0.0f - P[1][1];
    Pdot[2]= 0.0f - P[1][1];
    Pdot[3]=Q_gyro;
    
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
       
    angle_err = angle_m - y_angle;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
       
    y_angle   += K_0 * angle_err;
    q_bias  += K_1 * angle_err;
    y_angle_dot = gyro_m-q_bias;
}
short acc[3]={0};
short gyro[3]={0};
float f_acc[3]={0};
float f_gyro[3]={0};
short temp=0;
void sleep_enable()
{
    GPIO_ResetBits(GPIOD,GPIO_Pin_3);
}
void sleep_disable()
{
    GPIO_SetBits(GPIOD,GPIO_Pin_3);
}
void lock_brake()
{
    GPIO_SetBits(GPIOD,GPIO_Pin_4);
}
void unlock_brake()
{
    GPIO_ResetBits(GPIOD,GPIO_Pin_4);
}
void speed_forward()
{
    GPIO_ResetBits(GPIOD,GPIO_Pin_5);    
}
void speed_back()
{
    GPIO_SetBits(GPIOD,GPIO_Pin_5);
}
int main(void)
{
  uint8_t rx_buf[32] = {0};	
  uint8_t speed_dir = 0;
  int16_t cnt = 0;
  uint16_t dac_out = 0;
  uint16_t CRC16 = 0;
  MOTOR_TYPEDEF scooter= {0,MID_SPEED,FORWARD,0};
  NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	Stm32_Clock_Init(9);
    IO_Init();
	delay_init();	    	 //延时函数初始化
   LED_Init();
	uart_init(115200);	 	//串口初始化为9600

  Adc_Init(); 	
  Dac1_Init();
    Dac2_Init();  
   TIM3_Int_Init(399,719);//5ms
	//i2c_GPIO_Config();
	//MPU6050初始化
	//MPU6050_Init();  
    //printf("mpu6050 ID:0X%02X\n",MPU6050ReadID());

	while(!NRF24L01_Init())
	{
#ifdef TX_MODE
	printf("nrf state:unconnet-tx\n");
#endif
#ifdef RX_MODE
	printf("nrf state:unconnet-rx\n");
#endif  
		
		delay_ms(200);
		LED ^= 1;
	}
	LED = 1;
#ifdef TX_MODE
	printf("nrf state:connet-tx\n");
#endif
#ifdef RX_MODE
	printf("nrf state:connet-rx\n");
#endif   
  
	while(1)
	{ 
        if(!NRF24L01_IRQ)	
        {			
			if(NRF24L01_RxPacket(rx_buf))
			{
				 
				if(rx_buf[0]==0x26 && rx_buf[1]==0x14 && rx_buf[2]==0xb8)
				{
                    scooter.mode  = rx_buf[3];
                    scooter.dir   = rx_buf[4];
                    scooter.brake = rx_buf[5];
                    scooter.speed = (rx_buf[6]|(rx_buf[7]<<8));
                    DEBUG_INFO("scooter info: mode=%d dir=%d brake=%d speed=%d!!!\n",
                                scooter.mode,
                                scooter.dir,
                                scooter.brake,
                                scooter.speed);
				}				
			}
			else
			{
				 DEBUG_INFO("nrf no data\n");
			}
	    }
        if ( scooter.mode == SLEEP_MODE)
        {
            Dac1_Set_Vol(0);
            Dac2_Set_Vol(0); 
            sleep_enable();
        }
        else
        {   
            if( scooter.dir == BACK )
            {
                speed_back();
            }
            else
            {
                speed_forward();
            }
            switch (scooter.mode)
            {
                case FULL_SPEED:
                    speed_dir = 1;
                    break;
                case MID_SPEED:
                    speed_dir = 2;
                    break;           
                case LOW_SPEED:
                    speed_dir = 3;
                    break;  
                default:
                    speed_dir = 1;
                    break; 
            }
            dac_out = scooter.speed/speed_dir;
            if ( scooter.brake == 1)//刹车
            {
                if( dac_out > (4096/speed_dir) - 1000 )
                {
                    lock_brake();
                }
                else
                {
                    unlock_brake();
                }
                Dac2_Set_Vol(dac_out);
                Dac1_Set_Vol(0);
            }
            else
            {
                unlock_brake();
                Dac1_Set_Vol(dac_out);
                Dac2_Set_Vol(0);
            }
            
        }
		
        
	}

}
