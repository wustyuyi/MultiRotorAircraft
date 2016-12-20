#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"
#include "timer.h"
#include "adc.h"
#include "24l01.h"
#include "mpu6050.h"
#include "DataAcquisition.h"
#include "RemoteReceive.h"
#include "UIManage.h"
#include "MS5611.h"
void BspInit()
{
    const char *s;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(256000);//初始化串口波特率为115200
    Adc_Init();
    TIM3_Int_Init(4000,84-1);	//4ms,定时器时钟84M    
    PWM_Init();  
    s= get_lib_info();
    printf("\r\n%s\r\n",s);
    NRF24L01_Init();
	while(NRF24L01_Check())
	{
        printf("NRF not find\n");
 		delay_ms(200);
	}
    NRF24L01_RX_Mode();
    printf("NRF24L01 OK"); 	
    
    MPU6050_Init();
    if ( MPU6050_ADDRESS == MPU6050ReadID())
    {
        printf("Find MPU6050!\n");
    }
    else
    {
        printf("Can't find MPU6050!\n");
    }
    LED_Init();
    MS5611_Init();
    if ( WaitBaroInitOffset())
    {
        printf("MS5611 offset init secuess!\r\n");
    }
}



void TaskCreate()
{
    UartReciveTaskCreate();
    DataAcquisitionTaskCreate();
    UIManageTaskCreate();
}
void TaskStart()
{
    vTaskStartScheduler();
}
int main()
{   
    BspInit();
    TaskCreate();
    TaskStart();
    while(1)
    {
    }
    return -1;
}
