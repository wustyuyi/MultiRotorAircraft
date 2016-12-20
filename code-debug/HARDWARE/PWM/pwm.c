#include "pwm.h"
#include "led.h"
#include "usart.h"
 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器PWM 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//TIM14 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
//u32 TimerPeriod = 0;
//u32 ccr1 = 0;
//u32 ccr2 = 0;
//u32 ccr3 = 0;
//u32 ccr4 = 0;
void TIM8_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  //使能TIM8时钟
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                        
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);

    TIM_TimeBaseStructure.TIM_Prescaler = psc;   //Timer clock = sysclock /(TIM_Prescaler+1) = 168M
    TIM_TimeBaseStructure.TIM_Period = arr;    //Period = (TIM counter clock / TIM output clock) - 1 = 20K
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //死区设置
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//计数中断
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//脉宽调制模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//N极比较输出不使能

    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;   
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    TIM_OC1Init(TIM8,&TIM_OCInitStructure);
    TIM_OC2Init(TIM8,&TIM_OCInitStructure);
    TIM_OC3Init(TIM8,&TIM_OCInitStructure);
    TIM_OC4Init(TIM8,&TIM_OCInitStructure);
    TIM_Cmd(TIM8,ENABLE);
    TIM_CtrlPWMOutputs(TIM8,ENABLE); 									  
} 
void TIM2_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  //使能TIM8时钟
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;     //??????
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //???????!
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   //?????
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //?????PWM?????
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_TIM2);
    TIM_DeInit(TIM2);//???TIM2???
    
    TIM_TimeBaseStructure.TIM_Prescaler = psc;   //Timer clock = sysclock /(TIM_Prescaler+1) = 168M
    TIM_TimeBaseStructure.TIM_Period = arr;    //Period = (TIM counter clock / TIM output clock) - 1 = 20K
    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //死区设置
    //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//计数中断
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);  //?????,???????TIM1?TIM8??
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//脉宽调制模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE); //??TIM2???									  
} 
#define PWM 0
void PWM_Init()
{
    
 	TIM8_PWM_Init(2500-1,168-1);	//168M/4=42Mhz的计数频率,重装载值2000，所以PWM频率为 42M/2000=21Khz.     
    TIM2_PWM_Init(2500-1,84-1);
    TIM_SetCompare1(TIM8,PWM);	//修改比较值，修改占空比,LD
    TIM_SetCompare2(TIM8,PWM);	//修改比较值，修改占空比,RD
    TIM_SetCompare3(TIM8,PWM);	//修改比较值，修改占空比,RU
    TIM_SetCompare4(TIM8,PWM);	//修改比较值，修改占空比,LU
   
    TIM_SetCompare3(TIM2,PWM);	//修改比较值，修改占空比,RU
    TIM_SetCompare4(TIM2,PWM);	//修改比较值，修改占空比,LU    
}

