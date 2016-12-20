#include "pwm.h"
#include "led.h"
#include "usart.h"
 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ��PWM ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//TIM14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
//u32 TimerPeriod = 0;
//u32 ccr1 = 0;
//u32 ccr2 = 0;
//u32 ccr3 = 0;
//u32 ccr4 = 0;
void TIM8_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  //ʹ��TIM8ʱ��
  
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
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //��������
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//�����ж�
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//�������ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//N���Ƚ������ʹ��

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
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOCʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  //ʹ��TIM8ʱ��
  
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
    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //��������
    //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//�����ж�
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);  //?????,???????TIM1?TIM8??
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//�������ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE); //??TIM2???									  
} 
#define PWM 0
void PWM_Init()
{
    
 	TIM8_PWM_Init(2500-1,168-1);	//168M/4=42Mhz�ļ���Ƶ��,��װ��ֵ2000������PWMƵ��Ϊ 42M/2000=21Khz.     
    TIM2_PWM_Init(2500-1,84-1);
    TIM_SetCompare1(TIM8,PWM);	//�޸ıȽ�ֵ���޸�ռ�ձ�,LD
    TIM_SetCompare2(TIM8,PWM);	//�޸ıȽ�ֵ���޸�ռ�ձ�,RD
    TIM_SetCompare3(TIM8,PWM);	//�޸ıȽ�ֵ���޸�ռ�ձ�,RU
    TIM_SetCompare4(TIM8,PWM);	//�޸ıȽ�ֵ���޸�ռ�ձ�,LU
   
    TIM_SetCompare3(TIM2,PWM);	//�޸ıȽ�ֵ���޸�ռ�ձ�,RU
    TIM_SetCompare4(TIM2,PWM);	//�޸ıȽ�ֵ���޸�ռ�ձ�,LU    
}

