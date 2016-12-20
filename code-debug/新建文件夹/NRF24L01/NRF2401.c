#include "NRF2401.h"
#include "spi.h"
#include "stm32f10x_spi.h"
#include "delay.h"
#include "usart.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//NRF24L01��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
const uint8_t NRF_ADDRESS[TX_ADR_WIDTH]={0x1e,0x3c,0x5a,0x78,0x96};
    
uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x1e,0x3c,0x5a,0x78,0x96}; //���͵�ַ
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x1e,0x3c,0x5a,0x78,0x96}; //���͵�ַ

u8 NRF24L01_Init(void)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB,D,G�˿�ʱ��
    	
	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;	//PG6 7 ���� 	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //PB13/14/15����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��ָ��IO
  
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PG8 ����  
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);//PG6,7,8����					 
		 
    SPI2_Init();    		//��ʼ��SPI	 
 
	SPI_Cmd(SPI2, DISABLE); // SPI���費ʹ��

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI����
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//ʱ�����յ�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//���ݲ����ڵ�1��ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź����������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
			 
	
//	NRF24L01_CSN_1;			//SPIƬѡȡ�� 
	NRF24L01_CE_0; 			//ʹ��24L01

//	NRF24L01_Write_Reg(WRITE_REG_NRF+SETUP_AW,0x01);  //���õ�ַΪ3���ֽڳ���
 	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ

	NRF24L01_Write_Reg(WRITE_REG_NRF+RX_PW_P0, RX_PLOAD_WIDTH);
  	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ 
	 
  	NRF24L01_Write_Reg(WRITE_REG_NRF+SETUP_RETR,0xaa);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0);       //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x07);  //����TX�������,0db����,2Mbps,���������濪��
	NRF24L01_CE_1; 
   	 
	  
 

#ifdef TX_MODE 
	 NRF24L01_TX_Mode();
	if(NRF24L01_Check())return 1;
	else  return 0;
#endif	
#ifdef RX_MODE 
	 NRF24L01_RX_Mode();
	if(NRF24L01_Check())return 2;
	else  return 0;
#endif	
		 	 
}

 		 	 

//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[TX_ADR_WIDTH]={0};
	u8 i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,TX_ADR_WIDTH); //����д��ĵ�ַ  
	for(i=0;i<TX_ADR_WIDTH;i++)if(buf[i]!=TX_ADDRESS[i])return 0;//���24L01����	 							   	
	return 1;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN_0;                 //ʹ��SPI����
  	status =SPI2_ReadWriteByte(reg);//���ͼĴ����� 
  	SPI2_ReadWriteByte(value);      //д��Ĵ�����ֵ
  	NRF24L01_CSN_1;                 //��ֹSPI����	   
  	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN_0;          //ʹ��SPI����		
  	SPI2_ReadWriteByte(reg);   //���ͼĴ�����
  	reg_val=SPI2_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  	NRF24L01_CSN_1;          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN_0;           //ʹ��SPI����
  	status=SPI2_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI2_ReadWriteByte(0XFF);//��������
  	NRF24L01_CSN_1;       //�ر�SPI����
  	return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN_0;          //ʹ��SPI����
  	status = SPI2_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI2_ReadWriteByte(*pBuf++); //д������	 
  	NRF24L01_CSN_1;       //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}				   

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
 //	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	NRF24L01_CE_0;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,32);//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE_1;//��������	   

	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	while(1)
	{
		sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
		if(sta&TX_OK)//�������
		{
		//	LCD_Display_String(168,1,GREEN,BLUE, "TX OK ");
			return TX_OK;
		}
		if(sta&MAX_TX)
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ���
		//	LCD_Display_String(168,1,GREEN,BLUE,"TX MAX");
			return MAX_TX; 
		}
	}

}
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,32);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
	//	LCD_Display_String(168,1,GREEN,BLUE, "RX OK ");
		return 1; 
	}
//	LCD_Display_String(168,1,GREEN,BLUE, "RX NO ");	   
	return 0;//û�յ��κ�����
}					    
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE_0;	  
//  	LCD_Display_String(168,1,GREEN,BLUE, "RX OK ");
//	LCD_Display_String(168,1,GREEN,BLUE, "R_Mode");
  	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG, 0x3f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	   
     
  	NRF24L01_CE_1; //CEΪ��,�������ģʽ 
}						 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE_0;	    
	    //����NRF�Ĵ���
  // 	LCD_Display_String(168,1,GREEN,BLUE, "T_Mode");
  	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	
     
   
	NRF24L01_CE_1;//CEΪ��,10us����������
	//delay_ms(1);
}		  




