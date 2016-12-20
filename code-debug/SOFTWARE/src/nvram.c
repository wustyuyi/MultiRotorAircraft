#include "nvram.h"
#include "stmflash.h"
const char __FLASH__[16*1024] __attribute__((at(0x0800C000)));//����3��ʼ��ַ, 16 Kbytes  
char NVRAM[2048];
uint8_t NvramBuf[NVRAM_LENGTH];
uint16_t DataLen = 0;
PARAMETER *ParameterTail;

int a,b,c,d,e,f,g,h,i,j;

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:˫��(8λ)��
void FLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
        pBuffer[i] = *(__IO uint64_t*)ReadAddr;//��ȡ8���ֽ�
		ReadAddr+=8;//ƫ��8���ֽ�.	
	}
}
int FLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
    if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return-1;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
    FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*8;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{

        status=FLASH_EraseSector(FLASH_Sector_3,VoltageRange_3);//VCC=2.7~3.6V֮��!!
        //if(status!=FLASH_COMPLETE)return -2;	//����������		
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramDoubleWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=8;
			pBuffer++;
		} 
	}
    FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
} 

int NvramParameterCreate(uint8_t Handle,void *DataAddress,uint8_t  Length)
{
    PARAMETER *Parameter;
    if ( Handle >= PARAMETER_NUM )return -1;
    Parameter = ParameterTail + sizeof(PARAMETER) + ParameterTail->Length;
       
    Parameter->Handle = Handle;
    Parameter->Length = Length;
    Parameter->Next   = NULL;
    
    *((uint16_t *)&NvramBuf[PARAMETER_NUM*sizeof(PARAMETER)]) = DataLen + Length;//����data length
    ParameterTail = Parameter;
    return 0;
}
PARAMETER * NvramParameterCheckout(uint8_t Handle)
{
    PARAMETER *Parameter=NULL;
    Parameter = (PARAMETER *)(&NvramBuf[0]);
    while( NULL != Parameter->Next )
    {  
        if ( Handle == Parameter->Handle)
        {
            return Parameter;
        }
        Parameter = (PARAMETER *)( &NvramBuf[Parameter->Next]);      
    }
    return NULL;
}
int NvramParameterDelete(uint8_t Handle)
{

}
int NvramParameterRead(uint8_t Handle,void *DataAddress,uint8_t  Length)
{
    PARAMETER *Parameter;
    Parameter = NvramParameterCheckout(Handle);
    if (NULL == Parameter)return -1;
    if ( Length != Parameter->Length )return -2;
    
    DataAddress = Parameter + sizeof(PARAMETER);
    return 1;
}
int NvramParameterWrite(uint8_t Handle,void *DataAddress,uint8_t  Length)
{
    char *StartAddr = NULL;
    PARAMETER *Parameter;
    Parameter = NvramParameterCheckout(Handle);
    if (NULL == Parameter)return -1;
    
    if ( Length != Parameter->Length )return -2;
    
    StartAddr = (char *)Parameter + sizeof(PARAMETER);
    memcpy(StartAddr,DataAddress,Length);
    return 0;
}
int NvramParameterInit(uint8_t Handle,void* DataAddress)
{
    int ret;
    ret = NvramParameterRead(PARAMETER_0,&a,sizeof(a));
    if ( ret < 0 )
    {
        NvramParameterCreate(PARAMETER_0,&a,sizeof(a));
    }
   

}
int NvramInit()
{
    uint16_t cnt = 0;
    PARAMETER *Parameter;
    STMFLASH_Read(NVRAM_START_ADDR,(u32*)NvramBuf,NVRAM_LENGTH/2);
    DataLen = *((uint16_t *)&NvramBuf[PARAMETER_NUM*sizeof(PARAMETER)]);
    ParameterTail = (PARAMETER *)(&NvramBuf[0]);
    while( NULL != ParameterTail->Next )
    {  
        ParameterTail = (PARAMETER *)( &NvramBuf[Parameter->Next]);
        cnt ++;
        if(cnt>PARAMETER_NUM)return -1;
    }
    if ((uint8_t*)ParameterTail > &NvramBuf[STM_SECTOR_SIZE-sizeof(PARAMETER)])return -2;
    return 0;
}