#include "nvram.h"
#include "stmflash.h"
const char __FLASH__[16*1024] __attribute__((at(0x0800C000)));//扇区3起始地址, 16 Kbytes  
char NVRAM[2048];
uint8_t NvramBuf[NVRAM_LENGTH];
uint16_t DataLen = 0;
PARAMETER *ParameterTail;

int a,b,c,d,e,f,g,h,i,j;

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:双字(8位)数
void FLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
        pBuffer[i] = *(__IO uint64_t*)ReadAddr;//读取8个字节
		ReadAddr+=8;//偏移8个字节.	
	}
}
int FLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
    if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return-1;	//非法地址
	FLASH_Unlock();									//解锁 
    FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*8;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{

        status=FLASH_EraseSector(FLASH_Sector_3,VoltageRange_3);//VCC=2.7~3.6V之间!!
        //if(status!=FLASH_COMPLETE)return -2;	//发生错误了		
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramDoubleWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=8;
			pBuffer++;
		} 
	}
    FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 

int NvramParameterCreate(uint8_t Handle,void *DataAddress,uint8_t  Length)
{
    PARAMETER *Parameter;
    if ( Handle >= PARAMETER_NUM )return -1;
    Parameter = ParameterTail + sizeof(PARAMETER) + ParameterTail->Length;
       
    Parameter->Handle = Handle;
    Parameter->Length = Length;
    Parameter->Next   = NULL;
    
    *((uint16_t *)&NvramBuf[PARAMETER_NUM*sizeof(PARAMETER)]) = DataLen + Length;//更新data length
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