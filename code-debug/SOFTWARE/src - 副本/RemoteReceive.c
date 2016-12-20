/////////////////////////////////////////////////////////////////////////////////	 
//????????????
//??:Javen Chen
//??:V0.0.1
//????,?????
//Copyright(C) ???????????? @ 2014-2024
//All rights reserved	  
////////////////////////////////////////////////////////////////////////////////  

//Scheduler includes///////////////////////////////////////////////////////////
#include "sys.h"
#include "usart.h"
#include "string.h"
#include "24l01.h"
#include "crc16.h"
#include "RemoteReceive.h"
#include "DataAcquisition.h"
#include <stdlib.h>
NRF_MODE NrfMode = RxMode;

#define UART_QUE_NUM 4
#define CMD_QUE_NUM  8
SemaphoreHandle_t NRFReceiveSempht = NULL;
SemaphoreHandle_t NRFTransmitSempht = NULL;
xQueueHandle UartQueue    = NULL;
xQueueHandle CommandQueue = NULL;

#define USART_REC_LEN  			32  	//????????? 32	  	
uint8_t USART_RX_BUF[USART_REC_LEN];     //????,??USART_REC_LEN???.
u8 UartRecCnt = 0;
u8 UartLoop = 0;

////////////////////////////////////////////////////////////////////////////////
//???  :DataUpdateTask
//????:Update sensor data to controller
//????:void
//???  :void
//??    :Javen Chen
//???  :Javen Chen
//??    :V0.0.1
//////////////////////////////////////////////////////////////////////////////// 
void DataUpdateTask()
{
    while(1)
    {
        //your code
       vTaskDelay(1);
        taskYIELD();//???????????
    }

}

////////////////////////////////////////////////////////////////////////////////
//???  :CommandProcess
//????:Process command
//????:void
//???  :void
//??    :Javen Chen
//???  :Javen Chen
//??    :V0.0.1
//////////////////////////////////////////////////////////////////////////////// 
void CommandProcess()
{
    COMMAND CmdBuf = {0,0};
    portBASE_TYPE xStatus;
    memset(&CmdBuf,0,sizeof(CmdBuf));
    xStatus = xQueueReceive( CommandQueue,&CmdBuf,0 );
    if( pdPASS == xStatus)
    {
        //command received 
    
    }
}

////////////////////////////////////////////////////////////////////////////////
//???  :CommandAnalysis
//????:Analysis command
//????:char *ReceiveData,The data from USART
//???  :void
//??    :Javen Chen
//???  :Javen Chen
//??    :V0.0.1
//////////////////////////////////////////////////////////////////////////////// 
void CommandAnalysis( char *ReceiveData )
{
        uint8_t RxBuf[32]="";
   // printf("%-6.9s\n",ReceiveData);
     NRF24L01_TxPacket_Task((u8 *)ReceiveData);
}

////////////////////////////////////////////////////////////////////////////////
//???  :CommandAnalysisTask
//????:CommandAnalysisTask
//????:void *pvParameters
//???  :void
//??    :Javen Chen
//???  :Javen Chen
//??    :V0.0.1
//////////////////////////////////////////////////////////////////////////////// 
void CommandAnalysisTask(void *pvParameters)
{
    char ReceiveBuf[USART_REC_LEN];
    portBASE_TYPE xStatus;
    while(1)
    {
        memset(ReceiveBuf,0,USART_REC_LEN);
        xStatus = xQueueReceive( UartQueue,ReceiveBuf,portMAX_DELAY );//????
        CommandAnalysis( ReceiveBuf );
        taskYIELD();//???????????
    }
}

////////////////////////////////////////////////////////////////////////////////
//???  :UartReciveTask
//????:Recive data from uart
//????:void *pvParameters
//???  :void
//??    :Javen Chen
//???  :Javen Chen
//??    :V0.0.1
////////////////////////////////////////////////////////////////////////////////
void UartReciveTask( void *pvParameters )
{
    portBASE_TYPE xStatus;
	static portTickType xLastWakeTime;
    while(1){    
        if(UartLoop>0){
            UartLoop++;    
        }        
        if (UartLoop>2){
            if(UartRecCnt>0 && UartRecCnt<USART_REC_LEN){
                xStatus = xQueueSendToBack( UartQueue, &USART_RX_BUF, 0 );
                if( xStatus != pdPASS ){
                    DEBUG( "Could not send to the queue.\n" );
                }
                memset(USART_RX_BUF,0,USART_REC_LEN);
                UartLoop = 0;
                UartRecCnt = 0;  
                taskYIELD();//???????????
            }        
        }        
        vTaskDelayUntil(&xLastWakeTime,50);//???1000us???
	}
}
void NRFCommandProcess(uint8_t *data)
{
    uint16_t crc = 0;
    if ( data[0] != 0xCD )
    {
        DEBUG("command error\n");
        return;
    }
    crc = crc16((uint8_t *)data,12);
    if ( crc != *(uint16_t *)(data+12) )
    {
        DEBUG("crc error\n");
        return;
    }    
    switch (data[2])
    {
        case 0x01: 
            ThrottleR = *((uint16_t *) (data+4));
            RollPID.Target = (*((int16_t  *) (data+6)));
            PitchPID.Target =(*((int16_t  *) (data+8)));
            RollPID.Target  = RollPID.Target*0.1f;
            PitchPID.Target = PitchPID.Target*0.1f;            
           // *((int16_t  *) (command+10)) = PID_Yaw.Expect;
            break;
        
        default:
            DEBUG("command type error\n");
            break;
    }
    //DEBUG("NRF rec ok\n");
}
void NRFReciveTask( void *pvParameters )
{
    uint8_t Sta = 0;
    uint8_t RxBuf[32];
    while(1)
    {
        xSemaphoreTake( NRFReceiveSempht, portMAX_DELAY );
       //DEBUG("DEBUG\n");
        Sta = NRF24L01_RxPacket(RxBuf);
        if ( 0 == Sta )
        {
            NRFCommandProcess(RxBuf);
            //printf("%s\n",RxBuf);  
            taskYIELD();//???????????
        }        
    }

}
void UartReciveTaskCreate()
{
    NRFReceiveSempht = xSemaphoreCreateBinary();
    NRFTransmitSempht = xSemaphoreCreateBinary();
    UartQueue    = xQueueCreate( UART_QUE_NUM, USART_REC_LEN );
    CommandQueue = xQueueCreate( CMD_QUE_NUM, sizeof( COMMAND ) );
    xTaskCreate( UartReciveTask,     /* Pointer to the function that implements the task.              */
                 "UartRecive",   /* Text name for the task.  This is to facilitate debugging only. */
                 128,        /* Stack depth in words.                                          */
                 NULL,       /* We are not using the task parameter.                           */
                 2,          /* This task will run at priority 1.                              */
                 NULL );     /* We are not using the task handle.                              */
    xTaskCreate( NRFReciveTask,     /* Pointer to the function that implements the task.              */
                 "NRFRecive",   /* Text name for the task.  This is to facilitate debugging only. */
                 256,        /* Stack depth in words.                                          */
                 NULL,       /* We are not using the task parameter.                           */
                 2,          /* This task will run at priority 1.                              */
                 NULL );     /* We are not using the task handle.                              */    
    
    
    xTaskCreate( CommandAnalysisTask,     /* Pointer to the function that implements the task.              */
                 "CmdAnalysis",   /* Text name for the task.  This is to facilitate debugging only. */
                 256,        /* Stack depth in words.                                          */
                 NULL,       /* We are not using the task parameter.                           */
                 1,          /* This task will run at priority 1.                              */
                 NULL );     /* We are not using the task handle.                              */
    xTaskCreate( DataUpdateTask,     /* Pointer to the function that implements the task.              */
                 "DataUpdate",   /* Text name for the task.  This is to facilitate debugging only. */
                 128,        /* Stack depth in words.                                          */
                 NULL,       /* We are not using the task parameter.                           */
                 1,          /* This task will run at priority 1.                              */
                 NULL );     /* We are not using the task handle.                              */
}

void UartReceive(char *UartData)
{
    UartLoop = 1;		
    USART_RX_BUF[UartRecCnt] = *UartData;
    UartRecCnt ++;
    if( USART_REC_LEN == UartRecCnt )
    {
        UartRecCnt = 0;
    }
}
void NRFTransmitWhile()
{
    xSemaphoreTake( NRFTransmitSempht, portMAX_DELAY );
   // printf("Line5");
}

void NRFReceiveIntterupt()
{
    static BaseType_t xHigherPriorityTaskWoken;
    if ( RxMode == NrfMode )
    {
        //printf("Rec");
        xSemaphoreGiveFromISR( NRFReceiveSempht, &xHigherPriorityTaskWoken );
    }
    else
    {   
        //printf("Trn");
        xSemaphoreGiveFromISR( NRFTransmitSempht, &xHigherPriorityTaskWoken );
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}