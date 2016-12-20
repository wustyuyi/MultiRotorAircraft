#ifndef __REMOTERECEIVE_H__
#define __REMOTERECEIVE_H__
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
typedef enum{
    TxMode,
    RxMode
}NRF_MODE;

typedef enum{
    Uart,
    Remote,
    Bluetooth,
    Wifi
}COMMAND_SOURCE;

typedef enum{
    Throttle,
    Direction,
    Action
}COMMAND_TYPE;

typedef struct{
    COMMAND_SOURCE Source;
    COMMAND_TYPE   Cmd;
    char           *DataBuf;
}COMMAND;

extern void CommandProcess();
extern void UartReciveTaskCreate();
extern void UartReceive(char *UartData);
extern void NRFReceiveIntterupt();
extern void NRFTransmitWhile();
extern xQueueHandle CommandQueue;
extern NRF_MODE NrfMode;
#endif