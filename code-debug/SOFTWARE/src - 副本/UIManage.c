#include "UIManage.h"

#include "UIManage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
xQueueHandle UIQueue    = NULL;
void LEDCtrlTask()
{
    static portTickType xLastWakeTime;  
    while(1)
    {   
        //LED0 ^= 1;
        //LED1 ^= 1;
//        LED2 ^= 1;
//        LED3 ^= 1;
        vTaskDelayUntil( &xLastWakeTime,1000 ); 
    }

}
void UIManageTask()
{
    static UI UI_Last={0,0};
    UI UI_State={0,0};
    portBASE_TYPE xStatus;
    while(1)
    {
        xStatus = xQueueReceive( UIQueue,&UI_State,portMAX_DELAY ); 
        taskYIELD();//???????????
    }

}
void UIManageTaskCreate()
{
    UIQueue    = xQueueCreate( 4, sizeof(UI) );
    
    xTaskCreate(    UIManageTask,     /* Pointer to the function that implements the task.              */
                    "UIManageTask",   /* Text name for the task.  This is to facilitate debugging only. */
                    128,        /* Stack depth in words.                                          */
                    NULL,       /* We are not using the task parameter.                           */
                    1,          /* This task will run at priority 1.                              */
                    NULL );     /* We are not using the task handle.                              */    
    xTaskCreate(    LEDCtrlTask,     /* Pointer to the function that implements the task.              */
                    "LEDCtrlTask",   /* Text name for the task.  This is to facilitate debugging only. */
                    128,        /* Stack depth in words.                                          */
                    NULL,       /* We are not using the task parameter.                           */
                    1,          /* This task will run at priority 1.                              */
                    NULL );     /* We are not using the task handle.                              */       

}