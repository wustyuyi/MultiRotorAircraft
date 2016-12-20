#ifndef __UIMANAGE_H__
#define __UIMANAGE_H__
#include "led.h"

#define communication_module 1
#define battery_module       2
#define engine_module        3




typedef struct{
    uint8_t module_id;
    uint8_t modeule_state;
}UI;
void UIManageTaskCreate();
    
#endif//__UIMANAGE_H__