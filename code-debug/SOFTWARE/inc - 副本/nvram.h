#ifndef __NVRAM_H__
#define __NVRAM_H__
#include "sys.h"
#include "stmflash.h"
#include "string.h"
typedef struct{
    uint8_t  Handle;
    uint8_t  Length;
    uint16_t Next;
}PARAMETER;
//第一个Parameter在NVRAM起始处
#define STM_SECTOR_SIZE 1024
#define STM32_FLASH_SIZE 11
#define PARAMETER_NUM 32
#define NVRAM_START_ADDR (STM32_FLASH_BASE+(STM32_FLASH_SIZE-1)*STM_SECTOR_SIZE)
#define NVRAM_LENGTH  STM_SECTOR_SIZE


#define PARAMETER_0   0
#define PARAMETER_1   1
#define PARAMETER_2   2
#define PARAMETER_3   3
#define PARAMETER_4   4
#define PARAMETER_5   5
#define PARAMETER_6   6
#define PARAMETER_7   7
#define PARAMETER_8   8
#define PARAMETER_9   9


#endif //__NVRAM_H__