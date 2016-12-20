#ifndef __DATAACQUISITION_H__
#define __DATAACQUISITION_H__
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//#include "ControlAlgorithm.h"
#include "AttitudeAlgorithm.h"


extern PID_Typedef PitchPID;
extern PID_Typedef RollPID;
extern int16_t ThrottleR;

void DataAcquisitionTaskCreate(void);
void DataAcquisitionIntterupt(void);
#endif
