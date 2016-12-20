#include "mpu6050.h"
#include "delay.h"
#include "stdio.h"
#include "led.h"
#include "usart.h"
#include "adc.h"
#include "AttitudeAlgorithm.h"
#include "ControlAlgorithm.h"
#include "DataAcquisition.h"
#include "arm_math.h"
#include "outputdata.h"
#include "System.h"
#include "filter.h"
#include "MS5611.h"
SemaphoreHandle_t DataAcquisitionSempht = NULL;
xQueueHandle DataQueue = NULL;

static char offset_flag = 0;

int16_t ThrottleR = 0;
uint16_t battery = 360;


PID_Typedef PitchPID;
PID_Typedef RollPID;
PID_Typedef YawPID;

PID_Typedef PitchRataPID;
PID_Typedef RollRataPID;
PID_Typedef YawRataPID;


#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define SENSOR_MAX_G 8.0f		//constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W 1000.0f	//deg/s
#define ACC_SCALE  SENSOR_MAX_G/32768.0f
#define GYRO_SCALE  SENSOR_MAX_W/32768.0f
extern uint8_t accUpdated;
void DataCollection(IMU *imu)
{
    static uint8_t cnt = 0;
    //vTaskSuspendAll();

    if (offset_flag == 0)
    {
        for(cnt = 0; cnt < 10; cnt ++)
        {
            MPU6050ReadAcc ( &(imu->acc_adc_x) );//0.000244140625  //g=9.80665f //0.0023942 *9=0.0215478
            MPU6050ReadGyro( &(imu->gyro_adc_x) );//0.030517578125  //0.0005326321  
        }
        offset_flag = 1;
        return;
    }    
    MPU6050ReadAcc ( &(imu->acc_adc_x) );//0.000244140625  //g=9.80665f //0.0023942 *9=0.0215478
    MPU6050ReadGyro( &(imu->gyro_adc_x) );//0.030517578125  //0.0005326321  
    battery = Get_Adc(8)/6;
    
    imu->acc_offset_x = -380;
    imu->acc_offset_y = 210;
    imu->acc_offset_z = 226;
    imu->gyro_offset_x = 60;
    imu->gyro_offset_y = -15;
    imu->gyro_offset_z = 83;   

    imu->acc_raw_x = ((float)(imu->acc_adc_x + imu->acc_offset_x));
    imu->acc_raw_y = ((float)(imu->acc_adc_y + imu->acc_offset_y));
    imu->acc_raw_z = ((float)(imu->acc_adc_z + imu->acc_offset_z));
    
    imu->gyro_raw_x = ((float)(imu->gyro_adc_x + imu->gyro_offset_x));
    imu->gyro_raw_y = ((float)(imu->gyro_adc_y + imu->gyro_offset_y));
    imu->gyro_raw_z = ((float)(imu->gyro_adc_z + imu->gyro_offset_z));     
    
//    imu->acc_raw_x = LPF2pApply_1((float)(imu->acc_adc_x + imu->acc_offset_x));
//    imu->acc_raw_y = LPF2pApply_2((float)(imu->acc_adc_y + imu->acc_offset_y));
//    imu->acc_raw_z = LPF2pApply_3((float)(imu->acc_adc_z + imu->acc_offset_z));
//    
//    imu->gyro_raw_x = LPF2pApply_4((float)(imu->gyro_adc_x + imu->gyro_offset_x));
//    imu->gyro_raw_y = LPF2pApply_5((float)(imu->gyro_adc_y + imu->gyro_offset_y));
//    imu->gyro_raw_z = LPF2pApply_6((float)(imu->gyro_adc_z + imu->gyro_offset_z));    
    
    // rad/s   m/s^2  
    imu->acc_x = imu->acc_raw_x * ACC_SCALE * CONSTANTS_ONE_G ;
    imu->acc_y = imu->acc_raw_y * ACC_SCALE * CONSTANTS_ONE_G ;
    imu->acc_z = imu->acc_raw_z * ACC_SCALE * CONSTANTS_ONE_G ;

    imu->gyro_x = imu->gyro_raw_x * GYRO_SCALE * M_PI /180.f;
    imu->gyro_y = imu->gyro_raw_y * GYRO_SCALE * M_PI /180.f;
    imu->gyro_z = imu->gyro_raw_z * GYRO_SCALE * M_PI /180.f;    
    

    

    
    MahonyAHRSupdate( imu->gyro_x, imu->gyro_y, imu->gyro_z,
                      imu->acc_x,  imu->acc_y,  imu->acc_z,
                      0,  0,  0 );
    sensfusion6GetEulerRPY( &(imu->roll), &(imu->pitch), &(imu->yaw));
    q4ToDCM( &(imu->DCM[0][0]) );
    accUpdated = 1;
    imu->rollRate  = imu->gyro_x*180.f/M_PI;
    imu->pitchRate = imu->gyro_y*180.f/M_PI;
    imu->yawRate   = imu->gyro_z*180.f/M_PI;
    OutData[0] = imu->roll*10;
    OutData[1] = imu->rollRate;
    OutData[2] = imu->yaw*10;
    OutData[3] = imu->yawRate;
    OutPut_Data();

//    if ( cnt == 1)
//    {
//        MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);//start temperature transform 
//        AltitudeCombineThread();
//    }
//    if ( cnt == 3)
//    {
//        MS561101BA_GetTemperature(); //read temperature
//        MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);//start barometric pressure transform   
//    } 
//    if ( cnt == 6)
//    {
//        MS561101BA_getPressure();   //read barometric pressure    
//        cnt =0;
//    }
//    cnt ++;

   // xTaskResumeAll();
}

void DataProcessing(IMU *imu)
{
    if (offset_flag == 0)
    {
        return;
    }
/*    
    static uint8_t cnt = 0;
    float dPitch = 0.0f;
    static float angle_z = 0;


//    Kalman_Filter_Z(angle->Yaw,AngularVelocity->z,&f_angle.Yaw,&f_gyro.z);
//    OutData[0] = f_angle.Pitch;
//    OutData[1] = f_angle.Roll;
//    OutData[2] = f_gyro.x;
//    OutData[3] = f_gyro.y;
//   // OutPut_Data();
//    //return;
//   //if(f_gyro.z>5 || f_gyro.z<-5)
//    if ( fabsf(f_gyro.z) > 5)
//    angle_z += f_gyro.z*SYSTEM_CYCLE;
//    f_angle.Yaw = angle_z;
//   
//    PitchPID.Measure = f_angle.Pitch;
//    PitchPID.Diff = -f_gyro.x;

    RollPID.Measure = f_angle.Roll;
    RollPID.Diff = -f_gyro.y;  
    
    YawPID.Measure = 0;
    YawPID.Diff = f_gyro.z;
    //printf("YawPID.Diff=%d\n",(int)YawPID.Diff);
    PID_Postion(&YawPID,0);
    cnt++;
    if ( cnt == 5 )
    {
        PID_Postion(&PitchPID,0);
        PID_Postion(&RollPID,0);
        cnt = 0;
    }

    
    PitchRataPID.Measure = f_gyro.x;
    PitchRataPID.Target = PitchPID.Output;
    //PitchRataPID.Target = PitchPID.Target*3;
    //PitchRataPID.Target = 0;
    
    RollRataPID.Measure = f_gyro.y;
    RollRataPID.Target = RollPID.Output; 
    //RollRataPID.Target = RollPID.Target*3;
    //RollRataPID.Target = 0;


    PID_Postion(&PitchRataPID,1);
    PID_Postion(&RollRataPID,1);
    PID_Postion(&YawRataPID,1);    
//    PID_Normal(&PitchPID);
//    PID_Normal(&RollPID);
//    PID_Normal(&YawPID);
    acc.z = acc.z*0.1;
    //  PWM输出            基准油门         油门抑制         Pitch校正      Roll校正      Yaw校正   
    //PWM_Out(ThrottleR,(int16_t)acc.z,(int)PitchPID.Output,(int)RollPID.Output,(int)YawPID.Output);
    
    //PWM_Out(ThrottleR,0,0,0,(int)YawPID.Output);
    PWM_Out(ThrottleR,(int16_t)acc.z,(int)PitchRataPID.Output,(int)RollRataPID.Output,(int)YawPID.Output);
    //PWM_Out(ThrottleR,0,(int)PitchRataPID.Output,0,0);

*/   
}
void Data()
{
    static uint16_t cnt = 0;
    vTaskSuspendAll();
    //taskENTER_CRITICAL();
    DataCollection(&imu);
    DataProcessing(&imu);
    cnt++;
    if( 125 == cnt )
    {
        //printf("roll=%f\r\n",imu.roll);
//        LED0 ^= 1;
//        LED1 ^= 1;
        cnt = 0;
    }    
    xTaskResumeAll();
    //taskEXIT_CRITICAL();
}
void AnglePidParaInit()
{
    ThrottleR = 0;
    
    PitchPID.Kp = 7;//7.5;
    PitchPID.Ki = 0;//0;
    PitchPID.Kd = 0.2;//0.3;
 

    
    PitchPID.Target = 0;
    PitchPID.Measure = 0;
    PitchPID.Integral = 0;
    PitchPID.IntegralLimit = 50;
    PitchPID.Diff = 0;
    PitchPID.LastError = 0;
    PitchPID.Output = 0;
    
    RollPID.Kp = PitchPID.Kp;
    RollPID.Ki = PitchPID.Ki;
    RollPID.Kd = PitchPID.Kd;
    RollPID.Measure = PitchPID.Measure;
    RollPID.Target = PitchPID.Target;
    RollPID.Integral = PitchPID.Integral;
    RollPID.IntegralLimit = PitchPID.IntegralLimit;
    RollPID.Diff = PitchPID.Diff;
    RollPID.LastError = PitchPID.LastError;
    RollPID.Output = PitchPID.Output;
 
    YawPID.Kp = 0;
    YawPID.Ki = 0;
    YawPID.Kd = 40; //45
    YawPID.Measure = PitchPID.Measure;
    YawPID.Target = PitchPID.Target;
    YawPID.Integral = PitchPID.Integral;
    YawPID.IntegralLimit = PitchPID.IntegralLimit;
    YawPID.Diff = PitchPID.Diff;
    YawPID.LastError = PitchPID.LastError;
    YawPID.Output = PitchPID.Output;  

}
void AngleRataPidParaInit()
{
    //ThrottleR = 0;
    
    PitchRataPID.Kp = 11;//10;//9.5;
    PitchRataPID.Ki = 0;//5.6;//7.5;
    PitchRataPID.Kd = 0.1;//0.1;//0.1
 

    
    PitchRataPID.Target = 0;
    PitchRataPID.Measure = 0;
    PitchRataPID.Integral = 0;
    PitchRataPID.IntegralLimit = 50;
    PitchRataPID.Diff = 0;
    PitchRataPID.LastError = 0;
    PitchRataPID.Output = 0;
    
    RollRataPID.Kp = PitchRataPID.Kp;
    RollRataPID.Ki = PitchRataPID.Ki;
    RollRataPID.Kd = PitchRataPID.Kd;
    RollRataPID.Measure = PitchRataPID.Measure;
    RollRataPID.Target = PitchRataPID.Target;
    RollRataPID.Integral = PitchRataPID.Integral;
    RollRataPID.IntegralLimit = PitchRataPID.IntegralLimit;
    RollRataPID.Diff = PitchRataPID.Diff;
    RollRataPID.LastError = PitchRataPID.LastError;
    RollRataPID.Output = PitchRataPID.Output;

//    YawRataPID.Kp = 0;
//    YawRataPID.Ki = 0;
//    YawRataPID.Kd = 40; 
//    YawRataPID.Measure = PitchRataPID.Measure;
//    YawRataPID.Target = PitchRataPID.Target;
//    YawRataPID.Integral = PitchRataPID.Integral;
//    YawRataPID.IntegralLimit = PitchRataPID.IntegralLimit;
//    YawRataPID.Diff = PitchRataPID.Diff;
//    YawRataPID.LastError = PitchRataPID.LastError;
//    YawRataPID.Output = PitchRataPID.Output;  

}
// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);
void Filter_Init()
{
    LPF2pSetCutoffFreq_1(SYSTEM_FREQ,FILTER_FREQ);		//30Hz
    LPF2pSetCutoffFreq_2(SYSTEM_FREQ,FILTER_FREQ);
    LPF2pSetCutoffFreq_3(SYSTEM_FREQ,FILTER_FREQ);
    LPF2pSetCutoffFreq_4(SYSTEM_FREQ,FILTER_FREQ);
    LPF2pSetCutoffFreq_5(SYSTEM_FREQ,FILTER_FREQ);
    LPF2pSetCutoffFreq_6(SYSTEM_FREQ,FILTER_FREQ);
}
void DataAcquisitionTask()
{
    AnglePidParaInit();
    AngleRataPidParaInit();
    while(1)
    {
       xSemaphoreTake( DataAcquisitionSempht, portMAX_DELAY );
       Data();    
    }
}
void DataAcquisitionTaskCreate()
{
    DataAcquisitionSempht = xSemaphoreCreateBinary();
    xTaskCreate(    DataAcquisitionTask,        /* Pointer to the function that implements the task.              */
                    "DataAcquisitionTask",      /* Text name for the task.  This is to facilitate debugging only. */
                    512,                        /* Stack depth in words.                                          */
                    NULL,                       /* We are not using the task parameter.                           */
                    3,                          /* This task will run at priority 1.                              */
                    NULL );                     /* We are not using the task handle.                              */        
}
void DataAcquisitionIntterupt()
{   
    static BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR( DataAcquisitionSempht, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}