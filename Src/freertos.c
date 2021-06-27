/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */    
#include "arm_math.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h" 
#include "AT24Cxx.h" 
#include "tim.h"
#include "fatfs.h"
#include "sdio.h"
#include "GPS_UBLOX_NEO_M8N.h"	
#include "can.h"
#include "Wireless_AS32_TTL_1W.h"
#include "Data_Generic.h"
#include "math.h"
#include "HostPCDebug_AT24CxxMem.h"
#include "HostPCDisplay.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId PowerOnSelfTestTaskHandle;
osThreadId FirstPowerOn_Mot_DisableTaskHandle;
osThreadId FirstPowerOn_Mot_EnableTaskHandle;
osThreadId CarRunningPowerOn_Mot_EnableTaskHandle;
osThreadId ThrottleAndBrakes_FaultCheckTaskHandle;
osThreadId Brakes_FaultCheckTaskHandle;
osThreadId OffLine_Mot_FOR_FaultTaskHandle;
osThreadId OnLine_Mot_FOR_ClearFaultTaskHandle;
osThreadId ThrottleTaskHandle;
osThreadId LightsUpAfterBrakeTaskHandle;
osThreadId E_ABSTaskHandle;
osThreadId MotController_CoolTaskHandle;

osThreadId TractionControlModeJudgmentTaskHandle;
osThreadId Line_TractionControlTaskHandle;
osThreadId RollStabilizerBarSteeringEngineTaskHandle;
osThreadId DRSSteeringEngineTaskHandle;
osThreadId SuspensionLineShift_ZeroSettingTaskHandle;
osThreadId Mileage_SaveTaskHandle;
osThreadId Mileage_ClearTaskHandle;
osThreadId MotIGBTAndAir_DataGuardTaskHandle;
osThreadId LeakageCurrentLimitProcessTaskHandle;
osThreadId CAN1_OffLine_ProcessTaskHandle;
//osThreadId PowerBroken_ClearHighVolValueTaskHandle;
osThreadId LEDTaskHandle;

osThreadId GPSTaskHandle;
osThreadId Fatfs_SDCardTaskHandle;
/* USER CODE BEGIN Variables */
//UBaseType_t Start,End;																//堆栈使用深度查询变量，调试时使用
unsigned long ulIdleCycleCount=0UL;
void vApplicationIdleHook(void)													//定义空闲钩子函数，观察系统时间分配是否充裕，调试时使用
{
	ulIdleCycleCount++;
}

size_t FreeMem;																					//剩余堆栈空间									


FATFS SDFatFs;  																			/* File system object for SD card logical drive */
FIL MyFile;    																				/* File object */
FRESULT res;    																			/* FatFs function common result code */
uint32_t byteswritten, bytesread;                     		/* File write/read counts */
uint8_t wtext[] = "This is CSUFSAE working with FatFs"; 	/* File write buffer */
uint8_t rtext[100];	                               				/* File read buffer */    

volatile float TCS_pid_Out;	
float TCS_Adjust_Ability=1000.0;

uint8_t AT24xxTest1=100;
uint8_t AT24xxTest2;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void PowerOnSelfTestTask(void const * argument);
void FirstPowerOn_Mot_DisableTask(void const * argument);
void FirstPowerOn_Mot_EnableTask(void const * argument);
void CarRunningPowerOn_Mot_EnableTask(void const * argument);
void ThrottleAndBrakes_FaultCheckTask(void const * argument);
void Brakes_FaultCheckTask(void const * argument);
void OffLine_Mot_FOR_FaultTask(void const * argument);
void OnLine_Mot_FOR_ClearFaultTask(void const * argument);
void ThrottleTask(void const * argument);
void LightsUpAfterBrakeTask(void const * argument);
void E_ABSTask(void const * argument);
void MotController_CoolTask(void const * argument);

void TractionControlModeJudgmentTask(void const * argument);
void Line_TractionControlTask(void const * argument);
void RollStabilizerBarSteeringEngineTask(void const * argument);
void DRSSteeringEngineTask(void const * argument);
void SuspensionLineShift_ZeroSettingTask(void const * argument);
void Mileage_SaveTask(void const * argument);
void Mileage_ClearTask(void const * argument);
void MotIGBTAndAir_DataGuardTask(void const * argument);
void LeakageCurrentLimitProcessTask(void const * argument);
void CAN1_OffLine_ProcessTask(void const * argument);
//void PowerBroken_ClearHighVolValueTask(void const * argument);
void LEDTask(void const * argument);

void GPSTask(void const * argument);
void Fatfs_SDCardTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
	/* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */	
	/* definition and creation of FirstPowerOn_Mot_EnableTask */
	osThreadDef(PowerOnSelfTestTask, PowerOnSelfTestTask, osPriorityNormal, 0, 128);
  PowerOnSelfTestTaskHandle = osThreadCreate(osThread(PowerOnSelfTestTask), NULL);	
	/* definition and creation of FirstPowerOn_Mot_EnableTask */
	osThreadDef(FirstPowerOn_Mot_EnableTask, FirstPowerOn_Mot_EnableTask, osPriorityNormal, 0, 128);
  FirstPowerOn_Mot_EnableTaskHandle = osThreadCreate(osThread(FirstPowerOn_Mot_EnableTask), NULL);	
	/* definition and creation of CarRunningPowerOn_Mot_EnableTask */
	osThreadDef(CarRunningPowerOn_Mot_EnableTask, CarRunningPowerOn_Mot_EnableTask, osPriorityNormal, 0, 128);
	CarRunningPowerOn_Mot_EnableTaskHandle = osThreadCreate(osThread(CarRunningPowerOn_Mot_EnableTask), NULL);
	/* definition and creation of FirstPowerOn_Mot_DisableTask */
	osThreadDef(FirstPowerOn_Mot_DisableTask, FirstPowerOn_Mot_DisableTask, osPriorityNormal, 0, 128);
  FirstPowerOn_Mot_DisableTaskHandle = osThreadCreate(osThread(FirstPowerOn_Mot_DisableTask), NULL);	
	/* definition and creation of Brakes_FaultCheckTask */
	osThreadDef(Brakes_FaultCheckTask, Brakes_FaultCheckTask, osPriorityNormal, 0, 128);
  Brakes_FaultCheckTaskHandle = osThreadCreate(osThread(Brakes_FaultCheckTask), NULL);
	/* definition and creation of FirstPowerOn_Mot_DisableTask */
	osThreadDef(OffLine_Mot_FOR_FaultTask, OffLine_Mot_FOR_FaultTask, osPriorityNormal, 0, 128);
  OffLine_Mot_FOR_FaultTaskHandle = osThreadCreate(osThread(OffLine_Mot_FOR_FaultTask), NULL);
	/* definition and creation of OnLine_Mot_FOR_ClearFaultTask */
	osThreadDef(OnLine_Mot_FOR_ClearFaultTask, OnLine_Mot_FOR_ClearFaultTask, osPriorityNormal, 0, 128);
  OnLine_Mot_FOR_ClearFaultTaskHandle = osThreadCreate(osThread(OnLine_Mot_FOR_ClearFaultTask), NULL);	
	/* definition and creation of ThrottleTask */
	osThreadDef(ThrottleTask, ThrottleTask, osPriorityNormal, 0, 128);
  ThrottleTaskHandle = osThreadCreate(osThread(ThrottleTask), NULL);
	/* definition and creation of LightsUpAfterBrakeTask */
	osThreadDef(LightsUpAfterBrakeTask, LightsUpAfterBrakeTask, osPriorityNormal, 0, 128);
  LightsUpAfterBrakeTaskHandle = osThreadCreate(osThread(LightsUpAfterBrakeTask), NULL);
 /* definition and creation of MotController_CoolTask */
	osThreadDef(MotController_CoolTask, MotController_CoolTask, osPriorityNormal, 0, 128);
  MotController_CoolTaskHandle = osThreadCreate(osThread(MotController_CoolTask), NULL);
 
 
	/* definition and creation of TractionControlModeJudgmentTask */
	osThreadDef(TractionControlModeJudgmentTask, TractionControlModeJudgmentTask, osPriorityNormal, 0, 128);
  TractionControlModeJudgmentTaskHandle = osThreadCreate(osThread(TractionControlModeJudgmentTask), NULL);	
	/* definition and creation of RollStabilizerBarSteeringEngineTask */
	osThreadDef(RollStabilizerBarSteeringEngineTask, RollStabilizerBarSteeringEngineTask, osPriorityNormal, 0, 128);
  RollStabilizerBarSteeringEngineTaskHandle = osThreadCreate(osThread(RollStabilizerBarSteeringEngineTask), NULL);
	/* definition and creation of DRSSteeringEngineTask */
	osThreadDef(DRSSteeringEngineTask, DRSSteeringEngineTask, osPriorityNormal, 0, 128);
  DRSSteeringEngineTaskHandle = osThreadCreate(osThread(DRSSteeringEngineTask), NULL);	
	/* definition and creation of SuspensionLineShift_ZeroSettingTask */
	osThreadDef(SuspensionLineShift_ZeroSettingTask, SuspensionLineShift_ZeroSettingTask, osPriorityNormal, 0, 128);
  SuspensionLineShift_ZeroSettingTaskHandle = osThreadCreate(osThread(SuspensionLineShift_ZeroSettingTask), NULL);	
	/* definition and creation of Mileage_SaveTask */
	osThreadDef(Mileage_SaveTask, Mileage_SaveTask, osPriorityNormal, 0, 128);
  Mileage_SaveTaskHandle = osThreadCreate(osThread(Mileage_SaveTask), NULL);	
	/* definition and creation of Mileage_ClearTask */
	osThreadDef(Mileage_ClearTask, Mileage_ClearTask, osPriorityNormal, 0, 128);
  Mileage_ClearTaskHandle = osThreadCreate(osThread(Mileage_ClearTask), NULL);	
	/* definition and creation of IGBTAndAir_DataGuardTask */
	osThreadDef(MotIGBTAndAir_DataGuardTask, MotIGBTAndAir_DataGuardTask, osPriorityNormal, 0, 128);
  MotIGBTAndAir_DataGuardTaskHandle = osThreadCreate(osThread(MotIGBTAndAir_DataGuardTask), NULL);	
	/* definition and creation of LeakageCurrentLimitProcessTask */
	osThreadDef(LeakageCurrentLimitProcessTask, LeakageCurrentLimitProcessTask, osPriorityNormal, 0, 128);
  LeakageCurrentLimitProcessTaskHandle = osThreadCreate(osThread(LeakageCurrentLimitProcessTask), NULL);	
	/* definition and creation of CAN1_OffLine_ProcessTask */
	osThreadDef(CAN1_OffLine_ProcessTask, CAN1_OffLine_ProcessTask, osPriorityNormal, 0, 128);
  CAN1_OffLine_ProcessTaskHandle = osThreadCreate(osThread(CAN1_OffLine_ProcessTask), NULL);	
//	/* definition and creation of PowerBroken_ClearHighVolValueTask */
//	osThreadDef(PowerBroken_ClearHighVolValueTask, PowerBroken_ClearHighVolValueTask, osPriorityNormal, 0, 64);
//  PowerBroken_ClearHighVolValueTaskHandle = osThreadCreate(osThread(PowerBroken_ClearHighVolValueTask), NULL);
	/* definition and creation of LEDTask */
	osThreadDef(LEDTask, LEDTask, osPriorityNormal, 0, 64);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);
	
	/* definition and creation of GPSTask */
	osThreadDef(GPSTask, GPSTask, osPriorityNormal, 0, 128);
  GPSTaskHandle = osThreadCreate(osThread(GPSTask), NULL);
	/* definition and creation of Fatfs_SDCardTask */
	osThreadDef(Fatfs_SDCardTask, Fatfs_SDCardTask, osPriorityNormal, 0, 128);
  Fatfs_SDCardTaskHandle = osThreadCreate(osThread(Fatfs_SDCardTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */  

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}


/* USER CODE BEGIN Application */

/* PowerOnSelfTestTask function */
void PowerOnSelfTestTask(void const * argument)
{	
  /* USER CODE BEGIN PowerOnSelfTestTask */
  /* Infinite loop */		
	if (AT24Cxx_Check()){  //检查器件
		/*发送EEPROM故障信息给显示屏*/	
	}
	for(;;)
  { 	
		
		osDelay(100);		
  }
/* USER CODE END PowerOnSelfTestTask */
}   

/* FirstPowerOn_Mot_DisableTask function */
void FirstPowerOn_Mot_DisableTask(void const * argument)
{	
  /* USER CODE BEGIN FirstPowerOn_Mot_DisableTask */
  /* Infinite loop */		
	uint8_t PositiveRelayValue=0,MotorState_Data=0;
	osDelay(2000);
	for(uint8_t i=0;i<10;i++){
		CAN1_DATA_Send(ModeState_Reg,DISABLEMOT);
		Set_MotorState_Data(MotorState_n,&MotorState_Data);					//设置电机状态变量
		osDelay(2);
	}
	CSU_CAN1_Receive_Mot_Ready(MotSpeedValue_Reg,0x64);						//请求100ms发送一次电机实际转速值
	CSU_CAN1_Receive_Mot_Ready(MotCurrentValue_Reg,0x64);					//请求100ms发送一次电机实际电流值
	CSU_CAN1_Receive_Mot_Ready(MotPowerValue_Reg,0x64);						//请求100ms发送一次电机功率
	CSU_CAN1_Receive_Mot_Ready(DCBusVoltageValue_Reg,0x64);				//请求100ms发送一次控制器直流母线电压
	CSU_CAN1_Receive_Mot_Ready(Capacity_Reg,0x64);								//请求100ms发送一次Capacity I2xt
	CSU_CAN1_Receive_Mot_Ready(MotTemperature_Reg,0x64);					//请求100ms发送一次电机温度
	CSU_CAN1_Receive_Mot_Ready(IGBTTemperature_Reg,0x64);					//请求100ms发送一次IGBT温度
	CSU_CAN1_Receive_Mot_Ready(AirTemperature_Reg,0x64);					//请求100ms发送一次控制器温度
	CSU_CAN1_Receive_Mot_Ready(OutVoltage_Reg,0x64);							//请求100ms发送一次电机输出电压
	CSU_CAN1_Receive_Mot_Ready(WarningErrorMap_Reg,0x64);					//请求100ms发送一次Warning-Error map,存储错误和警告信息
	CSU_CAN1_Receive_Mot_Ready(StatusMap_Reg,0x64);								//请求100ms发送一次Status map,状态信息
	for(;;)
  { 	
		Get_CAN_Analysis_Data(PositiveRelayState_n,&PositiveRelayValue);
		if(PositiveRelayValue==0x01){														//电机电路通电
			for(uint8_t i=0;i<10;i++){
				CAN1_DATA_Send(ModeState_Reg,DISABLEMOT);
				Set_MotorState_Data(MotorState_n,&MotorState_Data);						//设置电机状态变量
				CSU_CAN1_Receive_Mot_Ready(MotSpeedValue_Reg,0x64);						//请求100ms发送一次电机实际转速值
				CSU_CAN1_Receive_Mot_Ready(MotCurrentValue_Reg,0x64);					//请求100ms发送一次电机实际电流值
				CSU_CAN1_Receive_Mot_Ready(MotPowerValue_Reg,0x64);						//请求100ms发送一次电机功率
				CSU_CAN1_Receive_Mot_Ready(DCBusVoltageValue_Reg,0x64);				//请求100ms发送一次控制器直流母线电压
				CSU_CAN1_Receive_Mot_Ready(Capacity_Reg,0x64);								//请求100ms发送一次Capacity I2xt
				CSU_CAN1_Receive_Mot_Ready(MotTemperature_Reg,0x64);					//请求100ms发送一次电机温度
				CSU_CAN1_Receive_Mot_Ready(IGBTTemperature_Reg,0x64);					//请求100ms发送一次IGBT温度
				CSU_CAN1_Receive_Mot_Ready(AirTemperature_Reg,0x64);					//请求100ms发送一次控制器温度
				CSU_CAN1_Receive_Mot_Ready(OutVoltage_Reg,0x64);							//请求100ms发送一次电机输出电压
				CSU_CAN1_Receive_Mot_Ready(WarningErrorMap_Reg,0x64);					//请求100ms发送一次Warning-Error map,存储错误和警告信息
				CSU_CAN1_Receive_Mot_Ready(StatusMap_Reg,0x64);								//请求100ms发送一次Status map,状态信息
				osDelay(2);
			}			
			vTaskDelete( NULL );			
		}	
		osDelay(2);		
  }
/* USER CODE END FirstPowerOn_Mot_DisableTask */
}    

/* FirstPowerOn_Mot_EnableTask function */
void FirstPowerOn_Mot_EnableTask(void const * argument)
{	
  /* USER CODE BEGIN FirstPowerOn_Mot_EnableTask */
  /* Infinite loop */	
	uint8_t CarStartFlag=0,CarSpeakerCtrl=0,MotorState_Data=0;
	float CarBrakePressureFront,CarBrakePressureBack;	
	float BrakePressure_Range_ActualHigh_Float,BrakePressure_Range_ActualLow_Float;
	for(;;)
  { 	
		Get_CAN_Analysis_Data(StartFlag_n,&CarStartFlag);
		Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
		Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);				
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualHigh_n,&BrakePressure_Range_ActualHigh_Float);	
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualLow_n,&BrakePressure_Range_ActualLow_Float);			
		if((CarBrakePressureFront+CarBrakePressureBack)/2.0-BrakePressure_Range_ActualLow_Float>0.1*(BrakePressure_Range_ActualHigh_Float-BrakePressure_Range_ActualLow_Float)&&CarStartFlag==1){	//刹车踩下10%量程，且按下启动按钮
			CarSpeakerCtrl=1;
			Set_Fan_Speaker_Taillight_Pump_CtrlData(SpeakerCtrl_n,&CarSpeakerCtrl);
			CAN2_DATA_Send(ECU_ECU2_CONTROL_ID);
			osDelay(2500);																																	//喇叭响2.5秒关闭，然后电机使能
			CarSpeakerCtrl=0;
			Set_Fan_Speaker_Taillight_Pump_CtrlData(SpeakerCtrl_n,&CarSpeakerCtrl);
			CAN2_DATA_Send(ECU_ECU2_CONTROL_ID);
			for(uint8_t i=0;i<10;i++){
				CAN1_DATA_Send(ModeState_Reg,ENABLEMOT);
				MotorState_Data=1;
				Set_MotorState_Data(MotorState_n,&MotorState_Data);														//设置电机状态变量
//				CAN1_DATA_Send(ACCRamp_Reg,0x2710);//10S																				//调整加速斜坡时间
//				CAN1_DATA_Send(DECRamp_Reg,0x2710);
				osDelay(2);
			}
			/* definition and creation of ThrottleAndBrakes_FaultCheckTask */
			osThreadDef(ThrottleAndBrakes_FaultCheckTask, ThrottleAndBrakes_FaultCheckTask, osPriorityNormal, 0, 128);
			ThrottleAndBrakes_FaultCheckTaskHandle = osThreadCreate(osThread(ThrottleAndBrakes_FaultCheckTask), NULL);
			vTaskDelete( NULL );			
		}	
		osDelay(20);		
  }
/* USER CODE END FirstPowerOn_Mot_EnableTask */
}

/* CarRunningPowerOn_Mot_EnableTask function */
void CarRunningPowerOn_Mot_EnableTask(void const * argument)
{	
  /* USER CODE BEGIN CarRunningPowerOn_Mot_EnableTask */
  /* Infinite loop */	
	uint8_t SuspensionLineShift_ZeroSetting_Flag=0,MotorState_Data=0;
	float CarBrakePressureFront,CarBrakePressureBack;	
	float BrakePressure_Range_ActualHigh_Float,BrakePressure_Range_ActualLow_Float;
	for(;;)
  { 	
		Get_CAN_Analysis_Data(SuspensionLineShift_ZeroSetting_n,&SuspensionLineShift_ZeroSetting_Flag);
		Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
		Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);				
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualHigh_n,&BrakePressure_Range_ActualHigh_Float);	
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualLow_n,&BrakePressure_Range_ActualLow_Float);			
		if((CarBrakePressureFront+CarBrakePressureBack)/2.0-BrakePressure_Range_ActualLow_Float>0.1*(BrakePressure_Range_ActualHigh_Float-BrakePressure_Range_ActualLow_Float)&&SuspensionLineShift_ZeroSetting_Flag==1){	//刹车踩下10%量程，且按下启动按钮
			CAN1_DATA_Send(ModeState_Reg,ENABLEMOT);
			MotorState_Data=1;
			Set_MotorState_Data(MotorState_n,&MotorState_Data);														//设置电机状态变量
			SuspensionLineShift_ZeroSetting_Flag=0;
			Set_CAN_Analysis_Data(SuspensionLineShift_ZeroSetting_n,&SuspensionLineShift_ZeroSetting_Flag);
		}	
		osDelay(20);		
  }
/* USER CODE END CarRunningPowerOn_Mot_EnableTask */
}
uint8_t temp1,temp2;
/* ThrottleAndBrakes_FaultCheckTask function */
void ThrottleAndBrakes_FaultCheckTask(void const * argument)
{	
  /* USER CODE BEGIN ThrottleAndBrakes_FaultCheckTask */
  /* Infinite loop */	
	float CarBrakePressureFront,CarBrakePressureBack,BrakePressure_Range_ActualHigh_Float,BrakePressure_Range_ActualLow_Float;	
	uint16_t CarAccPedalLeft,CarAccPedalRight,AccPeda_Range_High,AccPeda_Range_Low;
	static uint8_t Conflict_Disable_Flag=0,AccPedaSensor_Offline_Flag=0,BrakeSensor_Offline_Flag=0,Check_Switch=0;
	uint8_t IMD_BrakeReliability_Trigger_Data=0;
	uint8_t Throttle_Brake_InterferingState_Data=0;
	for(;;)
  { 	
		Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
		Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualHigh_n,&BrakePressure_Range_ActualHigh_Float);	
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualLow_n,&BrakePressure_Range_ActualLow_Float);	
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_High_n,&AccPeda_Range_High);
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_Low_n,&AccPeda_Range_Low);
		Get_Acc_Brake_Angle_LowVData(AccPedalLeft_n,&CarAccPedalLeft);
		Get_Acc_Brake_Angle_LowVData(AccPedalRight_n,&CarAccPedalRight);			
		if(Check_Switch==0){					
			if((fabsf(CarAccPedalLeft-CarAccPedalRight)>=0.30*(AccPeda_Range_High-AccPeda_Range_Low))&&Conflict_Disable_Flag==0){
				osDelay(100);
				Get_Acc_Brake_Angle_LowVData(AccPedalLeft_n,&CarAccPedalLeft);
				Get_Acc_Brake_Angle_LowVData(AccPedalRight_n,&CarAccPedalRight);		
				if(fabsf(CarAccPedalLeft-CarAccPedalRight)>=0.30*(AccPeda_Range_High-AccPeda_Range_Low)){
					xTaskNotifyGive(OffLine_Mot_FOR_FaultTaskHandle);											//两个油门传感器 数值相差踏板行程的10%持续时间100ms，通知切断动力输出
					Conflict_Disable_Flag=1;
				}			
			}
			else if((fabsf(CarAccPedalLeft-CarAccPedalRight)<0.30*(AccPeda_Range_High-AccPeda_Range_Low))&&Conflict_Disable_Flag==1){
				xTaskNotifyGive(OnLine_Mot_FOR_ClearFaultTaskHandle);	
				Conflict_Disable_Flag=0;
			}
			if((CarAccPedalLeft<10||CarAccPedalRight<10)&&AccPedaSensor_Offline_Flag==0){
				osDelay(100);
				Get_Acc_Brake_Angle_LowVData(AccPedalLeft_n,&CarAccPedalLeft);
				Get_Acc_Brake_Angle_LowVData(AccPedalRight_n,&CarAccPedalRight);
				if(CarAccPedalLeft<10||CarAccPedalRight<10){
					xTaskNotifyGive(OffLine_Mot_FOR_FaultTaskHandle);											//油门传感器有一个掉线且持续时间100ms，通知切断动力输出
					AccPedaSensor_Offline_Flag=1;
				}
			}
			else if((CarAccPedalLeft>10&&CarAccPedalRight>10)&&AccPedaSensor_Offline_Flag==1){
				xTaskNotifyGive(OnLine_Mot_FOR_ClearFaultTaskHandle);
				AccPedaSensor_Offline_Flag=0;
			}		
//			if(((CarBrakePressureFront<=0.1)||(CarBrakePressureBack<=0.1))&&(BrakeSensor_Offline_Flag==0)){			
//				osDelay(100);
//				Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
//				Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);
//				if(((CarBrakePressureFront<=0.1)||(CarBrakePressureBack<=0.1))&&(BrakeSensor_Offline_Flag==0)){							//制动油压传感器离线100ms,通知切断动力输出	
////					xTaskNotifyGive(OffLine_Mot_FOR_FaultTaskHandle);	
//					IMD_BrakeReliability_Trigger_Data=1;																										//断开ECU2中安全回路继电器
//					Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);
//					BrakeSensor_Offline_Flag=1;
//				}
//			}
//			else if(((CarBrakePressureFront>0.1)&&(CarBrakePressureBack>0.1))&&(BrakeSensor_Offline_Flag==1)){
////				xTaskNotifyGive(OnLine_Mot_FOR_ClearFaultTaskHandle);	
//				IMD_BrakeReliability_Trigger_Data=0;																										//断开ECU2中安全回路继电器
//				Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);
//				BrakeSensor_Offline_Flag=0;
//			}
			temp1=(CarAccPedalLeft+CarAccPedalRight)/2.0-AccPeda_Range_Low>0.25*(AccPeda_Range_High-AccPeda_Range_Low);
			temp2=(CarBrakePressureFront+CarBrakePressureBack)/2.0-BrakePressure_Range_ActualLow_Float>=0.1*(BrakePressure_Range_ActualHigh_Float-BrakePressure_Range_ActualLow_Float);
			if((CarAccPedalLeft+CarAccPedalRight)/2.0-AccPeda_Range_Low>0.25*(AccPeda_Range_High-AccPeda_Range_Low)&&(CarBrakePressureFront+CarBrakePressureBack)/2.0-BrakePressure_Range_ActualLow_Float>=0.1*(BrakePressure_Range_ActualHigh_Float-BrakePressure_Range_ActualLow_Float)){
					/* definition and creation of E_ABSTask */
					xTaskNotifyGive(OffLine_Mot_FOR_FaultTaskHandle);
					osThreadDef(E_ABSTask, E_ABSTask, osPriorityNormal, 0, 128);						//油门信号输出超过25%并制动踏板被踩下，切断动力输出，创建电子刹车功能任务
					E_ABSTaskHandle = osThreadCreate(osThread(E_ABSTask), NULL);		
					Conflict_Disable_Flag=0;																								//车子跑动过程中必须是可以检查油门故障，安全保障
					AccPedaSensor_Offline_Flag=0;																						//车子跑动过程中必须是可以检查油门在线，安全保障
					BrakeSensor_Offline_Flag=0;																							//车子跑动过程中必须是可以检查制动故障，安全保障			
					Throttle_Brake_InterferingState_Data=1;
					Set_Throttle_Brake_Interferingtate_Data(Throttle_Brake_InterferingState_n,&Throttle_Brake_InterferingState_Data);
			}
		}
		osDelay(20);		
  }
/* USER CODE END ThrottleAndBrakes_FaultCheckTask */
}

/* Brakes_FaultCheckTask function */
void Brakes_FaultCheckTask(void const * argument)
{	
  /* USER CODE BEGIN Brakes_FaultCheckTask */
  /* Infinite loop */		
	float CarBrakePressureFront,CarBrakePressureBack;	
	static uint8_t BrakeSensor_Offline_Flag=0,Check_Switch=0;
	uint8_t IMD_BrakeReliability_Trigger_Data=0;
	uint8_t Throttle_Brake_InterferingState_Data=0;
	for(;;)
  { 	
		if(Check_Switch==0){
			Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
			Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);
			if(((CarBrakePressureFront<=0.1)||(CarBrakePressureBack<=0.1))&&(BrakeSensor_Offline_Flag==0)){			
				osDelay(100);
				Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
				Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);
				if(((CarBrakePressureFront<=0.1)||(CarBrakePressureBack<=0.1))&&(BrakeSensor_Offline_Flag==0)){							//制动油压传感器离线100ms,通知切断动力输出	
					IMD_BrakeReliability_Trigger_Data=1;																										//断开ECU2中安全回路继电器
					Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);
					BrakeSensor_Offline_Flag=1;
				}
			}
			else if(((CarBrakePressureFront>0.1)&&(CarBrakePressureBack>0.1))&&(BrakeSensor_Offline_Flag==1)){
				IMD_BrakeReliability_Trigger_Data=0;																											//闭合ECU2中安全回路继电器
				Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);
				BrakeSensor_Offline_Flag=0;
			}
		}
		osDelay(20);
  }
/* USER CODE END Brakes_FaultCheckTask */
}

/* OffLine_Mot_FOR_FaultTask function */
void OffLine_Mot_FOR_FaultTask(void const * argument)
{	
  /* USER CODE BEGIN OffLine_Mot_FOR_FaultTask */
  /* Infinite loop */		
	uint8_t MotorState_Data=0;
	for(;;)
  { 	
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		for(uint8_t i=0;i<10;i++){
			CAN1_DATA_Send(ModeState_Reg,DISABLEMOT);																//有油门，刹车，等重大故障需要停车	
			MotorState_Data=0;
			Set_MotorState_Data(MotorState_n,&MotorState_Data);											//设置电机状态变量
			osDelay(2);
		}
  }
/* USER CODE END OffLine_Mot_FOR_FaultTask */
}

/* OnLine_Mot_FOR_ClearFaultTask function */
void OnLine_Mot_FOR_ClearFaultTask(void const * argument)
{	
  /* USER CODE BEGIN OnLine_Mot_FOR_ClearFaultTask */
  /* Infinite loop */		
	uint8_t MotorState_Data=0;
	for(;;)
  { 	
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		for(uint8_t i=0;i<10;i++){
			CAN1_DATA_Send(ModeState_Reg,ENABLEMOT);																//有油门，刹车，等重大故障被清除，方开动力	
			MotorState_Data=1;
			Set_MotorState_Data(MotorState_n,&MotorState_Data);											//设置电机状态变量
			osDelay(2);
		}
  }
/* USER CODE END OnLine_Mot_FOR_ClearFaultTask */
}

/* ThrottleTask function */
void ThrottleTask(void const * argument)
{	
  /* USER CODE BEGIN ThrottleTask */
  /* Infinite loop */	
	uint16_t CarAccPedalLeft,CarAccPedalRight,AccPeda_Range_High,AccPeda_Range_Low;
	float DigitalTorqueCtl1,DigitalTorqueCtl2;	
	uint8_t LineTCSFlag;
	for(;;)
  { 	
		Get_Acc_Brake_Angle_LowVData(AccPedalLeft_n,&CarAccPedalLeft);
		Get_Acc_Brake_Angle_LowVData(AccPedalRight_n,&CarAccPedalRight);
		Get_Line_CornersTCSState_Data(LineTCSState_n,&LineTCSFlag);
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_High_n,&AccPeda_Range_High);
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_Low_n,&AccPeda_Range_Low);
		if((CarAccPedalLeft+CarAccPedalRight)/2-AccPeda_Range_Low<0||(CarAccPedalLeft+CarAccPedalRight)/2-AccPeda_Range_Low<=150){			//有轻微波动
			DigitalTorqueCtl2=0;
		}
		else{
//			if(LineTCSFlag==0){
				DigitalTorqueCtl2=(float)((CarAccPedalLeft+CarAccPedalRight)/2-AccPeda_Range_Low)/(AccPeda_Range_High-AccPeda_Range_Low)*32767.0;
				if(DigitalTorqueCtl2>32767){
					DigitalTorqueCtl2=32767;
				}
//			}
//			else if(LineTCSFlag==1){
//				DigitalTorqueCtl1=(float)((CarAccPedalLeft+CarAccPedalRight)/2-AccPeda_Range_Low)/(AccPeda_Range_High-AccPeda_Range_Low)*32767.0;
//				if(DigitalTorqueCtl1>32767){
//					DigitalTorqueCtl1=32767;
//				}
//				DigitalTorqueCtl2=((float)((CarAccPedalLeft+CarAccPedalRight)/2-AccPeda_Range_Low)/(AccPeda_Range_High-AccPeda_Range_Low)-TCS_pid_Out/TCS_Adjust_Ability)*32767.0;
//				if(DigitalTorqueCtl2>DigitalTorqueCtl1){
//					DigitalTorqueCtl2=DigitalTorqueCtl1;
//				}
//				else if(DigitalTorqueCtl2<0){
//					DigitalTorqueCtl2=0;
//				}					
//			}
		}
		CAN1_DATA_Send(DigitalTorqueCtl_Reg,(uint16_t)DigitalTorqueCtl2);	
		osDelay(20);
  }
/* USER CODE END ThrottleTask */
}

/* LightsUpAfterBrakeTask function */
void LightsUpAfterBrakeTask(void const * argument)
{	
  /* USER CODE BEGIN LightsUpAfterBrakeTask */
  /* Infinite loop */	
	float CarBrakePressureFront,CarBrakePressureBack,BrakePressure_Range_ActualHigh_Float,BrakePressure_Range_ActualLow_Float;
	uint8_t CarTaillightCtrl=0,CarTaillightFlag=0;
	for(;;)
  { 	
		Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
		Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);		
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualHigh_n,&BrakePressure_Range_ActualHigh_Float);				
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualLow_n,&BrakePressure_Range_ActualLow_Float);		
		if((CarBrakePressureFront+CarBrakePressureBack)/2.0-BrakePressure_Range_ActualLow_Float>0.1*(BrakePressure_Range_ActualHigh_Float-BrakePressure_Range_ActualLow_Float)){
			CarTaillightFlag=1;																											 //踩下刹车，制动灯亮			
		}
		else{			
			CarTaillightFlag=0;																											 //松开刹车，制动灯灭			
		}
		if(CarTaillightCtrl!=CarTaillightFlag){
			CarTaillightCtrl=CarTaillightFlag;				
		}
		
		Set_Fan_Speaker_Taillight_Pump_CtrlData(TaillightCtrl_n,&CarTaillightCtrl);
		CAN2_DATA_Send(ECU_ECU2_CONTROL_ID);
		osDelay(100);
  }
/* USER CODE END LightsUpAfterBrakeTask */
}

/* E_ABSTask function */
void E_ABSTask(void const * argument)
{	
  /* USER CODE BEGIN E_ABSTask */
  /* Infinite loop */	
	uint16_t CarAccPedalLeft,CarAccPedalRight,AccPeda_Range_High,AccPeda_Range_Low;	
	uint8_t MotorState_Data=0,Throttle_Brake_InterferingState_Data;
	for(;;)
  { 	
		Get_Acc_Brake_Angle_LowVData(AccPedalLeft_n,&CarAccPedalLeft);
		Get_Acc_Brake_Angle_LowVData(AccPedalRight_n,&CarAccPedalRight);	
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_High_n,&AccPeda_Range_High);
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_Low_n,&AccPeda_Range_Low);	
		if((CarAccPedalLeft+CarAccPedalRight)/2.0-AccPeda_Range_Low<0.05*(AccPeda_Range_High-AccPeda_Range_Low)){
			CAN1_DATA_Send(ModeState_Reg,ENABLEMOT);
			MotorState_Data=1;
			Set_MotorState_Data(MotorState_n,&MotorState_Data);													//设置电机状态变量
			Throttle_Brake_InterferingState_Data=0;
			Set_Throttle_Brake_Interferingtate_Data(Throttle_Brake_InterferingState_n,&Throttle_Brake_InterferingState_Data);
			vTaskDelete(NULL);
		}
		osDelay(20);
  }
/* USER CODE END E_ABSTask */
}

/* MotController_CoolTask function */
void MotController_CoolTask(void const * argument)
{	
  /* USER CODE BEGIN MotController_CoolTask */
  /* Infinite loop */	
	float Car_Mot_IGBT_Temperature,Car_Mot_Air_Temperature;
	uint8_t CarWaterPumpCtrl;
	uint16_t MotCtrllerTempThreshold_High,MotCtrllerTempThreshold_Low;
	for(;;)
  { 
		Get_CAN_Analysis_Data(IGBT_Temperature_n,&Car_Mot_IGBT_Temperature);
		Get_CAN_Analysis_Data(Air_Temperature_n,&Car_Mot_Air_Temperature);	
		Get_Wireless_DataFrom_EEPROM(MotCtrllerTempThreshold_High_n,&MotCtrllerTempThreshold_High);		
		Get_Wireless_DataFrom_EEPROM(MotCtrllerTempThreshold_Low_n,&MotCtrllerTempThreshold_Low);		
		if(Car_Mot_IGBT_Temperature>MotCtrllerTempThreshold_High||Car_Mot_Air_Temperature>MotCtrllerTempThreshold_High){
			CarWaterPumpCtrl=1;
			Set_Fan_Speaker_Taillight_Pump_CtrlData(WaterpumpCtrl_n,&CarWaterPumpCtrl);//电机控制器温度过高，开启水泵降温
			CAN2_DATA_Send(ECU_ECU2_CONTROL_ID);	
		}
		else if(Car_Mot_IGBT_Temperature<MotCtrllerTempThreshold_Low&&Car_Mot_Air_Temperature<MotCtrllerTempThreshold_Low){
			CarWaterPumpCtrl=0;
			Set_Fan_Speaker_Taillight_Pump_CtrlData(WaterpumpCtrl_n,&CarWaterPumpCtrl);//电机控制器温度已降低到正常区间，关闭水泵
			CAN2_DATA_Send(ECU_ECU2_CONTROL_ID);	
		}
		osDelay(100);					//接收控制器温度和IGBT温度信息的周期为100ms，此处大于100ms
  }
/* USER CODE END MotController_CoolTask */
}

/* TractionControlModeJudgmentTask function */
void TractionControlModeJudgmentTask(void const * argument)
{	
  /* USER CODE BEGIN TractionControlModeJudgmentTask */
  /* Infinite loop */	
	uint8_t LineTCSMode=0,LineTCSFlag=0,CornersTCSMode=0,CornersTCSFlag=0;
	float SteeringWheelAngleTCS=0;
	for(;;)
  {
		Get_Acc_Brake_Angle_LowVData(SteeringWheelAngle_n,&SteeringWheelAngleTCS);
		Get_CAN_Analysis_Data(Corners_TCSToggleSwitch_n,&CornersTCSMode);
		Get_CAN_Analysis_Data(Line_TCSToggleSwitch_n,&LineTCSMode);		
		if(LineTCSMode==1&&CornersTCSMode==0){
			if(LineTCSFlag==0){
				LineTCSFlag=1;																			//进入直线牵引力控制模式,删除弯道牵引力控制任务			
				/* definition and creation of Line_TractionControlTask */
				osThreadDef(Line_TractionControlTask, Line_TractionControlTask, osPriorityNormal, 0, 128);
				Line_TractionControlTaskHandle = osThreadCreate(osThread(Line_TractionControlTask), NULL);																															
			}
		}
		else if(LineTCSMode==0&&CornersTCSMode==1){																																												
			if(LineTCSFlag==1){																		//进入弯道牵引力控制模式
				vTaskDelete(Line_TractionControlTaskHandle);				//删除直线牵引力控制任务
				LineTCSFlag=0;
			}
		}
		else{																										//如果不小心两个模式同时开启，则都不生效,(或者根据方向盘转角确定)
			if(LineTCSFlag==1){			
				vTaskDelete(Line_TractionControlTaskHandle);				//删除直线牵引力控制任务
				LineTCSFlag=0;
			}																											//删除弯道牵引力控制任务																																	
		}
		Set_Line_CornersTCSState_Data(LineTCSState_n,&LineTCSFlag);
		Set_Line_CornersTCSState_Data(CornersTCSState_n,&CornersTCSFlag);
		osDelay(100);
  }
  /* USER CODE END TractionControlModeJudgmentTask */
}

/* Line_TractionControlTask function */
void Line_TractionControlTask(void const * argument)
{	
  /* USER CODE BEGIN Line_TractionControlTask */
  /* Infinite loop */	
	arm_pid_instance_f32 TCS_PID_Instance;
	TCS_PID_Instance.state[0]=0;
	TCS_PID_Instance.state[1]=0;
	TCS_PID_Instance.state[2]=0;
	Get_Wireless_DataFrom_EEPROM(TCS_PID_Kp_n,&TCS_PID_Instance.Kp);		
	Get_Wireless_DataFrom_EEPROM(TCS_PID_Ki_n,&TCS_PID_Instance.Ki);		
	Get_Wireless_DataFrom_EEPROM(TCS_PID_Kd_n,&TCS_PID_Instance.Kd);
	arm_pid_init_f32(&TCS_PID_Instance,1);
	float TCS_pid_EK;
	float CurrentSlipRate,OptimalSlipRate;
	float TCS_Pid_Out_View=0;
	Get_Wireless_DataFrom_EEPROM(OptimalSlipRate_n,&OptimalSlipRate);
	for(;;)
  {				
		Get_CarSpeed_Mileage_SlipRate_Data(SlipRate_n,&CurrentSlipRate);
		if(CurrentSlipRate>0){
			TCS_pid_EK=CurrentSlipRate-OptimalSlipRate;
			TCS_pid_Out=arm_pid_f32(&TCS_PID_Instance,TCS_pid_EK);
			if(TCS_pid_Out>0){
				if(TCS_pid_Out>TCS_Adjust_Ability){																	//实验观察TCS_pid_Out的范围之后取定，暂取50
					TCS_pid_Out=TCS_Adjust_Ability;
				}
			}
			else if(TCS_pid_Out<-TCS_Adjust_Ability){															//实验观察TCS_pid_Out的范围之后取定，暂取50
					TCS_pid_Out=-TCS_Adjust_Ability;
			}		
		}
		else{
			TCS_pid_Out=0;																				//减速过程不做牵引力控制
		}
		TCS_Pid_Out_View=TCS_pid_Out;
		Set_TCS_PID_OUT_Data(TCS_PID_OUT_n,&TCS_Pid_Out_View);
		osDelay(20);
  }
  /* USER CODE END Line_TractionControlTask */
}

/* RollStabilizerBarSteeringEngineTask function */
void RollStabilizerBarSteeringEngineTask(void const * argument)
{	
  /* USER CODE BEGIN RollStabilizerBarSteeringEngineTask */
  /* Infinite loop */	
	static float StabilizerBar_SteeringEngine_Angle1_1;
	float StabilizerBar_SteeringEngine_Angle1_2;
	Get_CAN_Analysis_Data(StabilizerBar_Angle1_n,&StabilizerBar_SteeringEngine_Angle1_1);	
	AdjustAngle_SteeringEngine(steering_engine1_pwm,StabilizerBar_SteeringEngine_Angle1_1);
	AdjustAngle_SteeringEngine(steering_engine2_pwm,StabilizerBar_SteeringEngine_Angle1_1);
	for(;;)
  {						
		Get_CAN_Analysis_Data(StabilizerBar_Angle1_n,&StabilizerBar_SteeringEngine_Angle1_2);		
		if(fabs(StabilizerBar_SteeringEngine_Angle1_2-StabilizerBar_SteeringEngine_Angle1_1)>1){
			AdjustAngle_SteeringEngine(steering_engine1_pwm,StabilizerBar_SteeringEngine_Angle1_2);
			AdjustAngle_SteeringEngine(steering_engine2_pwm,StabilizerBar_SteeringEngine_Angle1_2);
			StabilizerBar_SteeringEngine_Angle1_1=StabilizerBar_SteeringEngine_Angle1_2;
		}
		osDelay(100);						//延时时间根据面板旋钮发数据的周期而定，暂定100ms
  }
  /* USER CODE END RollStabilizerBarSteeringEngineTask */
}

/* DRSSteeringEngineTask function */
void DRSSteeringEngineTask(void const * argument)
{	
  /* USER CODE BEGIN DRSSteeringEngineTask */
  /* Infinite loop */	
	float SteeringWheel_Angle_ForDRS;	
	uint8_t DRSSwitch=0,CarDRSState=0,CarBrake_State=0;
	uint8_t CarSpeed_Data=0;
	float DRS_SteeringWheel_Angle,Close_DRS_AdjustAngle,Open_DRS_AdjustAngle;
	float CarBrakePressureFront,CarBrakePressureBack,BrakePressure_Range_ActualHigh_Float,BrakePressure_Range_ActualLow_Float;	
	for(;;)
  {	
		Get_Acc_Brake_Angle_LowVData(SteeringWheelAngle_n,&SteeringWheel_Angle_ForDRS);
		Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&CarBrakePressureFront);
		Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&CarBrakePressureBack);
		Get_CarSpeed_Mileage_SlipRate_Data(CarSpeed_n,&CarSpeed_Data);
		Get_CAN_Analysis_Data(DRSSwitch2_n,&DRSSwitch);
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualHigh_n,&BrakePressure_Range_ActualHigh_Float);	
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_ActualLow_n,&BrakePressure_Range_ActualLow_Float);	
		Get_Wireless_DataFrom_EEPROM(DRS_SteeringWheel_Angle_n,&DRS_SteeringWheel_Angle);			
		Get_Wireless_DataFrom_EEPROM(Close_DRS_AdjustAngle_n,&Close_DRS_AdjustAngle);			
		Get_Wireless_DataFrom_EEPROM(Open_DRS_AdjustAngle_n,&Open_DRS_AdjustAngle);			
		if((DRSSwitch==1)&&(CarSpeed_Data>=50)){																																//如果DRS打开			
			CarBrake_State=(CarBrakePressureFront+CarBrakePressureBack)/2.0-BrakePressure_Range_ActualLow_Float>0.3*(BrakePressure_Range_ActualHigh_Float-BrakePressure_Range_ActualLow_Float);
			if(CarBrake_State||SteeringWheel_Angle_ForDRS>DRS_SteeringWheel_Angle||SteeringWheel_Angle_ForDRS<-DRS_SteeringWheel_Angle){
				Set_DRS_Data(DRS_Angle1_n,&Open_DRS_AdjustAngle);	
				Set_DRS_Data(DRS_Angle2_n,&Open_DRS_AdjustAngle);		
				CarDRSState=1;												
			}
			else{
				Set_DRS_Data(DRS_Angle1_n,&Close_DRS_AdjustAngle);	
				Set_DRS_Data(DRS_Angle2_n,&Close_DRS_AdjustAngle);		
				CarDRSState=0;
			}
			Set_DRSState_Data(DRSState_n,&CarDRSState);		
			CAN2_DATA_Send(ECU_ECU2_SteeringEngine_ID);
		}
		else{
			Set_DRS_Data(DRS_Angle1_n,&Open_DRS_AdjustAngle);	
			Set_DRS_Data(DRS_Angle2_n,&Open_DRS_AdjustAngle);		
			CarDRSState=0;
			Set_DRSState_Data(DRSState_n,&CarDRSState);		
			CAN2_DATA_Send(ECU_ECU2_SteeringEngine_ID);
		}
		osDelay(100);						
  }
  /* USER CODE END DRSSteeringEngineTask */
}

/* SuspensionLineShift_ZeroSettingTask function */
void SuspensionLineShift_ZeroSettingTask(void const * argument)
{	
  /* USER CODE BEGIN SuspensionLineShift_ZeroSettingTask */
  /* Infinite loop */	
	uint8_t SuspensionLineShift_ZeroSetting_Flag=0;
	uint16_t SuspensionLineShiftF1_ADMiddle_Data=0,SuspensionLineShiftF2_ADMiddle_Data=0,SuspensionLineShiftB1_ADMiddle_Data=0,SuspensionLineShiftB2_ADMiddle_Data=0;
	for(;;)
  {				
//		Get_CAN_Analysis_Data(SuspensionLineShift_ZeroSetting_n,&SuspensionLineShift_ZeroSetting_Flag);
//		if(SuspensionLineShift_ZeroSetting_Flag==1){
//			Get_SuspensionLineDisplacementData(SuspensionLineDistF1_ADValue_n,&SuspensionLineShiftF1_ADMiddle_Data);
//			Get_SuspensionLineDisplacementData(SuspensionLineDistF2_ADValue_n,&SuspensionLineShiftF2_ADMiddle_Data);
//			Get_SuspensionLineDisplacementData(SuspensionLineDistB1_ADValue_n,&SuspensionLineShiftB1_ADMiddle_Data);
//			Get_SuspensionLineDisplacementData(SuspensionLineDistB2_ADValue_n,&SuspensionLineShiftB2_ADMiddle_Data);
//			AT24Cxx_WriteOneByte(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID,(uint8_t)(SuspensionLineShiftF1_ADMiddle_Data>>8));
//			AT24Cxx_WriteOneByte(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID+1,(uint8_t)(SuspensionLineShiftF1_ADMiddle_Data&0x00FF));
//			AT24Cxx_WriteOneByte(SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_ID,(uint8_t)(SuspensionLineShiftF2_ADMiddle_Data>>8));
//			AT24Cxx_WriteOneByte(SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_ID+1,(uint8_t)(SuspensionLineShiftF2_ADMiddle_Data&0x00FF));
//			AT24Cxx_WriteOneByte(SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_ID,(uint8_t)(SuspensionLineShiftB1_ADMiddle_Data>>8));
//			AT24Cxx_WriteOneByte(SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_ID+1,(uint8_t)(SuspensionLineShiftB1_ADMiddle_Data&0x00FF));
//			AT24Cxx_WriteOneByte(SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_ID,(uint8_t)(SuspensionLineShiftB2_ADMiddle_Data>>8));
//			AT24Cxx_WriteOneByte(SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_ID+1,(uint8_t)(SuspensionLineShiftB2_ADMiddle_Data&0x00FF));
//			Set_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftF1_ADMiddle_Data);
//			Set_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftF2_ADMiddle_Data);
//			Set_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftB1_ADMiddle_Data);
//			Set_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftB2_ADMiddle_Data);
//			SuspensionLineShift_ZeroSetting_Flag=0;
//			Set_CAN_Analysis_Data(SuspensionLineShift_ZeroSetting_n,&SuspensionLineShift_ZeroSetting_Flag);
//		}	
    osDelay(100);
  }
  /* USER CODE END SuspensionLineShift_ZeroSettingTask */
}

/* Mileage_SaveTask function */
void Mileage_SaveTask(void const * argument)
{	
  /* USER CODE BEGIN Mileage_SaveTask */
  /* Infinite loop */	
	float Mileage_Data_Float;
	uint16_t Mileage_Data_u16;
	for(;;)
  {				
		//周期存储里程数据
		Get_CarSpeed_Mileage_SlipRate_Data(Mileage_n,&Mileage_Data_Float);
		Mileage_Data_u16=Mileage_Data_Float*10;
		AT24Cxx_WriteOneByte(Mileage_ID,(uint8_t)(Mileage_Data_u16>>8));
		AT24Cxx_WriteOneByte(Mileage_ID+1,(uint8_t)Mileage_Data_u16);
    osDelay(1000);
  }
  /* USER CODE END Mileage_SaveTask */
}

/* Mileage_ClearTask function */
void Mileage_ClearTask(void const * argument)
{	
  /* USER CODE BEGIN Mileage_ClearTask */
  /* Infinite loop */	
	uint8_t Mileage_Clear_Flag;
	float Mileage_Data_Float=0;
	for(;;)
  {				
		Get_CAN_Analysis_Data(Mileage_CLEAR_n,&Mileage_Clear_Flag);
		if(Mileage_Clear_Flag==1){
			Set_CarSpeed_Mileage_SlipRate_Data(Mileage_n,&Mileage_Data_Float);
			Mileage_Clear_Flag=0;
			Set_CAN_Analysis_Data(Mileage_CLEAR_n,&Mileage_Clear_Flag);
		}		
    osDelay(100);
  }
  /* USER CODE END Mileage_ClearTask */
}

/* MotIGBTAndAir_DataGuardTask function */
void MotIGBTAndAir_DataGuardTask(void const * argument)
{	
  /* USER CODE BEGIN LEDTask */
  /* Infinite loop */
	float IGBT_Temperature_Guard,Air_Temperature_Guard;
	uint16_t Mot_Temperature_Data=0;
	for(;;)
  {			
		Get_CAN_Analysis_Data(IGBT_Temperature_n,&IGBT_Temperature_Guard);
		Get_CAN_Analysis_Data(Air_Temperature_n,&Air_Temperature_Guard);
		Get_CAN_Analysis_Data(Mot_Temperature_n,&Mot_Temperature_Data);
		if(IGBT_Temperature_Guard<5.0||Air_Temperature_Guard<5.0)
		{
			CSU_CAN1_Receive_Mot_Ready(IGBTTemperature_Reg,0x64);					//请求100ms发送一次IGBT温度
			CSU_CAN1_Receive_Mot_Ready(AirTemperature_Reg,0x64);					//请求100ms发送一次控制器温度
		}
		if(Mot_Temperature_Data==0){
			CSU_CAN1_Receive_Mot_Ready(MotTemperature_Reg,0x64);					//请求100ms发送一次控制器温度
		}
    osDelay(100);
  }
  /* USER CODE END MotIGBTAndAir_DataGuardTask */
}

/* LeakageCurrentLimitProcessTask function */
void LeakageCurrentLimitProcessTask(void const * argument)
{	
  /* USER CODE BEGIN LeakageCurrentLimitProcessTask */
  /* Infinite loop */
	uint8_t LeakageCurrentLimit_State_Data=0,IMDOpenOrClose_Data=0,IMD_BrakeReliability_Trigger_Data=0,AMS_IMD_Reset_Input_Data=0,Clear_AMS_IMD_Reset_Input_Data=0;
	static uint8_t Check_Switch=0;
	for(;;)
  {			
		if(Check_Switch==0){
			Get_CAN_Analysis_Data(LeakageCurrentLimit_n,&LeakageCurrentLimit_State_Data);
			Get_CAN_Analysis_Data(AMS_IMD_Reset_Input_n,&AMS_IMD_Reset_Input_Data);		
			if(AMS_IMD_Reset_Input_Data){
				LeakageCurrentLimit_State_Data=0;
				Set_CAN_Analysis_Data(AMS_IMD_Reset_Input_n,&Clear_AMS_IMD_Reset_Input_Data);						
				AT24Cxx_WriteOneByte(LeakageCurrentLimit_State_Data_ID,LeakageCurrentLimit_State_Data);	//清除EEPROM中IMD故障状态
				IMD_BrakeReliability_Trigger_Data=0;																										//闭合BMS电源继电器
				Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);
				osDelay(2000);
				IMDOpenOrClose_Data=0;
				Set_IMDOpenOrClose_Data(IMDOpenOrClose_n,&IMDOpenOrClose_Data);													//关闭IMD指示灯
			}	
			else if(LeakageCurrentLimit_State_Data==1){																								//漏电超限一级告警
				IMDOpenOrClose_Data=1;
				Set_IMDOpenOrClose_Data(IMDOpenOrClose_n,&IMDOpenOrClose_Data);													//开启IMD指示灯
				AT24Cxx_WriteOneByte(LeakageCurrentLimit_State_Data_ID,LeakageCurrentLimit_State_Data);	//存储IMD故障状态到EEPROM
				IMD_BrakeReliability_Trigger_Data=1;																										//断开ECU2中安全回路继电器
				Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);
			}			
		}
    osDelay(100);
  }
  /* USER CODE END LeakageCurrentLimitProcessTask */
}

/* CAN1_OffLine_ProcessTask function */
void CAN1_OffLine_ProcessTask(void const * argument)
{	
  /* USER CODE BEGIN CAN1_OffLine_ProcessTask */
  /* Infinite loop */
	uint8_t LeakageCurrentLimit_State_Data=0,IMDOpenOrClose_Data=0,IMD_BrakeReliability_Trigger_Data=0,AMS_IMD_Reset_Input_Data=0,Clear_AMS_IMD_Reset_Input_Data=0;
	static uint8_t Check_Switch=0,Can1_OffLine_Flag=0;
	uint16_t Mot_Temperature_Data=1,Clear_Mot_Temperature_Data=0;
	Set_CAN_Analysis_Data(Mot_Temperature_n,&Mot_Temperature_Data);	
	osDelay(1000);
	for(;;)
  {			
		if(Check_Switch==0){			
			Get_CAN_Analysis_Data(AMS_IMD_Reset_Input_n,&AMS_IMD_Reset_Input_Data);	
			Get_CAN_Analysis_Data(Mot_Temperature_n,&Mot_Temperature_Data);	
			Set_CAN_Analysis_Data(Mot_Temperature_n,&Clear_Mot_Temperature_Data);			
			if(Mot_Temperature_Data==0&&Can1_OffLine_Flag==0){								
				IMD_BrakeReliability_Trigger_Data=1;																										//断开ECU2中安全回路继电器
				Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);	
				Can1_OffLine_Flag=1;
			}	
			else if((Mot_Temperature_Data!=0)&&(Can1_OffLine_Flag==1)){
				IMD_BrakeReliability_Trigger_Data=0;																										//闭合ECU2中安全回路继电器
				Set_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_Trigger_Data);		
			}			
		}
    osDelay(1000);
  }
  /* USER CODE END CAN1_OffLine_ProcessTask */
}

///* PowerBroken_ClearHighVolValueTask function */
//void PowerBroken_ClearHighVolValueTask(void const * argument)
//{	
//  /* USER CODE BEGIN PowerBroken_ClearHighVolValueTask */
//  /* Infinite loop */		
//	uint8_t PositiveRelayValue=0;
//	float BatteryTotal_V_RESET=0;
//	for(;;)
//  {				
//		Get_CAN_Analysis_Data(PositiveRelayState_n,&PositiveRelayValue);
//		if(PositiveRelayValue==0x00){
//			Set_CAN_Analysis_Data(BatteryTotal_V_n,&BatteryTotal_V_RESET);
//		}
//    osDelay(100);
//  }
//  /* USER CODE END PowerBroken_ClearHighVolValueTask */
//}

/* LEDTask function */
void LEDTask(void const * argument)
{	
  /* USER CODE BEGIN LEDTask */
  /* Infinite loop */	
//	Start=uxTaskGetStackHighWaterMark(NULL);	
	for(;;)
  {				
		FreeMem=xPortGetFreeHeapSize();													//查看堆栈剩余内存空间
		HAL_GPIO_TogglePin(ECU1_LED1_GPIO_Port, ECU1_LED1_Pin);		
    osDelay(200);
//		End=uxTaskGetStackHighWaterMark(NULL);
  }
  /* USER CODE END LEDTask */
}

/* GPSTask function */
void GPSTask(void const * argument)
{	
  /* USER CODE BEGIN GPSTask */
  /* Infinite loop */
	if (Get_GPS_NEO_M8N_Processed_Data(&huart6,&GPSX)){
		/*发送GPS故障信息给显示屏*/			
	}
  for(;;)
  {				
    osDelay(2);
  }
  /* USER CODE END GPSTask */
}

/* Fatfs_SDCardTask function */
void Fatfs_SDCardTask(void const * argument)
{	
  /* USER CODE BEGIN Fatfs_SDCardTask */
	MX_FATFS_Init();
//	res=f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0);
////	res=f_mkfs((TCHAR const*)SD_Path, 0, 0);								//格式化SD卡	
//	res=f_open(&MyFile, "CSUFSAE.txt", FA_OPEN_ALWAYS | FA_WRITE);
//	res=f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
//	res=f_close(&MyFile);
  /* Infinite loop */	
  for(;;)
  {				
    osDelay(100);
  }
  /* USER CODE END Fatfs_SDCardTask */
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

