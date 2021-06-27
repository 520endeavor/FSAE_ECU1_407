/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
	 


/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
	 
#include "gpio.h"
#include "HostPCDebug_AT24CxxMem.h"

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */
typedef enum{
	AccPedalLeft_n=200,									//左加速踏板油门值 
	AccPedalRight_n,										//右加速踏板油门值 
	SteeringWheelAngle_n,								//转向角度
	SteeringWheelAngleAdDigitalValue_n,	//转向角度AD值
	BrakePressureFront_n,								//刹车油压前
	BrakePressureBack_n,								//刹车油压后
	LowVoltageBatV_n,										//低压电池电压AD值
	LowVoltageBatState_n,								//低压电池状态
  SuspensionLineDistF1_ADValue_n,			//前悬架线位移1AD值
	SuspensionLineDistF2_ADValue_n,			//前悬架线位移2AD值
	SuspensionLineDistB1_ADValue_n,			//后悬架线位移1AD值
	SuspensionLineDistB2_ADValue_n,			//后悬架线位移2AD值
	SuspensionLineDisplacementF1_n,			//前悬架线位移1
	SuspensionLineDisplacementF2_n,			//前悬架线位移2
	SuspensionLineDisplacementB1_n,			//后悬架线位移1
	SuspensionLineDisplacementB2_n,			//后悬架线位移2
}AD7689GenericData_n; 
/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
//TimerHandle_t Get_ADValueGenericTimer_Handle(void);
void Get_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data);						//取值函数，搬运油门，油压，电压电池电压，转角等数据@AD7689GenericData_n
void Get_SuspensionLineDisplacementData(AD7689GenericData_n name_num,void * extern_data);			//取值函数，搬运4个悬架线位移等数据@AD7689GenericData_n
void Set_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data);						//修改函数，修改油门，油压，电压电池电压，转角等数据@AD7689GenericData_n
void Set_SuspensionLineDisplacementData(AD7689GenericData_n name_num,void * extern_data);			//修改函数，修改4个悬架线位移等数据@AD7689GenericData_n
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
