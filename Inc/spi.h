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
	AccPedalLeft_n=200,									//�����̤������ֵ 
	AccPedalRight_n,										//�Ҽ���̤������ֵ 
	SteeringWheelAngle_n,								//ת��Ƕ�
	SteeringWheelAngleAdDigitalValue_n,	//ת��Ƕ�ADֵ
	BrakePressureFront_n,								//ɲ����ѹǰ
	BrakePressureBack_n,								//ɲ����ѹ��
	LowVoltageBatV_n,										//��ѹ��ص�ѹADֵ
	LowVoltageBatState_n,								//��ѹ���״̬
  SuspensionLineDistF1_ADValue_n,			//ǰ������λ��1ADֵ
	SuspensionLineDistF2_ADValue_n,			//ǰ������λ��2ADֵ
	SuspensionLineDistB1_ADValue_n,			//��������λ��1ADֵ
	SuspensionLineDistB2_ADValue_n,			//��������λ��2ADֵ
	SuspensionLineDisplacementF1_n,			//ǰ������λ��1
	SuspensionLineDisplacementF2_n,			//ǰ������λ��2
	SuspensionLineDisplacementB1_n,			//��������λ��1
	SuspensionLineDisplacementB2_n,			//��������λ��2
}AD7689GenericData_n; 
/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
//TimerHandle_t Get_ADValueGenericTimer_Handle(void);
void Get_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data);						//ȡֵ�������������ţ���ѹ����ѹ��ص�ѹ��ת�ǵ�����@AD7689GenericData_n
void Get_SuspensionLineDisplacementData(AD7689GenericData_n name_num,void * extern_data);			//ȡֵ����������4��������λ�Ƶ�����@AD7689GenericData_n
void Set_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data);						//�޸ĺ������޸����ţ���ѹ����ѹ��ص�ѹ��ת�ǵ�����@AD7689GenericData_n
void Set_SuspensionLineDisplacementData(AD7689GenericData_n name_num,void * extern_data);			//�޸ĺ������޸�4��������λ�Ƶ�����@AD7689GenericData_n
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
