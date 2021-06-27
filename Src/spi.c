/**
  ******************************************************************************
  * File Name          : SPI.c
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
struct {
	uint16_t AccPedalLeft;
	uint16_t AccPedalRight;
	float SteeringWheelAngle;
	uint16_t SteeringWheelAngleAdDigitalValue;
	float BrakePressureFront;
	float BrakePressureBack;	
	float LowVoltageBatV;
	uint8_t LowVoltageBatState;	
}Acc_Brake_Angle_LowVData;
struct {
	uint16_t SuspensionLineDistF1_ADValue;
	uint16_t SuspensionLineDistF2_ADValue;
	uint16_t SuspensionLineDistB1_ADValue;
	uint16_t SuspensionLineDistB2_ADValue;
	float SuspensionLineDisplacementF1;
	float SuspensionLineDisplacementF2;
	float SuspensionLineDisplacementB1;
	float SuspensionLineDisplacementB2;
}SuspensionLineDisplacementData;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

osThreadId ADValueGenericTaskHandle;
//TimerHandle_t ADValueGenericTimer_Handle;					//软件定时器句柄
//TimerHandle_t Get_ADValueGenericTimer_Handle(void){
//return ADValueGenericTimer_Handle;
//}
//void ADValueGenericCallback(TimerHandle_t xTimer);//定时器回调函数
void ADValueGenericTask(void const * argument);
static void AD_FREERTOS_Init(void) 
{
	/*definition and creation of AutoReloadTimer */ 				//周期定时器，周期1ms(1个时钟节拍)，周期模式 //ID:1		      
//	ADValueGenericTimer_Handle=xTimerCreate((const char*)"ADValueGenericTimer",
//																			(TickType_t)1,
//																			(UBaseType_t)pdTRUE,
//																			(void*)1,
//																			(TimerCallbackFunction_t)ADValueGenericCallback);     
	/* definition and creation of ADValueGenericTask */
	osThreadDef(ADValueGenericTask, ADValueGenericTask, osPriorityNormal, 0, 128);
  ADValueGenericTaskHandle = osThreadCreate(osThread(ADValueGenericTask), NULL);	
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
	AD_FREERTOS_Init();
}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = ECU1_SPI1_CLK_Pin|ECU1_SPI1_MISO_Pin|ECU1_SPI1_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = ECU1_SPI2_CLK_Pin|ECU1_SPI2_MISO_Pin|ECU1_SPI2_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, ECU1_SPI1_CLK_Pin|ECU1_SPI1_MISO_Pin|ECU1_SPI1_MOSI_Pin);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, ECU1_SPI2_CLK_Pin|ECU1_SPI2_MISO_Pin|ECU1_SPI2_MOSI_Pin);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/*
*****************************************************************************
*@brief		纳秒延时函数
*@param		uint32_t t，延时大小，步长6纳秒
*@retval	None
*@par
*****************************************************************************
*/
static void AD7689_Delay_6ns(uint32_t t){
	while(t--);
}

/*
*****************************************************************************
*@brief		微秒延时函数
*@param		uint32_t cnt，延时大小，单位微秒
*@retval	None
*@par
*****************************************************************************
*/
static void AD7689_Delay_us(uint32_t cnt){
	uint32_t i,j;  
	for(i=0;i<cnt;i++)  
	{  
			for(j=0;j<35;j++);  
	}
}

/*
*****************************************************************************
*@brief		两个AD7618读取函数，注意SPI1读取的是AD2的数据，即通道AD8-AD15
					SPI2读取的是AD1的数据，即通道AD0-AD7
*@param		SPI_HandleTypeDef *spiHandle：SPI结构体句柄
*@param		float *data：存储AD模拟量数组首地址
*@param		uint8_t AD1_AD2_PS0: 0:AD0~AD7,1:AD8~AD15
*@retval	None
*@par
*****************************************************************************
*/
static void Get_AD7689Data(SPI_HandleTypeDef *spiHandle,uint16_t *data,uint8_t AD1_AD2_PS0){
	uint16_t CFG=0x3C49;
	uint8_t txData[2],rxData[2];
	uint8_t i=0,b=0;
	uint16_t a;	
	CFG=(CFG&0x3C7F)|(i<<7);
	txData[0]=(CFG<<2)>>8;											//内部基准源,REF=4.096,禁止序列器
	txData[1]=(CFG<<2)&0x00FF;		
	if(AD1_AD2_PS0==0){										//
		/*读取8个通道数据,其中两次是哑读*/	
		for (i=0;i<10;i++){			
			if(i==8){
				i=0;
				b=1;
			}		
			if(i==9){
				i=1;
				b=2;
			}		
			CFG=(CFG&0x3C7F)|(i<<7);
			txData[0]=(CFG<<2)>>8;										//内部基准源,REF=4.096,禁止序列器
			txData[1]=(CFG<<2)&0x00FF;		
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&rxData[0],1,100);//配置字，输入通道IN   i+2		
			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &rxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);											//过程中发生EOC
			a=(rxData[0]<<8)|rxData[1];							//接收的数据
			if(b==1){
				i=8;
			}
			if(b==2){
				i=9;
			}
			if(i>=2){
				data[i-2]=a;						//REF=4.096
			}
		}			
	 }	
	else if(AD1_AD2_PS0==1){										//使用SPI1读取AD2的数据
		/*读取8个通道数据,其中两次是哑读*/	
		for (i=0;i<10;i++){			
			if(i==8){
				i=0;
				b=1;
			}		
			if(i==9){
				i=1;
				b=2;
			}		
			CFG=(CFG&0x3C7F)|(i<<7);
			txData[0]=(CFG<<2)>>8;										//内部基准源,REF=4.096,禁止序列器
			txData[1]=(CFG<<2)&0x00FF;		
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&rxData[0],1,100);//配置字，输入通道IN   i+2		
			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &rxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);											//过程中发生EOC
			a=(rxData[0]<<8)|rxData[1];							//接收的数据
			if(b==1){
				i=8;
			}
			if(b==2){
				i=9;
			}
			if(i>=2){
				data[i-2]=a;						//REF=4.096
			}
		}			
	 }	
}
/*
*****************************************************************************
*@brief		AD7689采集AD数字量滤波函数
*@param		uint16_t *Ad7689_OriginalData：原始AD7689采集量数组首地址
*@param		uint16_t *Ad7689_FilterData：AD7689采集量经移动平均算法滤波后的数值 数组首地址
*@retval	none
*@par
*****************************************************************************
*/
static void Ad7689DataFilter(uint16_t *Ad7689_OriginalData,uint16_t *Ad7689_FilterData)
{
	static uint8_t Filter_Counter=0;
	static uint16_t Ad7689_TempData[5][16]={0};
	if(Filter_Counter<4){
		for(uint8_t i=0;i<16;i++)
		{
			Ad7689_TempData[Filter_Counter][i]=Ad7689_OriginalData[i];
			Ad7689_FilterData[i]=Ad7689_OriginalData[i];
		}
		Filter_Counter++;		
	}
	else {
		for(uint8_t i=0;i<16;i++)
		{
			Ad7689_TempData[Filter_Counter][i]=Ad7689_OriginalData[i];
			for(uint8_t ii=0;ii<5;ii++)
			{
				Ad7689_FilterData[i]+=Ad7689_TempData[ii][i]/5;
			}	
			for(uint8_t ii=0;ii<4;ii++)
			{
				Ad7689_TempData[ii][i]=Ad7689_TempData[ii+1][i];
			}	
		}		
	}
}

uint16_t Ad0Data[8]={0};
uint16_t Ad1Data[8]={0};
uint16_t Ad7689_OriginalData[16]={0},Ad7689_FilterData[16]={0};

/* ADValueGenericTask function */
void ADValueGenericTask(void const * argument)
{	
  /* USER CODE BEGIN ADValueGenericTask */
  /* Infinite loop */	
	float BrakePressure_Range_High_Float,BrakePressure_Range_Low_Float;
	uint16_t AccPeda_InitialDiff_Of_LandR,SterringWheel_ADMiddle,SterringWheel_TurnRange,SterringWheelAngle_Range;
	uint16_t SuspensionLineShiftF1_ADMiddle_Data,SuspensionLineShiftF2_ADMiddle_Data,SuspensionLineShiftB1_ADMiddle_Data,SuspensionLineShiftB2_ADMiddle_Data;
	for(;;)
  { 
		Get_AD7689Data(&hspi1,Ad0Data,0);
		Get_AD7689Data(&hspi1,Ad1Data,1);
//		for(uint8_t j=0;j<16;j++)
//		{
//			if(j<8){
//				Ad7689_OriginalData[j]=Ad0Data[j];
//			}
//			else
//			{
//				Ad7689_OriginalData[j]=Ad1Data[j-8];
//			}
//		}		
//		Ad7689DataFilter(Ad7689_OriginalData,Ad7689_FilterData);
		
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_High_n,&BrakePressure_Range_High_Float);
		Get_Wireless_DataFrom_EEPROM(BrakePressure_Range_Low_n,&BrakePressure_Range_Low_Float);
		Get_Wireless_DataFrom_EEPROM(AccPeda_InitialDiff_Of_LandR_n,&AccPeda_InitialDiff_Of_LandR);
		Get_Wireless_DataFrom_EEPROM(SterringWheel_ADMiddle_n,&SterringWheel_ADMiddle);
		Get_Wireless_DataFrom_EEPROM(SterringWheel_TurnRange_n,&SterringWheel_TurnRange);
		Get_Wireless_DataFrom_EEPROM(SterringWheelAngle_Range_n,&SterringWheelAngle_Range);
		Get_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftF1_ADMiddle_Data);
		Get_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftF2_ADMiddle_Data);
		Get_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftB1_ADMiddle_Data);
		Get_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_n,&SuspensionLineShiftB2_ADMiddle_Data);
		if(Ad0Data[0]-AccPeda_InitialDiff_Of_LandR<=0){
			Acc_Brake_Angle_LowVData.AccPedalLeft=0;
		}
		else{
			Acc_Brake_Angle_LowVData.AccPedalLeft=Ad0Data[0]-AccPeda_InitialDiff_Of_LandR;//AdDataMean[0];	//左加速踏板油门值 
		}			
		Acc_Brake_Angle_LowVData.AccPedalRight=Ad0Data[1];																								//右加速踏板油门值 
		Acc_Brake_Angle_LowVData.BrakePressureBack=Ad0Data[2]/62400.0*(BrakePressure_Range_High_Float-BrakePressure_Range_Low_Float)+BrakePressure_Range_Low_Float;				//刹车压力后
		Acc_Brake_Angle_LowVData.BrakePressureFront=Ad0Data[3]/62400.0*(BrakePressure_Range_High_Float-BrakePressure_Range_Low_Float)+BrakePressure_Range_Low_Float;			//刹车压力前
		
		Acc_Brake_Angle_LowVData.SteeringWheelAngleAdDigitalValue=Ad0Data[4];													//方向盘转角AD值
		if((float)(Ad0Data[4]-SterringWheel_ADMiddle)/SterringWheel_TurnRange*SterringWheelAngle_Range>=135.0){
			Acc_Brake_Angle_LowVData.SteeringWheelAngle=135.0;
		}
		else if((float)(Ad0Data[4]-SterringWheel_ADMiddle)/SterringWheel_TurnRange*SterringWheelAngle_Range<=-135.0){
			Acc_Brake_Angle_LowVData.SteeringWheelAngle=-135.0;
		}
		else{
			Acc_Brake_Angle_LowVData.SteeringWheelAngle=(float)(Ad0Data[4]-SterringWheel_ADMiddle)/SterringWheel_TurnRange*SterringWheelAngle_Range;	//方向盘转角,暂定左右各转135°
		}		
		Acc_Brake_Angle_LowVData.LowVoltageBatV=Ad0Data[5]/62400.0*5*4;												//低压电池电压
		if(Acc_Brake_Angle_LowVData.LowVoltageBatV<4.3*4&&Acc_Brake_Angle_LowVData.LowVoltageBatV>3.4*4){
			Acc_Brake_Angle_LowVData.LowVoltageBatState=0;						//低压电池 电压正常
		}
		else if(Acc_Brake_Angle_LowVData.LowVoltageBatV<=3.4*4){
			Acc_Brake_Angle_LowVData.LowVoltageBatState=1;						//低压电池 电压过低
		}
		else if(Acc_Brake_Angle_LowVData.LowVoltageBatV>=4.3*4){
			Acc_Brake_Angle_LowVData.LowVoltageBatState=2;						//低压电池 电压过高
		}
		else{
			Acc_Brake_Angle_LowVData.LowVoltageBatState=3;						//低压电池 电压状态未定义
		}		
		SuspensionLineDisplacementData.SuspensionLineDistF1_ADValue=Ad0Data[7];												//前悬架线位移1AD值
		SuspensionLineDisplacementData.SuspensionLineDistF2_ADValue=Ad0Data[6];												//前悬架线位移2AD值
		
		SuspensionLineDisplacementData.SuspensionLineDisplacementF1=(Ad0Data[7]-SuspensionLineShiftF1_ADMiddle_Data)/52800.0*50;							//前悬架线位移1，范围-50~50mm，但是参考电压为4V
		SuspensionLineDisplacementData.SuspensionLineDisplacementF2=(Ad0Data[6]-SuspensionLineShiftF2_ADMiddle_Data)/52800.0*50;							//前悬架线位移2，范围-50~50mm
		
		SuspensionLineDisplacementData.SuspensionLineDistB1_ADValue=Ad1Data[0];												//后悬架线位移1AD值
		SuspensionLineDisplacementData.SuspensionLineDistB2_ADValue=Ad1Data[1];												//后悬架线位移2AD值
		
		SuspensionLineDisplacementData.SuspensionLineDisplacementB1=(Ad1Data[0]-SuspensionLineShiftB1_ADMiddle_Data)/52800.0*50;							//前悬架线位移1，范围-50~50mm，但是参考电压为4V
		SuspensionLineDisplacementData.SuspensionLineDisplacementB2=(Ad1Data[1]-SuspensionLineShiftB2_ADMiddle_Data)/52800.0*50;

		osDelay(2);		
  }
/* USER CODE END ADValueGenericTask */
}

//void ADValueGenericCallback(TimerHandle_t xTimer)
//{
//	uint16_t AdData[8];
//	Get_AD7689Data(&hspi1,AdData);
//	Acc_Brake_Angle_LowVData.AccPedalLeft=AdData[0];		//左加速踏板油门值 
//	Acc_Brake_Angle_LowVData.AccPedalRight=AdData[1];		//右加速踏板油门值 
//	Acc_Brake_Angle_LowVData.SteeringWheelAngle=(AdData[2]/65536.0*4.096-4.096/2)/(4.096/2)*170;	//方向盘转角,暂定左右各转170°
//	Acc_Brake_Angle_LowVData.BrakePressureFront=AdData[4]/65536.0*20;												//刹车压力前
//	Acc_Brake_Angle_LowVData.BrakePressureBack=AdData[5]/65536.0*20;												//刹车压力后
//	Acc_Brake_Angle_LowVData.LowVoltageBatV=AdData[6]/65536.0*4.096*4;											//低压电池电压
//	Acc_Brake_Angle_LowVData.LowVoltageBatState=0;						//低压电池状态暂时置0
//}

/*
*****************************************************************************
*@brief		取值函数，搬运油门，油压，电压电池电压，转角等数据
*@param		TimGenericData_n name_num：索取数据对应的枚举变量
*@param		const void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data)
{
	if(name_num==AccPedalLeft_n){
		*(uint16_t*)extern_data=Acc_Brake_Angle_LowVData.AccPedalLeft;
	} 	
	else if(name_num==AccPedalRight_n){
		*(uint16_t*)extern_data=Acc_Brake_Angle_LowVData.AccPedalRight;
	}
	else if(name_num==BrakePressureFront_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.BrakePressureFront;
	}
	else if(name_num==BrakePressureBack_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.BrakePressureBack;
	}
	else if(name_num==SteeringWheelAngle_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.SteeringWheelAngle;
	}
	else if(name_num==SteeringWheelAngleAdDigitalValue_n){
		*(uint16_t*)extern_data=Acc_Brake_Angle_LowVData.SteeringWheelAngleAdDigitalValue;
	}	
	else if(name_num==LowVoltageBatV_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.LowVoltageBatV;
	}
	else if(name_num==LowVoltageBatState_n){
		*(uint8_t*)extern_data=Acc_Brake_Angle_LowVData.LowVoltageBatState;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运4个悬架线位移等数据
*@param		TimGenericData_n name_num：索取数据对应的枚举变量
*@param		const void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_SuspensionLineDisplacementData(AD7689GenericData_n name_num,void * extern_data)
{
	if(name_num==SuspensionLineDistF1_ADValue_n){
		*(uint16_t*)extern_data=SuspensionLineDisplacementData.SuspensionLineDistF1_ADValue;
	} 	
	else if(name_num==SuspensionLineDistF2_ADValue_n){
		*(uint16_t*)extern_data=SuspensionLineDisplacementData.SuspensionLineDistF2_ADValue;
	}
	else if(name_num==SuspensionLineDistB1_ADValue_n){
		*(uint16_t*)extern_data=SuspensionLineDisplacementData.SuspensionLineDistB1_ADValue;
	}
	else if(name_num==SuspensionLineDistB2_ADValue_n){
		*(uint16_t*)extern_data=SuspensionLineDisplacementData.SuspensionLineDistB2_ADValue;
	}		
	else if(name_num==SuspensionLineDisplacementF1_n){
		*(float*)extern_data=SuspensionLineDisplacementData.SuspensionLineDisplacementF1;
	} 	
	else if(name_num==SuspensionLineDisplacementF2_n){
		*(float*)extern_data=SuspensionLineDisplacementData.SuspensionLineDisplacementF2;
	}
	else if(name_num==SuspensionLineDisplacementB1_n){
		*(float*)extern_data=SuspensionLineDisplacementData.SuspensionLineDisplacementB1;
	}
	else if(name_num==SuspensionLineDisplacementB2_n){
		*(float*)extern_data=SuspensionLineDisplacementData.SuspensionLineDisplacementB2;
	}	
}

/*
*****************************************************************************
*@brief		修改函数，修改油门，油压，电压电池电压，转角等数据
*@param		TimGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data)
{
	if(name_num==AccPedalLeft_n){
		Acc_Brake_Angle_LowVData.AccPedalLeft=*(uint16_t*)extern_data;
	} 	
	else if(name_num==AccPedalRight_n){
		Acc_Brake_Angle_LowVData.AccPedalRight=*(uint16_t*)extern_data;
	}
	else if(name_num==BrakePressureFront_n){
		Acc_Brake_Angle_LowVData.BrakePressureFront=*(float*)extern_data;
	}
	else if(name_num==BrakePressureBack_n){
		Acc_Brake_Angle_LowVData.BrakePressureBack=*(float*)extern_data;
	}
	else if(name_num==SteeringWheelAngle_n){
		Acc_Brake_Angle_LowVData.SteeringWheelAngle=*(float*)extern_data;
	}
	else if(name_num==SteeringWheelAngleAdDigitalValue_n){
		Acc_Brake_Angle_LowVData.SteeringWheelAngleAdDigitalValue=*(uint16_t*)extern_data;
	}	
	else if(name_num==LowVoltageBatV_n){
		Acc_Brake_Angle_LowVData.LowVoltageBatV=*(float*)extern_data;
	}
	else if(name_num==LowVoltageBatState_n){
		Acc_Brake_Angle_LowVData.LowVoltageBatState=*(uint8_t*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改函数，修改4个悬架线位移等数据
*@param		TimGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_SuspensionLineDisplacementData(AD7689GenericData_n name_num,void * extern_data)
{
	if(name_num==SuspensionLineDistF1_ADValue_n){
		SuspensionLineDisplacementData.SuspensionLineDistF1_ADValue=*(float*)extern_data;
	} 	
	else if(name_num==SuspensionLineDistF2_ADValue_n){
		SuspensionLineDisplacementData.SuspensionLineDistF2_ADValue=*(float*)extern_data;
	}
	else if(name_num==SuspensionLineDistB1_ADValue_n){
		SuspensionLineDisplacementData.SuspensionLineDistB1_ADValue=*(float*)extern_data;
	}
	else if(name_num==SuspensionLineDistB2_ADValue_n){
		SuspensionLineDisplacementData.SuspensionLineDistB2_ADValue=*(float*)extern_data;
	}		
	else if(name_num==SuspensionLineDisplacementF1_n){
		SuspensionLineDisplacementData.SuspensionLineDisplacementF1=*(float*)extern_data;
	} 	
	else if(name_num==SuspensionLineDisplacementF2_n){
		SuspensionLineDisplacementData.SuspensionLineDisplacementF2=*(float*)extern_data;
	}
	else if(name_num==SuspensionLineDisplacementB1_n){
		SuspensionLineDisplacementData.SuspensionLineDisplacementB1=*(float*)extern_data;
	}
	else if(name_num==SuspensionLineDisplacementB2_n){
		SuspensionLineDisplacementData.SuspensionLineDisplacementB2=*(float*)extern_data;
	}		
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
