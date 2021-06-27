/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#include "can.h"

#include "gpio.h"

#include "usart.h"
/* USER CODE BEGIN 0 */
/*******�������ݽ���*********/

struct {	
	volatile float Mot_Actual_Rev;
	volatile float Act_Current;
	volatile float Mot_power;
	volatile float DC_BUS;
	volatile float Capacity_I2xt;
	volatile uint16_t Mot_Temperature;
	volatile float IGBT_Temperature;
	volatile float Air_Temperature;
	volatile float Mot_Vout;	
	volatile uint32_t Mot_Err;
	volatile uint32_t Mot_State;
}MotParameter;//���յ�����ݽ���

struct Bat_Parameter{
	volatile float BatteryTotal_V;
	volatile float BatteryTotal_I;
	volatile uint8_t Battery_SOC;
	volatile uint8_t Battery_SOH;
	volatile uint8_t Battery_State;
	volatile uint8_t Battery_Warning_Level;
	volatile uint8_t CommunicationLifeInfo;	
};//BMS_ECU_INFO���ݽ���

struct Bat_MAXV{
	volatile uint16_t MaxSingleVoltage; 
	volatile uint16_t MinSingleVoltage;
	volatile uint8_t MaxSingleVoltage_NUM;
	volatile uint8_t MinSingleVoltage_NUM;	
};//BMS_ECU_MAXV���ݽ���

struct Bat_MAXT{
	volatile float MaxSingleTemperature; 
	volatile float MinSingleTemperature;
	volatile uint8_t MaxSingleTemperature_NUM;
	volatile uint8_t MinSingleTemperature_NUM;
	volatile uint8_t CoolingControl;
	volatile uint8_t HeatingControl;	
};//BMS_ECU_MAXT���ݽ���

struct Bat_RELAY{
	volatile uint8_t PositiveRelayState; 
	volatile uint8_t NegativeRelayState;
	volatile uint8_t PrechargeRelayState;
	volatile uint8_t CarSlowChargeRelayState;
	volatile uint8_t OutCarFastChargeRelayState;
	volatile uint8_t ChargeState;
	volatile uint8_t ChargerOnlineState;
	volatile uint8_t OutCarChargerConnectState;
	volatile uint8_t CarChargerConnectState;	
	volatile float ChargeRequestVoltage;
	volatile uint16_t ChargeRequestCurrent;	
};//BMS_ECU_RELAY���ݽ���

struct Bat_POWER{
	volatile uint16_t MaxAllowChargeCurrent;
	volatile uint16_t MaxAllowDischargeCurrent;
	volatile float	MaxAllowChargePower;
	volatile float	MaxAllowDischargePower;
};//BMS_ECU_POWER���ݽ���

struct Bat_ALARM{
	volatile uint8_t MonomerOverVol;
	volatile uint8_t MonomerUnderVol;
	volatile uint8_t BatteHighT;
	volatile uint8_t BatteLowT;
	volatile uint8_t SingleVolBreak;
	volatile uint8_t SingleTempBreak;
	volatile uint8_t SingleVolDiffLarge;
	volatile uint8_t BatteTempDiffLarge;
	volatile uint8_t AllGroupOverVol;
	volatile uint8_t AllGroupUnderVol;
	volatile uint8_t SOCHigh;
	volatile uint8_t SOCLow;
	volatile uint8_t SteadyChargeOverCur;
	volatile uint8_t SteadyDischargeOverCur;
	volatile uint8_t TransientChargeOverCur;
	volatile uint8_t TransientDischargeOverCur;
	volatile uint8_t BSU_Offline;
	volatile uint8_t BSU_BalanceFault;
	volatile uint8_t LeakageCurrentLimit;
	volatile uint8_t PrechargeFail;
	volatile uint8_t RelayFail;
	volatile uint8_t BMU_Fail;
	volatile uint8_t ECU_ExchangeTimeout;
	volatile uint8_t BatteHVAbnormal;
	volatile uint8_t HallBreak;
};//BMS_ECU_ALARM���ݽ���

struct Bat_CELLV{
	volatile uint16_t BMS_ECU_CELLV[108];	
};//BMS_ECU_CELLV���ݽ���

struct Bat_CELLT{
	volatile float BMS_ECU_CELLT[36];	
};//BMS_ECU_CELLT���ݽ���

struct {
	struct Bat_Parameter Battery_Parameter;
	struct Bat_MAXV Battery_MAXV;
	struct Bat_MAXT Battery_MAXT;
	struct Bat_RELAY Battery_RELAY;
	struct Bat_POWER Battery_POWER;
	struct Bat_ALARM Battery_ALARM;
	struct Bat_CELLV Battery_CELLV;
	struct Bat_CELLT Battery_CELLT;
}Battery_INFO;//���������ݽ���

struct SteeringWheel{
	volatile uint8_t StartFlag;
	volatile uint8_t WarterBump;
	volatile uint8_t Line_TCSToggleSwitch;
	volatile uint8_t Corners_TCSToggleSwitch;
	volatile uint8_t DRSSwitch1;
	volatile uint8_t DRSSwitch2;
	volatile uint8_t Mileage_CLEAR;
	volatile uint8_t SuspensionLineShift_ZeroSetting;
	volatile float StabilizerBar_Angle1;
	volatile float StabilizerBar_Angle2;
	volatile uint8_t IMDSwitch;
	volatile uint8_t BrakeReliabilitySwitch;
	volatile uint8_t Throttle_Brake_Interfering_Block;
};//STERRINGWHEEL_ECU_INFO0���ݽ���

struct Mpu6050AndBackupKnob{
	volatile float mpu6050_Angle_X;
	volatile float mpu6050_Angle_Y;
	volatile float mpu6050_Angle_Z;
	volatile uint16_t BackupKnob;
};//STERRINGWHEEL_ECU_INFO1���ݽ���

struct {
	struct SteeringWheel SteeringWheel_Switch;
	struct Mpu6050AndBackupKnob mpu6050_angle_AndBackupKnob;
}SteeringWheel_INFO;//���շ�����������ݽ���

struct BackWheel{
	volatile uint8_t BackWheelSpeed_Lkm;
	volatile float BackWheelSpeed_Lm;
	volatile uint8_t BackWheelSpeed_Rkm;
	volatile float BackWheelSpeed_Rm;	
};//ECU2_ECU_INFO0���ݽ���

struct FrontWheel{
	volatile uint8_t FrontWheelSpeed_Lkm;
	volatile float FrontWheelSpeed_Lm;
	volatile uint8_t FrontWheelSpeed_Rkm;
	volatile float FrontWheelSpeed_Rm;	
};//ECU2_ECU_INFO1���ݽ���0~5Byte
struct TempSensor{
	volatile float TempSensorValue;
};//ECU2_ECU_INFO1����6~7Byte

struct {
	struct BackWheel BackWheelSpeed;
	struct FrontWheel FrontWheelSpeed;
	struct TempSensor TemperatureSensor;
}WheelSpeedAndTemp;//�����ٶȺ��¶����ݽ���

struct {
	volatile uint8_t AMS_IMD_Reset_Input;
}AMS_IMDReset_Input_INFO;//ECU2_ECU_INFO2

struct {
	volatile uint8_t WaterTank_Tstate;
	volatile uint8_t WaterPump_State;
	volatile uint8_t Fan_State;
	volatile uint8_t Taillight_State;
	volatile uint8_t Speeker_State;	
}Pump_Speeker_State;//ECU2_ECU_STATE���ݽ���
/*******�������ݽ���*********/

/*******����HCU_BMS���ݽ���*********/
struct{
	volatile uint8_t HCU_BMS_CMD_online_offline;
	volatile uint8_t HCU_BMS_CMD_ShutDown_Boot;
}HCU_BMS_CMD;
/*******����HCU_BMS���ݽ���*********/

static CanTxMsgTypeDef s1_TxMsg; //CAN1������Ϣ
static CanRxMsgTypeDef s1_RxMsg; //CAN1������Ϣ
static CanTxMsgTypeDef s2_TxMsg; //CAN2������Ϣ
static CanRxMsgTypeDef s2_RxMsg; //CAN2������Ϣ
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

osThreadId CAN1_ReceiveData_AnalysisTaskHandle;
osThreadId CAN2_ReceiveData_AnalysisTaskHandle;
osThreadId CAN1_TransmitTaskHandle;
osThreadId CAN2_TransmitTaskHandle;

TimerHandle_t CAN2_PeriodicSendTimer_Handle;					//CAN2���ڷ��������ʱ�����
TimerHandle_t Get_CAN2_PeriodicSendTimer_Handle(void){
	return CAN2_PeriodicSendTimer_Handle;
}

osMessageQId CAN1TransmitQueueHandle;									//CAN1�������ݶ���
osThreadId Get_CAN1TransmitQueueHandle(void){
	return CAN1TransmitQueueHandle;
}

osMessageQId CAN2TransmitQueueHandle;									//CAN2�������ݶ���
osThreadId Get_CAN2TransmitQueueHandle(void){
	return CAN2TransmitQueueHandle;
}

/*CANͨѶ�������*/
void CAN1_ReceiveData_AnalysisTask(void const * argument);//can1�������ݽ�������
void CAN2_ReceiveData_AnalysisTask(void const * argument);//can2�������ݽ�������

void CAN1_TransmitTask(void const * argument);
void CAN2_TransmitTask(void const * argument);
void CAN2_PeriodicSendCallback(TimerHandle_t xTimer);//CAN2���ڷ��Ͷ�ʱ���ص�����

static void CAN_FREERTOS_Init(void) 
{
	/* definition and creation of CAN1_ReceiveData_AnalysisTask */
	osThreadDef(CAN1_ReceiveData_AnalysisTask, CAN1_ReceiveData_AnalysisTask, osPriorityNormal, 0, 128);
	CAN1_ReceiveData_AnalysisTaskHandle = osThreadCreate(osThread(CAN1_ReceiveData_AnalysisTask), NULL);
	/* definition and creation of CAN2_ReceiveData_AnalysisTask */
	osThreadDef(CAN2_ReceiveData_AnalysisTask, CAN2_ReceiveData_AnalysisTask, osPriorityNormal, 0, 128);
	CAN2_ReceiveData_AnalysisTaskHandle = osThreadCreate(osThread(CAN2_ReceiveData_AnalysisTask), NULL);
	/* definition and creation of CAN2_ReceiveData_AnalysisTask */
	osThreadDef(CAN1_TransmitTask, CAN1_TransmitTask, osPriorityNormal, 0, 128);
	CAN1_TransmitTaskHandle = osThreadCreate(osThread(CAN1_TransmitTask), NULL);	
	/* definition and creation of CAN2_ReceiveData_AnalysisTask */
	osThreadDef(CAN2_TransmitTask, CAN2_TransmitTask, osPriorityNormal, 0, 128);
	CAN2_TransmitTaskHandle = osThreadCreate(osThread(CAN2_TransmitTask), NULL);
	
	/* Create the queue(s) */
	/* definition and creation of CAN1TransmitQueue */
  osMessageQDef(CAN1TransmitQueue,6, CAN1TransmitMessageTypedef);
  CAN1TransmitQueueHandle = osMessageCreate(osMessageQ(CAN1TransmitQueue), NULL);	
  /* definition and creation of CAN2TransmitQueue */
  osMessageQDef(CAN2TransmitQueue, 14, CAN2TransmitMessageTypedef);
  CAN2TransmitQueueHandle = osMessageCreate(osMessageQ(CAN2TransmitQueue), NULL);	
	
	/*definition and creation of CAN2_PeriodicSendTimer */ 				//���ڶ�ʱ��������1ms(1��ʱ�ӽ���)������ģʽ //ID:1		      
	CAN2_PeriodicSendTimer_Handle=xTimerCreate((const char*)"CAN2_PeriodicSend",
																			(TickType_t)100,
																			(UBaseType_t)pdTRUE,
																			(void*)1,
																			(TimerCallbackFunction_t)CAN2_PeriodicSendCallback);     
}



/* CAN1 init function */
void MX_CAN1_Init(void)							//CAN1������250K
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler =12;//6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_7TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
	APP_CAN_Config(&hcan1);		
	CAN_FREERTOS_Init();	
}
/* CAN2 init function */
void MX_CAN2_Init(void)					//CAN2������250K
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler =12;//6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_7TQ;
  hcan2.Init.BS2 = CAN_BS2_6TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = ENABLE;//DISABLE
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
	APP_CAN_Config(&hcan2);
}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = ECU1_CAN1RX_Pin|ECU1_CAN1TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = ECU1_CAN2RX_Pin|ECU1_CAN2TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOB, ECU1_CAN1RX_Pin|ECU1_CAN1TX_Pin);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, ECU1_CAN2RX_Pin|ECU1_CAN2TX_Pin);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);

  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void APP_CAN_Config(CAN_HandleTypeDef* canHandle){
	CAN_FilterConfTypeDef sFilterConfig;
	//���ù�����
	if(canHandle->Instance==CAN1){
		sFilterConfig.FilterIdHigh=(((uint32_t)MotTxID<<21)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)MotTxID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterMaskIdHigh=0xFFFF;
		sFilterConfig.FilterMaskIdLow=0xFFFF;		
		sFilterConfig.FilterNumber=0;								//������0
		sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterActivation=ENABLE;
		sFilterConfig.BankNumber=1;
		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
		//initialize the msg pointer
		hcan1	.pTxMsg=&s1_TxMsg;
		hcan1	.pRxMsg=&s1_RxMsg;
		//configuer Rx message
		hcan1 .pRxMsg->StdId=0x7FF	;//Max_Data = 0x7FF			
		hcan1 .pRxMsg->IDE=CAN_ID_STD;
		hcan1 .pRxMsg->RTR=CAN_RTR_DATA;		
//		hcan1 .pRxMsg->DLC=8;
		//configuer Tx message
		hcan1 .pTxMsg->StdId=MotRxID;//Max_Data = x7FF	
		hcan1 .pTxMsg->ExtId=0x0000;//Max_Data = 0x1FFFFFFF
		hcan1 .pTxMsg->IDE=CAN_ID_STD;
		hcan1 .pTxMsg->RTR=CAN_RTR_DATA;
		hcan1 .pTxMsg->DLC=8;			
	}
	else if(canHandle->Instance==CAN2){
		//configure the filter1,											BMS����ECU����0
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_INFO_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_INFO_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterMaskIdHigh=0xFFFF;
		sFilterConfig.FilterMaskIdLow=0xFFFF;		
		sFilterConfig.FilterNumber=1;								//������1
		sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterActivation=ENABLE;
		sFilterConfig.BankNumber=1;
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter2,											BMS����ECU����1
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_MAXV_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_MAXV_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;		
		sFilterConfig.FilterNumber=2;								//������2
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter3,											BMS����ECU����2
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_MAXT_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_MAXT_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterNumber=3;								//������3
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter4,											BMS����ECU����3
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_RELAY_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_RELAY_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;	
		sFilterConfig.FilterNumber=4;								//������4
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter5,											BMS����ECU����4
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_POWER_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_POWER_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;	
		sFilterConfig.FilterNumber=5;								//������5
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter6,											BMS����ECU����5
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_ALARM_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_ALARM_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;	
		sFilterConfig.FilterNumber=6;								//������6
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter7,											BMS����ECU����6-32,�����ѹ���
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_CELLV0_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_CELLV0_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterMaskIdHigh=(0xFFE0<<3)|0x0007;//0xFF07
		sFilterConfig.FilterMaskIdLow=0xFFFF;		
		sFilterConfig.FilterNumber=7;								//������7
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter8,											BMS����ECU����33-41,�����¶����
		sFilterConfig.FilterIdHigh=(((uint32_t)BMS_ECU_CELLT0_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)BMS_ECU_CELLT0_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterMaskIdHigh=(0xFFF0<<3)|0x0007;//0xFF87
		sFilterConfig.FilterMaskIdLow=0xFFFF;		
		sFilterConfig.FilterNumber=8;								//������8
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter9,											WHEEL����ECU����1�����ı��42-43
		sFilterConfig.FilterIdHigh=(((uint32_t)STERRINGWHEEL_ECU_INFO0_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)STERRINGWHEEL_ECU_INFO0_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterMaskIdHigh=0xFFFF;
		sFilterConfig.FilterMaskIdLow=(0xFEFF<<3)|0x0007;//0xF7FF	
		sFilterConfig.FilterNumber=9;								//������9
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
		//configure the filter10,											ECU2ģ�鷢��ECU����2�����ı��44-47
		sFilterConfig.FilterIdHigh=(((uint32_t)ECU2_ECU_INFO0_ID<<3)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)ECU2_ECU_INFO0_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterMaskIdHigh=0xFFFF;
		sFilterConfig.FilterMaskIdLow=(0xF8FF<<3)|0x0007;	//0xC7FF	
		sFilterConfig.FilterNumber=10;							//������10
		HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);		
		//initialize the msg pointer
		hcan2	.pTxMsg=&s2_TxMsg;
		hcan2	.pRxMsg=&s2_RxMsg;
		//configuer Rx message
		hcan1 .pRxMsg->ExtId=0x1FFFFFFF;//Max_Data = 0x1FFFFFFF	
		hcan2 .pRxMsg->IDE=CAN_ID_EXT;
		hcan2 .pRxMsg->RTR=CAN_RTR_DATA;
//		hcan2 .pRxMsg->DLC=8;
		//configuer Tx message
		hcan2 .pTxMsg->StdId=0x0000;//Max_Data = x7FF	
		hcan2 .pTxMsg->ExtId=BMS_ECU_INFO_ID;//Max_Data = 0x1FFFFFFF
		hcan2 .pTxMsg->IDE=CAN_ID_EXT;
		hcan2 .pTxMsg->RTR=CAN_RTR_DATA;
		hcan2 .pTxMsg->DLC=8;			
	}	
}

/*
*****************************************************************************
*@brief		CAN1��CAN2���ͺ���,ʹ��CAN���������ͱ��ģ��������ñ���ID,
					���ͱ��������׵�ַ���������ݳ��ȣ�����һ�α�����������һ�����ݡ�	
*@param		CAN_HandleTypeDef* canHandle��can�ṹ�����
*@param		uint32_t ID��32��չID������0x18FF1234
*@param		uint8_t *pData�����ͱ��������׵�ַ�����緢������Ϊ
					CharBuff[8]={0,1,2,3,4,5,6,7},�����ڶ�������ΪCharBuff���ɡ�					
*@param		uint8_t Size,�������ݵĳ��ȣ�ȡֵӦΪ1~8��		
*@retval	None
*@par
*****************************************************************************
*/
void CSU_CAN_Send(CAN_HandleTypeDef* canHandle,uint32_t ID,uint8_t *pData, uint8_t Size){	//
	uint8_t a=0;
	if(canHandle->Instance==CAN1){	
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
		hcan1.pTxMsg->IDE = CAN_ID_STD;			//��׼֡
		hcan1.pTxMsg->ExtId = ID;																				//������չ֡ ID uint32_t ����:0x00000201
		for(a=0;a<Size;a++){
			hcan1.pTxMsg->Data[a]=*(pData+a); 
		}
		hcan1.pTxMsg->DLC = Size;																				//�������ݳ��� 1-8
		HAL_CAN_Transmit(&hcan1,10);																		//CAN1
}
	else if(canHandle->Instance==CAN2){
		hcan2.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
		hcan2.pTxMsg->IDE = CAN_ID_EXT;			//��չ֡
		hcan2.pTxMsg->ExtId = ID;																				//������չ֡ ID uint32_t ����:0x19ff1234
		for(a=0;a<Size;a++){
			hcan2.pTxMsg->Data[a]=*(pData+a); 
		}
		hcan2.pTxMsg->DLC = Size;																				//�������ݳ��� 1-8
		HAL_CAN_Transmit(&hcan2,10);																		//CAN2	
	}
}

/*
*****************************************************************************
*@brief		CAN1���͵���������ָ��,ʹ��CAN1����������CANOpen���ģ����ͱ�׼֡��11λID
*@param		uint8_t REGID������������Ĵ���ID��������յ���������ֲ�
*@param		uint16_t Data��Ҫ���͵�16bits���ݡ�			
*@retval	None
*@par
*****************************************************************************
*/
void CSU_CAN1_Send_Mot(uint8_t REGID,uint16_t Data)
{
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;			//��׼֡
	hcan1.pTxMsg->StdId = MotRxID;			//���ͱ�׼֡ID uint16t ����:0x201
	hcan1.pTxMsg->Data[0]=REGID;
	hcan1.pTxMsg->Data[1]= Data&0x00ff;		//��λ��ǰ
	hcan1.pTxMsg->Data[2]= (Data&0xff00)>>8;	//��λ�ں�
	hcan1.pTxMsg->DLC =3;										//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);
	//������Ƕ��CSU_CAN_Send��������Ҫ�޸���㺯������������ֱ�Ӳ����ײ����ֱ�ۣ����º�����ͬ
//	uint8_t pData[3];
//	pData[0]=REGID;
//	pData[1]=Data&0x00ff;				//��λ��ǰ
//	pData[2]=(Data&0xff00)>>8;	//��λ�ں�
//	CSU_CAN_Send(&hcan1,MotRxID,pData, 3);	
}

/*
*****************************************************************************
*@brief		CAN1�������ݺ���
*@param		uint8_t RegID���Ĵ���ID
*@param		uint16_t Ctrl_Data���Ĵ���ID��Ӧ��������
*@retval	None
*@par
*****************************************************************************
*/
void CAN1_DATA_Send(uint8_t RegID,uint16_t Ctrl_Data)
{	
	CAN1TransmitMessageTypedef MotCtrl_Data;
	MotCtrl_Data.CANTransmitID=RegID;
	MotCtrl_Data.CANTransmitData=Ctrl_Data;
	xQueueSend(CAN1TransmitQueueHandle,&MotCtrl_Data,(TickType_t)0);
}

void CAN1_TransmitTask(void const * argument)
{
  /* USER CODE BEGIN CAN_TransmitTask */
  /* Infinite loop */
	CAN1TransmitMessageTypedef MotCtrl_Data;
	for(;;)
  {
		if(xQueueReceive(CAN1TransmitQueueHandle,&MotCtrl_Data,(TickType_t)0)==pdTRUE){
			CSU_CAN1_Send_Mot(MotCtrl_Data.CANTransmitID,MotCtrl_Data.CANTransmitData);
		}
		osDelay(1);
  }
  /* USER CODE END CAN_TransmitTask */
}

/*
*****************************************************************************
*@brief		CAN1���յ������ֵ���֡���ú���,�ú�����CSU_CAN1_SelectNum_Receive���ʹ��
					�øú����������ݣ�ʹ��CAN1����������CANOpen���ģ�11λID
*@param		uint8_t Data_RegID�����������Ϣ�洢ID
*@param		uint8_t Data_Period�����͵Ĳ���ֵ��Ϣ������
*@retval	None
*@par
*****************************************************************************
*/
void CSU_CAN1_Receive_Mot_Ready(uint8_t Data_RegID,uint8_t Data_Period)
{
	//���ͻ�ȡ��Ϣ����
	uint16_t DATA_TEMP=0;
	DATA_TEMP=(DATA_TEMP|Data_Period)<<8;
	DATA_TEMP=DATA_TEMP|Data_RegID;
	CSU_CAN1_Send_Mot(RequestInfo_Reg,DATA_TEMP);	
	//��ȡ��Ϣ
	hcan1 .pRxMsg->IDE=CAN_ID_STD;
	hcan1 .pRxMsg->RTR=CAN_RTR_DATA;
//	hcan1 .pRxMsg->DLC=3;
	//HAL_CAN_Receive_IT(&hcan1,CAN_FILTER_FIFO0);
}


/*
*****************************************************************************
*@brief		�޸ĵ����վ��ַ��������������õ�EEPROM��
*@param		uint16_t Data���´�վ��ַ��16λ������0x210���ú���ֻ��Ա�׼֡
					���ǵ������������������С��ģʽ��can���߷�������ʱ�ȷ��͵�λ���ٷ��͸�λ
*@retval	None
*@par
*****************************************************************************
*/
void CSU_CAN1_ModifySlaveAdd(uint16_t Data)
{
	//�޸ĵ����վ��ַ
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;			//��׼֡
	hcan1.pTxMsg->StdId = MotRxID;			//���ͱ�׼֡ID uint16t ����:0x201
	hcan1.pTxMsg->Data[0]=ModifySlaveAdd_Reg;
	hcan1.pTxMsg->Data[1]= Data&0x00ff;		//��λ��ǰ
	hcan1.pTxMsg->Data[2]= (Data&0xff00)>>8;	//��λ�ں�
	hcan1.pTxMsg->DLC =3;										//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);
	//�����µĽ���ID
	uint32_t MotRxID_New=0;
	MotRxID_New=MotRxID_New|Data;
	//�ѵ�ǰ���ò������浽EEPROM��
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;			//��׼֡
	hcan1.pTxMsg->StdId = MotRxID_New;			//���ͱ�׼֡ID uint16t ����:0x201
	hcan1.pTxMsg->Data[0]=SaveParameters_Reg;
	hcan1.pTxMsg->Data[1]= 0x00;		//��λ��ǰ
	hcan1.pTxMsg->Data[2]= 0x00;	//��λ�ں�
	hcan1.pTxMsg->DLC =3;										//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);	
}

	
/*
*****************************************************************************
*@brief		����MotParameter
*@param		uint8_t MotReg������������Ĵ�����ַ
*@retval	None
*@par
*****************************************************************************
*/
static void MotParameter_Analysis(uint8_t MotReg,uint8_t DataByteH,uint8_t DataByteL)
{
	uint16_t Mot_Data=(DataByteH<<8)+DataByteL;	
	switch(MotReg){
		case MotSpeedValue_Reg: 		
			MotParameter.Mot_Actual_Rev=Mot_Data;								//���ʵ��ת��
			break;
		case MotCurrentValue_Reg:		
			MotParameter.Act_Current=Mot_Data;									//���ʵ�ʵ���
			break;
		case MotPowerValue_Reg:	
			MotParameter.Mot_power=Mot_Data;										//�������		
			break;
		case DCBusVoltageValue_Reg:
			MotParameter.DC_BUS=Mot_Data;												//������ֱ��ĸ�ߵ�ѹ
			break;
		case Capacity_Reg:
			MotParameter.Capacity_I2xt=Mot_Data;								//Capacity I2xt
			break;
		case MotTemperature_Reg:
			MotParameter.Mot_Temperature=Mot_Data;							//����¶�
			break;
		case IGBTTemperature_Reg:	
			MotParameter.IGBT_Temperature=Mot_Data*0.01-160;		//IGBT�¶�
			break;
		case AirTemperature_Reg:
			MotParameter.Air_Temperature=Mot_Data*0.021739-237;	//�������¶�
			break;	
		case OutVoltage_Reg:
			MotParameter.Mot_Vout=Mot_Data;											//��������ѹ
			break;		
	}
}

/*
*****************************************************************************
*@brief		����Battery_Parameter
*@param		uint8_t *receive_data,������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_Parameter_Analysis(uint8_t *receive_data)
{
	Battery_INFO.Battery_Parameter.BatteryTotal_V=((*receive_data<<8)+*(receive_data+1))*0.1;//��������ܵ�ѹ
	Battery_INFO.Battery_Parameter.BatteryTotal_I=((*(receive_data+2)<<8)+*(receive_data+3))*0.1-1000;//��������ܵ�ѹ
	Battery_INFO.Battery_Parameter.Battery_SOC=*(receive_data+4);//SOC
	Battery_INFO.Battery_Parameter.Battery_SOH=*(receive_data+5);//SOH
	Battery_INFO.Battery_Parameter.Battery_State=*(receive_data+6)>>4;//�������״��
	Battery_INFO.Battery_Parameter.Battery_Warning_Level=*(receive_data+6)&0x0F;//������ظ澯����
	Battery_INFO.Battery_Parameter.CommunicationLifeInfo=*(receive_data+7);//ͨ��������Ϣ
}

/*
*****************************************************************************
*@brief		����Battery_MAXV
*@param		uint8_t *receive_data,������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_MAXV_Analysis(uint8_t *receive_data)
{
	Battery_INFO.Battery_MAXV.MaxSingleVoltage=(*receive_data<<8)+*(receive_data+1);//��ߵ����ѹ
	Battery_INFO.Battery_MAXV.MinSingleVoltage=(*(receive_data+2)<<8)+*(receive_data+3);//��͵����ѹ
	Battery_INFO.Battery_MAXV.MaxSingleVoltage_NUM=*(receive_data+4);//��ߵ����ѹ���
	Battery_INFO.Battery_MAXV.MinSingleVoltage_NUM=*(receive_data+5);//��͵����ѹ���		
}

/*
*****************************************************************************
*@brief		����Battery_MAXT
*@param		uint8_t *receive_data,������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_MAXT_Analysis(uint8_t *receive_data)
{
	Battery_INFO.Battery_MAXT.MaxSingleTemperature=*receive_data-40;	//��ߵ����¶�
	Battery_INFO.Battery_MAXT.MinSingleTemperature=*(receive_data+1)-40;//��͵����¶�
	Battery_INFO.Battery_MAXT.MaxSingleTemperature_NUM=*(receive_data+2);//��ߵ����¶����
	Battery_INFO.Battery_MAXT.MinSingleTemperature_NUM=*(receive_data+3);//��͵����¶����	
	Battery_INFO.Battery_MAXT.CoolingControl=*(receive_data+4);//��ȴ����
	Battery_INFO.Battery_MAXT.HeatingControl=*(receive_data+5);//���ȿ���
}

/*
*****************************************************************************
*@brief		����Battery_RELAY
*@param		uint8_t *receive_data,������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void BMS_ECU_RELAY_Analysis(uint8_t *receive_data)
{
	Battery_INFO.Battery_RELAY.PositiveRelayState=*receive_data>>6;	//�������ܸ���Ԥ��磬���أ����䣩���̵���״̬
	Battery_INFO.Battery_RELAY.NegativeRelayState=(*receive_data&0x30)>>4;//�ǳ��أ���䣩���̵���״̬
	Battery_INFO.Battery_RELAY.PrechargeRelayState=(*receive_data&0x0F)>>2;//Ԥ���̵���״̬
	Battery_INFO.Battery_RELAY.CarSlowChargeRelayState=*receive_data&0x03;//���أ����䣩�̵���״̬	
	Battery_INFO.Battery_RELAY.OutCarFastChargeRelayState=*(receive_data+1)>>6;//�ǳ��أ���䣩�̵���״̬	
	Battery_INFO.Battery_RELAY.ChargeState=*(receive_data+2)>>4;//���״̬
	Battery_INFO.Battery_RELAY.ChargerOnlineState=(*(receive_data+2)&0x08)>>3;//�����Ƿ�����
	Battery_INFO.Battery_RELAY.OutCarChargerConnectState=(*(receive_data+2)&0x04)>>2;//�ǳ��س�������ȷ��
	Battery_INFO.Battery_RELAY.CarChargerConnectState=*(receive_data+2)&0x03;//���س�������ȷ��
	Battery_INFO.Battery_RELAY.ChargeRequestVoltage=((*(receive_data+4)<<8)+*(receive_data+5))*0.1;//��������ѹ
	Battery_INFO.Battery_RELAY.ChargeRequestCurrent=(*(receive_data+6)<<8)+*(receive_data+7);//����������	
}

/*
*****************************************************************************
*@brief		����Battery_POWER
*@param		uint8_t *receive_data,������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_POWER_Analysis(uint8_t *receive_data)
{
	Battery_INFO.Battery_POWER.MaxAllowChargeCurrent=(*receive_data<<8)+*(receive_data+1);//������������
	Battery_INFO.Battery_POWER.MaxAllowDischargeCurrent=(*(receive_data+2)<<8)+*(receive_data+3);//�������ŵ����
	Battery_INFO.Battery_POWER.MaxAllowChargePower=((*(receive_data+4)<<8)+*(receive_data+5))*0.1;//��������繦��
	Battery_INFO.Battery_POWER.MaxAllowDischargePower=((*(receive_data+6)<<8)+*(receive_data+7))*0.1;//�������ŵ繦��
}

/*
*****************************************************************************
*@brief		����Battery_ALARM
*@param		uint8_t *receive_data,������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_ALARM_Analysis(uint8_t *receive_data)
{
	Battery_INFO.Battery_ALARM.MonomerOverVol=*receive_data>>6;//�����ѹ
	Battery_INFO.Battery_ALARM.MonomerUnderVol=(*receive_data&0x30)>>4;//����Ƿѹ
	Battery_INFO.Battery_ALARM.BatteHighT=(*receive_data&0x0F)>>2;//��ظ���
	Battery_INFO.Battery_ALARM.BatteLowT=*receive_data&0x03;//��ص���
	Battery_INFO.Battery_ALARM.SingleVolBreak=*(receive_data+1)>>7;//�����ѹ����
	Battery_INFO.Battery_ALARM.SingleTempBreak=(*(receive_data+1)&0x40)>>6;//�����¶ȶ���
	Battery_INFO.Battery_ALARM.SingleVolDiffLarge=(*(receive_data+1)&0x30)>>4;//����ѹ�����
	Battery_INFO.Battery_ALARM.BatteTempDiffLarge=(*(receive_data+1)&0x0F)>>2;//����²����
	Battery_INFO.Battery_ALARM.AllGroupOverVol=*(receive_data+1)&0x03;//�����ѹ
	Battery_INFO.Battery_ALARM.AllGroupUnderVol=*(receive_data+2)>>6;//����Ƿѹ
	Battery_INFO.Battery_ALARM.SOCHigh=(*(receive_data+2)&0x30)>>4;//�����SOC����
	Battery_INFO.Battery_ALARM.SOCLow=(*(receive_data+2)&0x0F)>>2;//�����SOC����
	Battery_INFO.Battery_ALARM.SteadyChargeOverCur=*(receive_data+2)&0x03;//��̬������
	Battery_INFO.Battery_ALARM.SteadyDischargeOverCur=*(receive_data+3)>>6;//��̬�ŵ����
	Battery_INFO.Battery_ALARM.TransientChargeOverCur=(*(receive_data+3)&0x30)>>4;//˲̬������
	Battery_INFO.Battery_ALARM.TransientDischargeOverCur=(*(receive_data+3)&0x0F)>>2;//˲̬�ŵ����
	Battery_INFO.Battery_ALARM.BSU_Offline=*(receive_data+3)&0x03;//BSU����
	Battery_INFO.Battery_ALARM.BSU_BalanceFault=*(receive_data+4)>>7;//BSU�������
	Battery_INFO.Battery_ALARM.LeakageCurrentLimit=(*(receive_data+4)&0x60)>>5;//©��������
	Battery_INFO.Battery_ALARM.PrechargeFail=(*(receive_data+4)&0x10)>>4;//Ԥ���ʧ��
	Battery_INFO.Battery_ALARM.RelayFail=(*(receive_data+4)&0x08)>>3;//�̵�������
	Battery_INFO.Battery_ALARM.BMU_Fail=(*(receive_data+4)&0x04)>>2;//BMU����
	Battery_INFO.Battery_ALARM.ECU_ExchangeTimeout=*(receive_data+4)&0x03;//ECUͨѶ��ʱ
	Battery_INFO.Battery_ALARM.BatteHVAbnormal=*(receive_data+5)>>7;//��ظ�ѹ�쳣
	Battery_INFO.Battery_ALARM.HallBreak=(*(receive_data+5)&0x40)>>6;//��������
}

/*
*****************************************************************************
*@brief		����Battery_CELLV
*@param		uint8_t *receive_data��������յ�������
*@param		uint32_t ID_Num��ID���
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_CELLV_Analysis(uint8_t *receive_data,uint32_t ID_Num)
{
	uint8_t j=ID_Num-BMS_ECU_CELLV0_NUM;
	for(uint8_t i=0;i<4;i++){
		Battery_INFO.Battery_CELLV.BMS_ECU_CELLV[i+j*4]=(*(receive_data+2*i)<<8)+*(receive_data+2*i+1);//�����ѹ
	}	
}

/*
*****************************************************************************
*@brief		����Battery_CELLT
*@param		uint8_t *receive_data��������յ�������
*@param		uint32_t ID_Num��ID���
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_CELLT_Analysis(uint8_t *receive_data,uint32_t ID_Num)
{
	uint8_t j=ID_Num-BMS_ECU_CELLT0_NUM;
	for(uint8_t i=0;i<4;i++){
		Battery_INFO.Battery_CELLT.BMS_ECU_CELLT[i+j*4]=(*(receive_data+2*i)<<8)+*(receive_data+2*i+1)-40;//�����¶�
	}	
}

/*
*****************************************************************************
*@brief		����Battery_INFO
*@param		uint8_t *receive_data,������յ�������
*@param		uint32_t receive_ID,�������ݵ�ID��Դ
*@retval	None
*@par
*****************************************************************************
*/
static void Battery_INFO_Analysis(uint8_t *receive_data,uint32_t receive_ID)
{	
	if(receive_ID==BMS_ECU_INFO_ID){
		Battery_Parameter_Analysis(receive_data);//����Battery_Parameter
	}
	else if(receive_ID==BMS_ECU_MAXV_ID){
		Battery_MAXV_Analysis(receive_data);//����Battery_Parameter
	}
	else if(receive_ID==BMS_ECU_MAXT_ID){
		Battery_MAXT_Analysis(receive_data);//����Battery_MAXT
	}
	else if(receive_ID==BMS_ECU_RELAY_ID){
		BMS_ECU_RELAY_Analysis(receive_data);//����Battery_RELAY
	}
	else if(receive_ID==BMS_ECU_POWER_ID){
		Battery_POWER_Analysis(receive_data);//����Battery_POWER
	}
	else if(receive_ID==BMS_ECU_ALARM_ID){
		Battery_ALARM_Analysis(receive_data);//����Battery_ALARM
	}
	else{
		for(uint32_t i=0;i<(uint32_t)27;i++)
		{
			if((~BMS_ECU_CELLV0_ID&receive_ID)==i<<16){
				Battery_CELLV_Analysis(receive_data,i+BMS_ECU_CELLV0_NUM);
			}	
		}
		for(uint32_t i=0;i<(uint32_t)9;i++)
		{
			if((~BMS_ECU_CELLT0_ID&receive_ID)==i<<16){
				Battery_CELLT_Analysis(receive_data,i+BMS_ECU_CELLT0_NUM);
			}	
		}
	}
}

/*
*****************************************************************************
*@brief		����SteeringWheel_Switch
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void SteeringWheel_Switch_Analysis(uint8_t *receive_data)
{
	SteeringWheel_INFO.SteeringWheel_Switch.StartFlag=*receive_data>>7;//�����ź�
	SteeringWheel_INFO.SteeringWheel_Switch.WarterBump=(*receive_data&0x40)>>6;//ˮ�ÿ����ź�
	SteeringWheel_INFO.SteeringWheel_Switch.Line_TCSToggleSwitch=(*receive_data&0x20)>>5;//����������
	SteeringWheel_INFO.SteeringWheel_Switch.Corners_TCSToggleSwitch=(*receive_data&0x10)>>4;//����������
	SteeringWheel_INFO.SteeringWheel_Switch.DRSSwitch1=(*receive_data&0x08)>>3;//DRS����ǰ
	SteeringWheel_INFO.SteeringWheel_Switch.DRSSwitch2=(*receive_data&0x04)>>2;//DRS���غ�	
	SteeringWheel_INFO.SteeringWheel_Switch.Mileage_CLEAR=(*receive_data&0x02)>>1;//��̼����㰴ť
	SteeringWheel_INFO.SteeringWheel_Switch.SuspensionLineShift_ZeroSetting=*receive_data&0x01;//������λ�Ƶ��㰴ť
	SteeringWheel_INFO.SteeringWheel_Switch.StabilizerBar_Angle1=((*(receive_data+1)<<8)+*(receive_data+2))/4095.0*90.0;//��λ����ť1��Ӧ���ȶ���Ƕ�
	SteeringWheel_INFO.SteeringWheel_Switch.StabilizerBar_Angle2=((*(receive_data+3)<<8)+*(receive_data+4))/4095.0*90.0;//��λ����ť2��Ӧ���ȶ���Ƕ�
	SteeringWheel_INFO.SteeringWheel_Switch.IMDSwitch=*(receive_data+5);//������ر�IMD
	SteeringWheel_INFO.SteeringWheel_Switch.BrakeReliabilitySwitch=*(receive_data+6);//������ر��ƶ��ɿ���
	SteeringWheel_INFO.SteeringWheel_Switch.Throttle_Brake_Interfering_Block=(*(receive_data+7))>>7;//�����ƶ����湦�� ����  0������  1���ر�  Ĭ������
}

/*
*****************************************************************************
*@brief		����mpu6050_angle
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/

static void mpu6050_angle_Analysis(uint8_t *receive_data)
{
	SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_X=((*receive_data<<8)+*(receive_data+1))*0.1;//mpu6050 X��Ƕ�
	SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_Y=((*(receive_data+2)<<8)+*(receive_data+3))*0.1;//mpu6050 Y��Ƕ�
	SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_Z=((*(receive_data+4)<<8)+*(receive_data+5))*0.1;//mpu6050 Z��Ƕ�
	SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.BackupKnob=((*(receive_data+6)<<8)+*(receive_data+7));//������ť
}

/*
*****************************************************************************
*@brief		����SteeringWheel_INFO
*@param		uint8_t *receive_data��������յ�������
*@param		uint32_t receive_ID,�������ݵ�ID��Դ
*@retval	None
*@par
*****************************************************************************
*/
static void SteeringWheel_INFO_Analysis(uint8_t *receive_data,uint32_t receive_ID)
{
	switch(receive_ID){
		case STERRINGWHEEL_ECU_INFO0_ID:
			SteeringWheel_Switch_Analysis(receive_data);//����Wheel_Switch
			break;
		case STERRINGWHEEL_ECU_INFO1_ID:
			mpu6050_angle_Analysis(receive_data);//����mpu6050_angle
			break;
	}
}

/*
*****************************************************************************
*@brief		����BackWheelSpeed
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void BackWheelSpeed_Analysis(uint8_t *receive_data)
{
	WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Lkm=*receive_data;//�����ٶ��󣬵;���(km)
	WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Lm=((*(receive_data+1)<<8)+*(receive_data+2))*0.001;//�����ٶ��󣬸߾���(m)
	WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Rkm=*(receive_data+3);//�����ٶ��ң��;���(km)
	WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Rm=((*(receive_data+4)<<8)+*(receive_data+5))*0.001;//�����ٶ��ң��߾���(m)
}

/*
*****************************************************************************
*@brief		����FrontWheelSpeed
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void FrontWheelSpeed_Analysis(uint8_t *receive_data)
{
	WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Lkm=*receive_data;//ǰ���ٶ��󣬵;���(km)
	WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Lm=((*(receive_data+1)<<8)+*(receive_data+2))*0.001;//ǰ���ٶ��󣬸߾���(m)
	WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Rkm=*(receive_data+3);//ǰ���ٶ��ң��;���(km)
	WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Rm=((*(receive_data+4)<<8)+*(receive_data+5))*0.001;//ǰ���ٶ��ң��߾���(m)
	WheelSpeedAndTemp.TemperatureSensor.TempSensorValue=((*(receive_data+6)<<8)+*(receive_data+7))*0.01-40;//�¶ȴ�������ֵ
}

/*
*****************************************************************************
*@brief		����WheelSpeedAndTemp
*@param		uint8_t *receive_data��������յ�������
*@param		uint32_t receive_ID,�������ݵ�ID��Դ
*@retval	None
*@par
*****************************************************************************
*/
static void WheelSpeedAndTemp_Analysis(uint8_t *receive_data,uint32_t receive_ID)
{
	switch(receive_ID){
		case ECU2_ECU_INFO0_ID:
			BackWheelSpeed_Analysis(receive_data);//����BackWheelSpeed
			break;
		case ECU2_ECU_INFO1_ID:
			FrontWheelSpeed_Analysis(receive_data);//����FrontWheelSpeed��TemperatureSensor
			break;
	}
}

/*
*****************************************************************************
*@brief		����AMS_IMDReset_Input_INFO
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void AMS_IMDReset_INFO_Analysis(uint8_t *receive_data)
{
	AMS_IMDReset_Input_INFO.AMS_IMD_Reset_Input=*receive_data>>7;//AMS,IMD��λ�ź�����
}

/*
*****************************************************************************
*@brief		����Pump_Speeker_State
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Pump_Speeker_State_Analysis(uint8_t *receive_data)
{
	Pump_Speeker_State.WaterTank_Tstate=*receive_data>>6;//ˮ���¶�״̬
	Pump_Speeker_State.WaterPump_State=(*receive_data&0x20)>>5;//ˮ��״̬
	Pump_Speeker_State.Fan_State=(*receive_data&0x10)>>4;//����״̬
	Pump_Speeker_State.Taillight_State=(*receive_data&0x08)>>3;//β��״̬
	Pump_Speeker_State.Speeker_State=(*receive_data&0x04)>>2;//����״̬
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);
	if (hcan->Instance==CAN1){		
		BaseType_t xHigherPriorityTaskWoken=pdFALSE;
		vTaskNotifyGiveFromISR(CAN1_ReceiveData_AnalysisTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
//	else if (hcan->Instance==CAN2){
//		BaseType_t xHigherPriorityTaskWoken=pdFALSE;
//		vTaskNotifyGiveFromISR(CAN2_ReceiveData_AnalysisTaskHandle, &xHigherPriorityTaskWoken);		
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		
//	}
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxCpltCallback could be implemented in the user file
   */
}

/************************Task Function**********************/
void CAN1_ReceiveData_AnalysisTask(void const * argument)
{	
  /* USER CODE BEGIN CAN1_ReceiveData_AnalysisTask */
  /* Infinite loop */	
	HAL_CAN_Receive_IT(&hcan1,CAN_FILTER_FIFO0);
	uint8_t motreg=0,databyteH=0,databyteL=0;	
  for(;;)
  {				
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		motreg=hcan1.pRxMsg->Data[0];		
		if(motreg==WarningErrorMap_Reg){
			MotParameter.Mot_Err=(hcan1.pRxMsg->Data[4]<<24)+(hcan1.pRxMsg->Data[3]<<16)+(hcan1.pRxMsg->Data[2]<<8)+hcan1.pRxMsg->Data[1];
		}	
		else if(motreg==StatusMap_Reg){
			MotParameter.Mot_State=(hcan1.pRxMsg->Data[4]<<24)+(hcan1.pRxMsg->Data[3]<<16)+(hcan1.pRxMsg->Data[2]<<8)+hcan1.pRxMsg->Data[1];
		}	
		else{
			databyteH=hcan1.pRxMsg->Data[2];
			databyteL=hcan1.pRxMsg->Data[1];
			MotParameter_Analysis(motreg,databyteH,databyteL);			
		}
		HAL_CAN_Receive_IT(&hcan1,CAN_FILTER_FIFO0);
  }	
  /* USER CODE END CAN1_ReceiveData_AnalysisTask */
}

void CAN2_ReceiveData_AnalysisTask(void const * argument)
{	
  /* USER CODE BEGIN CAN2_ReceiveData_AnalysisTask */
  /* Infinite loop */		
//	HAL_CAN_Receive_IT(&hcan2,CAN_FILTER_FIFO0);	
	uint8_t Receive_Data[8];
	uint32_t Receive_ID;
  for(;;)
  {				
//		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
		if(HAL_CAN_Receive(&hcan2, CAN_FILTER_FIFO0, 100)==HAL_OK){
			Receive_ID=hcan2.pRxMsg->ExtId;
			for(uint8_t i=0;i<8;i++){
				Receive_Data[i]=hcan2.pRxMsg->Data[i];
			}
			if(Receive_ID==STERRINGWHEEL_ECU_INFO0_ID||Receive_ID==STERRINGWHEEL_ECU_INFO1_ID){
				SteeringWheel_INFO_Analysis(Receive_Data,Receive_ID);
			}
			else if(Receive_ID==ECU2_ECU_INFO0_ID||Receive_ID==ECU2_ECU_INFO1_ID){
				WheelSpeedAndTemp_Analysis(Receive_Data,Receive_ID);
			}
			else if(Receive_ID==ECU2_ECU_INFO2_ID){
				AMS_IMDReset_INFO_Analysis(Receive_Data);
			}
			else if(Receive_ID==ECU2_ECU_STATE_ID){
				Pump_Speeker_State_Analysis(Receive_Data);
			}		
			else{
				Battery_INFO_Analysis(Receive_Data,Receive_ID);
			}
		}
//		HAL_CAN_Receive_IT(&hcan2,CAN_FILTER_FIFO0);
		osDelay(20);
  }		
  /* USER CODE END CAN2_ReceiveData_AnalysisTask */
}

/*
*****************************************************************************
*@brief		ȡֵ����������MotParameter����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/

static void Get_MotParameter(Analysis_Data_n name_num,void * extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case Mot_Actual_Rev_n:
			*(float*)extern_data=MotParameter.Mot_Actual_Rev;
			break;
		case Act_Current_n:
			*(float*)extern_data=MotParameter.Act_Current;
			break;
		case Mot_power_n:
			*(float*)extern_data=MotParameter.Mot_power;
			break;
		case DC_BUS_n:
			*(float*)extern_data=MotParameter.DC_BUS;
			break;
		case Capacity_I2xt_n:
			*(float*)extern_data=MotParameter.Capacity_I2xt;
			break;
		case Mot_Temperature_n:
			*(uint16_t*)extern_data=MotParameter.Mot_Temperature;
			break;
		case IGBT_Temperature_n:
			*(float*)extern_data=MotParameter.IGBT_Temperature;
			break;
		case Air_Temperature_n:
			*(float*)extern_data=MotParameter.Air_Temperature;
			break;
		case Mot_Vout_n:
			*(float*)extern_data=MotParameter.Mot_Vout;
			break;
		case Mot_Err_n:
			*(uint32_t*)extern_data=MotParameter.Mot_Err;
			break;
		case Mot_State_n:
			*(uint32_t*)extern_data=MotParameter.Mot_State;
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������Battery_Parameter����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_Battery_Parameter(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case BatteryTotal_V_n:
			*(float*)extern_data=Battery_INFO.Battery_Parameter.BatteryTotal_V;
			break;
		case BatteryTotal_I_n:
			*(float*)extern_data=Battery_INFO.Battery_Parameter.BatteryTotal_I;
			break;
		case Battery_SOC_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_Parameter.Battery_SOC;
			break;
		case Battery_SOH_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_Parameter.Battery_SOH;
			break;
		case Battery_State_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_Parameter.Battery_State;
			break;
		case Battery_Warning_Level_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_Parameter.Battery_Warning_Level;
			break;		
		case CommunicationLifeInfo_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_Parameter.CommunicationLifeInfo;
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������Battery_MAXV And Battery_MAXT����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_Battery_MAXV_And_MAXT(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case MaxSingleVoltage_n:
			*(uint16_t*)extern_data=Battery_INFO.Battery_MAXV.MaxSingleVoltage;
			break;
		case MinSingleVoltage_n:
			*(uint16_t*)extern_data=Battery_INFO.Battery_MAXV.MinSingleVoltage;
			break;
		case MaxSingleVoltage_NUM_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_MAXV.MaxSingleVoltage_NUM;
			break;
		case MinSingleVoltage_NUM_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_MAXV.MinSingleVoltage_NUM;
			break;
		case MaxSingleTemperature_n:
			*(float*)extern_data=Battery_INFO.Battery_MAXT.MaxSingleTemperature;
			break;
		case MinSingleTemperature_n:
			*(float*)extern_data=Battery_INFO.Battery_MAXT.MinSingleTemperature;
			break;		
		case MaxSingleTemperature_NUM_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_MAXT.MaxSingleTemperature_NUM;
			break;
		case MinSingleTemperature_NUM_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_MAXT.MinSingleTemperature_NUM;
			break;
		case CoolingControl_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_MAXT.CoolingControl;
			break;		
		case HeatingControl_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_MAXT.HeatingControl;
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������Battery_RELAY����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_Battery_RELAY(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case PositiveRelayState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.PositiveRelayState;
			break;
		case NegativeRelayState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.NegativeRelayState;
			break;
		case PrechargeRelayState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.PrechargeRelayState;
			break;
		case CarSlowChargeRelayState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.CarSlowChargeRelayState;
			break;
		case OutCarFastChargeRelayState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.OutCarFastChargeRelayState;
			break;
		case ChargeState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.ChargeState;
			break;		
		case ChargerOnlineState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.ChargerOnlineState;
			break;
		case OutCarChargerConnectState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.OutCarChargerConnectState;
			break;
		case CarChargerConnectState_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_RELAY.CarChargerConnectState;
			break;		
		case ChargeRequestVoltage_n:
			*(float*)extern_data=Battery_INFO.Battery_RELAY.ChargeRequestVoltage;
			break;
		case ChargeRequestCurrent_n:
			*(uint16_t*)extern_data=Battery_INFO.Battery_RELAY.ChargeRequestCurrent;
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������Battery_POWER����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_Battery_POWER(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case MaxAllowChargeCurrent_n:
			*(uint16_t*)extern_data=Battery_INFO.Battery_POWER.MaxAllowChargeCurrent;
			break;
		case MaxAllowDischargeCurrent_n:
			*(uint16_t*)extern_data=Battery_INFO.Battery_POWER.MaxAllowDischargeCurrent;
			break;
		case MaxAllowChargePower_n:
			*(float*)extern_data=Battery_INFO.Battery_POWER.MaxAllowChargePower;
			break;
		case MaxAllowDischargePower_n:
			*(float*)extern_data=Battery_INFO.Battery_POWER.MaxAllowDischargePower;
			break;		
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������Battery_ALARM����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_Battery_ALARM(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case MonomerOverVol_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.MonomerOverVol;
			break;
		case MonomerUnderVol_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.MonomerUnderVol;
			break;
		case BatteHighT_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.BatteHighT;
			break;
		case BatteLowT_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.BatteLowT;
			break;
		case SingleVolBreak_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.SingleVolBreak;
			break;
		case SingleTempBreak_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.SingleTempBreak;
			break;		
		case SingleVolDiffLarge_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.SingleVolDiffLarge;
			break;
		case BatteTempDiffLarge_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.BatteTempDiffLarge;
			break;
		case AllGroupOverVol_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.AllGroupOverVol;
			break;		
		case AllGroupUnderVol_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.AllGroupUnderVol;
			break;
		case SOCHigh_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.SOCHigh;
			break;
		case SOCLow_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.SOCLow;
			break;
		case SteadyChargeOverCur_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.SteadyChargeOverCur;
			break;		
		case SteadyDischargeOverCur_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.SteadyDischargeOverCur;
			break;
		case TransientChargeOverCur_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.TransientChargeOverCur;
			break;
		case TransientDischargeOverCur_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.TransientDischargeOverCur;
			break;
		case BSU_Offline_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.BSU_Offline;
			break;		
		case BSU_BalanceFault_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.BSU_BalanceFault;
			break;
		case LeakageCurrentLimit_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.LeakageCurrentLimit;
			break;		
		case PrechargeFail_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.PrechargeFail;
			break;
		case RelayFail_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.RelayFail;
			break;
		case BMU_Fail_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.BMU_Fail;
			break;		
		case ECU_ExchangeTimeout_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.ECU_ExchangeTimeout;
			break;
		case BatteHVAbnormal_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.BatteHVAbnormal;
			break;
		case HallBreak_n:
			*(uint8_t*)extern_data=Battery_INFO.Battery_ALARM.HallBreak;
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������Battery_INFO����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_Battery_INFO(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	if(name_num>=BatteryTotal_V_n&&name_num<=CommunicationLifeInfo_n){
		Get_Battery_Parameter(name_num,extern_data);
	}
	else if(name_num>=MaxSingleVoltage_n&&name_num<=HeatingControl_n){
		Get_Battery_MAXV_And_MAXT(name_num,extern_data);
	}
	else if(name_num>=PositiveRelayState_n&&name_num<=ChargeRequestCurrent_n){
		Get_Battery_RELAY(name_num,extern_data);
	}
	else if(name_num>=MaxAllowChargeCurrent_n&&name_num<=MaxAllowDischargePower_n){
		Get_Battery_POWER(name_num,extern_data);
	}
	else if(name_num>=MonomerOverVol_n&&name_num<=HallBreak_n){
		Get_Battery_ALARM(name_num,extern_data);
	}
	else if(name_num==BMS_ECU_CELLV_n){
		for(uint8_t i=0;i<108;i++){
			*((uint16_t*)extern_data+i)=Battery_INFO.Battery_CELLV.BMS_ECU_CELLV[i];//ȡ�׵�ַ��Ҫ108�����ݻ����ټ���ȡֵ
		}
	}
	else if(name_num==BMS_ECU_CELLT_n){
		for(uint8_t i=0;i<36;i++){		
			*((float*)extern_data+i)=Battery_INFO.Battery_CELLT.BMS_ECU_CELLT[i];//ȡ�׵�ַ��Ҫ36�����ݻ����ټ���ȡֵ
		}
	}	
}

/*
*****************************************************************************
*@brief		ȡֵ����������SteeringWheel_INFO����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_SteeringWheel_INFO(Analysis_Data_n name_num,void*extern_data)//*extern_data���������get����ֵ
{
	switch(name_num)
	{
		case StartFlag_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.StartFlag;
			break;
		case WarterBump_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.WarterBump;
			break;
		case Line_TCSToggleSwitch_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.Line_TCSToggleSwitch;
			break;
		case Corners_TCSToggleSwitch_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.Corners_TCSToggleSwitch;
			break;
		case DRSSwitch1_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.DRSSwitch1;
			break;
		case DRSSwitch2_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.DRSSwitch2;
			break;
		case Mileage_CLEAR_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.Mileage_CLEAR;
			break;
		case SuspensionLineShift_ZeroSetting_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.SuspensionLineShift_ZeroSetting;
			break;
		case StabilizerBar_Angle1_n:
			*(float*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.StabilizerBar_Angle1;
			break;
		case StabilizerBar_Angle2_n:
			*(float*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.StabilizerBar_Angle2;
			break;
		case IMDSwitch_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.IMDSwitch;
			break;		
		case BrakeReliabilitySwitch_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.BrakeReliabilitySwitch;
			break;
		case Throttle_Brake_Interfering_Block_n:
			*(uint8_t*)extern_data=SteeringWheel_INFO.SteeringWheel_Switch.Throttle_Brake_Interfering_Block;
			break;
		case mpu6050_Angle_X_n:
			*(float*)extern_data=SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_X;
			break;
		case mpu6050_Angle_Y_n:
			*(float*)extern_data=SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_Y;
			break;
		case mpu6050_Angle_Z_n:
			*(float*)extern_data=SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_Z;
			break;		
		case BackupKnob_n:
			*(uint16_t*)extern_data=SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.BackupKnob;
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������WheelSpeedAndTemp����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_WheelSpeedAndTemp(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case BackWheelSpeed_Lkm_n:
			*(uint8_t*)extern_data=WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Lkm;
			break;
		case BackWheelSpeed_Lm_n:
			*(float*)extern_data=WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Lm;
			break;
		case BackWheelSpeed_Rkm_n:
			*(uint8_t*)extern_data=WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Rkm;
			break;
		case BackWheelSpeed_Rm_n:
			*(float*)extern_data=WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Rm;
			break;
		case FrontWheelSpeed_Lkm_n:
			*(uint8_t*)extern_data=WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Lkm;
			break;
		case FrontWheelSpeed_Lm_n:
			*(float*)extern_data=WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Lm;
			break;		
		case FrontWheelSpeed_Rkm_n:
			*(uint8_t*)extern_data=WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Rkm;
			break;
		case FrontWheelSpeed_Rm_n:
			*(float*)extern_data=WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Rm;
			break;
		case TempSensorValue_n:
			*(float*)extern_data=WheelSpeedAndTemp.TemperatureSensor.TempSensorValue;
			break;	
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������AMS_IMDReset_Switch_Timer_Input_INFO����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_AMS_IMDReset_INFO(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	*(uint8_t*)extern_data=AMS_IMDReset_Input_INFO.AMS_IMD_Reset_Input;
}

/*
*****************************************************************************
*@brief		ȡֵ����������Pump_Speeker_State����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
static void Get_Pump_Speeker_State(Analysis_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case WaterTank_Tstate_n:
			*(uint8_t*)extern_data=Pump_Speeker_State.WaterTank_Tstate;
			break;
		case WaterPump_State_n:
			*(uint8_t*)extern_data=Pump_Speeker_State.WaterPump_State;
			break;
		case Fan_State_n:
			*(uint8_t*)extern_data=Pump_Speeker_State.Fan_State;
			break;
		case Taillight_State_n:
			*(uint8_t*)extern_data=Pump_Speeker_State.Taillight_State;
			break;
		case Speeker_State_n:
			*(uint8_t*)extern_data=Pump_Speeker_State.Speeker_State;
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������CAN1/CAN2���ղ������õ�����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_CAN_Analysis_Data(Analysis_Data_n name_num,void*extern_data)//*extern_data����freertos�е����룬�������get����ֵ
{
	if(name_num>=Mot_Actual_Rev_n&&name_num<=Mot_State_n){
		Get_MotParameter(name_num,extern_data);
	} 
	else if(name_num>=BatteryTotal_V_n&&name_num<=BMS_ECU_CELLT_n){
		Get_Battery_INFO(name_num,extern_data);
	}
	else if(name_num>=StartFlag_n&&name_num<=BackupKnob_n){
		Get_SteeringWheel_INFO(name_num,extern_data);
	}
	else if(name_num>=BackWheelSpeed_Lkm_n&&name_num<=TempSensorValue_n){
		Get_WheelSpeedAndTemp(name_num,extern_data);
	}
	else if(name_num==AMS_IMD_Reset_Input_n){
		Get_AMS_IMDReset_INFO(name_num,extern_data);
	}
	else if(name_num>=WaterTank_Tstate_n&&name_num<=Speeker_State_n){
		Get_Pump_Speeker_State(name_num,extern_data);
	}
}


/*
*****************************************************************************
*@brief		DRS���1�Ͷ��2������Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_ECU2_SteeringEngine_Protocol_Send(uint32_t Protocol_ID)
{
	CAN2TransmitMessageTypedef CAN2TransmitMessage;	
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;	
	float DRS_angle1,DRS_angle2;	
	Get_DRS_Data(DRS_Angle1_n,&DRS_angle1);											//ȡֵ����������DRS���1�Ƕ�����
	Get_DRS_Data(DRS_Angle2_n,&DRS_angle2);											//ȡֵ����������DRS���2�Ƕ�����
	CAN2TransmitMessage.CANTransmitData[0]=(uint16_t)(DRS_angle1*10)>>8;
	CAN2TransmitMessage.CANTransmitData[1]=(uint16_t)(DRS_angle1*10);
	CAN2TransmitMessage.CANTransmitData[2]=(uint16_t)(DRS_angle2*10)>>8;
	CAN2TransmitMessage.CANTransmitData[3]=(uint16_t)(DRS_angle2*10);
	for(uint8_t i=4;i<8;i++)
	{
		CAN2TransmitMessage.CANTransmitData[i]=0x00;	
	}	
	xQueueSendToFront(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		���ȣ����ȣ�β�ƣ����Run��ˮ�ã�IMD���ƶ��ɿ��Դ����źŵ�����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_ECU2_CONTROL_Protocol_Send(uint32_t Protocol_ID)
{
	CAN2TransmitMessageTypedef CAN2TransmitMessage;	
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;	
	uint8_t FanCtrl_INFO,SpeakerCtrl_INFO,TaillightCtrl_INFO,MotRunBackup_INFO,WaterpumpCtrl_INFO;
	uint8_t IMD_BrakeReliability_INFO,Reserve1_INFO,Reserve0_INFO;	
	Get_Fan_Speaker_Taillight_Pump_CtrlData(FanCtrl_n,&FanCtrl_INFO);
	Get_Fan_Speaker_Taillight_Pump_CtrlData(SpeakerCtrl_n,&SpeakerCtrl_INFO);
	Get_Fan_Speaker_Taillight_Pump_CtrlData(TaillightCtrl_n,&TaillightCtrl_INFO);
	Get_MotRun_BrakeReliab_Reserve_Data(MotRunBackup_n,&MotRunBackup_INFO);
	Get_Fan_Speaker_Taillight_Pump_CtrlData(WaterpumpCtrl_n,&WaterpumpCtrl_INFO);
	Get_MotRun_BrakeReliab_Reserve_Data(IMD_BrakeReliability_Trigger_n,&IMD_BrakeReliability_INFO);
	Get_MotRun_BrakeReliab_Reserve_Data(Reserve1_n,&Reserve1_INFO);
	Get_MotRun_BrakeReliab_Reserve_Data(Reserve0_n,&Reserve0_INFO);		
	CAN2TransmitMessage.CANTransmitData[0]=FanCtrl_INFO<<7|SpeakerCtrl_INFO<<6|TaillightCtrl_INFO<<5|MotRunBackup_INFO<<4|WaterpumpCtrl_INFO<<3|IMD_BrakeReliability_INFO<<2|Reserve1_INFO<<1|Reserve0_INFO;
	for(uint8_t i=1;i<8;i++)
	{
		CAN2TransmitMessage.CANTransmitData[i]=0x00;	
	}
	xQueueSendToFront(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		�������&״̬����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_MOTERR_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	uint32_t mot_err,mot_state;	
	Get_CAN_Analysis_Data(Mot_Err_n,&mot_err);
	Get_CAN_Analysis_Data(Mot_State_n,&mot_state);
	for(uint8_t i=0;i<4;i++)
	{
		CAN2TransmitMessage.CANTransmitData[i]=mot_err>>((3-i)*8);
		CAN2TransmitMessage.CANTransmitData[i+4]=mot_state>>((3-i)*8);
	}	
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		��ظ澯����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_BATERR_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;	
	CAN2TransmitMessage.CANTransmitData[0]=Battery_INFO.Battery_ALARM.MonomerOverVol<<6|Battery_INFO.Battery_ALARM.MonomerUnderVol<<4|Battery_INFO.Battery_ALARM.BatteHighT<<2|Battery_INFO.Battery_ALARM.BatteLowT;
	CAN2TransmitMessage.CANTransmitData[1]=Battery_INFO.Battery_ALARM.SingleVolBreak<<7|Battery_INFO.Battery_ALARM.SingleTempBreak<<6|Battery_INFO.Battery_ALARM.SingleVolDiffLarge<<4|Battery_INFO.Battery_ALARM.BatteTempDiffLarge<<2|Battery_INFO.Battery_ALARM.AllGroupOverVol;
	CAN2TransmitMessage.CANTransmitData[2]=Battery_INFO.Battery_ALARM.AllGroupUnderVol<<6|Battery_INFO.Battery_ALARM.SOCHigh<<4|Battery_INFO.Battery_ALARM.SOCLow<<2|Battery_INFO.Battery_ALARM.SteadyChargeOverCur;
	CAN2TransmitMessage.CANTransmitData[3]=Battery_INFO.Battery_ALARM.SteadyDischargeOverCur<<6|Battery_INFO.Battery_ALARM.TransientChargeOverCur<<4|Battery_INFO.Battery_ALARM.TransientDischargeOverCur<<2|Battery_INFO.Battery_ALARM.BSU_Offline;
	CAN2TransmitMessage.CANTransmitData[4]=Battery_INFO.Battery_ALARM.BSU_BalanceFault<<7|Battery_INFO.Battery_ALARM.LeakageCurrentLimit<<5|Battery_INFO.Battery_ALARM.PrechargeFail<<4|Battery_INFO.Battery_ALARM.RelayFail<<3|Battery_INFO.Battery_ALARM.BMU_Fail<<2|Battery_INFO.Battery_ALARM.ECU_ExchangeTimeout;
	CAN2TransmitMessage.CANTransmitData[5]=Battery_INFO.Battery_ALARM.BatteHVAbnormal<<7|Battery_INFO.Battery_ALARM.HallBreak<<6;								
	CAN2TransmitMessage.CANTransmitData[6]=0x00;
	CAN2TransmitMessage.CANTransmitData[7]=0x00;
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		����ģ����Ϣ����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_STATE_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	uint8_t LowVoltageState,WaterTank_State,waterpump_state,Fanstate,Taillightstate,Speekerstate;
	uint8_t MPU_6050State,Sd_CardState,Gps_State,Sim_State,Bat_State,Motor_State,IMD_OpenOrClose,OpenOrClose_5KW,LineTCSFlag,CornersTCSFlag;
	uint8_t CarDRSState,Throttle_Brake_InterferingState_Data;
	Get_Acc_Brake_Angle_LowVData(LowVoltageBatState_n,&LowVoltageState);
	Get_Pump_Speeker_State(WaterTank_Tstate_n,&WaterTank_State);
	Get_Pump_Speeker_State(WaterPump_State_n,&waterpump_state);
	Get_Pump_Speeker_State(Fan_State_n,&Fanstate);
	Get_Pump_Speeker_State(Taillight_State_n,&Taillightstate);
	Get_Pump_Speeker_State(Speeker_State_n,&Speekerstate);
	Get_Line_CornersTCSState_Data(LineTCSState_n,&LineTCSFlag);
	Get_Line_CornersTCSState_Data(CornersTCSState_n,&CornersTCSFlag);
	CAN2TransmitMessage.CANTransmitData[0]= LowVoltageState<<6|WaterTank_State<<4|waterpump_state<<3|Fanstate<<2|Taillightstate<<1|Speekerstate;
	Get_mpu6050State_Data(mpu6050State_n,&MPU_6050State);									//ȡֵ����������mpu6050״̬����
	Get_SDCardState_Data(SDCardState_n,&Sd_CardState);										//ȡֵ����������SD��״̬����
	Get_GPSState_Data(GPSState_n,&Gps_State);															//ȡֵ����������GPS״̬����
	Get_SIMState_Data(SIMState_n,&Sim_State);															//ȡֵ����������SIMState״̬����
	Get_BATState_Data(BATState_n,&Bat_State);															//ȡֵ���������˵��״̬����
	Get_MotorState_Data(MotorState_n,&Motor_State);												//ȡֵ����������MotorState״̬����
	Get_IMDOpenOrClose_Data(IMDOpenOrClose_n,&IMD_OpenOrClose);						//ȡֵ����������IMDOpenOrClose״̬����
	Get_5KWOpenOrClose_Data(_5KWOpenOrClose_n,&OpenOrClose_5KW);					//ȡֵ����������5KWOpenOrClose״̬����
	Get_DRSState_Data(DRSState_n,&CarDRSState);
	Get_Throttle_Brake_Interferingtate_Data(Throttle_Brake_InterferingState_n,&Throttle_Brake_InterferingState_Data);
	CAN2TransmitMessage.CANTransmitData[1]=MPU_6050State<<7|Sd_CardState<<6|Gps_State<<5|Sim_State<<4|Bat_State<<3|Motor_State<<2|IMD_OpenOrClose<<1|OpenOrClose_5KW;
	CAN2TransmitMessage.CANTransmitData[2]=LineTCSFlag<<7|CornersTCSFlag<<6|CarDRSState<<5|Throttle_Brake_InterferingState_Data<<4;
	for(uint8_t i=3;i<8;i++)
	{
		CAN2TransmitMessage.CANTransmitData[i]=0x00;	
	}
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		������Ϣ����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_INFO_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	uint8_t car_speed_data,AccPedalL_Open,AccPedalR_Open;
	float mileage_data;	
	float BrakePF,BrakePB;
	Get_CarSpeed_Mileage_SlipRate_Data(CarSpeed_n,&car_speed_data);
	Get_CarSpeed_Mileage_SlipRate_Data(Mileage_n,&mileage_data);
	Get_AccPeda_Open_Data(AccPedalLeft_Open_n,&AccPedalL_Open);
	Get_AccPeda_Open_Data(AccPedalRight_Open_n,&AccPedalR_Open);
	Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&BrakePF);
	Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&BrakePB);
	CAN2TransmitMessage.CANTransmitData[0]=car_speed_data;
	CAN2TransmitMessage.CANTransmitData[1]=(uint16_t)(mileage_data*1000)>>8;
	CAN2TransmitMessage.CANTransmitData[2]=(uint16_t)(mileage_data*1000);
	CAN2TransmitMessage.CANTransmitData[3]=AccPedalL_Open;
	CAN2TransmitMessage.CANTransmitData[4]=AccPedalR_Open;
	CAN2TransmitMessage.CANTransmitData[5]=BrakePF*10;
	CAN2TransmitMessage.CANTransmitData[6]=BrakePB*10;
	CAN2TransmitMessage.CANTransmitData[7]=0x00;
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		ת��Ƕ�����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_ANGLEAndSuspension_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	float Angle;	
	float SuspensionLinedF1,SuspensionLinedF2,SuspensionLinedB1,SuspensionLinedB2;
	uint16_t SterringWheelAngle_Range;
	Get_Acc_Brake_Angle_LowVData(SteeringWheelAngle_n,&Angle);
	Get_SuspensionLineDisplacementData(SuspensionLineDisplacementF1_n,&SuspensionLinedF1);
	Get_SuspensionLineDisplacementData(SuspensionLineDisplacementF2_n,&SuspensionLinedF2);
	Get_SuspensionLineDisplacementData(SuspensionLineDisplacementB1_n,&SuspensionLinedB1);
	Get_SuspensionLineDisplacementData(SuspensionLineDisplacementB2_n,&SuspensionLinedB2);
	Get_Wireless_DataFrom_EEPROM(SterringWheelAngle_Range_n,&SterringWheelAngle_Range);
	CAN2TransmitMessage.CANTransmitData[0]=(uint16_t)((Angle+SterringWheelAngle_Range)*10)>>8;
	CAN2TransmitMessage.CANTransmitData[1]=(uint16_t)((Angle+SterringWheelAngle_Range)*10);
	CAN2TransmitMessage.CANTransmitData[2]=SuspensionLinedF1+50;
	CAN2TransmitMessage.CANTransmitData[3]=SuspensionLinedF2+50;
	CAN2TransmitMessage.CANTransmitData[4]=SuspensionLinedB1+50;
	CAN2TransmitMessage.CANTransmitData[5]=SuspensionLinedB2+50;
	for(uint8_t i=6;i<8;i++)
	{
		CAN2TransmitMessage.CANTransmitData[i]=0x00;	
	}
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		���������Ϣ����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_BATINFO_Protocol_Send(uint32_t Protocol_ID)
{
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	float LowBatV;
	CAN2TransmitMessage.CANTransmitData[0]=(uint16_t)(Battery_INFO.Battery_Parameter.BatteryTotal_V*10)>>8;
	CAN2TransmitMessage.CANTransmitData[1]=(uint16_t)(Battery_INFO.Battery_Parameter.BatteryTotal_V*10);
	CAN2TransmitMessage.CANTransmitData[2]=(uint16_t)((Battery_INFO.Battery_Parameter.BatteryTotal_I+1000)*10)>>8;
	CAN2TransmitMessage.CANTransmitData[3]=(uint16_t)((Battery_INFO.Battery_Parameter.BatteryTotal_I+1000)*10);
	CAN2TransmitMessage.CANTransmitData[4]=(Battery_INFO.Battery_Parameter.Battery_SOC);
	CAN2TransmitMessage.CANTransmitData[5]=(Battery_INFO.Battery_Parameter.Battery_SOH);
	Get_Acc_Brake_Angle_LowVData(LowVoltageBatV_n,&LowBatV);
	CAN2TransmitMessage.CANTransmitData[6]=(uint8_t)(LowBatV*10);	
	CAN2TransmitMessage.CANTransmitData[7]=0x00;
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		���������͵�ѹ�����ŵ�����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_MAXV_Protocol_Send(uint32_t Protocol_ID)
{
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	CAN2TransmitMessage.CANTransmitData[0]=Battery_INFO.Battery_MAXV.MaxSingleVoltage>>8;
	CAN2TransmitMessage.CANTransmitData[1]=Battery_INFO.Battery_MAXV.MaxSingleVoltage;
	CAN2TransmitMessage.CANTransmitData[2]=Battery_INFO.Battery_MAXV.MinSingleVoltage>>8;
	CAN2TransmitMessage.CANTransmitData[3]=Battery_INFO.Battery_MAXV.MinSingleVoltage;
	CAN2TransmitMessage.CANTransmitData[4]=Battery_INFO.Battery_MAXV.MaxSingleVoltage_NUM;
	CAN2TransmitMessage.CANTransmitData[5]=Battery_INFO.Battery_MAXV.MinSingleVoltage_NUM;
	CAN2TransmitMessage.CANTransmitData[6]=0x00;
	CAN2TransmitMessage.CANTransmitData[7]=0x00;
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		�������¶ȼ����ŵ�����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_MAXT_Protocol_Send(uint32_t Protocol_ID)
{
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	CAN2TransmitMessage.CANTransmitData[0]=Battery_INFO.Battery_MAXT.MaxSingleTemperature+40;
	CAN2TransmitMessage.CANTransmitData[1]=Battery_INFO.Battery_MAXT.MinSingleTemperature+40;
	CAN2TransmitMessage.CANTransmitData[2]=Battery_INFO.Battery_MAXT.MaxSingleTemperature_NUM;
	CAN2TransmitMessage.CANTransmitData[3]=Battery_INFO.Battery_MAXT.MinSingleTemperature_NUM;
	CAN2TransmitMessage.CANTransmitData[4]=Battery_INFO.Battery_MAXT.CoolingControl;
	CAN2TransmitMessage.CANTransmitData[5]=Battery_INFO.Battery_MAXT.HeatingControl;
	CAN2TransmitMessage.CANTransmitData[6]=(uint16_t)((WheelSpeedAndTemp.TemperatureSensor.TempSensorValue+40)*100)>>8;
	CAN2TransmitMessage.CANTransmitData[7]=(uint16_t)((WheelSpeedAndTemp.TemperatureSensor.TempSensorValue+40)*100);
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		�����Ϣ0Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_MOTINFO0_Protocol_Send(uint32_t Protocol_ID)
{
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	CAN2TransmitMessage.CANTransmitData[0]=(uint16_t)MotParameter.Mot_Actual_Rev>>8;
	CAN2TransmitMessage.CANTransmitData[1]=(uint16_t)MotParameter.Mot_Actual_Rev;	
	CAN2TransmitMessage.CANTransmitData[2]=(uint16_t)MotParameter.Act_Current>>8;
	CAN2TransmitMessage.CANTransmitData[3]=(uint16_t)MotParameter.Act_Current;
	CAN2TransmitMessage.CANTransmitData[4]=(uint16_t)MotParameter.Mot_power>>8;
	CAN2TransmitMessage.CANTransmitData[5]=(uint16_t)MotParameter.Mot_power;
	CAN2TransmitMessage.CANTransmitData[6]=(uint16_t)MotParameter.DC_BUS>>8;
	CAN2TransmitMessage.CANTransmitData[7]=(uint16_t)MotParameter.DC_BUS;	
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		�����Ϣ1Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_MOTINFO1_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	CAN2TransmitMessage.CANTransmitData[0]=(uint16_t)MotParameter.Capacity_I2xt>>8;
	CAN2TransmitMessage.CANTransmitData[1]=(uint16_t)MotParameter.Capacity_I2xt;	
	CAN2TransmitMessage.CANTransmitData[2]=(uint16_t)MotParameter.Mot_Temperature>>8;
	CAN2TransmitMessage.CANTransmitData[3]=(uint16_t)MotParameter.Mot_Temperature;
	CAN2TransmitMessage.CANTransmitData[4]=(uint16_t)((MotParameter.IGBT_Temperature+160)*100)>>8;
	CAN2TransmitMessage.CANTransmitData[5]=(uint16_t)((MotParameter.IGBT_Temperature+160)*100);
	CAN2TransmitMessage.CANTransmitData[6]=(uint16_t)((MotParameter.Air_Temperature+237)/0.021739)>>8;
	CAN2TransmitMessage.CANTransmitData[7]=(uint16_t)((MotParameter.Air_Temperature+237)/0.021739);	
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		�����Ϣ2Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_STERRINGWHEEL_MOTINFO2_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	CAN2TransmitMessage.CANTransmitData[0]=(uint16_t)MotParameter.Mot_Vout>>8;
	CAN2TransmitMessage.CANTransmitData[1]=(uint16_t)MotParameter.Mot_Vout;		
	for(uint8_t i=2;i<8;i++)
	{
		CAN2TransmitMessage.CANTransmitData[i]=0x00;	
	}
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		HCU_BMS_CMDЭ�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU_HCU_BMS_CMD_Protocol_Send(uint32_t Protocol_ID)
{	
	CAN2TransmitMessageTypedef CAN2TransmitMessage;
	CAN2TransmitMessage.CANTransmitID=Protocol_ID;
	CAN2TransmitMessage.CANTransmitData[0]=HCU_BMS_CMD.HCU_BMS_CMD_online_offline;
	CAN2TransmitMessage.CANTransmitData[1]=HCU_BMS_CMD.HCU_BMS_CMD_ShutDown_Boot;		
	for(uint8_t i=2;i<8;i++)
	{
		CAN2TransmitMessage.CANTransmitData[i]=0x00;	
	}
	xQueueSendToBack(CAN2TransmitQueueHandle,&CAN2TransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		CAN2�������ݺ���
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
void CAN2_DATA_Send(uint32_t Protocol_ID)
{	
	if(Protocol_ID==ECU_ECU2_SteeringEngine_ID){		
		ECU_ECU2_SteeringEngine_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_ECU2_CONTROL_ID){		
		ECU_ECU2_CONTROL_Protocol_Send(Protocol_ID);		
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_MOTERR_ID){		
		ECU_STERRINGWHEEL_MOTERR_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_BATERR_ID){
		ECU_STERRINGWHEEL_BATERR_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_STATE_ID){
		ECU_STERRINGWHEEL_STATE_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_INFO_ID){
		ECU_STERRINGWHEEL_INFO_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_ANGLE_ID){
		ECU_STERRINGWHEEL_ANGLEAndSuspension_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_BATINFO_ID){
		ECU_STERRINGWHEEL_BATINFO_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_MAXV_ID){
		ECU_STERRINGWHEEL_MAXV_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_MAXT_ID){
		ECU_STERRINGWHEEL_MAXT_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_MOTINFO0_ID){
		ECU_STERRINGWHEEL_MOTINFO0_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_MOTINFO1_ID){
		ECU_STERRINGWHEEL_MOTINFO1_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU_STERRINGWHEEL_MOTINFO2_ID){
		ECU_STERRINGWHEEL_MOTINFO2_Protocol_Send(Protocol_ID);
	}	
	else if(Protocol_ID==ECU_HCU_BMS_CMD_ID){
		ECU_HCU_BMS_CMD_Protocol_Send(Protocol_ID);							//����������Ч����ʵ�ʲ�����
	}
}

void CAN2_PeriodicSendCallback(TimerHandle_t xTimer)
{
	//��������
//	CAN2_DATA_Send(ECU_ECU2_SteeringEngine_ID);								//ECU���͸�ECU2ģ�鱨�ģ�DRS���
//	CAN2_DATA_Send(ECU_ECU2_CONTROL_ID);											//ECU���͸�ECU2ģ�鱨�ģ����ȡ����ȡ�β�ơ����RUN�źţ����ã���ˮ�ÿ����źš��̵���6IMD���ƶ��ɿ��Դ����ź� 
	//���ڷ�������
	CAN2_DATA_Send(ECU_STERRINGWHEEL_MOTERR_ID);							//ECU���͸�STERRINGWHEEL���ģ�������󡢾����״̬�����Ϣ
	CAN2_DATA_Send(ECU_STERRINGWHEEL_BATERR_ID);							//ECU���͸�STERRINGWHEEL���ģ�BMS���󡢾��������Ϣ
	CAN2_DATA_Send(ECU_STERRINGWHEEL_STATE_ID);								//ECU���͸�STERRINGWHEEL���ģ�����״̬��Ϣ����SD����GPS����ѹ���״̬&ˮ���¶�״̬��
	CAN2_DATA_Send(ECU_STERRINGWHEEL_INFO_ID);								//ECU���͸�STERRINGWHEEL���ģ�������Ϣ�������٣���̣�����&�ƶ�̤���
	CAN2_DATA_Send(ECU_STERRINGWHEEL_ANGLE_ID);								//ECU���͸�STERRINGWHEEL���ģ�������ת����Ϣ
	CAN2_DATA_Send(ECU_STERRINGWHEEL_BATINFO_ID);							//ECU���͸�STERRINGWHEEL���ģ���������Ϣ������������ܵ�ѹ&�ܵ�����SOC��SOH����ѹ��ص�ѹ��ˮ���¶ȵ�
	CAN2_DATA_Send(ECU_STERRINGWHEEL_MAXV_ID);								//ECU���͸�STERRINGWHEEL���ģ����&��͵����ѹ��������
	CAN2_DATA_Send(ECU_STERRINGWHEEL_MAXT_ID);								//ECU���͸�STERRINGWHEEL���ģ����&��͵����¶ȼ������У���ȴ&���ȿ���
	CAN2_DATA_Send(ECU_STERRINGWHEEL_MOTINFO0_ID);						//ECU���͸�STERRINGWHEEL���ģ������Ϣ��ر��ġ���ʵ��ת�٣�ʵ�ʵ�����������ʣ�ֱ�����ߵ�ѹ��
	CAN2_DATA_Send(ECU_STERRINGWHEEL_MOTINFO1_ID);						//ECU���͸�STERRINGWHEEL���ģ������Ϣ��ر��ġ���Capacity I2xt������¶ȣ�IGBT�¶ȣ��������¶�
	CAN2_DATA_Send(ECU_STERRINGWHEEL_MOTINFO2_ID);						//ECU���͸�STERRINGWHEEL���ģ������Ϣ��ر��ġ�����������������ѹ
}

void CAN2_TransmitTask(void const * argument)
{
  /* USER CODE BEGIN CAN_TransmitTask */
  /* Infinite loop */
	if(Get_CAN2_PeriodicSendTimer_Handle()!=NULL){
		xTimerReset(Get_CAN2_PeriodicSendTimer_Handle(),10);			//��λ���ڶ�ʱ��
		xTimerStart(Get_CAN2_PeriodicSendTimer_Handle(),10);			//�������ڶ�ʱ����CAN2_DATA_Send������100ms������CAN2���Ͷ����з�������
	}
	CAN2TransmitMessageTypedef ECU_ECU2AndSterringWheel_Data;
	for(;;)
  {
		if(xQueueReceive(CAN2TransmitQueueHandle,&ECU_ECU2AndSterringWheel_Data,(TickType_t)0)==pdTRUE){
			CSU_CAN_Send(&hcan2,ECU_ECU2AndSterringWheel_Data.CANTransmitID,ECU_ECU2AndSterringWheel_Data.CANTransmitData, 8);
		}
		osDelay(2);
  }
  /* USER CODE END CAN_TransmitTask */
}



/*
*****************************************************************************
*@brief		�޸�MotParameter����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_MotParameter(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case Mot_Actual_Rev_n:
			MotParameter.Mot_Actual_Rev=*(float*)extern_data;
			break;
		case Act_Current_n:
			MotParameter.Act_Current=*(float*)extern_data;
			break;
		case Mot_power_n:
			MotParameter.Mot_power=*(float*)extern_data;
			break;
		case DC_BUS_n:
			MotParameter.DC_BUS=*(float*)extern_data;
			break;
		case Capacity_I2xt_n:
			MotParameter.Capacity_I2xt=*(float*)extern_data;
			break;
		case Mot_Temperature_n:
			MotParameter.Mot_Temperature=*(uint16_t*)extern_data;
			break;
		case IGBT_Temperature_n:
			MotParameter.IGBT_Temperature=*(float*)extern_data;
			break;
		case Air_Temperature_n:
			MotParameter.Air_Temperature=*(float*)extern_data;
			break;
		case Mot_Vout_n:
			MotParameter.Mot_Vout=*(float*)extern_data;
			break;
		case Mot_Err_n:
			MotParameter.Mot_Err=*(uint32_t*)extern_data;
			break;
		case Mot_State_n:
			MotParameter.Mot_State=*(uint32_t*)extern_data;
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Battery_Parameter����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_Battery_Parameter(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case BatteryTotal_V_n:
			Battery_INFO.Battery_Parameter.BatteryTotal_V=*(float*)extern_data;
			break;
		case BatteryTotal_I_n:
			Battery_INFO.Battery_Parameter.BatteryTotal_I=*(float*)extern_data;
			break;
		case Battery_SOC_n:
			Battery_INFO.Battery_Parameter.Battery_SOC=*(uint8_t*)extern_data;
			break;
		case Battery_SOH_n:
			Battery_INFO.Battery_Parameter.Battery_SOH=*(uint8_t*)extern_data;
			break;
		case Battery_State_n:
			Battery_INFO.Battery_Parameter.Battery_State=*(uint8_t*)extern_data;
			break;
		case Battery_Warning_Level_n:
			Battery_INFO.Battery_Parameter.Battery_Warning_Level=*(uint8_t*)extern_data;
			break;		
		case CommunicationLifeInfo_n:
			Battery_INFO.Battery_Parameter.CommunicationLifeInfo=*(uint8_t*)extern_data;
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Battery_MAXV And Battery_MAXT����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_Battery_MAXV_And_MAXT(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case MaxSingleVoltage_n:
			Battery_INFO.Battery_MAXV.MaxSingleVoltage=*(uint16_t*)extern_data;
			break;
		case MinSingleVoltage_n:
			Battery_INFO.Battery_MAXV.MinSingleVoltage=*(uint16_t*)extern_data;
			break;
		case MaxSingleVoltage_NUM_n:
			Battery_INFO.Battery_MAXV.MaxSingleVoltage_NUM=*(uint8_t*)extern_data;
			break;
		case MinSingleVoltage_NUM_n:
			Battery_INFO.Battery_MAXV.MinSingleVoltage_NUM=*(uint8_t*)extern_data;
			break;
		case MaxSingleTemperature_n:
			Battery_INFO.Battery_MAXT.MaxSingleTemperature=*(float*)extern_data;
			break;
		case MinSingleTemperature_n:
			Battery_INFO.Battery_MAXT.MinSingleTemperature=*(float*)extern_data;
			break;		
		case MaxSingleTemperature_NUM_n:
			Battery_INFO.Battery_MAXT.MaxSingleTemperature_NUM=*(uint8_t*)extern_data;
			break;
		case MinSingleTemperature_NUM_n:
			Battery_INFO.Battery_MAXT.MinSingleTemperature_NUM=*(uint8_t*)extern_data;
			break;
		case CoolingControl_n:
			Battery_INFO.Battery_MAXT.CoolingControl=*(uint8_t*)extern_data;
			break;		
		case HeatingControl_n:
			Battery_INFO.Battery_MAXT.HeatingControl=*(uint8_t*)extern_data;
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Battery_RELAY����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_Battery_RELAY(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case PositiveRelayState_n:
			Battery_INFO.Battery_RELAY.PositiveRelayState=*(uint8_t*)extern_data;
			break;
		case NegativeRelayState_n:
			Battery_INFO.Battery_RELAY.NegativeRelayState=*(uint8_t*)extern_data;
			break;
		case PrechargeRelayState_n:
			Battery_INFO.Battery_RELAY.PrechargeRelayState=*(uint8_t*)extern_data;
			break;
		case CarSlowChargeRelayState_n:
			Battery_INFO.Battery_RELAY.CarSlowChargeRelayState=*(uint8_t*)extern_data;
			break;
		case OutCarFastChargeRelayState_n:
			Battery_INFO.Battery_RELAY.OutCarFastChargeRelayState=*(uint8_t*)extern_data;
			break;
		case ChargeState_n:
			Battery_INFO.Battery_RELAY.ChargeState=*(uint8_t*)extern_data;
			break;		
		case ChargerOnlineState_n:
			Battery_INFO.Battery_RELAY.ChargerOnlineState=*(uint8_t*)extern_data;
			break;
		case OutCarChargerConnectState_n:
			Battery_INFO.Battery_RELAY.OutCarChargerConnectState=*(uint8_t*)extern_data;
			break;
		case CarChargerConnectState_n:
			Battery_INFO.Battery_RELAY.CarChargerConnectState=*(uint8_t*)extern_data;
			break;		
		case ChargeRequestVoltage_n:
			Battery_INFO.Battery_RELAY.ChargeRequestVoltage=*(float*)extern_data;
			break;
		case ChargeRequestCurrent_n:
			Battery_INFO.Battery_RELAY.ChargeRequestCurrent=*(uint16_t*)extern_data;
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Battery_POWER����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_Battery_POWER(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case MaxAllowChargeCurrent_n:
			Battery_INFO.Battery_POWER.MaxAllowChargeCurrent=*(uint16_t*)extern_data;
			break;
		case MaxAllowDischargeCurrent_n:
			Battery_INFO.Battery_POWER.MaxAllowDischargeCurrent=*(uint16_t*)extern_data;
			break;
		case MaxAllowChargePower_n:
			Battery_INFO.Battery_POWER.MaxAllowChargePower=*(float*)extern_data;
			break;
		case MaxAllowDischargePower_n:
			Battery_INFO.Battery_POWER.MaxAllowDischargePower=*(float*)extern_data;
			break;		
	}
}

/*
*****************************************************************************
*@brief		�޸�Battery_ALARM����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_Battery_ALARM(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case MonomerOverVol_n:
			Battery_INFO.Battery_ALARM.MonomerOverVol=*(uint8_t*)extern_data;
			break;
		case MonomerUnderVol_n:
			Battery_INFO.Battery_ALARM.MonomerUnderVol=*(uint8_t*)extern_data;
			break;
		case BatteHighT_n:
			Battery_INFO.Battery_ALARM.BatteHighT=*(uint8_t*)extern_data;
			break;
		case BatteLowT_n:
			Battery_INFO.Battery_ALARM.BatteLowT=*(uint8_t*)extern_data;
			break;
		case SingleVolBreak_n:
			Battery_INFO.Battery_ALARM.SingleVolBreak=*(uint8_t*)extern_data;
			break;
		case SingleTempBreak_n:
			Battery_INFO.Battery_ALARM.SingleTempBreak=*(uint8_t*)extern_data;
			break;		
		case SingleVolDiffLarge_n:
			Battery_INFO.Battery_ALARM.SingleVolDiffLarge=*(uint8_t*)extern_data;
			break;
		case BatteTempDiffLarge_n:
			Battery_INFO.Battery_ALARM.BatteTempDiffLarge=*(uint8_t*)extern_data;
			break;
		case AllGroupOverVol_n:
			Battery_INFO.Battery_ALARM.AllGroupOverVol=*(uint8_t*)extern_data;
			break;		
		case AllGroupUnderVol_n:
			Battery_INFO.Battery_ALARM.AllGroupUnderVol=*(uint8_t*)extern_data;
			break;
		case SOCHigh_n:
			Battery_INFO.Battery_ALARM.SOCHigh=*(uint8_t*)extern_data;
			break;
		case SOCLow_n:
			Battery_INFO.Battery_ALARM.SOCLow=*(uint8_t*)extern_data;
			break;
		case SteadyChargeOverCur_n:
			Battery_INFO.Battery_ALARM.SteadyChargeOverCur=*(uint8_t*)extern_data;
			break;		
		case SteadyDischargeOverCur_n:
			Battery_INFO.Battery_ALARM.SteadyDischargeOverCur=*(uint8_t*)extern_data;
			break;
		case TransientChargeOverCur_n:
			Battery_INFO.Battery_ALARM.TransientChargeOverCur=*(uint8_t*)extern_data;
			break;
		case TransientDischargeOverCur_n:
			Battery_INFO.Battery_ALARM.TransientDischargeOverCur=*(uint8_t*)extern_data;
			break;
		case BSU_Offline_n:
			Battery_INFO.Battery_ALARM.BSU_Offline=*(uint8_t*)extern_data;
			break;		
		case BSU_BalanceFault_n:
			Battery_INFO.Battery_ALARM.BSU_BalanceFault=*(uint8_t*)extern_data;
			break;
		case LeakageCurrentLimit_n:
			Battery_INFO.Battery_ALARM.LeakageCurrentLimit=*(uint8_t*)extern_data;
			break;		
		case PrechargeFail_n:
			Battery_INFO.Battery_ALARM.PrechargeFail=*(uint8_t*)extern_data;
			break;
		case RelayFail_n:
			Battery_INFO.Battery_ALARM.RelayFail=*(uint8_t*)extern_data;
			break;
		case BMU_Fail_n:
			Battery_INFO.Battery_ALARM.BMU_Fail=*(uint8_t*)extern_data;
			break;		
		case ECU_ExchangeTimeout_n:
			Battery_INFO.Battery_ALARM.ECU_ExchangeTimeout=*(uint8_t*)extern_data;
			break;
		case BatteHVAbnormal_n:
			Battery_INFO.Battery_ALARM.BatteHVAbnormal=*(uint8_t*)extern_data;
			break;
		case HallBreak_n:
			Battery_INFO.Battery_ALARM.HallBreak=*(uint8_t*)extern_data;
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Battery_INFO����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_Battery_INFO(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	if(name_num>=BatteryTotal_V_n&&name_num<=CommunicationLifeInfo_n){
		Set_Battery_Parameter(name_num,extern_data);
	}
	else if(name_num>=MaxSingleVoltage_n&&name_num<=HeatingControl_n){
		Set_Battery_MAXV_And_MAXT(name_num,extern_data);
	}
	else if(name_num>=PositiveRelayState_n&&name_num<=ChargeRequestCurrent_n){
		Set_Battery_RELAY(name_num,extern_data);
	}
	else if(name_num>=MaxAllowChargeCurrent_n&&name_num<=MaxAllowDischargePower_n){
		Set_Battery_POWER(name_num,extern_data);
	}
	else if(name_num>=MonomerOverVol_n&&name_num<=HallBreak_n){
		Set_Battery_ALARM(name_num,extern_data);
	}
	else if(name_num==BMS_ECU_CELLV_n){
		for(uint8_t i=0;i<108;i++){
			Battery_INFO.Battery_CELLV.BMS_ECU_CELLV[i]=*((uint16_t*)extern_data+i);//ȡ�׵�ַ��Ҫ108�����ݻ����ټ���ȡֵ
		}
	}
	else if(name_num==BMS_ECU_CELLT_n){
		for(uint8_t i=0;i<36;i++){		
			Battery_INFO.Battery_CELLT.BMS_ECU_CELLT[i]=*((float*)extern_data+i);//ȡ�׵�ַ��Ҫ36�����ݻ����ټ���ȡֵ
		}
	}	
}

/*
*****************************************************************************
*@brief		�޸�SteeringWheel_INFO����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_SteeringWheel_INFO(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case StartFlag_n:
			SteeringWheel_INFO.SteeringWheel_Switch.StartFlag=*(uint8_t*)extern_data;
			break;
		case WarterBump_n:
			SteeringWheel_INFO.SteeringWheel_Switch.WarterBump=*(uint8_t*)extern_data;
			break;
		case Line_TCSToggleSwitch_n:
			SteeringWheel_INFO.SteeringWheel_Switch.Line_TCSToggleSwitch=*(uint8_t*)extern_data;
			break;
		case Corners_TCSToggleSwitch_n:
			SteeringWheel_INFO.SteeringWheel_Switch.Corners_TCSToggleSwitch=*(uint8_t*)extern_data;
			break;
		case DRSSwitch1_n:
			SteeringWheel_INFO.SteeringWheel_Switch.DRSSwitch1=*(uint8_t*)extern_data;
			break;
		case DRSSwitch2_n:
			SteeringWheel_INFO.SteeringWheel_Switch.DRSSwitch2=*(uint8_t*)extern_data;
			break;
		case Mileage_CLEAR_n:		
			SteeringWheel_INFO.SteeringWheel_Switch.Mileage_CLEAR=*(uint8_t*)extern_data;
			break;	
		case SuspensionLineShift_ZeroSetting_n:
			SteeringWheel_INFO.SteeringWheel_Switch.SuspensionLineShift_ZeroSetting=*(uint8_t*)extern_data;
			break;		
		case StabilizerBar_Angle1_n:
			SteeringWheel_INFO.SteeringWheel_Switch.StabilizerBar_Angle1=*(float*)extern_data;
			break;
		case StabilizerBar_Angle2_n:
			SteeringWheel_INFO.SteeringWheel_Switch.StabilizerBar_Angle2=*(float*)extern_data;
			break;
		case IMDSwitch_n:
			SteeringWheel_INFO.SteeringWheel_Switch.IMDSwitch=*(uint8_t*)extern_data;
			break;		
		case BrakeReliabilitySwitch_n:
			SteeringWheel_INFO.SteeringWheel_Switch.BrakeReliabilitySwitch=*(uint8_t*)extern_data;
			break;
		case Throttle_Brake_Interfering_Block_n:
			SteeringWheel_INFO.SteeringWheel_Switch.Throttle_Brake_Interfering_Block=*(uint8_t*)extern_data;
			break;
		case mpu6050_Angle_X_n:
			SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_X=*(float*)extern_data;
			break;
		case mpu6050_Angle_Y_n:
			SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_Y=*(float*)extern_data;
			break;
		case mpu6050_Angle_Z_n:
			SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.mpu6050_Angle_Z=*(float*)extern_data;
			break;		
		case BackupKnob_n:
			SteeringWheel_INFO.mpu6050_angle_AndBackupKnob.BackupKnob=*(uint16_t*)extern_data;
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�WheelSpeedAndTemp����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_WheelSpeedAndTemp(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case BackWheelSpeed_Lkm_n:
			WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Lkm=*(uint8_t*)extern_data;
			break;
		case BackWheelSpeed_Lm_n:
			WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Lm=*(float*)extern_data;
			break;
		case BackWheelSpeed_Rkm_n:
			WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Rkm=*(uint8_t*)extern_data;
			break;
		case BackWheelSpeed_Rm_n:
			WheelSpeedAndTemp.BackWheelSpeed.BackWheelSpeed_Rm=*(float*)extern_data;
			break;
		case FrontWheelSpeed_Lkm_n:
			WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Lkm=*(uint8_t*)extern_data;
			break;
		case FrontWheelSpeed_Lm_n:
			WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Lm=*(float*)extern_data;
			break;		
		case FrontWheelSpeed_Rkm_n:
			WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Rkm=*(uint8_t*)extern_data;
			break;
		case FrontWheelSpeed_Rm_n:
			WheelSpeedAndTemp.FrontWheelSpeed.FrontWheelSpeed_Rm=*(float*)extern_data;
			break;
		case TempSensorValue_n:
			WheelSpeedAndTemp.TemperatureSensor.TempSensorValue=*(float*)extern_data;
			break;	
	}
}

/*
*****************************************************************************
*@brief		�޸�AMS_IMDReset_Switch_Timer_Input_INFO����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_AMS_IMDReset_INFO(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	AMS_IMDReset_Input_INFO.AMS_IMD_Reset_Input=*(uint8_t*)extern_data;
}

/*
*****************************************************************************
*@brief		�޸�Pump_Speeker_State����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
static void Set_Pump_Speeker_State(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case WaterTank_Tstate_n:
			Pump_Speeker_State.WaterTank_Tstate=*(uint8_t*)extern_data;
			break;
		case WaterPump_State_n:
			Pump_Speeker_State.WaterPump_State=*(uint8_t*)extern_data;
			break;
		case Fan_State_n:
			Pump_Speeker_State.Fan_State=*(uint8_t*)extern_data;
			break;
		case Taillight_State_n:
			Pump_Speeker_State.Taillight_State=*(uint8_t*)extern_data;
			break;
		case Speeker_State_n:
			Pump_Speeker_State.Speeker_State=*(uint8_t*)extern_data;
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�CAN1/CAN2���ղ������õ�����
*@param		Analysis_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_CAN_Analysis_Data(Analysis_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	if(name_num>=Mot_Actual_Rev_n&&name_num<=Mot_State_n){
		Set_MotParameter(name_num,extern_data);
	} 
	else if(name_num>=BatteryTotal_V_n&&name_num<=BMS_ECU_CELLT_n){
		Set_Battery_INFO(name_num,extern_data);
	}
	else if(name_num>=StartFlag_n&&name_num<=BackupKnob_n){
		Set_SteeringWheel_INFO(name_num,extern_data);
	}
	else if(name_num>=BackWheelSpeed_Lkm_n&&name_num<=TempSensorValue_n){
		Set_WheelSpeedAndTemp(name_num,extern_data);
	}
	else if(name_num==AMS_IMD_Reset_Input_n){
		Set_AMS_IMDReset_INFO(name_num,extern_data);
	}
	else if(name_num>=WaterTank_Tstate_n&&name_num<=Speeker_State_n){
		Set_Pump_Speeker_State(name_num,extern_data);
	}
}

/*
*****************************************************************************
*@brief		��ȡECU_HCU_BMS_CMD��������
*@param		HCU_BMS_CMD_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_HCU_BMS_CMD_Data(HCU_BMS_CMD_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	if(name_num==HCU_BMS_CMD_online_offline_n){
		*(uint8_t*)extern_data=HCU_BMS_CMD.HCU_BMS_CMD_online_offline;
	} 	
	else if(name_num==HCU_BMS_CMD_ShutDown_Boot_n){
		*(uint8_t*)extern_data=HCU_BMS_CMD.HCU_BMS_CMD_ShutDown_Boot;
	}
}

/*
*****************************************************************************
*@brief		�޸�ECU_HCU_BMS_CMD���͵���������
*@param		HCU_BMS_CMD_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_HCU_BMS_CMD_Data(HCU_BMS_CMD_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	if(name_num==HCU_BMS_CMD_online_offline_n){
		HCU_BMS_CMD.HCU_BMS_CMD_online_offline=*(uint8_t*)extern_data;
	} 	
	else if(name_num==HCU_BMS_CMD_ShutDown_Boot_n){
		HCU_BMS_CMD.HCU_BMS_CMD_ShutDown_Boot=*(uint8_t*)extern_data;
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
