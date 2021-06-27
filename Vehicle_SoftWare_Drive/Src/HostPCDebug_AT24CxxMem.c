#include "HostPCDebug_AT24CxxMem.h"

volatile uint8_t WirelessAnalysis_Data[5]={0};
volatile uint8_t WirelessReceice_Data_OneByte;


struct{
	volatile float BrakePressure_Range_High;							//制动踏板油压传感器压力量程高10.0Mpa
	volatile float BrakePressure_Range_Low;								//制动踏板油压传感器压力量程低0.0Mpa
	volatile float BrakePressure_Range_ActualHigh;				//制动踏板油压传感实际压力值高
	volatile float BrakePressure_Range_ActualLow;					//制动踏板油压传感实际压力值低
	volatile uint16_t AccPeda_Range_High;									//油门踏板行程高65535
	volatile uint16_t AccPeda_Range_Low;									//油门踏板行程低0
	volatile uint16_t AccPeda_InitialDiff_Of_LandR;				//油门踏板左右传感器初始值差异			
	volatile uint16_t SterringWheelAngle_Range;						//方向盘转角传感器最大转动角度，左右各135°
	volatile uint16_t SterringWheel_ADMiddle;							//方向盘中间位置AD数字量
	volatile uint16_t SterringWheel_TurnRange;						//方向盘左右转动AD数字量变化范围值
	volatile uint16_t MotCtrllerTempThreshold_High;				//电机控制器温度升到60℃，开启水泵给电机控制器降温
	volatile uint16_t MotCtrllerTempThreshold_Low;				//电机控制器温度降到50℃，关闭水泵
	volatile uint16_t Line_Corners_TCSMode_Angle;					//当牵引力直线模式开关打开，且方向盘转角小于该角度，LineTCS生效，弯道模式开关打开，且方向盘转角大于该角度，CornersTCS生效
	volatile float DRS_SteeringWheel_Angle;								//当方向盘转动角度大于此值，打开DRS功能
	volatile float Close_DRS_AdjustAngle;									//DRS功能关闭时，舵机保持的角度
	volatile float Open_DRS_AdjustAngle;									//DRS功能打开后，舵机保持的角度
	volatile float TCS_PID_Kp;											 		 	//TCS Proportional Coefficient
	volatile float TCS_PID_Ki;														//TCS Integral Coefficient
	volatile float TCS_PID_Kd;														//TCS Differential Coefficient
	volatile float OptimalSlipRate;												//牵引力控制最优滑移率
	volatile uint16_t SuspensionLineDistF1_ADMiddle;			//前悬架线位移左的中间位置AD数据
	volatile uint16_t SuspensionLineDistF2_ADMiddle;			//前悬架线位移右的中间位置AD数据
	volatile uint16_t SuspensionLineDistB1_ADMiddle;			//后悬架线位移左的中间位置AD数据
	volatile uint16_t SuspensionLineDistB2_ADMiddle;			//后悬架线位移右的中间位置AD数据
}Wireless_ReceiveAndTrnsmit1Data;

struct {
	volatile float TCS_PID_OUT;
}TCS_PID_OUT_VIEW;

struct{
	volatile uint16_t SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting;			//前悬架线位移左的中间位置AD数据
	volatile uint16_t SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting;			//前悬架线位移右的中间位置AD数据
	volatile uint16_t SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting;			//后悬架线位移左的中间位置AD数据
	volatile uint16_t SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting;			//后悬架线位移右的中间位置AD数据
}SuspensionLineShift_ADMiddle_For_Key_ZeroSetting;
//发送给上位机的数据

osThreadId Wireless_ReceiveLabview_SendMemTaskHandle;
osThreadId EEPROMMemoryTaskHandle;
osThreadId EEPROMreadAndGenericDataTaskHandle;
osThreadId SendLabviewReadbackData_Period50TaskHandle;
osThreadId SendLabviewModule2Data_Period100TaskHandle;
osThreadId SendLabviewModule2Data_Period20TaskHandle;
osThreadId Wireless_Transmit1And2_LabviewTaskHandle;

osMessageQId EEPROMMemoryQueueHandle;																		//EEPROM存储内容队列

osMessageQId WirelessTransmit1And2_LabviewQueueHandle;									//Wireless发送内容1和2队列
osThreadId Get_WirelessTransmit1And2_LabviewQueueHandle(void){
	return WirelessTransmit1And2_LabviewQueueHandle;
}

void Wireless_ReceiveLabview_SendMemTask(void const * argument);
void EEPROMMemoryTask(void const * argument);
void EEPROMreadAndGenericDataTask(void const * argument);
void SendLabviewReadbackData_Period50Task(void const * argument);
void SendLabviewModule2Data_Period100Task(void const * argument);
void SendLabviewModule2Data_Period20Task(void const * argument);
void Wireless_Transmit1And2_LabviewTask(void const * argument);

void SendDataToEEPROM(uint8_t *Wireless_data,uint8_t Memory_ID);			//向EEPROM内却娣泡队列 投递数据函数
	
void Wireless_FREERTOS_Init(void)
{
	/* definition and creation of Wireless_ReceiveData_SendMemTask */
	osThreadDef(Wireless_ReceiveLabview_SendMemTask, Wireless_ReceiveLabview_SendMemTask, osPriorityNormal, 0, 128);
	Wireless_ReceiveLabview_SendMemTaskHandle = osThreadCreate(osThread(Wireless_ReceiveLabview_SendMemTask), NULL);
	/* definition and creation of EEPROMMemoryTask */
	osThreadDef(EEPROMMemoryTask, EEPROMMemoryTask, osPriorityNormal, 0, 128);
  EEPROMMemoryTaskHandle = osThreadCreate(osThread(EEPROMMemoryTask), NULL);	
	/* definition and creation of EEPROMreadAndGenericDataTask */
	osThreadDef(EEPROMreadAndGenericDataTask, EEPROMreadAndGenericDataTask, osPriorityNormal, 0, 128);
  EEPROMreadAndGenericDataTaskHandle = osThreadCreate(osThread(EEPROMreadAndGenericDataTask), NULL);	
	/* definition and creation of SendLabviewReadbackData_Period50Task */
	osThreadDef(SendLabviewReadbackData_Period50Task, SendLabviewReadbackData_Period50Task, osPriorityNormal, 0, 128);
	SendLabviewReadbackData_Period50TaskHandle = osThreadCreate(osThread(SendLabviewReadbackData_Period50Task), NULL);
	/* definition and creation of SendLabviewModule2Data_Period100Task */
	osThreadDef(SendLabviewModule2Data_Period100Task, SendLabviewModule2Data_Period100Task, osPriorityNormal, 0, 128);
	SendLabviewModule2Data_Period100TaskHandle = osThreadCreate(osThread(SendLabviewModule2Data_Period100Task), NULL);
	/* definition and creation of SendLabviewModule2Data_Period20Task */
	osThreadDef(SendLabviewModule2Data_Period20Task, SendLabviewModule2Data_Period20Task, osPriorityNormal, 0, 128);
	SendLabviewModule2Data_Period20TaskHandle = osThreadCreate(osThread(SendLabviewModule2Data_Period20Task), NULL);
	/* definition and creation of Wireless_Transmit1And2_LabviewTask */
	osThreadDef(Wireless_Transmit1And2_LabviewTask, Wireless_Transmit1And2_LabviewTask, osPriorityNormal, 0, 128);
	Wireless_Transmit1And2_LabviewTaskHandle = osThreadCreate(osThread(Wireless_Transmit1And2_LabviewTask), NULL);
	
	/* definition and creation of EEPROMMemoryQueue */
  osMessageQDef(EEPROMMemoryQueue, WirelessReceiveDataNum, EEPROMMemoryMessageTypedef);
  EEPROMMemoryQueueHandle = osMessageCreate(osMessageQ(EEPROMMemoryQueue), NULL);
	/* definition and creation of WirelessTransmitLabviewQueue */
  osMessageQDef(WirelessTransmit1And2_LabviewQueue, WirelessReceiveDataNum+WirelessTransmitData2Num, WirelessTransmit1And2_MessageTypedef);
  WirelessTransmit1And2_LabviewQueueHandle = osMessageCreate(osMessageQ(WirelessTransmit1And2_LabviewQueue), NULL);
} 

/*
*****************************************************************************
*@brief		无线接收数据解析函数
*@param		uint8_t Wireless_ReceiveData：传入接收到的数据
*@param		uint8_t *Analysis_Data：解析数据
*@retval	None
*@par
*****************************************************************************
*/
static uint8_t Wireless_ReceiveData_Analysis(uint8_t Wireless_ReceiveData,uint8_t *Analysis_Data)
{	
	static uint8_t WirelessRxCnt = 0;
	Analysis_Data[WirelessRxCnt]=Wireless_ReceiveData;	
	if (Analysis_Data[0]!=(uint8_t)(Transmit1AndReceiveLabviewHeader>>8)){
			WirelessRxCnt=0;
			return 0;
	}
	if(WirelessRxCnt==1){
		if (Analysis_Data[1]!=(uint8_t)Transmit1AndReceiveLabviewHeader){
			WirelessRxCnt=0;
			return 0;
		}
	}
	WirelessRxCnt++;
	if (WirelessRxCnt<5) {
		return 0;
	}
	else{
		WirelessRxCnt=0;
		return 1;
	}
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	if(huart->Instance==UART4){
		if(Wireless_ReceiveData_Analysis(WirelessReceice_Data_OneByte,WirelessAnalysis_Data)){	
			BaseType_t xHigherPriorityTaskWoken=pdFALSE;
			vTaskNotifyGiveFromISR(Wireless_ReceiveLabview_SendMemTaskHandle, &xHigherPriorityTaskWoken);		
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);				
		}	
		HAL_UART_Receive_IT(&huart4, &WirelessReceice_Data_OneByte, 1);			
	}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}

/* Wireless_ReceiveLabview_SendMemTask function */
void Wireless_ReceiveLabview_SendMemTask(void const * argument)
{	
  /* USER CODE BEGIN Wireless_ReceiveLabview_SendMemTask */
  /* Infinite loop */
	HAL_UART_Receive_IT(&huart4, &WirelessReceice_Data_OneByte, 1);	
	uint8_t receive_data[2];
  for(;;)
  {	
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		receive_data[0]=WirelessAnalysis_Data[3];
		receive_data[1]=WirelessAnalysis_Data[4];
		SendDataToEEPROM(receive_data,WirelessAnalysis_Data[2]);				
  }
  /* USER CODE END Wireless_ReceiveLabview_SendMemTask */
}

/*
*****************************************************************************
*@brief		向EEPROM投递数据函数
*@param		uint8_t *Wireless_data：传入Wireless接收到的数据
*@param		uint8_t Memory_ID：存储ID
*@retval	None
*@par
*****************************************************************************
*/
void SendDataToEEPROM(uint8_t *Wireless_data,uint8_t Memory_ID)
{
	EEPROMMemoryMessageTypedef EEPROMMemoryMessage;
	if(Memory_ID>=BrakePressure_Range_High_ID&&Memory_ID<=SuspensionLineDistB2_ADMiddle_ID&&~(Memory_ID%2)){
		EEPROMMemoryMessage.EEPROMMemoryID=Memory_ID;
		EEPROMMemoryMessage.EEPROMMemoryData[0]=*Wireless_data;
		EEPROMMemoryMessage.EEPROMMemoryData[1]=*(Wireless_data+1);
		xQueueSend(EEPROMMemoryQueueHandle,&EEPROMMemoryMessage,(TickType_t)0 );
	}
}

/* EEPROMMemoryTask function */
void EEPROMMemoryTask(void const * argument)
{	
  /* USER CODE BEGIN EEPROMMemoryTask */
  /* Infinite loop */
	if (AT24Cxx_Check()){  //检查器件
		/*发送EEPROM故障信息给显示屏*/	
	}
	EEPROMMemoryMessageTypedef EEPROMMemoryMessage;
  for(;;)
  {	
		if(xQueueReceive(EEPROMMemoryQueueHandle,&EEPROMMemoryMessage,(TickType_t)0)==pdTRUE){
			AT24Cxx_WriteOneByte(EEPROMMemoryMessage.EEPROMMemoryID,EEPROMMemoryMessage.EEPROMMemoryData[0]);
			AT24Cxx_WriteOneByte(EEPROMMemoryMessage.EEPROMMemoryID+1,EEPROMMemoryMessage.EEPROMMemoryData[1]);
			xTaskNotify(EEPROMreadAndGenericDataTaskHandle,(uint32_t)EEPROMMemoryMessage.EEPROMMemoryID,eSetValueWithOverwrite);		
		}		
    osDelay(10);
  }
  /* USER CODE END EEPROMMemoryTask */
}

/*
*****************************************************************************
*@brief		生成ECU1中要使用数值，数值来自Labview
*@param		uint8_t *Read_data：EEPROM中读出的数据
*@param		uint8_t Memory_ID：EEPROM中存储数字的ID
*@retval	None
*@par
*****************************************************************************
*/
static void DataFromLabviewGeneric(uint8_t *Read_data,uint8_t Memory_ID)
{
	if(Memory_ID==BrakePressure_Range_High_ID){
		Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_High=0.1*((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==BrakePressure_Range_Low_ID){
		Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_Low=0.1*((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==BrakePressure_Range_ActualHigh_ID){
		Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualHigh=0.1*((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==BrakePressure_Range_ActualLow_ID){
		Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualLow=0.1*((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==AccPeda_Range_High_ID){
		Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_High=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==AccPeda_Range_Low_ID){
		Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_Low=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==AccPeda_InitialDiff_Of_LandR_ID){
		Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR=((Read_data[0]<<8)+Read_data[1]);
	}	
	if(Memory_ID==SterringWheelAngle_Range_ID){
		Wireless_ReceiveAndTrnsmit1Data.SterringWheelAngle_Range=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==SterringWheel_ADMiddle_ID){
		Wireless_ReceiveAndTrnsmit1Data.SterringWheel_ADMiddle=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==SterringWheel_TurnRange_ID){
		Wireless_ReceiveAndTrnsmit1Data.SterringWheel_TurnRange=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==MotCtrllerTempThreshold_High_ID){
		Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_High=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==MotCtrllerTempThreshold_Low_ID){
		Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_Low=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==Line_Corners_TCSMode_Angle_ID){
		Wireless_ReceiveAndTrnsmit1Data.Line_Corners_TCSMode_Angle=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==DRS_SteeringWheel_Angle_ID){
		Wireless_ReceiveAndTrnsmit1Data.DRS_SteeringWheel_Angle=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==Close_DRS_AdjustAngle_ID){
		Wireless_ReceiveAndTrnsmit1Data.Close_DRS_AdjustAngle=((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==Open_DRS_AdjustAngle_ID){
		Wireless_ReceiveAndTrnsmit1Data.Open_DRS_AdjustAngle=((Read_data[0]<<8)+Read_data[1]);	
	}
	if(Memory_ID==TCS_PID_Kp_ID){
		Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kp=0.1*((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==TCS_PID_Ki_ID){
		Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Ki=0.1*((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==TCS_PID_Kd_ID){
		Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kd=0.1*((Read_data[0]<<8)+Read_data[1]);
	}
	if(Memory_ID==OptimalSlipRate_ID){
		Wireless_ReceiveAndTrnsmit1Data.OptimalSlipRate=0.01*((Read_data[0]<<8)+Read_data[1]);	
	}
	if(Memory_ID==SuspensionLineDistF1_ADMiddle_ID){
		Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF1_ADMiddle=(Read_data[0]<<8)+Read_data[1];	
	}
	if(Memory_ID==SuspensionLineDistF2_ADMiddle_ID){
		Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF2_ADMiddle=(Read_data[0]<<8)+Read_data[1];	
	}
	if(Memory_ID==SuspensionLineDistB1_ADMiddle_ID){
		Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB1_ADMiddle=(Read_data[0]<<8)+Read_data[1];	
	}
	if(Memory_ID==SuspensionLineDistB2_ADMiddle_ID){
		Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB2_ADMiddle=(Read_data[0]<<8)+Read_data[1];	
	}
}

/* EEPROMreadAndGenericDataTask function */
void EEPROMreadAndGenericDataTask(void const * argument)
{	
  /* USER CODE BEGIN EEPROMreadAndGenericDataTask */
  /* Infinite loop */
	if (AT24Cxx_Check()){  //检查器件
		/*发送EEPROM故障信息给显示屏*/	
	}
	uint8_t Memory_ID;
	uint8_t Read_Data[2];
  for(;;)
  {	
		xTaskNotifyWait( 0,0xFFFFFFFF,(uint32_t*)&Memory_ID,portMAX_DELAY);
		Read_Data[0]=AT24Cxx_ReadOneByte(Memory_ID);
		Read_Data[1]=AT24Cxx_ReadOneByte(Memory_ID+1);
		DataFromLabviewGeneric(Read_Data,Memory_ID);
	}
  /* USER CODE END EEPROMreadAndGenericDataTask */
}

/*
*****************************************************************************
*@brief		ECU1接收Labview回执发送数据协议化函数
*@param		uint8_t Labview_ID：Labview中接收数据的ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU1_Labview_Readback_Send(uint8_t Labview_ID)
{	
	WirelessTransmit1And2_MessageTypedef WirelessTransmitMessage;
	WirelessTransmitMessage.WirelessTransmitID=Labview_ID;
	if(Labview_ID==BrakePressure_Range_High_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_High*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_High*10;
	}
	if(Labview_ID==BrakePressure_Range_Low_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_Low*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_Low*10;
	}
	if(Labview_ID==BrakePressure_Range_ActualHigh_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualHigh*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualHigh*10;
	}	
	if(Labview_ID==BrakePressure_Range_ActualLow_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualLow*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualLow*10;
	}	
	if(Labview_ID==AccPeda_Range_High_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_High>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_High;
	}
	if(Labview_ID==AccPeda_Range_Low_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_Low>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_Low;
	}
	if(Labview_ID==AccPeda_InitialDiff_Of_LandR_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR;
	}
	if(Labview_ID==SterringWheelAngle_Range_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.SterringWheelAngle_Range>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.SterringWheelAngle_Range;
	}
	if(Labview_ID==SterringWheel_ADMiddle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.SterringWheel_ADMiddle>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.SterringWheel_ADMiddle;
	}
	if(Labview_ID==SterringWheel_TurnRange_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.SterringWheel_TurnRange>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.SterringWheel_TurnRange;
	}	 
	if(Labview_ID==MotCtrllerTempThreshold_High_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_High>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_High;
	}
	if(Labview_ID==MotCtrllerTempThreshold_Low_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_Low>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_Low;
	}
	if(Labview_ID==Line_Corners_TCSMode_Angle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.Line_Corners_TCSMode_Angle>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.Line_Corners_TCSMode_Angle;
	}
	if(Labview_ID==DRS_SteeringWheel_Angle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.DRS_SteeringWheel_Angle)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(uint16_t)Wireless_ReceiveAndTrnsmit1Data.DRS_SteeringWheel_Angle;
	}
	if(Labview_ID==Close_DRS_AdjustAngle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.Close_DRS_AdjustAngle)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(uint16_t)Wireless_ReceiveAndTrnsmit1Data.Close_DRS_AdjustAngle;
	}
	if(Labview_ID==Open_DRS_AdjustAngle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.Open_DRS_AdjustAngle)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(uint16_t)Wireless_ReceiveAndTrnsmit1Data.Open_DRS_AdjustAngle;
	}
	if(Labview_ID==TCS_PID_Kp_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kp*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kp*10;
	}
	if(Labview_ID==TCS_PID_Ki_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Ki*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Ki*10;
	}
	if(Labview_ID==TCS_PID_Kd_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kd*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kd*10;
	}
	if(Labview_ID==OptimalSlipRate_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Wireless_ReceiveAndTrnsmit1Data.OptimalSlipRate*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.OptimalSlipRate*100;
	}
	if(Labview_ID==SuspensionLineDistF1_ADMiddle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF1_ADMiddle>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF1_ADMiddle;
	}
	if(Labview_ID==SuspensionLineDistF2_ADMiddle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF2_ADMiddle>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF2_ADMiddle;
	}
	if(Labview_ID==SuspensionLineDistB1_ADMiddle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB1_ADMiddle>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB1_ADMiddle;
	}
	if(Labview_ID==SuspensionLineDistB2_ADMiddle_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB2_ADMiddle>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB2_ADMiddle;
	}
	xQueueSend(WirelessTransmit1And2_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );		
}

/*
*****************************************************************************
*@brief		ECU1发送Labview模块2数据协议化函数
*@param		uint8_t Labview_ID：Labview中接收数据的ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU1_Labview_Module2Data_Send(uint8_t Labview_ID)
{
	float BrakePressureFront_Display,BrakePressureBack_Display;
	uint16_t AccPedalLeft_Display,AccPedalRight_Display,SteeringWheelAngleAdValue_Display;
	float FrontWheelSpeed_Lm_Display,FrontWheelSpeed_Rm_Display,BackWheelSpeed_Lm_Display,BackWheelSpeed_Rm_Display,CurrentSlipRate_Display;
	uint16_t SuspensionLineDistF1_ADData,SuspensionLineDistF2_ADData,SuspensionLineDistB1_ADData,SuspensionLineDistB2_ADData;
	WirelessTransmit1And2_MessageTypedef WirelessTransmitMessage;
	WirelessTransmitMessage.WirelessTransmitID=Labview_ID;
		
	if(Labview_ID==BrakePressureFrontDisplay_ID){
		Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&BrakePressureFront_Display);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(BrakePressureFront_Display*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=BrakePressureFront_Display*10;
	}
	else if(Labview_ID==BrakePressureBackDisplay_ID){
		Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&BrakePressureBack_Display);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(BrakePressureBack_Display*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=BrakePressureBack_Display*10;
	}
	else if(Labview_ID==AccPedaLeftAdDigitalValueDisplay_ID){
		Get_Acc_Brake_Angle_LowVData(AccPedalLeft_n,&AccPedalLeft_Display);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(AccPedalLeft_Display+Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=AccPedalLeft_Display+Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR;
	}
	else if(Labview_ID==AccPedaRightAdDigitalValueDisplay_ID){
		Get_Acc_Brake_Angle_LowVData(AccPedalRight_n,&AccPedalRight_Display);		
		WirelessTransmitMessage.WirelessTransmitData[0]=AccPedalRight_Display>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=AccPedalRight_Display;
	}
	else if(Labview_ID==SterringWheelAdDigitalValueDisplay_ID){
		Get_Acc_Brake_Angle_LowVData(SteeringWheelAngleAdDigitalValue_n,&SteeringWheelAngleAdValue_Display);		
		WirelessTransmitMessage.WirelessTransmitData[0]=SteeringWheelAngleAdValue_Display>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=SteeringWheelAngleAdValue_Display;
	}
	else if(Labview_ID==WheelSpeedFrontDisplay_ID){
		Get_CAN_Analysis_Data(FrontWheelSpeed_Lm_n,&FrontWheelSpeed_Lm_Display);		
		Get_CAN_Analysis_Data(FrontWheelSpeed_Rm_n,&FrontWheelSpeed_Rm_Display);	
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((FrontWheelSpeed_Lm_Display+FrontWheelSpeed_Rm_Display)/2*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=((FrontWheelSpeed_Lm_Display+FrontWheelSpeed_Rm_Display)/2*10);
	}
	else if(Labview_ID==WheelSpeedBackDisplay_ID){
		Get_CAN_Analysis_Data(BackWheelSpeed_Lm_n,&BackWheelSpeed_Lm_Display);		
		Get_CAN_Analysis_Data(BackWheelSpeed_Rm_n,&BackWheelSpeed_Rm_Display);	
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((BackWheelSpeed_Lm_Display+BackWheelSpeed_Rm_Display)/2*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=((BackWheelSpeed_Lm_Display+BackWheelSpeed_Rm_Display)/2*10);
	}	
	else if(Labview_ID==CurrentSlipRateDisplay_ID){
		Get_CarSpeed_Mileage_SlipRate_Data(SlipRate_n,&CurrentSlipRate_Display);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((CurrentSlipRate_Display+10)*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(CurrentSlipRate_Display+10)*100;
	}		
	else if(Labview_ID==SuspensionLineDistF1_ADView_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDistF1_ADValue_n,&SuspensionLineDistF1_ADData);
		WirelessTransmitMessage.WirelessTransmitData[0]=SuspensionLineDistF1_ADData>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=SuspensionLineDistF1_ADData;
	}	
	else if(Labview_ID==SuspensionLineDistF2_ADView_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDistF2_ADValue_n,&SuspensionLineDistF2_ADData);
		WirelessTransmitMessage.WirelessTransmitData[0]=SuspensionLineDistF2_ADData>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=SuspensionLineDistF2_ADData;
	}	
	else if(Labview_ID==SuspensionLineDistB1_ADView_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDistB1_ADValue_n,&SuspensionLineDistB1_ADData);
		WirelessTransmitMessage.WirelessTransmitData[0]=SuspensionLineDistB1_ADData>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=SuspensionLineDistB1_ADData;
	}	
	else if(Labview_ID==SuspensionLineDistB2_ADView_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDistB2_ADValue_n,&SuspensionLineDistB2_ADData);
		WirelessTransmitMessage.WirelessTransmitData[0]=SuspensionLineDistB2_ADData>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=SuspensionLineDistB2_ADData;
	}	
	else if(Labview_ID==TCS_PID_OUT_View_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(10*(TCS_PID_OUT_VIEW.TCS_PID_OUT+3000))>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(uint16_t)(10*(TCS_PID_OUT_VIEW.TCS_PID_OUT+3000));
	}		
	xQueueSend(WirelessTransmit1And2_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );
}


/*
*****************************************************************************
*@brief		Wireless发送数据函数,发送模块1和2的数据
*@param		uint8_t Labview_ID：Labview中接收数据ID
*@retval	None
*@par
*****************************************************************************
*/
void Wireless_DATA_Send1And2(uint8_t Labview_ID)
{	
	if(Labview_ID>=BrakePressure_Range_High_ID&&Labview_ID<=SuspensionLineDistB2_ADMiddle_ID&&~(Labview_ID%2)){
		ECU1_Labview_Readback_Send(Labview_ID);
	}
	else if(Labview_ID>=BrakePressureFrontDisplay_ID&&Labview_ID<=TCS_PID_OUT_View_ID&&~(Labview_ID%2)){
		ECU1_Labview_Module2Data_Send(Labview_ID);
	}
	//get 其他的数据，协议化再发送，需另做函数
}

/* SendLabviewReadbackData_Period50Task function */
void SendLabviewReadbackData_Period50Task(void const * argument)
{	
  /* USER CODE BEGIN SendLabviewReadbackData_Period50Task */
  /* Infinite loop */
	uint8_t i=0;
	for(;;)
  {	
//		Wireless_DATA_Send1And2((uint8_t)i*2);
//		i++;
//		if(i==WirelessReceiveDataNum){
//			i=0;			
//		}
		osDelay(50);
  }
  /* USER CODE END SendLabviewReadbackData_Period50Task */
}

/* SendLabviewModule2Data_Period100Task function */
void SendLabviewModule2Data_Period100Task(void const * argument)
{	
  /* USER CODE BEGIN SendLabviewModule2Data_Period100Task */
  /* Infinite loop */
	for(;;)
  {	
//		for(uint8_t i=0;i<WirelessTransmitData2Num;i++)
//		{
//			if((BrakePressureFrontDisplay_ID+(uint8_t)i*2!=CurrentSlipRateDisplay_ID)&&(BrakePressureFrontDisplay_ID+(uint8_t)i*2!=TCS_PID_OUT_View_ID)){
//				Wireless_DATA_Send1And2(BrakePressureFrontDisplay_ID+(uint8_t)i*2);
//			}
//		}		
		osDelay(100);
  }
  /* USER CODE END SendLabviewModule2Data_Period100Task */
}

/* SendLabviewModule2Data_Period20Task function */
void SendLabviewModule2Data_Period20Task(void const * argument)
{	
  /* USER CODE BEGIN SendLabviewModule2Data_Period20Task */
  /* Infinite loop */
	for(;;)
  {	
//		Wireless_DATA_Send1And2(CurrentSlipRateDisplay_ID);
//		Wireless_DATA_Send1And2(TCS_PID_OUT_View_ID);
		osDelay(20);
  }
  /* USER CODE END SendLabviewModule2Data_Period20Task */
}

/* Wireless_Transmit1And2_LabviewTask function */
void Wireless_Transmit1And2_LabviewTask(void const * argument)
{	
  /* USER CODE BEGIN Wireless_Transmit1And2_LabviewTask */
  /* Infinite loop */
	WirelessTransmit1And2_MessageTypedef WirelessTransmit_Message;
	uint8_t WirelessTransmit_Data[5];	
  for(;;)
  {	
		if(xQueueReceive(WirelessTransmit1And2_LabviewQueueHandle,&WirelessTransmit_Message,(TickType_t)0)==pdTRUE){
			WirelessTransmit_Data[2]=WirelessTransmit_Message.WirelessTransmitID;
			if(WirelessTransmit_Data[2]>=BrakePressure_Range_High_ID&&WirelessTransmit_Data[2]<=SuspensionLineDistB2_ADMiddle_ID&&~(WirelessTransmit_Data[2]%2)){
				WirelessTransmit_Data[0]=Transmit1AndReceiveLabviewHeader>>8;
				WirelessTransmit_Data[1]=(uint8_t)Transmit1AndReceiveLabviewHeader;
			}
			else if(WirelessTransmit_Data[2]>=BrakePressureFrontDisplay_ID&&WirelessTransmit_Data[2]<=SuspensionLineDistB2_ADView_ID&&~(WirelessTransmit_Data[2]%2)){
				WirelessTransmit_Data[0]=TransmitLabviewModule2Header>>8;
				WirelessTransmit_Data[1]=(uint8_t)TransmitLabviewModule2Header;
			}			
			WirelessTransmit_Data[3]=WirelessTransmit_Message.WirelessTransmitData[0];
			WirelessTransmit_Data[4]=WirelessTransmit_Message.WirelessTransmitData[1];
			HAL_UART_Transmit_DMA(&huart4,WirelessTransmit_Data,5);
		}
		osDelay(2);
  }
  /* USER CODE END Wireless_Transmit1And2_LabviewTask */
}

/*
*****************************************************************************
*@brief		取值函数，搬运Wireless接收并解析好的数据
*@param		Wireless_ReceiveData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Wireless_DataFrom_EEPROM(Wireless_ReceiveData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	switch(name_num)
	{
		case BrakePressure_Range_High_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_High;
			break;
		case BrakePressure_Range_Low_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_Low;
			break;
		case BrakePressure_Range_ActualHigh_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualHigh;
			break;
		case BrakePressure_Range_ActualLow_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualLow;
			break;		
		case AccPeda_Range_High_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_High;
			break;
		case AccPeda_Range_Low_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_Low;
			break;
		case AccPeda_InitialDiff_Of_LandR_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR;
			break;		
		case SterringWheelAngle_Range_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.SterringWheelAngle_Range;
			break;
		case SterringWheel_ADMiddle_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.SterringWheel_ADMiddle;
			break;		
		case SterringWheel_TurnRange_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.SterringWheel_TurnRange;
			break;
		case MotCtrllerTempThreshold_High_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_High;
			break;
		case MotCtrllerTempThreshold_Low_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_Low;
			break;
		case Line_Corners_TCSMode_Angle_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.Line_Corners_TCSMode_Angle;
			break;
		case DRS_SteeringWheel_Angle_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.DRS_SteeringWheel_Angle;
			break;
		case Close_DRS_AdjustAngle_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.Close_DRS_AdjustAngle;
			break;
		case Open_DRS_AdjustAngle_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.Open_DRS_AdjustAngle;
			break;
		case TCS_PID_Kp_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kp;
			break;
		case TCS_PID_Ki_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Ki;
			break;
		case TCS_PID_Kd_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kd;
			break;
		case OptimalSlipRate_n:
			*(float*)extern_data=Wireless_ReceiveAndTrnsmit1Data.OptimalSlipRate;
			break;	
		case SuspensionLineDistF1_ADMiddle_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF1_ADMiddle;
			break;	
		case SuspensionLineDistF2_ADMiddle_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF2_ADMiddle;
			break;	
		case SuspensionLineDistB1_ADMiddle_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB1_ADMiddle;
			break;	
		case SuspensionLineDistB2_ADMiddle_n:
			*(uint16_t*)extern_data=Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB2_ADMiddle;
			break;	
	}
}

void Get_TCS_PID_OUT_Data(Wireless_ReceiveData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==TCS_PID_OUT_n)
	{
		*(float*)extern_data=TCS_PID_OUT_VIEW.TCS_PID_OUT;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运 悬架线位移调零时，悬架线位移所处位置的AD值
*@param		SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num：获取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	switch(name_num)
	{
		case SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_n:
			*(uint16_t*)extern_data=SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting;
			break;
		case SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_n:
			*(uint16_t*)extern_data=SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting;
			break;
		case SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_n:
			*(uint16_t*)extern_data=SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting;
			break;
		case SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_n:
			*(uint16_t*)extern_data=SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting;
			break;		
	}	
}

/*
*****************************************************************************
*@brief		修改Wireless接收并解析好的数据
*@param		Wireless_ReceiveData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Wireless_DataFrom_EEPROM(Wireless_ReceiveData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	switch(name_num)
	{
		case BrakePressure_Range_High_n:
			Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_High=*(float*)extern_data;
			break;
		case BrakePressure_Range_Low_n:
			Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_Low=*(float*)extern_data;
			break;
		case BrakePressure_Range_ActualHigh_n:
			Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualHigh=*(float*)extern_data;
			break;
		case BrakePressure_Range_ActualLow_n:
			Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualLow=*(float*)extern_data;
			break;
		case AccPeda_Range_High_n:
			Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_High=*(uint16_t*)extern_data;
			break;
		case AccPeda_Range_Low_n:
			Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_Low=*(uint16_t*)extern_data;
			break;
		case AccPeda_InitialDiff_Of_LandR_n:
			Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR=*(uint16_t*)extern_data;
			break;
		case SterringWheelAngle_Range_n:
			Wireless_ReceiveAndTrnsmit1Data.SterringWheelAngle_Range=*(uint16_t*)extern_data;
			break;
		case SterringWheel_ADMiddle_n:
			Wireless_ReceiveAndTrnsmit1Data.SterringWheel_ADMiddle=*(uint16_t*)extern_data;
			break;		
		case SterringWheel_TurnRange_n:
			Wireless_ReceiveAndTrnsmit1Data.SterringWheel_TurnRange=*(uint16_t*)extern_data;
			break;			
		case MotCtrllerTempThreshold_High_n:
			Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_High=*(uint16_t*)extern_data;
			break;
		case MotCtrllerTempThreshold_Low_n:
			Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_Low=*(uint16_t*)extern_data;
			break;
		case Line_Corners_TCSMode_Angle_n:
			Wireless_ReceiveAndTrnsmit1Data.Line_Corners_TCSMode_Angle=*(uint16_t*)extern_data;
			break;
		case DRS_SteeringWheel_Angle_n:
			Wireless_ReceiveAndTrnsmit1Data.DRS_SteeringWheel_Angle=*(float*)extern_data;
			break;
		case Close_DRS_AdjustAngle_n:
			Wireless_ReceiveAndTrnsmit1Data.Close_DRS_AdjustAngle=*(float*)extern_data;
			break;
		case Open_DRS_AdjustAngle_n:
			Wireless_ReceiveAndTrnsmit1Data.Open_DRS_AdjustAngle=*(float*)extern_data;
			break;
		case TCS_PID_Kp_n:
			Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kp=*(float*)extern_data;
			break;
		case TCS_PID_Ki_n:
			Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Ki=*(float*)extern_data;
			break;
		case TCS_PID_Kd_n:
			Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kd=*(float*)extern_data;
			break;
		case OptimalSlipRate_n:
			Wireless_ReceiveAndTrnsmit1Data.OptimalSlipRate=*(float*)extern_data;
			break;		
		case SuspensionLineDistF1_ADMiddle_n:
			Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF1_ADMiddle=*(uint16_t*)extern_data;
			break;
		case SuspensionLineDistF2_ADMiddle_n:
			Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF2_ADMiddle=*(uint16_t*)extern_data;
			break;
		case SuspensionLineDistB1_ADMiddle_n:
			Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB1_ADMiddle=*(uint16_t*)extern_data;
			break;
		case SuspensionLineDistB2_ADMiddle_n:
			Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB2_ADMiddle=*(uint16_t*)extern_data;
			break;
	}
}

void Set_TCS_PID_OUT_Data(Wireless_ReceiveData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==TCS_PID_OUT_n)
	{
		TCS_PID_OUT_VIEW.TCS_PID_OUT=*(float*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改函数，修改 悬架线位移调零时，悬架线位移所处位置的AD值
*@param		SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num：获取数据对应的枚举变量
*@param		void*extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	switch(name_num)
	{
		case SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_n:
			SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting=*(uint16_t*)extern_data;
			break;
		case SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_n:
			SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting=*(uint16_t*)extern_data;
			break;
		case SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_n:
			SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting=*(uint16_t*)extern_data;
			break;
		case SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_n:
			SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting=*(uint16_t*)extern_data;
			break;		
	}	
}

/*
*****************************************************************************
*@brief		EEPROM数据清除函数，只运行一次
*@param		None
*@retval	None
*@par
*****************************************************************************
*/
static void CLEARData_For_EEPROM(void)
{
	for(uint8_t i=0;i<2*WirelessReceiveDataNum;i++)
	{
		AT24Cxx_WriteOneByte(i,0);
	}
	AT24Cxx_WriteOneByte(Mileage_ID,0);
	AT24Cxx_WriteOneByte(Mileage_ID+1,0);
	AT24Cxx_WriteOneByte(LeakageCurrentLimit_State_Data_ID,0);
	for(uint8_t i=0;i<4;i++)
	{
		AT24Cxx_WriteOneByte(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID+2*i,0);
		AT24Cxx_WriteOneByte(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID+2*i+1,0);
	}
}

/*
*****************************************************************************
*@brief		初次上电读EEPROM数据函数
*@param		None
*@retval	None
*@par
*****************************************************************************
*/
void FirstPowerOnReadEEPROMInit(void)
{
//	CLEARData_For_EEPROM();
	float Mileage_Init=0;
	uint8_t LeakageCurrentLimit_Data=0;
	Mileage_Init=((AT24Cxx_ReadOneByte(Mileage_ID)<<8)+AT24Cxx_ReadOneByte(Mileage_ID+1))/10.0;
	Set_CarSpeed_Mileage_SlipRate_Data(Mileage_n,&Mileage_Init);
	LeakageCurrentLimit_Data=AT24Cxx_ReadOneByte(LeakageCurrentLimit_State_Data_ID);
	Set_CAN_Analysis_Data(LeakageCurrentLimit_n,&LeakageCurrentLimit_Data);
	Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_High=0.1*((AT24Cxx_ReadOneByte(BrakePressure_Range_High_ID)<<8)+AT24Cxx_ReadOneByte(BrakePressure_Range_High_ID+1));
	Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_Low=0.1*((AT24Cxx_ReadOneByte(BrakePressure_Range_Low_ID)<<8)+AT24Cxx_ReadOneByte(BrakePressure_Range_Low_ID+1));
	Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualHigh=0.1*((AT24Cxx_ReadOneByte(BrakePressure_Range_ActualHigh_ID)<<8)+AT24Cxx_ReadOneByte(BrakePressure_Range_ActualHigh_ID+1));
	Wireless_ReceiveAndTrnsmit1Data.BrakePressure_Range_ActualLow=0.1*((AT24Cxx_ReadOneByte(BrakePressure_Range_ActualLow_ID)<<8)+AT24Cxx_ReadOneByte(BrakePressure_Range_ActualLow_ID+1));
	Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_High=(AT24Cxx_ReadOneByte(AccPeda_Range_High_ID)<<8)+AT24Cxx_ReadOneByte(AccPeda_Range_High_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.AccPeda_Range_Low=(AT24Cxx_ReadOneByte(AccPeda_Range_Low_ID)<<8)+AT24Cxx_ReadOneByte(AccPeda_Range_Low_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.AccPeda_InitialDiff_Of_LandR=(AT24Cxx_ReadOneByte(AccPeda_InitialDiff_Of_LandR_ID)<<8)+AT24Cxx_ReadOneByte(AccPeda_InitialDiff_Of_LandR_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.SterringWheelAngle_Range=(AT24Cxx_ReadOneByte(SterringWheelAngle_Range_ID)<<8)+AT24Cxx_ReadOneByte(SterringWheelAngle_Range_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.SterringWheel_ADMiddle=(AT24Cxx_ReadOneByte(SterringWheel_ADMiddle_ID)<<8)+AT24Cxx_ReadOneByte(SterringWheel_ADMiddle_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.SterringWheel_TurnRange=(AT24Cxx_ReadOneByte(SterringWheel_TurnRange_ID)<<8)+AT24Cxx_ReadOneByte(SterringWheel_TurnRange_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_High=(AT24Cxx_ReadOneByte(MotCtrllerTempThreshold_High_ID)<<8)+AT24Cxx_ReadOneByte(MotCtrllerTempThreshold_High_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.MotCtrllerTempThreshold_Low=(AT24Cxx_ReadOneByte(MotCtrllerTempThreshold_Low_ID)<<8)+AT24Cxx_ReadOneByte(MotCtrllerTempThreshold_Low_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.Line_Corners_TCSMode_Angle=(AT24Cxx_ReadOneByte(Line_Corners_TCSMode_Angle_ID)<<8)+AT24Cxx_ReadOneByte(Line_Corners_TCSMode_Angle_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.DRS_SteeringWheel_Angle=(AT24Cxx_ReadOneByte(DRS_SteeringWheel_Angle_ID)<<8)+AT24Cxx_ReadOneByte(DRS_SteeringWheel_Angle_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.Close_DRS_AdjustAngle=(AT24Cxx_ReadOneByte(Close_DRS_AdjustAngle_ID)<<8)+AT24Cxx_ReadOneByte(Close_DRS_AdjustAngle_ID+1);
	Wireless_ReceiveAndTrnsmit1Data.Open_DRS_AdjustAngle=(AT24Cxx_ReadOneByte(Open_DRS_AdjustAngle_ID)<<8)+AT24Cxx_ReadOneByte(Open_DRS_AdjustAngle_ID+1);	
	Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kp=0.1*((AT24Cxx_ReadOneByte(TCS_PID_Kp_ID)<<8)+AT24Cxx_ReadOneByte(TCS_PID_Kp_ID+1));
	Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Ki=0.1*((AT24Cxx_ReadOneByte(TCS_PID_Ki_ID)<<8)+AT24Cxx_ReadOneByte(TCS_PID_Ki_ID+1));
	Wireless_ReceiveAndTrnsmit1Data.TCS_PID_Kd=0.1*((AT24Cxx_ReadOneByte(TCS_PID_Kd_ID)<<8)+AT24Cxx_ReadOneByte(TCS_PID_Kd_ID+1));
	Wireless_ReceiveAndTrnsmit1Data.OptimalSlipRate=0.01*((AT24Cxx_ReadOneByte(OptimalSlipRate_ID)<<8)+AT24Cxx_ReadOneByte(OptimalSlipRate_ID+1));		
	Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF1_ADMiddle=(AT24Cxx_ReadOneByte(SuspensionLineDistF1_ADMiddle_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineDistF1_ADMiddle_ID+1);	
	Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistF2_ADMiddle=(AT24Cxx_ReadOneByte(SuspensionLineDistF2_ADMiddle_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineDistF2_ADMiddle_ID+1);	
	Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB1_ADMiddle=(AT24Cxx_ReadOneByte(SuspensionLineDistB1_ADMiddle_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineDistB1_ADMiddle_ID+1);	
	Wireless_ReceiveAndTrnsmit1Data.SuspensionLineDistB2_ADMiddle=(AT24Cxx_ReadOneByte(SuspensionLineDistB2_ADMiddle_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineDistB2_ADMiddle_ID+1);	
	SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting=(AT24Cxx_ReadOneByte(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID+1);
	SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting=(AT24Cxx_ReadOneByte(SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_ID+1);
	SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting=(AT24Cxx_ReadOneByte(SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_ID+1);
	SuspensionLineShift_ADMiddle_For_Key_ZeroSetting.SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting=(AT24Cxx_ReadOneByte(SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_ID)<<8)+AT24Cxx_ReadOneByte(SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_ID+1);
}

