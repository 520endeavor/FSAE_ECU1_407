#include "HostPCDisplay.h"

osThreadId SendLabviewModule3Data_Period100TaskHandle;
osThreadId Wireless_Transmit3_LabviewTaskHandle;

osMessageQId WirelessTransmit3_LabviewQueueHandle;									//Wireless发送内容队列

void SendLabviewModule3Data_Period100Task(void const * argument);
void Wireless_Transmit3_LabviewTask(void const * argument);

void WirelessModule2_FREERTOS_Init(void)
{	
	/* definition and creation of SendLabviewModule2Data_Period100Task */
	osThreadDef(SendLabviewModule3Data_Period100Task, SendLabviewModule3Data_Period100Task, osPriorityNormal, 0, 128);
	SendLabviewModule3Data_Period100TaskHandle = osThreadCreate(osThread(SendLabviewModule3Data_Period100Task), NULL);
	/* definition and creation of Wireless_Transmit3_LabviewTask */
	osThreadDef(Wireless_Transmit3_LabviewTask, Wireless_Transmit3_LabviewTask, osPriorityNormal, 0, 128);
	Wireless_Transmit3_LabviewTaskHandle = osThreadCreate(osThread(Wireless_Transmit3_LabviewTask), NULL);
	
	/* definition and creation of WirelessTransmit3_LabviewQueue */
  osMessageQDef(WirelessTransmit3_LabviewQueue, WirelessTransmitData3Num, WirelessTransmit3_MessageTypedef);
  WirelessTransmit3_LabviewQueueHandle = osMessageCreate(osMessageQ(WirelessTransmit3_LabviewQueue), NULL);
} 

/*
*****************************************************************************
*@brief		ECU1发送Labview模块3数据协议化函数
*@param		uint8_t Labview_ID：Labview中接收数据的ID
*@retval	None
*@par
*****************************************************************************
*/
void ECU1_Labview_Module3Data_Send(uint8_t Labview_ID)
{
	WirelessTransmit3_MessageTypedef WirelessTransmitMessage;
	WirelessTransmitMessage.WirelessTransmitID=Labview_ID;		
	uint8_t AccPedalL_Open,AccPedalR_Open,MaxV_Num,MinV_Num,MaxT_Num,MinT_Num,CarSpeed;	
	uint16_t MaxVoltage,MinVoltage;
	float BrakePressureF,BrakePressureB,SteeringWheelAngle,CarMileage,PowerBatTotalCurrent,PowerBatTotalVoltage,LowVoltageBatV,MaxTemperature,MinTemperature;
	float SuspensionLineDistF1,SuspensionLineDistF2,SuspensionLineDistB1,SuspensionLineDistB2;
	float MotIGBTTemperature_Data,MotAiremperature_Data,GPSLongitude_Data,GPSLatitude_Data,GPSAltitude_Data;
	float WheelSpeed_Front1,WheelSpeed_Front2,WheelSpeed_Back1,WheelSpeed_Back2;
	uint8_t WaterPumpStatus_Data,SpeakerStatus_Data,TaillightStatus_Data;	
	if((Labview_ID>=AccPedalLeft_Open_ID&&Labview_ID<=MinTemperatureNum_ID)||(Labview_ID>=WaterPumpStatus_ID&&Labview_ID<=TCSStatus_ID)){
		WirelessTransmitMessage.WirelessTransmitData[1]=0;
	}
	//开辟单字节区域
	if(Labview_ID==AccPedalLeft_Open_ID){		
		Get_AccPeda_Open_Data(AccPedalLeft_Open_n,&AccPedalL_Open);
		WirelessTransmitMessage.WirelessTransmitData[0]=AccPedalL_Open;
	}
	else if(Labview_ID==AccPedalRight_Open_ID){
		Get_AccPeda_Open_Data(AccPedalRight_Open_n,&AccPedalR_Open);
		WirelessTransmitMessage.WirelessTransmitData[0]=AccPedalR_Open;
	}
	else if(Labview_ID==BrakePressureFront_ID){
		Get_Acc_Brake_Angle_LowVData(BrakePressureFront_n,&BrakePressureF);
		WirelessTransmitMessage.WirelessTransmitData[0]=BrakePressureF*10;
	}
	else if(Labview_ID==BrakePressureBack_ID){
		Get_Acc_Brake_Angle_LowVData(BrakePressureBack_n,&BrakePressureB);
		WirelessTransmitMessage.WirelessTransmitData[0]=BrakePressureB*10;
	}
	else if(Labview_ID==MaxVoltageNum_ID){
		Get_CAN_Analysis_Data(MaxSingleVoltage_NUM_n,&MaxV_Num);
		WirelessTransmitMessage.WirelessTransmitData[0]=MaxV_Num;
	}
	else if(Labview_ID==MinVoltageNum_ID){
		Get_CAN_Analysis_Data(MinSingleVoltage_NUM_n,&MinV_Num);
		WirelessTransmitMessage.WirelessTransmitData[0]=MinV_Num;
	}
	else if(Labview_ID==MaxTemperatureNum_ID){
		Get_CAN_Analysis_Data(MaxSingleTemperature_NUM_n,&MaxT_Num);
		WirelessTransmitMessage.WirelessTransmitData[0]=MaxT_Num;		
	}
	else if(Labview_ID==MinTemperatureNum_ID){
		Get_CAN_Analysis_Data(MinSingleTemperature_NUM_n,&MinT_Num);
		WirelessTransmitMessage.WirelessTransmitData[0]=MinT_Num;		
	}
	//开辟两字节区域
	else if(Labview_ID==SterringWheelAngle_ID){
		Get_Acc_Brake_Angle_LowVData(SteeringWheelAngle_n,&SteeringWheelAngle);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(SteeringWheelAngle+135)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=SteeringWheelAngle+135;		
	}
	else if(Labview_ID==CarSpeed_ID){
		Get_CarSpeed_Mileage_SlipRate_Data(CarSpeed_n,&CarSpeed);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(CarSpeed*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=CarSpeed*10;		
	}
	else if(Labview_ID==CarMileage_ID){
		Get_CarSpeed_Mileage_SlipRate_Data(Mileage_n,&CarMileage);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(CarMileage*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=CarMileage*10;		
	}
	else if(Labview_ID==PowerBatTotalCurrent_ID){
		Get_CAN_Analysis_Data(BatteryTotal_I_n,&PowerBatTotalCurrent);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((PowerBatTotalCurrent+1000)*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(PowerBatTotalCurrent+1000)*10;		
	}
	else if(Labview_ID==PowerBatTotalVoltage_ID){
		Get_CAN_Analysis_Data(BatteryTotal_V_n,&PowerBatTotalVoltage);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(PowerBatTotalVoltage*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=PowerBatTotalVoltage*10;		
	}
	else if(Labview_ID==MaxVoltage_ID){
		Get_CAN_Analysis_Data(MaxSingleVoltage_n,&MaxVoltage);
		WirelessTransmitMessage.WirelessTransmitData[0]=MaxVoltage>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=MaxVoltage;		
	}		
	else if(Labview_ID==MinVoltage_ID){
		Get_CAN_Analysis_Data(MinSingleVoltage_n,&MinVoltage);
		WirelessTransmitMessage.WirelessTransmitData[0]=MinVoltage>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=MinVoltage;		
	}	
	else if(Labview_ID==LowVoltageBat_Voltage_ID){
		Get_Acc_Brake_Angle_LowVData(LowVoltageBatV_n,&LowVoltageBatV);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(LowVoltageBatV*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=LowVoltageBatV*10;		
	}	
	else if(Labview_ID==MaxTemperature_ID){
		Get_CAN_Analysis_Data(MaxSingleTemperature_n,&MaxTemperature);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((MaxTemperature+40)*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(MaxTemperature+40)*10;		
	}
	else if(Labview_ID==MinTemperature_ID){
		Get_CAN_Analysis_Data(MinSingleTemperature_n,&MinTemperature);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((MinTemperature+40)*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(MinTemperature+40)*10;		
	}
	else if(Labview_ID==MotIGBTTemperature_ID){
		Get_CAN_Analysis_Data(IGBT_Temperature_n,&MotIGBTTemperature_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(MotIGBTTemperature_Data*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=MotIGBTTemperature_Data*10;		
	}
	else if(Labview_ID==MotAiremperature_ID){
		Get_CAN_Analysis_Data(Air_Temperature_n,&MotAiremperature_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(MotAiremperature_Data*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=MotAiremperature_Data*10;		
	}
	else if(Labview_ID==GPSLongitudeData_ID){
		Get_NEO_M8N_GPRMC_AnalysisData(GPS_Longitude_n,&GPSLongitude_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((GPSLongitude_Data+180)*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(GPSLongitude_Data+180)*10;		
	}
	else if(Labview_ID==GPSLatitudeData_ID){
		Get_NEO_M8N_GPRMC_AnalysisData(GPS_Latitude_n,&GPSLatitude_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((GPSLatitude_Data+90)*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=(GPSLatitude_Data+90)*10;		
	}	
	else if(Labview_ID==GPSAltitudeData_ID){
		Get_NEO_M8N_GPRMC_AnalysisData(GPS_Altitude_n,&GPSAltitude_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(GPSAltitude_Data*10)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=GPSAltitude_Data*10;		
	}
	else if(Labview_ID==SuspensionLineDisplacementF1_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDisplacementF1_n,&SuspensionLineDistF1);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((SuspensionLineDistF1+50)*100)>>8;		
		WirelessTransmitMessage.WirelessTransmitData[1]=(SuspensionLineDistF1+50)*100;
	}
	else if(Labview_ID==SuspensionLineDisplacementF2_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDisplacementF2_n,&SuspensionLineDistF2);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((SuspensionLineDistF2+50)*100)>>8;		
		WirelessTransmitMessage.WirelessTransmitData[1]=(SuspensionLineDistF2+50)*100;
	}
	else if(Labview_ID==SuspensionLineDisplacementB1_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDisplacementB1_n,&SuspensionLineDistB1);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((SuspensionLineDistB1+50)*100)>>8;	
		WirelessTransmitMessage.WirelessTransmitData[1]=(SuspensionLineDistB1+50)*100;		
	}	
	else if(Labview_ID==SuspensionLineDisplacementB2_ID){
		Get_SuspensionLineDisplacementData(SuspensionLineDisplacementB2_n,&SuspensionLineDistB2);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)((SuspensionLineDistB2+50)*100)>>8;	
		WirelessTransmitMessage.WirelessTransmitData[1]=(SuspensionLineDistB2+50)*100;		
	}	
	else if(Labview_ID==WheelSpeedF1_ID){
		Get_CAN_Analysis_Data(FrontWheelSpeed_Lm_n,&WheelSpeed_Front1);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(WheelSpeed_Front1*1000)>>8;	
		WirelessTransmitMessage.WirelessTransmitData[1]=WheelSpeed_Front1*1000;		
	}
	else if(Labview_ID==WheelSpeedF2_ID){
		Get_CAN_Analysis_Data(FrontWheelSpeed_Rm_n,&WheelSpeed_Front2);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(WheelSpeed_Front2*1000)>>8;	
		WirelessTransmitMessage.WirelessTransmitData[1]=WheelSpeed_Front2*1000;		
	}
	else if(Labview_ID==WheelSpeedB1_ID){
		Get_CAN_Analysis_Data(BackWheelSpeed_Lm_n,&WheelSpeed_Back1);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(WheelSpeed_Back1*1000)>>8;	
		WirelessTransmitMessage.WirelessTransmitData[1]=WheelSpeed_Back1*1000;		
	}
	else if(Labview_ID==WheelSpeedB2_ID){
		Get_CAN_Analysis_Data(BackWheelSpeed_Rm_n,&WheelSpeed_Back2);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(WheelSpeed_Back2*1000)>>8;	
		WirelessTransmitMessage.WirelessTransmitData[1]=WheelSpeed_Back2*1000;		
	}	
	//新增单字节区域
	else if(Labview_ID==WaterPumpStatus_ID){
		Get_Fan_Speaker_Taillight_Pump_CtrlData(WaterpumpCtrl_n,&WaterPumpStatus_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=WaterPumpStatus_Data;	
	}
	else if(Labview_ID==SpeakerStatus_ID){
		Get_Fan_Speaker_Taillight_Pump_CtrlData(SpeakerCtrl_n,&SpeakerStatus_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=SpeakerStatus_Data;
	}
	else if(Labview_ID==TaillightStatus_ID){
		Get_Fan_Speaker_Taillight_Pump_CtrlData(TaillightCtrl_n,&TaillightStatus_Data);
		WirelessTransmitMessage.WirelessTransmitData[0]=TaillightStatus_Data;
	}
	else if(Labview_ID==TCSStatus_ID){
		uint8_t LineTCSStatus,CornersTCSStatus;
		Get_Line_CornersTCSState_Data(LineTCSState_n,&LineTCSStatus);
		Get_Line_CornersTCSState_Data(CornersTCSState_n,&CornersTCSStatus);
		WirelessTransmitMessage.WirelessTransmitData[0]=LineTCSStatus+CornersTCSStatus;
	}				
	if((Labview_ID>=AccPedalLeft_Open_ID&&Labview_ID<=MinTemperatureNum_ID)||(Labview_ID>=SterringWheelAngle_ID&&Labview_ID<=WheelSpeedB2_ID)||(Labview_ID>=WaterPumpStatus_ID&&Labview_ID<=TCSStatus_ID)){
		xQueueSend(WirelessTransmit3_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );
	}
}


/* SendLabviewModule3Data_Period100Task function */
void SendLabviewModule3Data_Period100Task(void const * argument)
{	
  /* USER CODE BEGIN SendLabviewModule3Data_Period100Task */
  /* Infinite loop */
	for(;;)
  {	
		//开辟单字节区域
		for(uint8_t i=0;i<=MinTemperatureNum_ID-AccPedalLeft_Open_ID;i++)
		{
			ECU1_Labview_Module3Data_Send(AccPedalLeft_Open_ID+i);
		}		
		//开辟两字节区域
		for(uint8_t i=0;i<=WheelSpeedB2_ID-SterringWheelAngle_ID;i++)
		{
			ECU1_Labview_Module3Data_Send(SterringWheelAngle_ID+i);
		}	
		//新增单字节区域
		for(uint8_t i=0;i<=TCSStatus_ID-WaterPumpStatus_ID;i++)
		{
			ECU1_Labview_Module3Data_Send(WaterPumpStatus_ID+i);
		}	
		osDelay(100);
  }
  /* USER CODE END SendLabviewModule3Data_Period100Task */
}

/* Wireless_Transmit3_LabviewTask function */
void Wireless_Transmit3_LabviewTask(void const * argument)
{	
  /* USER CODE BEGIN Wireless_Transmit3_LabviewTask */
  /* Infinite loop */
	WirelessTransmit3_MessageTypedef WirelessTransmit_Message;
	uint8_t WirelessTransmit_1ByteData[3];	
	uint8_t WirelessTransmit_2ByteData[4];
	WirelessTransmit_1ByteData[0]=TransmitLabviewModule3Header;
	WirelessTransmit_2ByteData[0]=TransmitLabviewModule3Header;
  for(;;)
  {	
		if(xQueueReceive(WirelessTransmit3_LabviewQueueHandle,&WirelessTransmit_Message,(TickType_t)0)==pdTRUE){
			if((WirelessTransmit_Message.WirelessTransmitID>=AccPedalLeft_Open_ID&&WirelessTransmit_Message.WirelessTransmitID<=MinTemperatureNum_ID)||(WirelessTransmit_Message.WirelessTransmitID>=WaterPumpStatus_ID&&WirelessTransmit_Message.WirelessTransmitID<=TCSStatus_ID)){
				WirelessTransmit_1ByteData[1]=WirelessTransmit_Message.WirelessTransmitID;
				WirelessTransmit_1ByteData[2]=WirelessTransmit_Message.WirelessTransmitData[0];
				HAL_UART_Transmit_DMA(&huart4,WirelessTransmit_1ByteData,3);
			}
			else if(WirelessTransmit_Message.WirelessTransmitID>=SterringWheelAngle_ID&&WirelessTransmit_Message.WirelessTransmitID<=WheelSpeedB2_ID){
				WirelessTransmit_2ByteData[1]=WirelessTransmit_Message.WirelessTransmitID;
				WirelessTransmit_2ByteData[2]=WirelessTransmit_Message.WirelessTransmitData[0];
				WirelessTransmit_2ByteData[3]=WirelessTransmit_Message.WirelessTransmitData[1];
				HAL_UART_Transmit_DMA(&huart4,WirelessTransmit_2ByteData,4);
			}			
		}
		osDelay(2);
  }
  /* USER CODE END Wireless_Transmit3_LabviewTask */
}

