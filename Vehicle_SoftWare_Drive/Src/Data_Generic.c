#include "Data_Generic.h"

struct{
	volatile float DRS_Angle1;
	volatile float DRS_Angle2;
}DRS_Angle;//ECU_ ECU2_SteeringEngine ,DRS 舵机角度信息

struct {
	volatile uint8_t FanCtrl;
	volatile uint8_t SpeakerCtrl;
	volatile uint8_t TaillightCtrl;
	volatile uint8_t MotRunBackup;
	volatile uint8_t WaterpumpCtrl;
	volatile uint8_t IMD_BrakeReliability_Trigger;
	volatile uint8_t Reserve1;
	volatile uint8_t Reserve0;
}Speaker_Taillight_Pump_Ctrl;//ECU_ ECU2_CONTROL信息

struct{
	volatile uint8_t mpu6050State;
	volatile uint8_t SDCardState;
	volatile uint8_t GPSState;
	volatile uint8_t SIMState;
	volatile uint8_t BATState;
	volatile uint8_t MotorState;
	volatile uint8_t IMDOpenOrClose;
	volatile uint8_t _5KWOpenOrClose;
	volatile uint8_t LineTCSState;
	volatile uint8_t CornersTCSState;
	volatile uint8_t DRSState;
	volatile uint8_t Throttle_Brake_InterferingState;
}VehicleModuleState;//ECU_ STERRINGWHEEL_STATE,车上模块状态信息

struct{
	volatile uint8_t CarSpeed;
	volatile float Mileage;
	volatile float SlipRate;
}CarSpeed_Mileage_SlipRate;//ECU_ STERRINGWHEEL_INFO ,整车速度,里程,滑移率信息

struct{
	volatile uint8_t AccPedalLeft_Open;
	volatile uint8_t AccPedalRight_Open;
}AccPeda_Open_Data;//ECU_ STERRINGWHEEL_INFO ,整车油门开度信息


osThreadId CarSpeed_Mileage_SlipRateGenericTaskHandle;

void CarSpeed_Mileage_SlipRateGenericTask(void const * argument);

void Data_GenericFREERTOS_Init(void)
{
	/* definition and creation of CarSpeed_Mileage_SlipRateGenericTask */
	osThreadDef(CarSpeed_Mileage_SlipRateGenericTask, CarSpeed_Mileage_SlipRateGenericTask, osPriorityNormal, 0, 128);
	CarSpeed_Mileage_SlipRateGenericTaskHandle = osThreadCreate(osThread(CarSpeed_Mileage_SlipRateGenericTask), NULL);		
}

/* CarSpeed_Mileage_SlipRateGenericTask function */
void CarSpeed_Mileage_SlipRateGenericTask(void const * argument)
{	
  /* USER CODE BEGIN CarSpeed_Mileage_SlipRateGenericTask */
  /* Infinite loop */	
	float CarSpeed_BackWheelSpeed_Lm,CarSpeed_BackWheelSpeed_Rm,CarSpeed_FrontWheelSpeed_Lm,CarSpeed_FrontWheelSpeed_Rm;
	float BackWheel_SpeedMean,FrontWheel_SpeedMean,OptimalSlipRate;
	uint16_t AccPedalL,AccPedalR,AccPeda_Range_High,AccPeda_Range_Low;
	for(;;)
  {				
		Get_CAN_Analysis_Data(BackWheelSpeed_Lm_n,&CarSpeed_BackWheelSpeed_Lm);
		Get_CAN_Analysis_Data(BackWheelSpeed_Rm_n,&CarSpeed_BackWheelSpeed_Rm);
		Get_CAN_Analysis_Data(FrontWheelSpeed_Lm_n,&CarSpeed_FrontWheelSpeed_Lm);
		Get_CAN_Analysis_Data(FrontWheelSpeed_Rm_n,&CarSpeed_FrontWheelSpeed_Rm);			
		CarSpeed_Mileage_SlipRate.CarSpeed=(CarSpeed_FrontWheelSpeed_Lm+CarSpeed_FrontWheelSpeed_Rm)/2.0*3.6;
		CarSpeed_Mileage_SlipRate.Mileage+=(CarSpeed_FrontWheelSpeed_Lm+CarSpeed_FrontWheelSpeed_Rm)/2.0*0.01/1000;
		BackWheel_SpeedMean=(CarSpeed_BackWheelSpeed_Lm+CarSpeed_BackWheelSpeed_Rm)/2.0;
		FrontWheel_SpeedMean=(CarSpeed_FrontWheelSpeed_Lm+CarSpeed_FrontWheelSpeed_Rm)/2.0;
		if(FrontWheel_SpeedMean!=0){
			if((BackWheel_SpeedMean-FrontWheel_SpeedMean)/FrontWheel_SpeedMean<-10){
				CarSpeed_Mileage_SlipRate.SlipRate=-10;
			}
			else if((BackWheel_SpeedMean-FrontWheel_SpeedMean)/FrontWheel_SpeedMean>10){
				CarSpeed_Mileage_SlipRate.SlipRate=10;
			}
			else{
				CarSpeed_Mileage_SlipRate.SlipRate=(BackWheel_SpeedMean-FrontWheel_SpeedMean)/FrontWheel_SpeedMean;
			}			
		}
		else if(BackWheel_SpeedMean!=0){
			Get_Wireless_DataFrom_EEPROM(OptimalSlipRate_n,&OptimalSlipRate);
			CarSpeed_Mileage_SlipRate.SlipRate=OptimalSlipRate/2.0;
		}
		else if(BackWheel_SpeedMean==0){
			CarSpeed_Mileage_SlipRate.SlipRate=0;
		}
		Get_Acc_Brake_Angle_LowVData(AccPedalLeft_n,&AccPedalL);
		Get_Acc_Brake_Angle_LowVData(AccPedalRight_n,&AccPedalR);
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_High_n,&AccPeda_Range_High);
		Get_Wireless_DataFrom_EEPROM(AccPeda_Range_Low_n,&AccPeda_Range_Low);
		if(AccPedalL<AccPeda_Range_Low){
			AccPeda_Open_Data.AccPedalLeft_Open=0;
		}
		else{
			if((float)(AccPedalL-AccPeda_Range_Low)/(AccPeda_Range_High-AccPeda_Range_Low)*100>=100){
				AccPeda_Open_Data.AccPedalLeft_Open=100;
			}
			else{			
				AccPeda_Open_Data.AccPedalLeft_Open=(float)(AccPedalL-AccPeda_Range_Low)/(AccPeda_Range_High-AccPeda_Range_Low)*100;
			}
		}	
		if(AccPedalR<AccPeda_Range_Low){
			AccPeda_Open_Data.AccPedalRight_Open=0;
		}
		else{
			if((float)(AccPedalR-AccPeda_Range_Low)/(AccPeda_Range_High-AccPeda_Range_Low)*100>=100){
				AccPeda_Open_Data.AccPedalRight_Open=100;
			}
			else{
				AccPeda_Open_Data.AccPedalRight_Open=(float)(AccPedalR-AccPeda_Range_Low)/(AccPeda_Range_High-AccPeda_Range_Low)*100;
			}
		}			
    osDelay(10);
  }
  /* USER CODE END CarSpeed_Mileage_SlipRateGenericTask */
}


/*
*****************************************************************************
*@brief		取值函数，搬运DRS数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_DRS_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==DRS_Angle1_n){
		*(float*)extern_data=DRS_Angle.DRS_Angle1;
	} 
	else if(name_num==DRS_Angle2_n){
		*(float*)extern_data=DRS_Angle.DRS_Angle2;
	}	
}

/*
*****************************************************************************
*@brief		取值函数，搬运风扇，喇叭，尾灯，水泵控制信号数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Fan_Speaker_Taillight_Pump_CtrlData(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==FanCtrl_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.FanCtrl;
	}
	else if(name_num==SpeakerCtrl_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.SpeakerCtrl;
	} 
	else if(name_num==TaillightCtrl_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.TaillightCtrl;
	}	
	else if(name_num==WaterpumpCtrl_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.WaterpumpCtrl;
	}	
}

/*
*****************************************************************************
*@brief		取值函数，搬运MotRun,制动可靠性及保留信号数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_MotRun_BrakeReliab_Reserve_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==MotRunBackup_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.MotRunBackup;
	}
	else if(name_num==IMD_BrakeReliability_Trigger_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.IMD_BrakeReliability_Trigger;
	} 
	else if(name_num==Reserve1_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.Reserve1;
	}	
	else if(name_num==Reserve0_n){
		*(uint8_t*)extern_data=Speaker_Taillight_Pump_Ctrl.Reserve0;
	}	
}

/*
*****************************************************************************
*@brief		取值函数，搬运mpu6050状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_mpu6050State_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==mpu6050State_n){
		*(uint8_t*)extern_data=VehicleModuleState.mpu6050State;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运SD卡状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_SDCardState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==SDCardState_n){
		*(uint8_t*)extern_data=VehicleModuleState.SDCardState;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运GPS状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_GPSState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==GPSState_n){
		*(uint8_t*)extern_data=VehicleModuleState.GPSState;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运SIMState状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_SIMState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==SIMState_n){
		*(uint8_t*)extern_data=VehicleModuleState.SIMState;
	}
}
	
/*
*****************************************************************************
*@brief		取值函数，搬运电池状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_BATState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==BATState_n){
		*(uint8_t*)extern_data=VehicleModuleState.BATState;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运MotorState状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_MotorState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==MotorState_n){
		*(uint8_t*)extern_data=VehicleModuleState.MotorState;
	}
}
	
/*
*****************************************************************************
*@brief		取值函数，搬运IMDOpenOrClose状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_IMDOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==IMDOpenOrClose_n){
		*(uint8_t*)extern_data=VehicleModuleState.IMDOpenOrClose;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运5KWOpenOrClose状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_5KWOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==_5KWOpenOrClose_n){
		*(uint8_t*)extern_data=VehicleModuleState._5KWOpenOrClose;
	}
}
	
/*
*****************************************************************************
*@brief		取值函数，搬运TCS状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Line_CornersTCSState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==LineTCSState_n){
		*(uint8_t*)extern_data=VehicleModuleState.LineTCSState;
	}
	if(name_num==CornersTCSState_n){
		*(uint8_t*)extern_data=VehicleModuleState.CornersTCSState;
	}
}

/*
*****************************************************************************
*@brief		取值函数，搬运DRS状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_DRSState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==DRSState_n){
		VehicleModuleState.DRSState=*(uint8_t*)extern_data;
	}	
}

/*
*****************************************************************************
*@brief		取值函数，搬运油门刹车干涉 开启状态数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Throttle_Brake_Interferingtate_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==Throttle_Brake_InterferingState_n){
		VehicleModuleState.Throttle_Brake_InterferingState=*(uint8_t*)extern_data;
	}	
}

/*
*****************************************************************************
*@brief		取值函数，搬运车速和里程数据
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_CarSpeed_Mileage_SlipRate_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==CarSpeed_n){
		*(uint8_t*)extern_data=CarSpeed_Mileage_SlipRate.CarSpeed;
	}
	else if(name_num==Mileage_n){
		*(float*)extern_data=CarSpeed_Mileage_SlipRate.Mileage;
	}
	else if(name_num==SlipRate_n){
		*(float*)extern_data=CarSpeed_Mileage_SlipRate.SlipRate;
	}	
}

/*
*****************************************************************************
*@brief		取值函数，搬运加速踏板左右传感器开度
*@param		ControlerGenericData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_AccPeda_Open_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==AccPedalLeft_Open_n){
		*(uint8_t*)extern_data=AccPeda_Open_Data.AccPedalLeft_Open;
	}
	else if(name_num==AccPedalRight_Open_n){
		*(uint8_t*)extern_data=AccPeda_Open_Data.AccPedalRight_Open;
	}	
}

/*
*****************************************************************************
*@brief		修改DRS数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_DRS_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	if(name_num==DRS_Angle1_n){
		DRS_Angle.DRS_Angle1=*(float*)extern_data;
	} 
	else if(name_num==DRS_Angle2_n){
		DRS_Angle.DRS_Angle2=*(float*)extern_data;
	}	
}

/*
*****************************************************************************
*@brief		修改风扇，喇叭，尾灯，水泵控制信号数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Fan_Speaker_Taillight_Pump_CtrlData(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==FanCtrl_n){
		Speaker_Taillight_Pump_Ctrl.FanCtrl=*(uint8_t*)extern_data;
	}
	else if(name_num==SpeakerCtrl_n){
		Speaker_Taillight_Pump_Ctrl.SpeakerCtrl=*(uint8_t*)extern_data;
	} 
	else if(name_num==TaillightCtrl_n){
		Speaker_Taillight_Pump_Ctrl.TaillightCtrl=*(uint8_t*)extern_data;
	}	
	else if(name_num==WaterpumpCtrl_n){
		Speaker_Taillight_Pump_Ctrl.WaterpumpCtrl=*(uint8_t*)extern_data;
	}	
}

/*
*****************************************************************************
*@brief		修改MotRun,制动可靠性及保留信号数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_MotRun_BrakeReliab_Reserve_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==MotRunBackup_n){
		Speaker_Taillight_Pump_Ctrl.MotRunBackup=*(uint8_t*)extern_data;
	}
	else if(name_num==IMD_BrakeReliability_Trigger_n){
		Speaker_Taillight_Pump_Ctrl.IMD_BrakeReliability_Trigger=*(uint8_t*)extern_data;
	} 
	else if(name_num==Reserve1_n){
		Speaker_Taillight_Pump_Ctrl.Reserve1=*(uint8_t*)extern_data;
	}	
	else if(name_num==Reserve0_n){
		Speaker_Taillight_Pump_Ctrl.Reserve0=*(uint8_t*)extern_data;
	}	
}

/*
*****************************************************************************
*@brief		修改mpu6050状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_mpu6050State_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==mpu6050State_n){
		VehicleModuleState.mpu6050State=*(uint8_t*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改SD卡状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_SDCardState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==SDCardState_n){
		VehicleModuleState.SDCardState=*(uint8_t*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改GPS状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_GPSState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==GPSState_n){
		VehicleModuleState.GPSState=*(uint8_t*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改SIMState状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_SIMState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==SIMState_n){
		VehicleModuleState.SIMState=*(uint8_t*)extern_data;
	}
}
	
/*
*****************************************************************************
*@brief		修改电池状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_BATState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==BATState_n){
		VehicleModuleState.BATState=*(uint8_t*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改MotorState状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_MotorState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==MotorState_n){
		VehicleModuleState.MotorState=*(uint8_t*)extern_data;
	}
}
	
/*
*****************************************************************************
*@brief		修改IMDOpenOrClose状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_IMDOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==IMDOpenOrClose_n){
		VehicleModuleState.IMDOpenOrClose=*(uint8_t*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改5KWOpenOrClose状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_5KWOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==_5KWOpenOrClose_n){
		VehicleModuleState._5KWOpenOrClose=*(uint8_t*)extern_data;
	}
}
	
/*
*****************************************************************************
*@brief		修改TCS状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Line_CornersTCSState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==LineTCSState_n){
		VehicleModuleState.LineTCSState=*(uint8_t*)extern_data;
	}
	if(name_num==CornersTCSState_n){
		VehicleModuleState.CornersTCSState=*(uint8_t*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改DRS状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_DRSState_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==DRSState_n){
		VehicleModuleState.DRSState=*(uint8_t*)extern_data;
	}	
}

/*
*****************************************************************************
*@brief		修改油门刹车干涉 开启状态数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Throttle_Brake_Interferingtate_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==Throttle_Brake_InterferingState_n){
		*(uint8_t*)extern_data=VehicleModuleState.Throttle_Brake_InterferingState;
	}	
}

/*
*****************************************************************************
*@brief		修改车速和里程数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_CarSpeed_Mileage_SlipRate_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==CarSpeed_n){
		CarSpeed_Mileage_SlipRate.CarSpeed=*(uint8_t*)extern_data;
	}
	else if(name_num==Mileage_n){
		CarSpeed_Mileage_SlipRate.Mileage=*(float*)extern_data;
	}
	else if(name_num==SlipRate_n){
		CarSpeed_Mileage_SlipRate.SlipRate=*(float*)extern_data;
	}
}

/*
*****************************************************************************
*@brief		修改加速踏板左右传感器开度数据
*@param		ControlerGenericData_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_AccPeda_Open_Data(ControlerGenericData_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	if(name_num==AccPedalLeft_Open_n){
		AccPeda_Open_Data.AccPedalLeft_Open=*(uint8_t*)extern_data;
	}
	else if(name_num==AccPedalRight_Open_n){
		AccPeda_Open_Data.AccPedalRight_Open=*(uint8_t*)extern_data;
	}	
}


