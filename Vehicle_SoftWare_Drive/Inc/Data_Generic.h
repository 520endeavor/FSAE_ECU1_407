#ifndef __DATA_GENERIC_H
#define __DATA_GENERIC_H

#include "stm32f4xx_hal.h"
#include "Can.h"
#include "HostPCDebug_AT24CxxMem.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

typedef enum{
	/**********DRS舵机角度控制**********/
	DRS_Angle1_n=150,//和接收的分区编号
	DRS_Angle2_n,
	/**********DRS舵机角度控制**********/
	/**********风扇、水泵，喇叭等控制**********/	
	FanCtrl_n,
	SpeakerCtrl_n,
	TaillightCtrl_n,
	MotRunBackup_n,
	WaterpumpCtrl_n,
	IMD_BrakeReliability_Trigger_n,
	Reserve1_n,
	Reserve0_n,
	/**********风扇、水泵，喇叭等控制**********/	
	/**********整车模块状态数据发送**********/
	mpu6050State_n,
	SDCardState_n,
	GPSState_n,
	SIMState_n,
	BATState_n,
	MotorState_n,
	IMDOpenOrClose_n,
	_5KWOpenOrClose_n,
	LineTCSState_n,
	CornersTCSState_n,
	DRSState_n,
	Throttle_Brake_InterferingState_n,
	/**********整车模块状态数据发送**********/	
	/**********整车状态数据发送**********/
	CarSpeed_n,
	Mileage_n,
	SlipRate_n,
	AccPedalLeft_Open_n,
	AccPedalRight_Open_n,
	/**********整车状态数据发送**********/	
}ControlerGenericData_n;

void Data_GenericFREERTOS_Init(void);									//Data_Generic中建立任务的函数

void Get_DRS_Data(ControlerGenericData_n name_num,void*extern_data);														//取值函数，搬运DRS数据@ControlerGenericData_n
void Get_Fan_Speaker_Taillight_Pump_CtrlData(ControlerGenericData_n name_num,void*extern_data);	//取值函数，搬运风扇，喇叭，尾灯，水泵控制信号数据@ControlerGenericData_n
void Get_MotRun_BrakeReliab_Reserve_Data(ControlerGenericData_n name_num,void*extern_data);			//取值函数，搬运MotRun,制动可靠性及保留信号数据@ControlerGenericData_n
void Get_mpu6050State_Data(ControlerGenericData_n name_num,void*extern_data);										//取值函数，搬运mpu6050状态数据@ControlerGenericData_n
void Get_SDCardState_Data(ControlerGenericData_n name_num,void*extern_data);										//取值函数，搬运SD卡状态数据@ControlerGenericData_n
void Get_GPSState_Data(ControlerGenericData_n name_num,void*extern_data);												//取值函数，搬运GPS状态数据@ControlerGenericData_n
void Get_SIMState_Data(ControlerGenericData_n name_num,void*extern_data);												//取值函数，搬运SIMState状态数据@ControlerGenericData_n
void Get_BATState_Data(ControlerGenericData_n name_num,void*extern_data);												//取值函数，搬运电池状态数据@ControlerGenericData_n
void Get_MotorState_Data(ControlerGenericData_n name_num,void*extern_data);											//取值函数，搬运MotorState状态数据@ControlerGenericData_n
void Get_IMDOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//取值函数，搬运IMDOpenOrClose状态数据@ControlerGenericData_n
void Get_5KWOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//取值函数，搬运5KWOpenOrClose状态数据@ControlerGenericData_n
void Get_Line_CornersTCSState_Data(ControlerGenericData_n name_num,void*extern_data);						//取值函数，搬运TCS状态数据@ControlerGenericData_n
void Get_DRSState_Data(ControlerGenericData_n name_num,void*extern_data);												//取值函数，搬运DRS状态数据@ControlerGenericData_n
void Get_Throttle_Brake_Interferingtate_Data(ControlerGenericData_n name_num,void*extern_data);	//取值函数，搬运油门刹车干涉 开启状态数据
void Get_CarSpeed_Mileage_SlipRate_Data(ControlerGenericData_n name_num,void*extern_data);			//取值函数，搬运车速和里程数据@ControlerGenericData_n
void Get_AccPeda_Open_Data(ControlerGenericData_n name_num,void*extern_data);										//取值函数，搬运加速踏板左右传感器开度数据

void Set_DRS_Data(ControlerGenericData_n name_num,void*extern_data);														//修改DRS数据@ControlerGenericData_n
void Set_Fan_Speaker_Taillight_Pump_CtrlData(ControlerGenericData_n name_num,void*extern_data);	//修改风扇，喇叭，尾灯，水泵控制信号数据@ControlerGenericData_n
void Set_MotRun_BrakeReliab_Reserve_Data(ControlerGenericData_n name_num,void*extern_data);			//修改MotRun,制动可靠性及保留信号数据@ControlerGenericData_n
void Set_mpu6050State_Data(ControlerGenericData_n name_num,void*extern_data);										//修改mpu6050状态数据@ControlerGenericData_n
void Set_SDCardState_Data(ControlerGenericData_n name_num,void*extern_data);										//修改SD卡状态数据@ControlerGenericData_n
void Set_GPSState_Data(ControlerGenericData_n name_num,void*extern_data);												//修改GPS状态数据@ControlerGenericData_n
void Set_SIMState_Data(ControlerGenericData_n name_num,void*extern_data);												//修改SIMState状态数据@ControlerGenericData_n
void Set_BATState_Data(ControlerGenericData_n name_num,void*extern_data);												//修改电池状态数据@ControlerGenericData_n
void Set_MotorState_Data(ControlerGenericData_n name_num,void*extern_data);											//修改MotorState状态数据@ControlerGenericData_n
void Set_IMDOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//修改IMDOpenOrClose状态数据@ControlerGenericData_n
void Set_5KWOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//修改5KWOpenOrClose状态数据@ControlerGenericData_n
void Set_Line_CornersTCSState_Data(ControlerGenericData_n name_num,void*extern_data);						//修改TCS状态数据@ControlerGenericData_n
void Set_DRSState_Data(ControlerGenericData_n name_num,void*extern_data);												//修改DRS状态数据@ControlerGenericData_n
void Set_Throttle_Brake_Interferingtate_Data(ControlerGenericData_n name_num,void*extern_data);	//修改油门刹车干涉 开启状态数据
void Set_CarSpeed_Mileage_SlipRate_Data(ControlerGenericData_n name_num,void*extern_data);			//修改车速和里程数据@ControlerGenericData_n
void Set_AccPeda_Open_Data(ControlerGenericData_n name_num,void*extern_data);										//修改加速踏板左右传感器开度数据@ControlerGenericData_n

#endif

