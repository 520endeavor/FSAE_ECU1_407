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
	/**********DRS����Ƕȿ���**********/
	DRS_Angle1_n=150,//�ͽ��յķ������
	DRS_Angle2_n,
	/**********DRS����Ƕȿ���**********/
	/**********���ȡ�ˮ�ã����ȵȿ���**********/	
	FanCtrl_n,
	SpeakerCtrl_n,
	TaillightCtrl_n,
	MotRunBackup_n,
	WaterpumpCtrl_n,
	IMD_BrakeReliability_Trigger_n,
	Reserve1_n,
	Reserve0_n,
	/**********���ȡ�ˮ�ã����ȵȿ���**********/	
	/**********����ģ��״̬���ݷ���**********/
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
	/**********����ģ��״̬���ݷ���**********/	
	/**********����״̬���ݷ���**********/
	CarSpeed_n,
	Mileage_n,
	SlipRate_n,
	AccPedalLeft_Open_n,
	AccPedalRight_Open_n,
	/**********����״̬���ݷ���**********/	
}ControlerGenericData_n;

void Data_GenericFREERTOS_Init(void);									//Data_Generic�н�������ĺ���

void Get_DRS_Data(ControlerGenericData_n name_num,void*extern_data);														//ȡֵ����������DRS����@ControlerGenericData_n
void Get_Fan_Speaker_Taillight_Pump_CtrlData(ControlerGenericData_n name_num,void*extern_data);	//ȡֵ���������˷��ȣ����ȣ�β�ƣ�ˮ�ÿ����ź�����@ControlerGenericData_n
void Get_MotRun_BrakeReliab_Reserve_Data(ControlerGenericData_n name_num,void*extern_data);			//ȡֵ����������MotRun,�ƶ��ɿ��Լ������ź�����@ControlerGenericData_n
void Get_mpu6050State_Data(ControlerGenericData_n name_num,void*extern_data);										//ȡֵ����������mpu6050״̬����@ControlerGenericData_n
void Get_SDCardState_Data(ControlerGenericData_n name_num,void*extern_data);										//ȡֵ����������SD��״̬����@ControlerGenericData_n
void Get_GPSState_Data(ControlerGenericData_n name_num,void*extern_data);												//ȡֵ����������GPS״̬����@ControlerGenericData_n
void Get_SIMState_Data(ControlerGenericData_n name_num,void*extern_data);												//ȡֵ����������SIMState״̬����@ControlerGenericData_n
void Get_BATState_Data(ControlerGenericData_n name_num,void*extern_data);												//ȡֵ���������˵��״̬����@ControlerGenericData_n
void Get_MotorState_Data(ControlerGenericData_n name_num,void*extern_data);											//ȡֵ����������MotorState״̬����@ControlerGenericData_n
void Get_IMDOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//ȡֵ����������IMDOpenOrClose״̬����@ControlerGenericData_n
void Get_5KWOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//ȡֵ����������5KWOpenOrClose״̬����@ControlerGenericData_n
void Get_Line_CornersTCSState_Data(ControlerGenericData_n name_num,void*extern_data);						//ȡֵ����������TCS״̬����@ControlerGenericData_n
void Get_DRSState_Data(ControlerGenericData_n name_num,void*extern_data);												//ȡֵ����������DRS״̬����@ControlerGenericData_n
void Get_Throttle_Brake_Interferingtate_Data(ControlerGenericData_n name_num,void*extern_data);	//ȡֵ��������������ɲ������ ����״̬����
void Get_CarSpeed_Mileage_SlipRate_Data(ControlerGenericData_n name_num,void*extern_data);			//ȡֵ���������˳��ٺ��������@ControlerGenericData_n
void Get_AccPeda_Open_Data(ControlerGenericData_n name_num,void*extern_data);										//ȡֵ���������˼���̤�����Ҵ�������������

void Set_DRS_Data(ControlerGenericData_n name_num,void*extern_data);														//�޸�DRS����@ControlerGenericData_n
void Set_Fan_Speaker_Taillight_Pump_CtrlData(ControlerGenericData_n name_num,void*extern_data);	//�޸ķ��ȣ����ȣ�β�ƣ�ˮ�ÿ����ź�����@ControlerGenericData_n
void Set_MotRun_BrakeReliab_Reserve_Data(ControlerGenericData_n name_num,void*extern_data);			//�޸�MotRun,�ƶ��ɿ��Լ������ź�����@ControlerGenericData_n
void Set_mpu6050State_Data(ControlerGenericData_n name_num,void*extern_data);										//�޸�mpu6050״̬����@ControlerGenericData_n
void Set_SDCardState_Data(ControlerGenericData_n name_num,void*extern_data);										//�޸�SD��״̬����@ControlerGenericData_n
void Set_GPSState_Data(ControlerGenericData_n name_num,void*extern_data);												//�޸�GPS״̬����@ControlerGenericData_n
void Set_SIMState_Data(ControlerGenericData_n name_num,void*extern_data);												//�޸�SIMState״̬����@ControlerGenericData_n
void Set_BATState_Data(ControlerGenericData_n name_num,void*extern_data);												//�޸ĵ��״̬����@ControlerGenericData_n
void Set_MotorState_Data(ControlerGenericData_n name_num,void*extern_data);											//�޸�MotorState״̬����@ControlerGenericData_n
void Set_IMDOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//�޸�IMDOpenOrClose״̬����@ControlerGenericData_n
void Set_5KWOpenOrClose_Data(ControlerGenericData_n name_num,void*extern_data);									//�޸�5KWOpenOrClose״̬����@ControlerGenericData_n
void Set_Line_CornersTCSState_Data(ControlerGenericData_n name_num,void*extern_data);						//�޸�TCS״̬����@ControlerGenericData_n
void Set_DRSState_Data(ControlerGenericData_n name_num,void*extern_data);												//�޸�DRS״̬����@ControlerGenericData_n
void Set_Throttle_Brake_Interferingtate_Data(ControlerGenericData_n name_num,void*extern_data);	//�޸�����ɲ������ ����״̬����
void Set_CarSpeed_Mileage_SlipRate_Data(ControlerGenericData_n name_num,void*extern_data);			//�޸ĳ��ٺ��������@ControlerGenericData_n
void Set_AccPeda_Open_Data(ControlerGenericData_n name_num,void*extern_data);										//�޸ļ���̤�����Ҵ�������������@ControlerGenericData_n

#endif

