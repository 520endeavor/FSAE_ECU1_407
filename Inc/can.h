/**
  ******************************************************************************
  * File Name          : CAN.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
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

#include "spi.h"	 
#include "Data_Generic.h"
#include "HostPCDebug_AT24CxxMem.h"
/* USER CODE END Includes */
 
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
/*------------ECU&���������--------------------*/
#define MotRxID		0x201			//����������ձ���ID
#define MotTxID		0x181			//���������������ID	 
/*--------������������ܶ���*/
#define	ENABLEMOT			0x0000			//ʹ�ܿ�����
#define DISABLEMOT		0X0004			//��ʹ�ܿ�����
#define ACCRampTime		500					//ACC-Rampʱ��Ϊ500ms
#define	DECRampTime		500					//DEC-Rampʱ��Ϊ500ms
/*--------����������Ĵ�������*/
#define ModifySlaveAdd_Reg		(uint8_t)0x68			//�޸Ĵ�վ��ַ�Ĵ���
#define SaveParameters_Reg		(uint8_t)0x84			//�����޸Ĳ�����EEPROM,���Ĵ�����д���������ݣ�0x00,0x00
#define ModeState_Reg					(uint8_t)0x51			//ģʽ״̬��ENABLE or DISABLE
#define DigitalSpeedCtl_Reg		(uint8_t)0x31			//���ַ�ʽ����ת�٣�����ֵΪ0X0000-0x7FFF(0-32767)����Ӧ����0-100%
#define ACCRamp_Reg						(uint8_t)0x35			//����б��
#define DECRamp_Reg						(uint8_t)0xed			//����б��	 
#define DigitalTorqueCtl_Reg	(uint8_t)0x90			//���ַ�ʽ����Ť�أ�����ֵΪ0X0000-0x7FF8(0-32760)����Ӧ����0-100%
/*����Register*/
#define RequestInfo_Reg				(uint8_t)0x3d			//���ܼĴ����������ͼĴ�����Ϣ
#define MotSpeedValue_Reg			(uint8_t)0x30			//���ʵ��ת��ֵ
#define MotCurrentValue_Reg		(uint8_t)0x20			//���ʵ�ʵ���ֵ
#define	MotPowerValue_Reg			(uint8_t)0xf6			//�������
#define DCBusVoltageValue_Reg	(uint8_t)0xeb			//������ֱ��ĸ�ߵ�ѹ
#define Capacity_Reg					(uint8_t)0x45			//Capacity I2xt
#define MotTemperature_Reg		(uint8_t)0x49			//����¶�
#define IGBTTemperature_Reg		(uint8_t)0x4a			//IGBT�¶�
#define AirTemperature_Reg		(uint8_t)0x4b			//�������¶�
#define OutVoltage_Reg				(uint8_t)0x8a			//��������ѹ
#define WarningErrorMap_Reg		(uint8_t)0x8f			//Warning-Error map,�洢����;�����Ϣ
#define	StatusMap_Reg					(uint8_t)0x40			//Status map,״̬��Ϣ
/*------------ECU����ECU2ģ�鱨��ID����---------------*/
#define ECU_ECU2_SteeringEngine_ID		0x18FF1E4A		//ECU���͸�ECU2ģ�鱨�ģ�DRS���
#define ECU_ECU2_CONTROL_ID						0x18FF1F4A		//ECU���͸�ECU2ģ�鱨�ģ����ȡ����ȡ�β�ơ����RUN�źţ����ã���ˮ�ÿ����źš��̵���6IMD���ƶ��ɿ��Դ����ź� 
/*------------ECU����WHEEL����ID����---------------*/
#define ECU_STERRINGWHEEL_MOTERR_ID		0x18FF204A		//ECU���͸�STERRINGWHEEL���ģ�������󡢾����״̬�����Ϣ
#define ECU_STERRINGWHEEL_BATERR_ID		0x18FF214A		//ECU���͸�STERRINGWHEEL���ģ�BMS���󡢾��������Ϣ
#define ECU_STERRINGWHEEL_STATE_ID		0x18FF224A		//ECU���͸�STERRINGWHEEL���ģ�����״̬��Ϣ����SD����GPS����ѹ���״̬&ˮ���¶�״̬��
#define ECU_STERRINGWHEEL_INFO_ID			0x18FF234A	 	//ECU���͸�STERRINGWHEEL���ģ�������Ϣ�������٣���̣�����&�ƶ�̤���
#define ECU_STERRINGWHEEL_ANGLE_ID		0x18FF244A	 	//ECU���͸�STERRINGWHEEL���ģ�������ת����Ϣ
#define ECU_STERRINGWHEEL_BATINFO_ID	0x18FF254A	 	//ECU���͸�STERRINGWHEEL���ģ���������Ϣ������������ܵ�ѹ&�ܵ�����SOC��SOH����ѹ��ص�ѹ��ˮ���¶ȵ�
#define ECU_STERRINGWHEEL_MAXV_ID			0x18FF264A	 	//ECU���͸�STERRINGWHEEL���ģ����&��͵����ѹ��������
#define ECU_STERRINGWHEEL_MAXT_ID			0x18FF274A	 	//ECU���͸�STERRINGWHEEL���ģ����&��͵����¶ȼ������У���ȴ&���ȿ���
#define ECU_STERRINGWHEEL_MOTINFO0_ID	0x18FF284A	 	//ECU���͸�STERRINGWHEEL���ģ������Ϣ��ر��ġ���ʵ��ת�٣�ʵ�ʵ�����������ʣ�ֱ�����ߵ�ѹ��
#define ECU_STERRINGWHEEL_MOTINFO1_ID	0x18FF294A	 	//ECU���͸�STERRINGWHEEL���ģ������Ϣ��ر��ġ���Capacity I2xt������¶ȣ�IGBT�¶ȣ��������¶�
#define ECU_STERRINGWHEEL_MOTINFO2_ID	0x18FF2A4A	 	//ECU���͸�STERRINGWHEEL���ģ������Ϣ��ر��ġ�����������������ѹ
/*------------ECU����BMS����ID����---------------*/
#define ECU_HCU_BMS_CMD_ID      			0x1801F340		//ECU���͸�HCU_BMS���ģ����Ƶ����Ͽ��Ϳ�����ѹ����Լ��ػ�����
/*------------ECU����BMS����ID����Ŷ���---------------*/
#define BMS_ECU_INFO_ID				0x186040F3		//BMS����ECU����0���������Ϣ��ء�����������ܵ�ѹ&�ܵ�����SOC��SOH���������״̬��������ظ澯����ͨ��������Ϣ��
#define BMS_ECU_INFO_NUM			0							//ECU���ձ��ı��0 
#define BMS_ECU_MAXV_ID				0x186140F3		//BMS����ECU����1�����&��͵����ѹ��������
#define BMS_ECU_MAXV_NUM			1							//ECU���ձ��ı��1	 
#define BMS_ECU_MAXT_ID				0x186240F3		//BMS����ECU����2�����&��͵����¶ȼ������У���ȴ&���ȿ���
#define BMS_ECU_MAXT_NUM			2							//ECU���ձ��ı��2	 
#define BMS_ECU_RELAY_ID			0x186340F3		//BMS����ECU����3���̵��������״̬��ر��ġ����������ܸ���Ԥ�䣬���أ����䣩��磬�ǳ��أ���䣩���ȼ̵���״̬���������״̬
#define BMS_ECU_RELAY_NUM			3							//ECU���ձ��ı��3	 
#define BMS_ECU_POWER_ID			0x186440F3		//BMS����ECU����4����������ŵ����&����
#define BMS_ECU_POWER_NUM			4							//ECU���ձ��ı��4 
#define BMS_ECU_ALARM_ID			0x186540F3		//BMS����ECU����5��������Ϣ
#define BMS_ECU_ALARM_NUM			5							//ECU���ձ��ı��5 
/*-----------------------�����ѹ���--------------------*/
#define BMS_ECU_CELLV0_ID			0x180050F3		//BMS����ECU����6�������ѹ��Ϣ
#define BMS_ECU_CELLV0_NUM		6							//ECU���ձ��ı��6
#define BMS_ECU_CELLV1_ID			0x180150F3		//BMS����ECU����7�������ѹ��Ϣ
#define BMS_ECU_CELLV1_NUM		7							//ECU���ձ��ı��7
#define BMS_ECU_CELLV2_ID			0x180250F3		//BMS����ECU����8�������ѹ��Ϣ
#define BMS_ECU_CELLV2_NUM		8							//ECU���ձ��ı��8
#define BMS_ECU_CELLV3_ID			0x180350F3		//BMS����ECU����9�������ѹ��Ϣ
#define BMS_ECU_CELLV3_NUM		9							//ECU���ձ��ı��9
#define BMS_ECU_CELLV4_ID			0x180450F3		//BMS����ECU����10�������ѹ��Ϣ
#define BMS_ECU_CELLV4_NUM		10						//ECU���ձ��ı��10
#define BMS_ECU_CELLV5_ID			0x180550F3		//BMS����ECU����11�������ѹ��Ϣ
#define BMS_ECU_CELLV5_NUM		11						//ECU���ձ��ı��11
#define BMS_ECU_CELLV6_ID			0x180650F3		//BMS����ECU����12�������ѹ��Ϣ
#define BMS_ECU_CELLV6_NUM		12						//ECU���ձ��ı��12
#define BMS_ECU_CELLV7_ID			0x180750F3		//BMS����ECU����13�������ѹ��Ϣ
#define BMS_ECU_CELLV7_NUM		13						//ECU���ձ��ı��13
#define BMS_ECU_CELLV8_ID			0x180850F3		//BMS����ECU����14�������ѹ��Ϣ
#define BMS_ECU_CELLV8_NUM		14						//ECU���ձ��ı��14
#define BMS_ECU_CELLV9_ID			0x180950F3		//BMS����ECU����15�������ѹ��Ϣ
#define BMS_ECU_CELLV9_NUM		15						//ECU���ձ��ı��15
#define BMS_ECU_CELLV10_ID		0x180A50F3		//BMS����ECU����16�������ѹ��Ϣ
#define BMS_ECU_CELLV10_NUM		16						//ECU���ձ��ı��16
#define BMS_ECU_CELLV11_ID		0x180B50F3		//BMS����ECU����17�������ѹ��Ϣ
#define BMS_ECU_CELLV11_NUM		17						//ECU���ձ��ı��17
#define BMS_ECU_CELLV12_ID		0x180C50F3		//BMS����ECU����18�������ѹ��Ϣ
#define BMS_ECU_CELLV12_NUM		18						//ECU���ձ��ı��18
#define BMS_ECU_CELLV13_ID		0x180D50F3		//BMS����ECU����19�������ѹ��Ϣ
#define BMS_ECU_CELLV13_NUM		19						//ECU���ձ��ı��19
#define BMS_ECU_CELLV14_ID		0x180E50F3		//BMS����ECU����20�������ѹ��Ϣ
#define BMS_ECU_CELLV14_NUM		20						//ECU���ձ��ı��20
#define BMS_ECU_CELLV15_ID		0x180F50F3		//BMS����ECU����21�������ѹ��Ϣ
#define BMS_ECU_CELLV15_NUM		21						//ECU���ձ��ı��21
#define BMS_ECU_CELLV16_ID		0x181050F3		//BMS����ECU����22�������ѹ��Ϣ
#define BMS_ECU_CELLV16_NUM		22						//ECU���ձ��ı��22
#define BMS_ECU_CELLV17_ID		0x181150F3		//BMS����ECU����23�������ѹ��Ϣ
#define BMS_ECU_CELLV17_NUM		23						//ECU���ձ��ı��23
#define BMS_ECU_CELLV18_ID		0x181250F3		//BMS����ECU����24�������ѹ��Ϣ
#define BMS_ECU_CELLV18_NUM		24						//ECU���ձ��ı��24
#define BMS_ECU_CELLV19_ID		0x181350F3		//BMS����ECU����25�������ѹ��Ϣ
#define BMS_ECU_CELLV19_NUM		25						//ECU���ձ��ı��25
#define BMS_ECU_CELLV20_ID		0x181450F3		//BMS����ECU����26�������ѹ��Ϣ
#define BMS_ECU_CELLV20_NUM		26						//ECU���ձ��ı��26
#define BMS_ECU_CELLV21_ID		0x181550F3		//BMS����ECU����27�������ѹ��Ϣ
#define BMS_ECU_CELLV21_NUM		27						//ECU���ձ��ı��27
#define BMS_ECU_CELLV22_ID		0x181650F3		//BMS����ECU����28�������ѹ��Ϣ
#define BMS_ECU_CELLV22_NUM		28						//ECU���ձ��ı��28
#define BMS_ECU_CELLV23_ID		0x181750F3		//BMS����ECU����29�������ѹ��Ϣ
#define BMS_ECU_CELLV23_NUM		29						//ECU���ձ��ı��29
#define BMS_ECU_CELLV24_ID		0x181850F3		//BMS����ECU����30�������ѹ��Ϣ
#define BMS_ECU_CELLV24_NUM		30						//ECU���ձ��ı��30
#define BMS_ECU_CELLV25_ID		0x181950F3		//BMS����ECU����31�������ѹ��Ϣ
#define BMS_ECU_CELLV25_NUM		31						//ECU���ձ��ı��31
#define BMS_ECU_CELLV26_ID		0x181A50F3		//BMS����ECU����32�������ѹ��Ϣ
#define BMS_ECU_CELLV26_NUM		32						//ECU���ձ��ı��32
/*-----------------------�����¶����--------------------*/
#define BMS_ECU_CELLT0_ID			0x185050F3		//BMS����ECU����33�������¶���Ϣ(ÿ�����İ����ĸ������¶�)
#define BMS_ECU_CELLT0_NUM		33						//ECU���ձ��ı��33
#define BMS_ECU_CELLT1_ID			0x185150F3		//BMS����ECU����34�������¶���Ϣ
#define BMS_ECU_CELLT1_NUM		34						//ECU���ձ��ı��34
#define BMS_ECU_CELLT2_ID			0x185250F3		//BMS����ECU����35�������¶���Ϣ
#define BMS_ECU_CELLT2_NUM		35						//ECU���ձ��ı��35
#define BMS_ECU_CELLT3_ID			0x185350F3		//BMS����ECU����36�������¶���Ϣ
#define BMS_ECU_CELLT3_NUM		36						//ECU���ձ��ı��36
#define BMS_ECU_CELLT4_ID			0x185450F3		//BMS����ECU����37�������¶���Ϣ
#define BMS_ECU_CELLT4_NUM		37						//ECU���ձ��ı��37
#define BMS_ECU_CELLT5_ID			0x185550F3		//BMS����ECU����38�������¶���Ϣ
#define BMS_ECU_CELLT5_NUM		38						//ECU���ձ��ı��38
#define BMS_ECU_CELLT6_ID			0x185650F3		//BMS����ECU����39�������¶���Ϣ
#define BMS_ECU_CELLT6_NUM		39						//ECU���ձ��ı��39
#define BMS_ECU_CELLT7_ID			0x185750F3		//BMS����ECU����40�������¶���Ϣ
#define BMS_ECU_CELLT7_NUM		40						//ECU���ձ��ı��40
#define BMS_ECU_CELLT8_ID			0x185850F3		//BMS����ECU����41�������¶���Ϣ
#define BMS_ECU_CELLT8_NUM		41						//ECU���ձ��ı��41
/*------------ECU����WHEEL����ID����Ŷ���---------------*/
#define	STERRINGWHEEL_ECU_INFO0_ID		0x18FF3C4B		//STERRINGWHEEL����ECU����1�������̿�����Ϣ����������ˮ�ÿ��ƣ��������أ���λ����DRS���أ�����or�ر�IMD&�ƶ��ɿ���
#define	STERRINGWHEEL_ECU_INFO0_NUM	42						//ECU���ձ��ı��42	 
#define	STERRINGWHEEL_ECU_INFO1_ID		0x18FF3D4B		//STERRINGWHEEL����ECU����2��mpu6050����,�����ǽǼ��ٶ�����
#define	STERRINGWHEEL_ECU_INFO1_NUM	43						//ECU���ձ��ı��43	
/*------------ECU����ECU2����ID����Ŷ���---------------*/
#define	ECU2_ECU_INFO0_ID			0x18FF5A4E		//ECU2ģ�鷢��ECU����1�����������Ϣ���������ٶ�
#define	ECU2_ECU_INFO0_NUM		44						//ECU���ձ��ı��44
#define	ECU2_ECU_INFO1_ID			0x18FF5B4E		//ECU2ģ�鷢��ECU����2�����������Ϣ����ǰ���ٶȣ�ˮ���¶ȵ�
#define	ECU2_ECU_INFO1_NUM		45						//ECU���ձ��ı��45
#define	ECU2_ECU_INFO2_ID			0x18FF5C4E		//ECU2ģ�鷢��ECU����3��AMS,IMD��λ�ź�����
#define	ECU2_ECU_INFO2_NUM		46						//ECU���ձ��ı��46
#define	ECU2_ECU_STATE_ID			0x18FF5D4E		//ECU2ģ�鷢��ECU����4����ѹ��أ�ˮ���¶ȣ�ˮ�ã����ȣ�β�ƣ����ȵ�״̬��Ϣ
#define	ECU2_ECU_STATE_NUM		47						//ECU���ձ��ı��47

/**********����ö������**********/

typedef enum{
	/**********������ݽ���**********/	
	Mot_Actual_Rev_n=1,
	Act_Current_n,
	Mot_power_n,
	DC_BUS_n,
	Capacity_I2xt_n,
	Mot_Temperature_n,
	IGBT_Temperature_n,
	Air_Temperature_n,
	Mot_Vout_n,
	Mot_Err_n,
	Mot_State_n,	
	/**********������ݽ���**********/
	
	/**********���������ݽ���**********/	
	//BMS_ECU_INFO���ݽ���
	BatteryTotal_V_n,
	BatteryTotal_I_n,
	Battery_SOC_n,
	Battery_SOH_n,
	Battery_State_n,
	Battery_Warning_Level_n,
	CommunicationLifeInfo_n,
	//BMS_ECU_MAXV���ݽ���
	MaxSingleVoltage_n, 
	MinSingleVoltage_n,
	MaxSingleVoltage_NUM_n,
	MinSingleVoltage_NUM_n,	
	//BMS_ECU_MAXT���ݽ���
	MaxSingleTemperature_n, 
	MinSingleTemperature_n,
	MaxSingleTemperature_NUM_n,
	MinSingleTemperature_NUM_n,
	CoolingControl_n,
	HeatingControl_n,	
	//BMS_ECU_RELAY���ݽ���
	PositiveRelayState_n, 
	NegativeRelayState_n,
	PrechargeRelayState_n,
	CarSlowChargeRelayState_n,
	OutCarFastChargeRelayState_n,
	ChargeState_n,
	ChargerOnlineState_n,
	OutCarChargerConnectState_n,
	CarChargerConnectState_n,	
	ChargeRequestVoltage_n,
	ChargeRequestCurrent_n,	
	//BMS_ECU_POWER���ݽ���
	MaxAllowChargeCurrent_n,
	MaxAllowDischargeCurrent_n,
	MaxAllowChargePower_n,
	MaxAllowDischargePower_n,
	//BMS_ECU_ALARM���ݽ���
	MonomerOverVol_n,
	MonomerUnderVol_n,
	BatteHighT_n,
	BatteLowT_n,
	SingleVolBreak_n,
	SingleTempBreak_n,
	SingleVolDiffLarge_n,
	BatteTempDiffLarge_n,
	AllGroupOverVol_n,
	AllGroupUnderVol_n,
	SOCHigh_n,
	SOCLow_n,
	SteadyChargeOverCur_n,
	SteadyDischargeOverCur_n,
	TransientChargeOverCur_n,
	TransientDischargeOverCur_n,
	BSU_Offline_n,
	BSU_BalanceFault_n,
	LeakageCurrentLimit_n,
	PrechargeFail_n,
	RelayFail_n,
	BMU_Fail_n,
	ECU_ExchangeTimeout_n,
	BatteHVAbnormal_n,
	HallBreak_n,
	//BMS_ECU_CELLV���ݽ���
	BMS_ECU_CELLV_n,																							//һ����ȡ108��ĵ�ѹ
	//BMS_ECU_CELLT���ݽ���
	BMS_ECU_CELLT_n,																							//һ����ȡ36���¶�	
	/**********���������ݽ���**********/
	
	/**********���շ�����������ݽ���**********/	
	//STERRINGWHEEL_ECU_INFO0���ݽ���
	StartFlag_n,
	WarterBump_n,
	Line_TCSToggleSwitch_n,
	Corners_TCSToggleSwitch_n,
	DRSSwitch1_n,
	DRSSwitch2_n,
	Mileage_CLEAR_n,
	SuspensionLineShift_ZeroSetting_n,
	StabilizerBar_Angle1_n,
	StabilizerBar_Angle2_n,
	IMDSwitch_n,
	BrakeReliabilitySwitch_n,
	Throttle_Brake_Interfering_Block_n,
	//STERRINGWHEEL_ECU_INFO1���ݽ���
	mpu6050_Angle_X_n,
	mpu6050_Angle_Y_n,
	mpu6050_Angle_Z_n,
	BackupKnob_n,	
	/**********���շ�����������ݽ���**********/
	
	/**********�����ٶȺ��¶����ݽ���**********/	
	//ECU2_ECU_INFO0���ݽ���
	BackWheelSpeed_Lkm_n,
	BackWheelSpeed_Lm_n,
	BackWheelSpeed_Rkm_n,
	BackWheelSpeed_Rm_n,
	//ECU2_ECU_INFO1���ݽ���0~5Byte
	FrontWheelSpeed_Lkm_n,
	FrontWheelSpeed_Lm_n,
	FrontWheelSpeed_Rkm_n,
	FrontWheelSpeed_Rm_n,
	TempSensorValue_n,//ECU2_ECU_INFO1����6~7Byt	
	/**********�����ٶȺ��¶����ݽ���**********/
	
	/**********AMS_IMD��λ�ź�����**********/	
	//ECU2_ECU_INFO2
	AMS_IMD_Reset_Input_n,		
	/**********AMS_IMD��λ�ź�����**********/
	
	/**********ˮ�ã����ȣ����ȣ�β�Ƶ��ź�**********/	
	//ECU2_ECU_STATE���ݽ���
	WaterTank_Tstate_n,
	WaterPump_State_n,
	Fan_State_n,
	Taillight_State_n,
	Speeker_State_n	,
	/**********ˮ�ã����ȣ����ȣ�β�Ƶ��ź�**********/
} Analysis_Data_n;
/**********����ö������**********/

/**********����HCU_BMS_CMDö������**********/
typedef enum{
	HCU_BMS_CMD_online_offline_n=145,
	HCU_BMS_CMD_ShutDown_Boot_n,
}HCU_BMS_CMD_n;

/**********����HCU_BMS_CMDö������**********/

typedef struct{
	uint8_t CANTransmitID;
	uint16_t CANTransmitData;
}CAN1TransmitMessageTypedef;

typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmitData[8];
}CAN2TransmitMessageTypedef;



/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void APP_CAN_Config(CAN_HandleTypeDef* canHandle);
void CSU_CAN_Send(CAN_HandleTypeDef* canHandle,uint32_t ID,uint8_t *pData, uint8_t Size);//CAN���ͺ���,ʹ��CAN���������ͱ���
void CSU_CAN1_Send_Mot(uint8_t REGID,uint16_t Data);//CAN1���͵���������ָ��
void CSU_CAN1_Receive_Mot_Ready(uint8_t Data_RegID,uint8_t Data_Period);//CAN1���ͻ�ȡ�����Ϣ����,���յ����Ϣ׼�������������CAN1�Ľ��պ���	
void CSU_CAN1_ModifySlaveAdd(uint16_t Data);//�޸ĵ����վ��ַ��������������õ�EEPROM��

void Get_CAN_Analysis_Data(Analysis_Data_n name_num,void*extern_data);//ȡֵ����������CAN1/CAN2���ղ������õ�����,@Analysis_Data_n

void CAN1_DATA_Send(uint8_t RegID,uint16_t Ctrl_Data);		//CAN1���͵���������ݺ���
void CAN2_DATA_Send(uint32_t Protocol_ID);								//CAN2����Э�����ݺ���

osThreadId Get_CAN1TransmitQueueHandle(void);							//��ȡCAN1���Ͷ��о��
osThreadId Get_CAN2TransmitQueueHandle(void);							//��ȡCAN2���Ͷ��о��
TimerHandle_t Get_CAN2_PeriodicSendTimer_Handle(void);		//��ȡCAN2���ڷ��������ʱ�����

void Set_CAN_Analysis_Data(Analysis_Data_n name_num,void*extern_data);//�޸�CAN1/CAN2���ղ������õ�����

void Get_HCU_BMS_CMD_Data(HCU_BMS_CMD_n name_num,void*extern_data);		//��ȡECU_HCU_BMS_CMD��������
void Set_HCU_BMS_CMD_Data(HCU_BMS_CMD_n name_num,void*extern_data);		//�޸�ECU_HCU_BMS_CMD���͵���������
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
