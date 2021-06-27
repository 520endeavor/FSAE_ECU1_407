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
/*------------ECU&电机控制器--------------------*/
#define MotRxID		0x201			//电机控制器收报文ID
#define MotTxID		0x181			//电机控制器发报文ID	 
/*--------电机控制器功能定义*/
#define	ENABLEMOT			0x0000			//使能控制器
#define DISABLEMOT		0X0004			//不使能控制器
#define ACCRampTime		500					//ACC-Ramp时间为500ms
#define	DECRampTime		500					//DEC-Ramp时间为500ms
/*--------电机控制器寄存器定义*/
#define ModifySlaveAdd_Reg		(uint8_t)0x68			//修改从站地址寄存器
#define SaveParameters_Reg		(uint8_t)0x84			//保存修改参数到EEPROM,往寄存器中写入两个数据：0x00,0x00
#define ModeState_Reg					(uint8_t)0x51			//模式状态，ENABLE or DISABLE
#define DigitalSpeedCtl_Reg		(uint8_t)0x31			//数字方式控制转速，设置值为0X0000-0x7FFF(0-32767)，对应油门0-100%
#define ACCRamp_Reg						(uint8_t)0x35			//加速斜坡
#define DECRamp_Reg						(uint8_t)0xed			//减速斜坡	 
#define DigitalTorqueCtl_Reg	(uint8_t)0x90			//数字方式控制扭矩，设置值为0X0000-0x7FF8(0-32760)，对应油门0-100%
/*接收Register*/
#define RequestInfo_Reg				(uint8_t)0x3d			//功能寄存器，请求发送寄存器信息
#define MotSpeedValue_Reg			(uint8_t)0x30			//电机实际转速值
#define MotCurrentValue_Reg		(uint8_t)0x20			//电机实际电流值
#define	MotPowerValue_Reg			(uint8_t)0xf6			//电机功率
#define DCBusVoltageValue_Reg	(uint8_t)0xeb			//控制器直流母线电压
#define Capacity_Reg					(uint8_t)0x45			//Capacity I2xt
#define MotTemperature_Reg		(uint8_t)0x49			//电机温度
#define IGBTTemperature_Reg		(uint8_t)0x4a			//IGBT温度
#define AirTemperature_Reg		(uint8_t)0x4b			//控制器温度
#define OutVoltage_Reg				(uint8_t)0x8a			//电机输出电压
#define WarningErrorMap_Reg		(uint8_t)0x8f			//Warning-Error map,存储错误和警告信息
#define	StatusMap_Reg					(uint8_t)0x40			//Status map,状态信息
/*------------ECU发给ECU2模块报文ID定义---------------*/
#define ECU_ECU2_SteeringEngine_ID		0x18FF1E4A		//ECU发送给ECU2模块报文，DRS舵机
#define ECU_ECU2_CONTROL_ID						0x18FF1F4A		//ECU发送给ECU2模块报文，风扇、喇叭、尾灯、电机RUN信号（备用）、水泵控制信号、继电器6IMD和制动可靠性触发信号 
/*------------ECU发给WHEEL报文ID定义---------------*/
#define ECU_STERRINGWHEEL_MOTERR_ID		0x18FF204A		//ECU发送给STERRINGWHEEL报文，电机错误、警告和状态相关信息
#define ECU_STERRINGWHEEL_BATERR_ID		0x18FF214A		//ECU发送给STERRINGWHEEL报文，BMS错误、警告相关信息
#define ECU_STERRINGWHEEL_STATE_ID		0x18FF224A		//ECU发送给STERRINGWHEEL报文，整车状态信息——SD卡，GPS，低压电池状态&水箱温度状态等
#define ECU_STERRINGWHEEL_INFO_ID			0x18FF234A	 	//ECU发送给STERRINGWHEEL报文，整车信息——车速，里程，加速&制动踏板等
#define ECU_STERRINGWHEEL_ANGLE_ID		0x18FF244A	 	//ECU发送给STERRINGWHEEL报文，方向盘转角信息
#define ECU_STERRINGWHEEL_BATINFO_ID	0x18FF254A	 	//ECU发送给STERRINGWHEEL报文，电池相关信息——动力电池总电压&总电流，SOC，SOH，低压电池电压，水箱温度等
#define ECU_STERRINGWHEEL_MAXV_ID			0x18FF264A	 	//ECU发送给STERRINGWHEEL报文，最高&最低单体电压及其序列
#define ECU_STERRINGWHEEL_MAXT_ID			0x18FF274A	 	//ECU发送给STERRINGWHEEL报文，最高&最低单体温度及其序列，冷却&加热控制
#define ECU_STERRINGWHEEL_MOTINFO0_ID	0x18FF284A	 	//ECU发送给STERRINGWHEEL报文，电机信息相关报文——实际转速，实际电流，电机功率，直流总线电压等
#define ECU_STERRINGWHEEL_MOTINFO1_ID	0x18FF294A	 	//ECU发送给STERRINGWHEEL报文，电机信息相关报文——Capacity I2xt，电机温度，IGBT温度，控制器温度
#define ECU_STERRINGWHEEL_MOTINFO2_ID	0x18FF2A4A	 	//ECU发送给STERRINGWHEEL报文，电机信息相关报文——电机控制器输出电压
/*------------ECU发给BMS报文ID定义---------------*/
#define ECU_HCU_BMS_CMD_ID      			0x1801F340		//ECU发送给HCU_BMS报文，控制电池箱断开和开启高压输出以及关机命令
/*------------ECU接收BMS报文ID及编号定义---------------*/
#define BMS_ECU_INFO_ID				0x186040F3		//BMS发给ECU报文0，电池箱信息相关——动力电池总电压&总电流，SOC，SOH，动力电池状态，动力电池告警级别，通信生命信息等
#define BMS_ECU_INFO_NUM			0							//ECU接收报文编号0 
#define BMS_ECU_MAXV_ID				0x186140F3		//BMS发给ECU报文1，最高&最低单体电压及其序列
#define BMS_ECU_MAXV_NUM			1							//ECU接收报文编号1	 
#define BMS_ECU_MAXT_ID				0x186240F3		//BMS发给ECU报文2，最高&最低单体温度及其序列，冷却&加热控制
#define BMS_ECU_MAXT_NUM			2							//ECU接收报文编号2	 
#define BMS_ECU_RELAY_ID			0x186340F3		//BMS发给ECU报文3，继电器及充电状态相关报文——总正，总负，预充，车载（慢充）充电，非车载（快充）充电等继电器状态及其它充电状态
#define BMS_ECU_RELAY_NUM			3							//ECU接收报文编号3	 
#define BMS_ECU_POWER_ID			0x186440F3		//BMS发给ECU报文4，最大允许充放电电流&功率
#define BMS_ECU_POWER_NUM			4							//ECU接收报文编号4 
#define BMS_ECU_ALARM_ID			0x186540F3		//BMS发给ECU报文5，警告信息
#define BMS_ECU_ALARM_NUM			5							//ECU接收报文编号5 
/*-----------------------单体电压相关--------------------*/
#define BMS_ECU_CELLV0_ID			0x180050F3		//BMS发给ECU报文6，单体电压信息
#define BMS_ECU_CELLV0_NUM		6							//ECU接收报文编号6
#define BMS_ECU_CELLV1_ID			0x180150F3		//BMS发给ECU报文7，单体电压信息
#define BMS_ECU_CELLV1_NUM		7							//ECU接收报文编号7
#define BMS_ECU_CELLV2_ID			0x180250F3		//BMS发给ECU报文8，单体电压信息
#define BMS_ECU_CELLV2_NUM		8							//ECU接收报文编号8
#define BMS_ECU_CELLV3_ID			0x180350F3		//BMS发给ECU报文9，单体电压信息
#define BMS_ECU_CELLV3_NUM		9							//ECU接收报文编号9
#define BMS_ECU_CELLV4_ID			0x180450F3		//BMS发给ECU报文10，单体电压信息
#define BMS_ECU_CELLV4_NUM		10						//ECU接收报文编号10
#define BMS_ECU_CELLV5_ID			0x180550F3		//BMS发给ECU报文11，单体电压信息
#define BMS_ECU_CELLV5_NUM		11						//ECU接收报文编号11
#define BMS_ECU_CELLV6_ID			0x180650F3		//BMS发给ECU报文12，单体电压信息
#define BMS_ECU_CELLV6_NUM		12						//ECU接收报文编号12
#define BMS_ECU_CELLV7_ID			0x180750F3		//BMS发给ECU报文13，单体电压信息
#define BMS_ECU_CELLV7_NUM		13						//ECU接收报文编号13
#define BMS_ECU_CELLV8_ID			0x180850F3		//BMS发给ECU报文14，单体电压信息
#define BMS_ECU_CELLV8_NUM		14						//ECU接收报文编号14
#define BMS_ECU_CELLV9_ID			0x180950F3		//BMS发给ECU报文15，单体电压信息
#define BMS_ECU_CELLV9_NUM		15						//ECU接收报文编号15
#define BMS_ECU_CELLV10_ID		0x180A50F3		//BMS发给ECU报文16，单体电压信息
#define BMS_ECU_CELLV10_NUM		16						//ECU接收报文编号16
#define BMS_ECU_CELLV11_ID		0x180B50F3		//BMS发给ECU报文17，单体电压信息
#define BMS_ECU_CELLV11_NUM		17						//ECU接收报文编号17
#define BMS_ECU_CELLV12_ID		0x180C50F3		//BMS发给ECU报文18，单体电压信息
#define BMS_ECU_CELLV12_NUM		18						//ECU接收报文编号18
#define BMS_ECU_CELLV13_ID		0x180D50F3		//BMS发给ECU报文19，单体电压信息
#define BMS_ECU_CELLV13_NUM		19						//ECU接收报文编号19
#define BMS_ECU_CELLV14_ID		0x180E50F3		//BMS发给ECU报文20，单体电压信息
#define BMS_ECU_CELLV14_NUM		20						//ECU接收报文编号20
#define BMS_ECU_CELLV15_ID		0x180F50F3		//BMS发给ECU报文21，单体电压信息
#define BMS_ECU_CELLV15_NUM		21						//ECU接收报文编号21
#define BMS_ECU_CELLV16_ID		0x181050F3		//BMS发给ECU报文22，单体电压信息
#define BMS_ECU_CELLV16_NUM		22						//ECU接收报文编号22
#define BMS_ECU_CELLV17_ID		0x181150F3		//BMS发给ECU报文23，单体电压信息
#define BMS_ECU_CELLV17_NUM		23						//ECU接收报文编号23
#define BMS_ECU_CELLV18_ID		0x181250F3		//BMS发给ECU报文24，单体电压信息
#define BMS_ECU_CELLV18_NUM		24						//ECU接收报文编号24
#define BMS_ECU_CELLV19_ID		0x181350F3		//BMS发给ECU报文25，单体电压信息
#define BMS_ECU_CELLV19_NUM		25						//ECU接收报文编号25
#define BMS_ECU_CELLV20_ID		0x181450F3		//BMS发给ECU报文26，单体电压信息
#define BMS_ECU_CELLV20_NUM		26						//ECU接收报文编号26
#define BMS_ECU_CELLV21_ID		0x181550F3		//BMS发给ECU报文27，单体电压信息
#define BMS_ECU_CELLV21_NUM		27						//ECU接收报文编号27
#define BMS_ECU_CELLV22_ID		0x181650F3		//BMS发给ECU报文28，单体电压信息
#define BMS_ECU_CELLV22_NUM		28						//ECU接收报文编号28
#define BMS_ECU_CELLV23_ID		0x181750F3		//BMS发给ECU报文29，单体电压信息
#define BMS_ECU_CELLV23_NUM		29						//ECU接收报文编号29
#define BMS_ECU_CELLV24_ID		0x181850F3		//BMS发给ECU报文30，单体电压信息
#define BMS_ECU_CELLV24_NUM		30						//ECU接收报文编号30
#define BMS_ECU_CELLV25_ID		0x181950F3		//BMS发给ECU报文31，单体电压信息
#define BMS_ECU_CELLV25_NUM		31						//ECU接收报文编号31
#define BMS_ECU_CELLV26_ID		0x181A50F3		//BMS发给ECU报文32，单体电压信息
#define BMS_ECU_CELLV26_NUM		32						//ECU接收报文编号32
/*-----------------------单体温度相关--------------------*/
#define BMS_ECU_CELLT0_ID			0x185050F3		//BMS发给ECU报文33，单体温度信息(每个报文包含四个单体温度)
#define BMS_ECU_CELLT0_NUM		33						//ECU接收报文编号33
#define BMS_ECU_CELLT1_ID			0x185150F3		//BMS发给ECU报文34，单体温度信息
#define BMS_ECU_CELLT1_NUM		34						//ECU接收报文编号34
#define BMS_ECU_CELLT2_ID			0x185250F3		//BMS发给ECU报文35，单体温度信息
#define BMS_ECU_CELLT2_NUM		35						//ECU接收报文编号35
#define BMS_ECU_CELLT3_ID			0x185350F3		//BMS发给ECU报文36，单体温度信息
#define BMS_ECU_CELLT3_NUM		36						//ECU接收报文编号36
#define BMS_ECU_CELLT4_ID			0x185450F3		//BMS发给ECU报文37，单体温度信息
#define BMS_ECU_CELLT4_NUM		37						//ECU接收报文编号37
#define BMS_ECU_CELLT5_ID			0x185550F3		//BMS发给ECU报文38，单体温度信息
#define BMS_ECU_CELLT5_NUM		38						//ECU接收报文编号38
#define BMS_ECU_CELLT6_ID			0x185650F3		//BMS发给ECU报文39，单体温度信息
#define BMS_ECU_CELLT6_NUM		39						//ECU接收报文编号39
#define BMS_ECU_CELLT7_ID			0x185750F3		//BMS发给ECU报文40，单体温度信息
#define BMS_ECU_CELLT7_NUM		40						//ECU接收报文编号40
#define BMS_ECU_CELLT8_ID			0x185850F3		//BMS发给ECU报文41，单体温度信息
#define BMS_ECU_CELLT8_NUM		41						//ECU接收报文编号41
/*------------ECU接收WHEEL报文ID及编号定义---------------*/
#define	STERRINGWHEEL_ECU_INFO0_ID		0x18FF3C4B		//STERRINGWHEEL发给ECU报文1，方向盘控制信息——启动，水泵控制，拨动开关，电位器，DRS开关，启动or关闭IMD&制动可靠性
#define	STERRINGWHEEL_ECU_INFO0_NUM	42						//ECU接收报文编号42	 
#define	STERRINGWHEEL_ECU_INFO1_ID		0x18FF3D4B		//STERRINGWHEEL发给ECU报文2，mpu6050数据,陀螺仪角加速度数据
#define	STERRINGWHEEL_ECU_INFO1_NUM	43						//ECU接收报文编号43	
/*------------ECU接收ECU2报文ID及编号定义---------------*/
#define	ECU2_ECU_INFO0_ID			0x18FF5A4E		//ECU2模块发给ECU报文1，霍尔相关信息——后轮速度
#define	ECU2_ECU_INFO0_NUM		44						//ECU接收报文编号44
#define	ECU2_ECU_INFO1_ID			0x18FF5B4E		//ECU2模块发给ECU报文2，霍尔相关信息——前轮速度，水箱温度等
#define	ECU2_ECU_INFO1_NUM		45						//ECU接收报文编号45
#define	ECU2_ECU_INFO2_ID			0x18FF5C4E		//ECU2模块发给ECU报文3，AMS,IMD复位信号输入
#define	ECU2_ECU_INFO2_NUM		46						//ECU接收报文编号46
#define	ECU2_ECU_STATE_ID			0x18FF5D4E		//ECU2模块发给ECU报文4，低压电池，水箱温度，水泵，风扇，尾灯，喇叭等状态信息
#define	ECU2_ECU_STATE_NUM		47						//ECU接收报文编号47

/**********接收枚举类型**********/

typedef enum{
	/**********电机数据解析**********/	
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
	/**********电机数据解析**********/
	
	/**********电池相关数据解析**********/	
	//BMS_ECU_INFO数据解析
	BatteryTotal_V_n,
	BatteryTotal_I_n,
	Battery_SOC_n,
	Battery_SOH_n,
	Battery_State_n,
	Battery_Warning_Level_n,
	CommunicationLifeInfo_n,
	//BMS_ECU_MAXV数据解析
	MaxSingleVoltage_n, 
	MinSingleVoltage_n,
	MaxSingleVoltage_NUM_n,
	MinSingleVoltage_NUM_n,	
	//BMS_ECU_MAXT数据解析
	MaxSingleTemperature_n, 
	MinSingleTemperature_n,
	MaxSingleTemperature_NUM_n,
	MinSingleTemperature_NUM_n,
	CoolingControl_n,
	HeatingControl_n,	
	//BMS_ECU_RELAY数据解析
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
	//BMS_ECU_POWER数据解析
	MaxAllowChargeCurrent_n,
	MaxAllowDischargeCurrent_n,
	MaxAllowChargePower_n,
	MaxAllowDischargePower_n,
	//BMS_ECU_ALARM数据解析
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
	//BMS_ECU_CELLV数据解析
	BMS_ECU_CELLV_n,																							//一次性取108块的电压
	//BMS_ECU_CELLT数据解析
	BMS_ECU_CELLT_n,																							//一次性取36个温度	
	/**********电池相关数据解析**********/
	
	/**********接收方向盘相关数据解析**********/	
	//STERRINGWHEEL_ECU_INFO0数据解析
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
	//STERRINGWHEEL_ECU_INFO1数据解析
	mpu6050_Angle_X_n,
	mpu6050_Angle_Y_n,
	mpu6050_Angle_Z_n,
	BackupKnob_n,	
	/**********接收方向盘相关数据解析**********/
	
	/**********四轮速度和温度数据解析**********/	
	//ECU2_ECU_INFO0数据解析
	BackWheelSpeed_Lkm_n,
	BackWheelSpeed_Lm_n,
	BackWheelSpeed_Rkm_n,
	BackWheelSpeed_Rm_n,
	//ECU2_ECU_INFO1数据解析0~5Byte
	FrontWheelSpeed_Lkm_n,
	FrontWheelSpeed_Lm_n,
	FrontWheelSpeed_Rkm_n,
	FrontWheelSpeed_Rm_n,
	TempSensorValue_n,//ECU2_ECU_INFO1数据6~7Byt	
	/**********四轮速度和温度数据解析**********/
	
	/**********AMS_IMD复位信号输入**********/	
	//ECU2_ECU_INFO2
	AMS_IMD_Reset_Input_n,		
	/**********AMS_IMD复位信号输入**********/
	
	/**********水泵，风扇，喇叭，尾灯等信号**********/	
	//ECU2_ECU_STATE数据解析
	WaterTank_Tstate_n,
	WaterPump_State_n,
	Fan_State_n,
	Taillight_State_n,
	Speeker_State_n	,
	/**********水泵，风扇，喇叭，尾灯等信号**********/
} Analysis_Data_n;
/**********接收枚举类型**********/

/**********发送HCU_BMS_CMD枚举类型**********/
typedef enum{
	HCU_BMS_CMD_online_offline_n=145,
	HCU_BMS_CMD_ShutDown_Boot_n,
}HCU_BMS_CMD_n;

/**********发送HCU_BMS_CMD枚举类型**********/

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
void CSU_CAN_Send(CAN_HandleTypeDef* canHandle,uint32_t ID,uint8_t *pData, uint8_t Size);//CAN发送函数,使用CAN控制器发送报文
void CSU_CAN1_Send_Mot(uint8_t REGID,uint16_t Data);//CAN1发送电机控制相关指令
void CSU_CAN1_Receive_Mot_Ready(uint8_t Data_RegID,uint8_t Data_Period);//CAN1发送获取电机消息请求,接收电机消息准备函数，后紧随CAN1的接收函数	
void CSU_CAN1_ModifySlaveAdd(uint16_t Data);//修改电机从站地址，并保存参数配置到EEPROM中

void Get_CAN_Analysis_Data(Analysis_Data_n name_num,void*extern_data);//取值函数，搬运CAN1/CAN2接收并解析好的数据,@Analysis_Data_n

void CAN1_DATA_Send(uint8_t RegID,uint16_t Ctrl_Data);		//CAN1发送电机控制数据函数
void CAN2_DATA_Send(uint32_t Protocol_ID);								//CAN2发送协议数据函数

osThreadId Get_CAN1TransmitQueueHandle(void);							//获取CAN1发送队列句柄
osThreadId Get_CAN2TransmitQueueHandle(void);							//获取CAN2发送队列句柄
TimerHandle_t Get_CAN2_PeriodicSendTimer_Handle(void);		//获取CAN2周期发送软件定时器句柄

void Set_CAN_Analysis_Data(Analysis_Data_n name_num,void*extern_data);//修改CAN1/CAN2接收并解析好的数据

void Get_HCU_BMS_CMD_Data(HCU_BMS_CMD_n name_num,void*extern_data);		//获取ECU_HCU_BMS_CMD命令数据
void Set_HCU_BMS_CMD_Data(HCU_BMS_CMD_n name_num,void*extern_data);		//修改ECU_HCU_BMS_CMD发送的命令数据
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
