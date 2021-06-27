#ifndef __HOSTPCDISPLAY_H
#define __HOSTPCDISPLAY_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "can.h"
#include "spi.h"	 
#include "Data_Generic.h"
#include "HostPCDebug_AT24CxxMem.h"
#include "GPS_UBLOX_NEO_M8N.h"	

/*------------ECU1——Labview--------------------*/
#define TransmitLabviewModule3Header		(uint8_t)0xA8					//ECU1发送Labview模块3数据表头
//开辟单字节区域
#define AccPedalLeft_Open_ID										 0x71
#define AccPedalRight_Open_ID										 0x72
#define BrakePressureFront_ID										 0x73
#define BrakePressureBack_ID										 0x74
#define MaxVoltageNum_ID												 0x75
#define MinVoltageNum_ID												 0x76
#define MaxTemperatureNum_ID										 0x77
#define MinTemperatureNum_ID										 0x78
//开辟两字节区域
#define SterringWheelAngle_ID										 0x7D						 
#define CarSpeed_ID												 			 0x7E
#define CarMileage_ID														 0x7F
#define PowerBatTotalCurrent_ID									 0x80
#define PowerBatTotalVoltage_ID									 0x81
#define MaxVoltage_ID														 0x82
#define MinVoltage_ID                            0x83
#define LowVoltageBat_Voltage_ID	 	 	   				 0x84
#define MaxTemperature_ID												 0x85
#define MinTemperature_ID												 0x86
#define MotIGBTTemperature_ID										 0x87
#define MotAiremperature_ID										   0x88
#define GPSLongitudeData_ID										   0x89
#define GPSLatitudeData_ID										   0x8A
#define GPSAltitudeData_ID										   0x8B
#define SuspensionLineDisplacementF1_ID					 0x8C
#define SuspensionLineDisplacementF2_ID					 0x8D
#define SuspensionLineDisplacementB1_ID					 0x8E
#define SuspensionLineDisplacementB2_ID					 0x8F
#define WheelSpeedF1_ID					 								 0x90
#define WheelSpeedF2_ID					 								 0x91
#define WheelSpeedB1_ID					 								 0x92
#define WheelSpeedB2_ID					 								 0x93
//新增单字节区域
#define WaterPumpStatus_ID										   0xA0
#define SpeakerStatus_ID										  	 0xA1
#define TaillightStatus_ID										   0xA2
#define TCSStatus_ID										   			 0xA3

#define WirelessTransmitData3Num								 35			//发送数据的ID数


typedef struct{
	uint8_t WirelessTransmitID;
	uint8_t WirelessTransmitData[2];
}WirelessTransmit3_MessageTypedef;

void WirelessModule2_FREERTOS_Init(void);											//Wireless通信中建立任务的函数

//void ECU1_Labview_Module3Data_Send(uint8_t Labview_ID);				//投递Wireless发送内容到队列的函数，发送模块3的数据


 
#endif

