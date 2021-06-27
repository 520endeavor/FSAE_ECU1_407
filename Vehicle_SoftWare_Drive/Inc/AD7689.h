#ifndef __AD7689_H
#define __AD7689_H 

#include "spi.h"
#include "gpio.h"
#include "can.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

typedef enum{
	LowVoltageBatV_n=125,						//低压电池电压AD值
	AccPedalOpeningLeft_n,					//油门传感器AD值
	AccPedalOpeningRight_n,					//油门传感器AD值
	BrakePressureFront_n,						//刹车油压传感器AD值
	BrakePressureBack_n,						//刹车油压传感器AD值
	SteeringAngle_n,								//转向角度传感器AD值
}AD7689GenericData_n; 

void Get_AD7689Data(SPI_HandleTypeDef *spiHandle,float *data);														//两个AD7618读取函数，注意SPI1读取AD2,SPI2读取AD1

void AD7689_Get_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data);//取值函数，搬运油门，油压，电压电池电压，转角等数据


#endif

