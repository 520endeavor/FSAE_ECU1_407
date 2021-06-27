#ifndef __AD7689_H
#define __AD7689_H 

#include "spi.h"
#include "gpio.h"
#include "can.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

typedef enum{
	LowVoltageBatV_n=125,						//��ѹ��ص�ѹADֵ
	AccPedalOpeningLeft_n,					//���Ŵ�����ADֵ
	AccPedalOpeningRight_n,					//���Ŵ�����ADֵ
	BrakePressureFront_n,						//ɲ����ѹ������ADֵ
	BrakePressureBack_n,						//ɲ����ѹ������ADֵ
	SteeringAngle_n,								//ת��Ƕȴ�����ADֵ
}AD7689GenericData_n; 

void Get_AD7689Data(SPI_HandleTypeDef *spiHandle,float *data);														//����AD7618��ȡ������ע��SPI1��ȡAD2,SPI2��ȡAD1

void AD7689_Get_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data);//ȡֵ�������������ţ���ѹ����ѹ��ص�ѹ��ת�ǵ�����


#endif

