#ifndef __HOSTPCDEBUG_AT24CXXMEM_H
#define __HOSTPCDEBUG_AT24CXXMEM_H

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "Wireless_AS32_TTL_1W.h"
#include "AT24Cxx.h" 
#include "Data_Generic.h"
#include "spi.h"
#include "can.h"
//#include "HostPCDisplay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
/*------------�洢��EEPROM�еĵ�ַ------------*/
#define Mileage_ID 														0xFF						//������ݣ����ֽ�
#define LeakageCurrentLimit_State_Data_ID			0x0101					//IMD ©�糬�� ״̬���ݣ����ֽ�		
/*------------Labview����ECU1&ECU1����LabviewID--------------------*/
//ECU1����&����ģ��1,����IDͬΪEEPROM�еĴ洢��ַ
#define Transmit1AndReceiveLabviewHeader		(uint16_t)0xA6A6		  //ECU1����&���͵����ݱ�ͷ
#define BrakePressure_Range_High_ID								0x00 						//�ƶ�̤����ѹ������ѹ�����̸�����ID���������ߴ��ڽ���&����
#define BrakePressure_Range_Low_ID								0x02						//�ƶ�̤����ѹ������ѹ�����̵�����ID���������ߴ��ڽ���&����
#define BrakePressure_Range_ActualHigh_ID					0x04						//�ƶ�̤����ѹ������ʵ�ʸ�ID���������ߴ��ڽ���&����
#define BrakePressure_Range_ActualLow_ID					0x06						//�ƶ�̤����ѹ������ʵ�ʵIDͣ��������ߴ��ڽ���&����
#define AccPeda_Range_High_ID											0x08						//����̤���г���������ID���������ߴ��ڽ���&����
#define AccPeda_Range_Low_ID											0x0A						//����̤���г���������ID���������ߴ��ڽ���&����
#define AccPeda_InitialDiff_Of_LandR_ID						0x0C						//���Ҽ���̤���ʼʱ����ֵID���������ߴ��ڽ���&����
#define SterringWheelAngle_Range_ID		 						0x0E						//������ת�Ǵ��������ת���Ƕ�����ID���������ߴ��ڽ���&����
#define SterringWheel_ADMiddle_ID									0x10						//�������м�λ��AD������ID���������ߴ��ڽ���&����
#define SterringWheel_TurnRange_ID								0x12						//����������ת����ΧAD������ ID���������ߴ��ڽ���&����
#define MotCtrllerTempThreshold_High_ID						0x14						//����ˮ�ÿ����ĵ���������¶�����ID���������ߴ��ڽ���&����
#define MotCtrllerTempThreshold_Low_ID						0x16						//����ˮ�ùرյĵ���������¶�����ID���������ߴ��ڽ���&����
#define Line_Corners_TCSMode_Angle_ID							0x18						//����ǣ����ģʽѡ��ķ����̽Ƕ�����ID���������ߴ��ڽ���&����
#define DRS_SteeringWheel_Angle_ID								0x1A						//����DRS�����ķ����̽Ƕ�����ID���������ߴ��ڽ���&����
#define Close_DRS_AdjustAngle_ID									0x1C						//����DRSʱ��β���ֵĽǶ�����ID���������ߴ��ڽ���&����
#define Open_DRS_AdjustAngle_ID										0x1E						//�ر�DRSʱ��β���ֵĽǶ�����ID���������ߴ��ڽ���&����
#define TCS_PID_Kp_ID															0x20						//ǣ�������Ʊ���ϵ������ID���������ߴ��ڽ���&����
#define TCS_PID_Ki_ID															0x22						//ǣ�������ƻ���ϵ������ID���������ߴ��ڽ���&����
#define TCS_PID_Kd_ID															0x24						//ǣ��������΢��ϵ������ID���������ߴ��ڽ���&����
#define OptimalSlipRate_ID												0x26						//ǣ�����������Ż���������ID���������ߴ��ڽ���&����
#define SuspensionLineDistF1_ADMiddle_ID					0x28						//ǰ������λ������м�λ��AD����ID���������ߴ��ڽ���&����
#define SuspensionLineDistF2_ADMiddle_ID					0x2A						//ǰ������λ���ҵ��м�λ��AD����ID���������ߴ��ڽ���&����
#define SuspensionLineDistB1_ADMiddle_ID					0x2C						//��������λ������м�λ��AD����ID���������ߴ��ڽ���&����
#define SuspensionLineDistB2_ADMiddle_ID					0x2E						//��������λ���ҵ��м�λ��AD����ID���������ߴ��ڽ���&����
#define WirelessReceiveDataNum										24							//���߽���Labview����������
//ECU1����ģ��2
#define TransmitLabviewModule2Header		(uint16_t)0xA7A7					//ECU1���͵�Labviewģ��2���ݱ�ͷ
#define BrakePressureFrontDisplay_ID							0x64						//ɲ����ѹǰ������ʾID,�������ߴ��ڷ��͵���λ��
#define BrakePressureBackDisplay_ID								0x66						//ɲ����ѹ��������ʾID,�������ߴ��ڷ��͵���λ��
#define AccPedaLeftAdDigitalValueDisplay_ID				0x68						//����̤����AD������ID,�������ߴ��ڷ��͵���λ��
#define AccPedaRightAdDigitalValueDisplay_ID			0x6A						//����̤����AD������ID,�������ߴ��ڷ��͵���λ��
#define SterringWheelAdDigitalValueDisplay_ID			0x6C						//������AD������ID,�������ߴ��ڷ��͵���λ��
#define WheelSpeedFrontDisplay_ID									0x6E						//ǰ������ID,�������ߴ��ڷ��͵���λ��
#define WheelSpeedBackDisplay_ID									0x70						//��������ID,�������ߴ��ڷ��͵���λ��
#define CurrentSlipRateDisplay_ID									0x72						//��ǰ������ID,�������ߴ��ڷ��͵���λ��
#define SuspensionLineDistF1_ADView_ID					 	0x74						//ǰ������λ����ǰAD���ݵ�ID���������ߴ��ڷ��͵���λ��
#define SuspensionLineDistF2_ADView_ID					 	0x76						//ǰ������λ���ҵ�ǰAD���ݵ�ID���������ߴ��ڷ��͵���λ��
#define SuspensionLineDistB1_ADView_ID					 	0x78						//��������λ����ǰAD���ݵ�ID���������ߴ��ڷ��͵���λ��
#define SuspensionLineDistB2_ADView_ID						0x7A						//��������λ���ҵ�ǰAD���ݵ�ID���������ߴ��ڷ��͵���λ��
#define TCS_PID_OUT_View_ID												0x7C						//TCS��PID����������ݵ�ID���������ߴ��ڷ��͵���λ��
#define WirelessTransmitData2Num									13							//���߷��͵�Labviewģ��2����������

//�洢������λ�Ƶ���ADֵ��EEPROM�еĴ洢��ַ
#define SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID  	0x30 	//ǰ������λ������м�λ��AD����
#define SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_ID		0x32	//ǰ������λ���ҵ��м�λ��AD����
#define SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_ID		0x34	//��������λ������м�λ��AD����
#define SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_ID		0x36	//��������λ���ҵ��м�λ��AD����

typedef struct{
	uint8_t EEPROMMemoryID;
	uint8_t EEPROMMemoryData[2];
}EEPROMMemoryMessageTypedef;


typedef struct{
	uint8_t WirelessTransmitID;
	uint8_t WirelessTransmitData[2];
}WirelessTransmit1And2_MessageTypedef;


typedef enum{
	BrakePressure_Range_High_n=250,		
	BrakePressure_Range_Low_n,
	BrakePressure_Range_ActualHigh_n,
	BrakePressure_Range_ActualLow_n,
	AccPeda_Range_High_n,
	AccPeda_Range_Low_n,	
	AccPeda_InitialDiff_Of_LandR_n,
	SterringWheelAngle_Range_n,
	SterringWheel_ADMiddle_n,
	SterringWheel_TurnRange_n,
	MotCtrllerTempThreshold_High_n,
	MotCtrllerTempThreshold_Low_n,
	Line_Corners_TCSMode_Angle_n,
	DRS_SteeringWheel_Angle_n,
	Close_DRS_AdjustAngle_n,
	Open_DRS_AdjustAngle_n,
	TCS_PID_Kp_n,
	TCS_PID_Ki_n,
	TCS_PID_Kd_n,
	OptimalSlipRate_n,
	SuspensionLineDistF1_ADMiddle_n,
	SuspensionLineDistF2_ADMiddle_n,
	SuspensionLineDistB1_ADMiddle_n,
	SuspensionLineDistB2_ADMiddle_n,
	
	TCS_PID_OUT_n,
}Wireless_ReceiveData_n;

typedef enum{
	SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_n=290,	//ǰ������λ������м�λ��AD����
	SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_n,			//ǰ������λ���ҵ��м�λ��AD����
	SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_n,			//��������λ������м�λ��AD����
	SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_n			//��������λ���ҵ��м�λ��AD����
}SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n;

void Wireless_FREERTOS_Init(void);															//Wirelessͨ���н�������ĺ���

osThreadId Get_WirelessTransmit1And2_LabviewQueueHandle(void);	//���Wireless���͵�Labview���ݵĶ���

void Wireless_DATA_Send1And2(uint8_t Labview_ID);								//Ͷ��Wireless�������ݵ����еĺ���������ģ��1��2������

void FirstPowerOnReadEEPROMInit(void);													//��һ���ϵ��ȡEEPROM������

void Get_Wireless_DataFrom_EEPROM(Wireless_ReceiveData_n name_num,void*extern_data);//ȡֵ����������Wireless���ղ������õ�����,@Wireless_ReceiveData_n
void Get_TCS_PID_OUT_Data(Wireless_ReceiveData_n name_num,void*extern_data);//ȡֵ����������TCS_PID_OUT�������@Wireless_ReceiveData_n
void Get_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num,void*extern_data);//ȡֵ���������� ������λ�Ƶ���ʱ��������λ������λ�õ�ADֵ
void Set_Wireless_DataFrom_EEPROM(Wireless_ReceiveData_n name_num,void*extern_data);//�޸�Wireless���ղ������õ�����,@Wireless_ReceiveData_n
void Set_TCS_PID_OUT_Data(Wireless_ReceiveData_n name_num,void*extern_data);//�޸�TCS_PID_OUT��ر���@Wireless_ReceiveData_n
void Set_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num,void*extern_data);//�޸ĺ������޸� ������λ�Ƶ���ʱ��������λ������λ�õ�ADֵ
#endif

