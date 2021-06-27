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
/*------------´æ´¢ÔÚEEPROMÖĞµÄµØÖ·------------*/
#define Mileage_ID 														0xFF						//Àï³ÌÊı¾İ£¬Á½×Ö½Ú
#define LeakageCurrentLimit_State_Data_ID			0x0101					//IMD Â©µç³¬ÏŞ ×´Ì¬Êı¾İ£¬µ¥×Ö½Ú		
/*------------Labview¡ª¡ªECU1&ECU1¡ª¡ªLabviewID--------------------*/
//ECU1½ÓÊÕ&·¢ËÍÄ£¿é1,¸ÃÁĞIDÍ¬ÎªEEPROMÖĞµÄ´æ´¢µØÖ·
#define Transmit1AndReceiveLabviewHeader		(uint16_t)0xA6A6		  //ECU1½ÓÊÕ&·¢ËÍµÄÊı¾İ±íÍ·
#define BrakePressure_Range_High_ID								0x00 						//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Ñ¹Á¦Á¿³Ì¸ßÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define BrakePressure_Range_Low_ID								0x02						//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Ñ¹Á¦Á¿³ÌµÍÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define BrakePressure_Range_ActualHigh_ID					0x04						//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Êµ¼Ê¸ßID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define BrakePressure_Range_ActualLow_ID					0x06						//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Êµ¼ÊµIDÍ£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define AccPeda_Range_High_ID											0x08						//¼ÓËÙÌ¤°åĞĞ³ÌÉÏÏŞÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define AccPeda_Range_Low_ID											0x0A						//¼ÓËÙÌ¤°åĞĞ³ÌÏÂÏŞÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define AccPeda_InitialDiff_Of_LandR_ID						0x0C						//×óÓÒ¼ÓËÙÌ¤°å³õÊ¼Ê±²îÒìÖµID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define SterringWheelAngle_Range_ID		 						0x0E						//·½ÏòÅÌ×ª½Ç´«¸ĞÆ÷×î´ó×ª¶¯½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define SterringWheel_ADMiddle_ID									0x10						//·½ÏòÅÌÖĞ¼äÎ»ÖÃADÊı×ÖÁ¿ID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define SterringWheel_TurnRange_ID								0x12						//·½ÏòÅÌ×óÓÒ×ª¶¯·¶Î§ADÊı×ÖÁ¿ ID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define MotCtrllerTempThreshold_High_ID						0x14						//¿ØÖÆË®±Ã¿ªÆôµÄµç»ú¿ØÖÆÆ÷ÎÂ¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define MotCtrllerTempThreshold_Low_ID						0x16						//¿ØÖÆË®±Ã¹Ø±ÕµÄµç»ú¿ØÖÆÆ÷ÎÂ¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define Line_Corners_TCSMode_Angle_ID							0x18						//¿ØÖÆÇ£ÒıÁ¦Ä£Ê½Ñ¡ÔñµÄ·½ÏòÅÌ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define DRS_SteeringWheel_Angle_ID								0x1A						//¿ØÖÆDRS¿ªÆôµÄ·½ÏòÅÌ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define Close_DRS_AdjustAngle_ID									0x1C						//¿ªÆôDRSÊ±£¬Î²Òí±£³ÖµÄ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define Open_DRS_AdjustAngle_ID										0x1E						//¹Ø±ÕDRSÊ±£¬Î²Òí±£³ÖµÄ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define TCS_PID_Kp_ID															0x20						//Ç£ÒıÁ¦¿ØÖÆ±ÈÀıÏµÊıÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define TCS_PID_Ki_ID															0x22						//Ç£ÒıÁ¦¿ØÖÆ»ı·ÖÏµÊıÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define TCS_PID_Kd_ID															0x24						//Ç£ÒıÁ¦¿ØÖÆÎ¢·ÖÏµÊıÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define OptimalSlipRate_ID												0x26						//Ç£ÒıÁ¦¿ØÖÆ×îÓÅ»¬ÒÆÂÊÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define SuspensionLineDistF1_ADMiddle_ID					0x28						//Ç°Ğü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define SuspensionLineDistF2_ADMiddle_ID					0x2A						//Ç°Ğü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define SuspensionLineDistB1_ADMiddle_ID					0x2C						//ºóĞü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define SuspensionLineDistB2_ADMiddle_ID					0x2E						//ºóĞü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
#define WirelessReceiveDataNum										24							//ÎŞÏß½ÓÊÕLabviewµÄÊı¾İÊıÁ¿
//ECU1·¢ËÍÄ£¿é2
#define TransmitLabviewModule2Header		(uint16_t)0xA7A7					//ECU1·¢ËÍµ½LabviewÄ£¿é2Êı¾İ±íÍ·
#define BrakePressureFrontDisplay_ID							0x64						//É²³µÓÍÑ¹Ç°Êı¾İÏÔÊ¾ID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define BrakePressureBackDisplay_ID								0x66						//É²³µÓÍÑ¹ºóÊı¾İÏÔÊ¾ID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define AccPedaLeftAdDigitalValueDisplay_ID				0x68						//¼ÓËÙÌ¤°å×óADÊı×ÖÁ¿ID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define AccPedaRightAdDigitalValueDisplay_ID			0x6A						//¼ÓËÙÌ¤°åÓÒADÊı×ÖÁ¿ID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define SterringWheelAdDigitalValueDisplay_ID			0x6C						//·½ÏòÅÌADÊı×ÖÁ¿ID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define WheelSpeedFrontDisplay_ID									0x6E						//Ç°ÂÖÂÖËÙID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define WheelSpeedBackDisplay_ID									0x70						//ºóÂÖÂÖËÙID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define CurrentSlipRateDisplay_ID									0x72						//µ±Ç°»¬ÒÆÂÊID,ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define SuspensionLineDistF1_ADView_ID					 	0x74						//Ç°Ğü¼ÜÏßÎ»ÒÆ×óµ±Ç°ADÊı¾İµÄID£¬ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define SuspensionLineDistF2_ADView_ID					 	0x76						//Ç°Ğü¼ÜÏßÎ»ÒÆÓÒµ±Ç°ADÊı¾İµÄID£¬ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define SuspensionLineDistB1_ADView_ID					 	0x78						//ºóĞü¼ÜÏßÎ»ÒÆ×óµ±Ç°ADÊı¾İµÄID£¬ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define SuspensionLineDistB2_ADView_ID						0x7A						//ºóĞü¼ÜÏßÎ»ÒÆÓÒµ±Ç°ADÊı¾İµÄID£¬ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define TCS_PID_OUT_View_ID												0x7C						//TCSµÄPIDµ÷½ÚÊä³öÊı¾İµÄID£¬ÓÃÓÚÎŞÏß´®¿Ú·¢ËÍµ½ÉÏÎ»»ú
#define WirelessTransmitData2Num									13							//ÎŞÏß·¢ËÍµ½LabviewÄ£¿é2µÄÊı¾İÊıÁ¿

//´æ´¢Ğü¼ÜÏßÎ»ÒÆµ÷ÁãADÖµµ½EEPROMÖĞµÄ´æ´¢µØÖ·
#define SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_ID  	0x30 	//Ç°Ğü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İ
#define SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_ID		0x32	//Ç°Ğü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İ
#define SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_ID		0x34	//ºóĞü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İ
#define SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_ID		0x36	//ºóĞü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İ

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
	SuspensionLineShiftF1_ADMiddle_For_Key_ZeroSetting_n=290,	//Ç°Ğü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İ
	SuspensionLineShiftF2_ADMiddle_For_Key_ZeroSetting_n,			//Ç°Ğü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İ
	SuspensionLineShiftB1_ADMiddle_For_Key_ZeroSetting_n,			//ºóĞü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İ
	SuspensionLineShiftB2_ADMiddle_For_Key_ZeroSetting_n			//ºóĞü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İ
}SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n;

void Wireless_FREERTOS_Init(void);															//WirelessÍ¨ĞÅÖĞ½¨Á¢ÈÎÎñµÄº¯Êı

osThreadId Get_WirelessTransmit1And2_LabviewQueueHandle(void);	//´æ·ÅWireless·¢ËÍµ½LabviewÊı¾İµÄ¶ÓÁĞ

void Wireless_DATA_Send1And2(uint8_t Labview_ID);								//Í¶µİWireless·¢ËÍÄÚÈİµ½¶ÓÁĞµÄº¯Êı£¬·¢ËÍÄ£¿é1ºÍ2µÄÊı¾İ

void FirstPowerOnReadEEPROMInit(void);													//µÚÒ»´ÎÉÏµç¶ÁÈ¡EEPROMÖĞÊı¾İ

void Get_Wireless_DataFrom_EEPROM(Wireless_ReceiveData_n name_num,void*extern_data);//È¡Öµº¯Êı£¬°áÔËWireless½ÓÊÕ²¢½âÎöºÃµÄÊı¾İ,@Wireless_ReceiveData_n
void Get_TCS_PID_OUT_Data(Wireless_ReceiveData_n name_num,void*extern_data);//È¡Öµº¯Êı£¬°áÔËTCS_PID_OUTÏà¹ØÊı¾İ@Wireless_ReceiveData_n
void Get_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num,void*extern_data);//È¡Öµº¯Êı£¬°áÔË Ğü¼ÜÏßÎ»ÒÆµ÷ÁãÊ±£¬Ğü¼ÜÏßÎ»ÒÆËù´¦Î»ÖÃµÄADÖµ
void Set_Wireless_DataFrom_EEPROM(Wireless_ReceiveData_n name_num,void*extern_data);//ĞŞ¸ÄWireless½ÓÊÕ²¢½âÎöºÃµÄÊı¾İ,@Wireless_ReceiveData_n
void Set_TCS_PID_OUT_Data(Wireless_ReceiveData_n name_num,void*extern_data);//ĞŞ¸ÄTCS_PID_OUTÏà¹Ø±äÁ¿@Wireless_ReceiveData_n
void Set_SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_Data(SuspensionLineShift_ADMiddle_For_Key_ZeroSetting_n name_num,void*extern_data);//ĞŞ¸Äº¯Êı£¬ĞŞ¸Ä Ğü¼ÜÏßÎ»ÒÆµ÷ÁãÊ±£¬Ğü¼ÜÏßÎ»ÒÆËù´¦Î»ÖÃµÄADÖµ
#endif

