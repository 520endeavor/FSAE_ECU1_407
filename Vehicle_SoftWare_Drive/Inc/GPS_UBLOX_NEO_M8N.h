
/*ʹ��UBLOX_NEO_M8N֮ǰ������λ��U-center������������ݽ�ΪGPRMC��Ϣ*/

#ifndef __GPS_UBLOX_NEO_M8N_H
#define __GPS_UBLOX_NEO_M8N_H 

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "string.h"	

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
/** 
  * @brief  GPS Status structures definition  
  */  
typedef enum 
{
	GPS_Longitude_n=300,
	GPS_Latitude_n,
	GPS_Altitude_n,
	GPS_StatusData_n,
}NEO_M8N_GPS_Data_n;
	
	
typedef enum 
{
  GPS_ANALYSIS_OK= 0x00U,
  GPS_MINSSING_GPRMC_ERROR= 0x01U,
  GPS_INVALID_SYMBOL_WARNING= 0x02U
} GPS_StatusTypeDef;
extern GPS_StatusTypeDef GPS_Status;

/*������Ϣ*/
typedef struct  
{										    
 	uint8_t num;			//���Ǳ��
	uint8_t eledeg;		//��������
	uint16_t azideg;	//���Ƿ�λ��
	uint8_t sn;				//�����		   
}NEO_M8N_slmsg;  

/*UTCʱ����Ϣ*/
typedef struct  
{										    
 	uint16_t year;		//���
	uint8_t month;		//�·�
	uint8_t date;			//����
	uint8_t hour; 		//Сʱ
	uint8_t min; 			//����
	uint8_t sec; 			//����
}NEO_M8N_utc_time; 

/*GPS���������Ϣ*/
 typedef struct  
{										    
 	uint8_t svnum;						//�ɼ�������
	NEO_M8N_slmsg slmsg[12];			//���12������
	NEO_M8N_utc_time utc;				//UTCʱ��
	uint32_t latitude;				//γ�� ������100000��,ʵ��Ҫ����100000
	uint8_t nshemi;						//��γ/��γ,N:��γ;S:��γ				  
	uint32_t longitude;			  //���� ������100000��,ʵ��Ҫ����100000
	uint8_t ewhemi;						//����/����,E:����;W:����
	uint8_t gpssta;						//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.				  
 	uint8_t posslnum;					//���ڶ�λ��������,0~12.
 	uint8_t possl[12];				//���ڶ�λ�����Ǳ��
	uint8_t fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	uint16_t pdop;						//λ�þ������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t hdop;						//ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t vdop;						//��ֱ�������� 0~500,��Ӧʵ��ֵ0~50.0 
	int altitude;			 				//���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m	 
	uint16_t speed;						//��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ	 
}NEO_M8N_MSG;

extern NEO_M8N_MSG GPSX;

/*Application Functions*/

void GPS_UBLOX_FREERTOS_Init(void);										//GPS���պ����н�������ĺ���

uint8_t NEO_M8N_Comma_Pos(uint8_t *buf,uint8_t cx);				//��buf����õ���cx���������ڵ�λ��
uint32_t NEO_M8N_Pow(uint8_t m,uint8_t n);									//m^n����
int NEO_M8N_Str2num(uint8_t *buf,uint8_t*dx);							//strת��Ϊ����,��','����'*'����
uint8_t NEO_M8N_GPRMC_Analysis(NEO_M8N_MSG *gpsx,uint8_t *buf);	//����GPRMC��Ϣ
uint8_t Get_GPS_NEO_M8N_Processed_Data(UART_HandleTypeDef *uartHandle,NEO_M8N_MSG *gpsx);//�õ�������GPS��γ�ȵ���Ϣ,GPS��γ�ȵ���ֵ������*gpsxָ��Ľṹ����

void Get_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data);//ȡֵ����������GPS���ղ������õ�����,@NEO_M8N_GPS_Data_n
void Set_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data);//�޸�GPS���ղ������õ�����,@NEO_M8N_GPS_Data_n

#endif

