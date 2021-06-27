
/*使用UBLOX_NEO_M8N之前现在上位机U-center中设置输出内容仅为GPRMC信息*/

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

/*卫星信息*/
typedef struct  
{										    
 	uint8_t num;			//卫星编号
	uint8_t eledeg;		//卫星仰角
	uint16_t azideg;	//卫星方位角
	uint8_t sn;				//信噪比		   
}NEO_M8N_slmsg;  

/*UTC时间信息*/
typedef struct  
{										    
 	uint16_t year;		//年份
	uint8_t month;		//月份
	uint8_t date;			//日期
	uint8_t hour; 		//小时
	uint8_t min; 			//分钟
	uint8_t sec; 			//秒钟
}NEO_M8N_utc_time; 

/*GPS输出数据信息*/
 typedef struct  
{										    
 	uint8_t svnum;						//可见卫星数
	NEO_M8N_slmsg slmsg[12];			//最多12颗卫星
	NEO_M8N_utc_time utc;				//UTC时间
	uint32_t latitude;				//纬度 分扩大100000倍,实际要除以100000
	uint8_t nshemi;						//北纬/南纬,N:北纬;S:南纬				  
	uint32_t longitude;			  //经度 分扩大100000倍,实际要除以100000
	uint8_t ewhemi;						//东经/西经,E:东经;W:西经
	uint8_t gpssta;						//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.				  
 	uint8_t posslnum;					//用于定位的卫星数,0~12.
 	uint8_t possl[12];				//用于定位的卫星编号
	uint8_t fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
	uint16_t pdop;						//位置精度因子 0~500,对应实际值0~50.0
	uint16_t hdop;						//水平精度因子 0~500,对应实际值0~50.0
	uint16_t vdop;						//垂直精度因子 0~500,对应实际值0~50.0 
	int altitude;			 				//海拔高度,放大了10倍,实际除以10.单位:0.1m	 
	uint16_t speed;						//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时	 
}NEO_M8N_MSG;

extern NEO_M8N_MSG GPSX;

/*Application Functions*/

void GPS_UBLOX_FREERTOS_Init(void);										//GPS接收函数中建立任务的函数

uint8_t NEO_M8N_Comma_Pos(uint8_t *buf,uint8_t cx);				//从buf里面得到第cx个逗号所在的位置
uint32_t NEO_M8N_Pow(uint8_t m,uint8_t n);									//m^n函数
int NEO_M8N_Str2num(uint8_t *buf,uint8_t*dx);							//str转换为数字,以','或者'*'结束
uint8_t NEO_M8N_GPRMC_Analysis(NEO_M8N_MSG *gpsx,uint8_t *buf);	//分析GPRMC信息
uint8_t Get_GPS_NEO_M8N_Processed_Data(UART_HandleTypeDef *uartHandle,NEO_M8N_MSG *gpsx);//得到换算后的GPS经纬度等信息,GPS经纬度等数值保存在*gpsx指向的结构体中

void Get_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data);//取值函数，搬运GPS接收并解析好的数据,@NEO_M8N_GPS_Data_n
void Set_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data);//修改GPS接收并解析好的数据,@NEO_M8N_GPS_Data_n

#endif

