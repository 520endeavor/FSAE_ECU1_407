
/*使用UBLOX_NEO_M8N之前现在上位机U-center中设置输出内容仅为GPRMC信息*/
#include "GPS_UBLOX_NEO_M8N.h"	

GPS_StatusTypeDef GPS_Status;//定义GPS状态枚举变量
static uint8_t  GPS_Example_Data[]="$GPRMC,105230.00,A,2809.01555,N,11256.12232,E,0.022,,170717,,,A*74";
static uint16_t GPS_4SIZE=4*sizeof(GPS_Example_Data);     

struct{
	float GPS_Longitude;
	float GPS_latitude;
	float GPS_Altitude;
	uint8_t GPS_StatusData;
}NEO_M8N_GPS_Data;

osThreadId GPS_DataGenericTaskHandle;

void GPS_DataGenericTask(void const * argument);

void GPS_UBLOX_FREERTOS_Init(void)
{	
	/* definition and creation of GPS_DataGenericTask */
	osThreadDef(GPS_DataGenericTask, GPS_DataGenericTask, osPriorityNormal, 0, 128);
	GPS_DataGenericTaskHandle = osThreadCreate(osThread(GPS_DataGenericTask), NULL);	
} 

/*
*****************************************************************************
*@brief		从buf里面得到第cx个逗号所在的位置
*@retval	返回值:0~0XFE,代表逗号所在位置的偏移。0XFF,代表不存在第cx个逗号	
*@par
*****************************************************************************
*/
uint8_t NEO_M8N_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z'){
			return 0XFF;										//遇到'*'或者非法字符,则不存在第cx个逗号
		}
		if(*buf==','){
			cx--;
		}
		buf++;
	}
	return buf-p;	 
}

/*
*****************************************************************************
*@brief		m^n函数
*@param		m:底，n:幂
*@retval	返回值:m^n次方.
*@par
*****************************************************************************
*/
uint32_t NEO_M8N_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--){
		result*=m;    
	}
	return result;
}

/*
*****************************************************************************
*@brief		str转换为数字,以','或者'*'结束
*@param		buf:数字存储区
*@param		dx:小数点位数,返回给调用函数
*@retval	返回值:转换后的数值
*@par
*****************************************************************************
*/
int NEO_M8N_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) 														//得到整数和小数的长度
	{
		if(*p=='-'){											//是负数
			mask|=0X02;p++;
		}											
		if(*p==','||(*p=='*'))						//遇到结束了	
			break;																					
		if(*p=='.'){											//遇到小数点了
			mask|=0X01;p++;
		}											
		else if(*p>'9'||(*p<'0'))					//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01){
			flen++;
		}
		else{
			ilen++;
		}
		p++;
	}
	if(mask&0X02){
		buf++;													 //去掉负号
	}
	for(i=0;i<ilen;i++)								 //得到整数部分数据
	{  
		ires+=NEO_M8N_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5){
		flen=5;													 //最多取5位小数
	}
	*dx=flen;	 												 //小数点位数
	for(i=0;i<flen;i++)								 //得到小数部分数据
	{  
		fres+=NEO_M8N_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NEO_M8N_Pow(10,flen)+fres;
	if(mask&0X02){
		res=-res;		
	}		
	return res;
}	  					

/*
*****************************************************************************
*@brief		分析GPRMC信息
*@param		gpsx:nmea信息结构体
*@param		buf:接收到的GPS数据缓冲区首地址
*@retval	返回值：GPS_ANALYSIS_OK解析成功，其它解析遇到问题
*@par
*****************************************************************************
*/
uint8_t NEO_M8N_GPRMC_Analysis(NEO_M8N_MSG *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;  
	p1=(uint8_t*)strstr((const char *)buf,"GPRMC");			//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	if (p1==NULL){
		return GPS_MINSSING_GPRMC_ERROR;									//返回GPRMC数据缺失信息
	}
	posx=NEO_M8N_Comma_Pos(p1,1);										 		//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NEO_M8N_Str2num(p1+posx,&dx)/NEO_M8N_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;										//解析GPS数据时遇到非法字符
	}
	posx=NEO_M8N_Comma_Pos(p1,3);													//得到纬度
	if(posx!=0XFF)
	{
		temp=NEO_M8N_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NEO_M8N_Pow(10,dx+2);						//得到°
		rs=temp%NEO_M8N_Pow(10,dx+2);												//得到'		 
		gpsx->latitude=gpsx->latitude*NEO_M8N_Pow(10,5)+(rs*NEO_M8N_Pow(10,5-dx))/60;//转换为° 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;										//解析GPS数据时遇到非法字符
	}
	posx=NEO_M8N_Comma_Pos(p1,4);													//南纬还是北纬 
	if(posx!=0XFF){
		gpsx->nshemi=*(p1+posx);	
	}		
	else{
		return GPS_INVALID_SYMBOL_WARNING;										//解析GPS数据时遇到非法字符
	}
 	posx=NEO_M8N_Comma_Pos(p1,5);													//得到经度
	if(posx!=0XFF)
	{												  
		temp=NEO_M8N_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NEO_M8N_Pow(10,dx+2);						//得到°
		rs=temp%NEO_M8N_Pow(10,dx+2);													//得到'		 
		gpsx->longitude=gpsx->longitude*NEO_M8N_Pow(10,5)+(rs*NEO_M8N_Pow(10,5-dx))/60;//转换为° 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;											//解析GPS数据时遇到非法字符
	}
	posx=NEO_M8N_Comma_Pos(p1,6);														//东经还是西经
	if(posx!=0XFF){
		gpsx->ewhemi=*(p1+posx);		 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;											//解析GPS数据时遇到非法字符
	}
	posx=NEO_M8N_Comma_Pos(p1,9);														//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NEO_M8N_Str2num(p1+posx,&dx);		 								//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;											//解析GPS数据时遇到非法字符
	}	
	return GPS_ANALYSIS_OK;																					//解析正常
}

/*
*****************************************************************************
*@brief		得到换算后的GPS经纬度等信息，当经纬度、时间、日期等信息出现为0时，可能是GPS信号缺失
*@param		UART_HandleTypeDef uartHandle：串口结构体句柄
*@param		nmea_msg *gpsx:nmea信息结构体,GPS经纬度等数值保存在*gpsx指向的结构体中
*@retval	None
*@par
*****************************************************************************
*/
uint8_t Get_GPS_NEO_M8N_Processed_Data(UART_HandleTypeDef *uartHandle,NEO_M8N_MSG *gpsx)
{	
	uint8_t GPS_Receive_4Databuf[GPS_4SIZE];
	HAL_UART_Receive_DMA(uartHandle, GPS_Receive_4Databuf,GPS_4SIZE);
	return(NEO_M8N_GPRMC_Analysis(gpsx,GPS_Receive_4Databuf));
}

///*
//*****************************************************************************
//*@brief		GPS校验和计算
//*@param		buf:数据缓存区首地址
//*@param		len:数据长度
//*@param		cka,ckb:两个校验结果.
//*@retval	None
//*@par
//*****************************************************************************
//*/
//void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb)
//{
//	uint16_t i;
//	*cka=0;*ckb=0;
//	for(i=0;i<len;i++)
//	{
//		*cka=*cka+buf[i];
//		*ckb=*ckb+*cka;
//	}
//}

NEO_M8N_MSG GPSX;
/* GPS_DataGenericTask function */
void GPS_DataGenericTask(void const * argument)
{	
  /* USER CODE BEGIN GPS_DataGenericTask */
  /* Infinite loop */
	for(;;)
  {	
		NEO_M8N_GPS_Data.GPS_StatusData=Get_GPS_NEO_M8N_Processed_Data(&huart6,&GPSX);
		if(GPSX.nshemi=='N'){
			NEO_M8N_GPS_Data.GPS_latitude=GPSX.latitude/100000.0;
		}
		else if(GPSX.nshemi=='S'){
			NEO_M8N_GPS_Data.GPS_latitude=0-GPSX.latitude/100000.0;
		}
		if(GPSX.ewhemi=='W'){
			NEO_M8N_GPS_Data.GPS_Longitude=GPSX.longitude/100000.0;
		}
		else if(GPSX.ewhemi=='E'){
			NEO_M8N_GPS_Data.GPS_Longitude=0-GPSX.longitude/100000.0;
		}							
		NEO_M8N_GPS_Data.GPS_Altitude=GPSX.altitude/10.0;
		osDelay(50);
  }
  /* USER CODE END GPS_DataGenericTask */
}

/*
*****************************************************************************
*@brief		取值函数，搬运GPS接收并解析好的数据
*@param		NEO_M8N_GPS_Data_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	switch(name_num)
	{
		case GPS_Longitude_n:
			*(float*)extern_data=NEO_M8N_GPS_Data.GPS_Longitude;
			break;		
		case GPS_Latitude_n:
			*(float*)extern_data=NEO_M8N_GPS_Data.GPS_latitude;
			break;
		case GPS_Altitude_n:
			*(float*)extern_data=NEO_M8N_GPS_Data.GPS_Altitude;
			break;		
		case GPS_StatusData_n:
			*(uint8_t*)extern_data=NEO_M8N_GPS_Data.GPS_StatusData;
		break;
	}
}

/*
*****************************************************************************
*@brief		修改GPS接收并解析好的数据
*@param		NEO_M8N_GPS_Data_n name_num：修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data)//*extern_data是修改目标值变量
{
	switch(name_num)
	{
		case GPS_Longitude_n:
			NEO_M8N_GPS_Data.GPS_Longitude=*(float*)extern_data;
			break;		
		case GPS_Latitude_n:
			NEO_M8N_GPS_Data.GPS_latitude=*(float*)extern_data;
			break;
		case GPS_Altitude_n:
			NEO_M8N_GPS_Data.GPS_Altitude=*(float*)extern_data;
			break;		
		case GPS_StatusData_n:
			NEO_M8N_GPS_Data.GPS_StatusData=*(uint8_t*)extern_data;
		break;
	}
}