
/*ʹ��UBLOX_NEO_M8N֮ǰ������λ��U-center������������ݽ�ΪGPRMC��Ϣ*/
#include "GPS_UBLOX_NEO_M8N.h"	

GPS_StatusTypeDef GPS_Status;//����GPS״̬ö�ٱ���
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
*@brief		��buf����õ���cx���������ڵ�λ��
*@retval	����ֵ:0~0XFE,����������λ�õ�ƫ�ơ�0XFF,�������ڵ�cx������	
*@par
*****************************************************************************
*/
uint8_t NEO_M8N_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z'){
			return 0XFF;										//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
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
*@brief		m^n����
*@param		m:�ף�n:��
*@retval	����ֵ:m^n�η�.
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
*@brief		strת��Ϊ����,��','����'*'����
*@param		buf:���ִ洢��
*@param		dx:С����λ��,���ظ����ú���
*@retval	����ֵ:ת�������ֵ
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
	while(1) 														//�õ�������С���ĳ���
	{
		if(*p=='-'){											//�Ǹ���
			mask|=0X02;p++;
		}											
		if(*p==','||(*p=='*'))						//����������	
			break;																					
		if(*p=='.'){											//����С������
			mask|=0X01;p++;
		}											
		else if(*p>'9'||(*p<'0'))					//�зǷ��ַ�
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
		buf++;													 //ȥ������
	}
	for(i=0;i<ilen;i++)								 //�õ�������������
	{  
		ires+=NEO_M8N_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5){
		flen=5;													 //���ȡ5λС��
	}
	*dx=flen;	 												 //С����λ��
	for(i=0;i<flen;i++)								 //�õ�С����������
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
*@brief		����GPRMC��Ϣ
*@param		gpsx:nmea��Ϣ�ṹ��
*@param		buf:���յ���GPS���ݻ������׵�ַ
*@retval	����ֵ��GPS_ANALYSIS_OK�����ɹ�������������������
*@par
*****************************************************************************
*/
uint8_t NEO_M8N_GPRMC_Analysis(NEO_M8N_MSG *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;  
	p1=(uint8_t*)strstr((const char *)buf,"GPRMC");			//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	if (p1==NULL){
		return GPS_MINSSING_GPRMC_ERROR;									//����GPRMC����ȱʧ��Ϣ
	}
	posx=NEO_M8N_Comma_Pos(p1,1);										 		//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NEO_M8N_Str2num(p1+posx,&dx)/NEO_M8N_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;										//����GPS����ʱ�����Ƿ��ַ�
	}
	posx=NEO_M8N_Comma_Pos(p1,3);													//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NEO_M8N_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NEO_M8N_Pow(10,dx+2);						//�õ���
		rs=temp%NEO_M8N_Pow(10,dx+2);												//�õ�'		 
		gpsx->latitude=gpsx->latitude*NEO_M8N_Pow(10,5)+(rs*NEO_M8N_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;										//����GPS����ʱ�����Ƿ��ַ�
	}
	posx=NEO_M8N_Comma_Pos(p1,4);													//��γ���Ǳ�γ 
	if(posx!=0XFF){
		gpsx->nshemi=*(p1+posx);	
	}		
	else{
		return GPS_INVALID_SYMBOL_WARNING;										//����GPS����ʱ�����Ƿ��ַ�
	}
 	posx=NEO_M8N_Comma_Pos(p1,5);													//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NEO_M8N_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NEO_M8N_Pow(10,dx+2);						//�õ���
		rs=temp%NEO_M8N_Pow(10,dx+2);													//�õ�'		 
		gpsx->longitude=gpsx->longitude*NEO_M8N_Pow(10,5)+(rs*NEO_M8N_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;											//����GPS����ʱ�����Ƿ��ַ�
	}
	posx=NEO_M8N_Comma_Pos(p1,6);														//������������
	if(posx!=0XFF){
		gpsx->ewhemi=*(p1+posx);		 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;											//����GPS����ʱ�����Ƿ��ַ�
	}
	posx=NEO_M8N_Comma_Pos(p1,9);														//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NEO_M8N_Str2num(p1+posx,&dx);		 								//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	}
	else{
		return GPS_INVALID_SYMBOL_WARNING;											//����GPS����ʱ�����Ƿ��ַ�
	}	
	return GPS_ANALYSIS_OK;																					//��������
}

/*
*****************************************************************************
*@brief		�õ�������GPS��γ�ȵ���Ϣ������γ�ȡ�ʱ�䡢���ڵ���Ϣ����Ϊ0ʱ��������GPS�ź�ȱʧ
*@param		UART_HandleTypeDef uartHandle�����ڽṹ����
*@param		nmea_msg *gpsx:nmea��Ϣ�ṹ��,GPS��γ�ȵ���ֵ������*gpsxָ��Ľṹ����
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
//*@brief		GPSУ��ͼ���
//*@param		buf:���ݻ������׵�ַ
//*@param		len:���ݳ���
//*@param		cka,ckb:����У����.
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
*@brief		ȡֵ����������GPS���ղ������õ�����
*@param		NEO_M8N_GPS_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data)//*extern_data�������get����ֵ
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
*@brief		�޸�GPS���ղ������õ�����
*@param		NEO_M8N_GPS_Data_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_NEO_M8N_GPRMC_AnalysisData(NEO_M8N_GPS_Data_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
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