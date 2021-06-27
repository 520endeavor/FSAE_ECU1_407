#include "AD7689.h"	


struct{
	uint8_t AccPedalOpeningLeft;
	uint8_t AccPedalOpeningRight;
	float BrakePressureFront;
	float BrakePressureBack;
	float SteeringAngle;	
	float LowVoltageBatV;
}Acc_Brake_Angle_LowVData;

/*
*****************************************************************************
*@brief		������ʱ����
*@param		uint32_t t����ʱ��С������6����
*@retval	None
*@par
*****************************************************************************
*/
static void AD7689_Delay_6ns(uint32_t t){
	while(t--);
}

/*
*****************************************************************************
*@brief		΢����ʱ����
*@param		uint32_t cnt����ʱ��С����λ΢��
*@retval	None
*@par
*****************************************************************************
*/
static void AD7689_Delay_us(uint32_t cnt){
	uint32_t i,j;  
	for(i=0;i<cnt;i++)  
	{  
			for(j=0;j<35;j++);  
	}
}

/*
*****************************************************************************
*@brief		����AD7618��ȡ������ע��SPI1��ȡ����AD2�����ݣ���ͨ��AD8-AD15
					SPI2��ȡ����AD1�����ݣ���ͨ��AD0-AD7
*@param		SPI_HandleTypeDef *spiHandle��SPI�ṹ����
*@param		float *data���洢ADģ���������׵�ַ
*@retval	None
*@par
*****************************************************************************
*/
void Get_AD7689Data(SPI_HandleTypeDef *spiHandle,float *data){
	uint16_t CFG=0x3C49;
	uint8_t txData[2],rxData[2];
	uint8_t DumbrxData[2];					//�洢�ƶ�����
	uint8_t i=0,b=0;
	uint16_t a;
	/*�ƶ�����*/
	CFG=(CFG&0x3C7F)|(i<<7);
	txData[0]=(CFG<<2)>>8;											//�ڲ���׼Դ,REF=4.096,��ֹ������
	txData[1]=(CFG<<2)&0x00FF;		
	if(spiHandle->Instance==SPI2){										//ʹ��SPI2��ȡAD1������
		for(uint8_t j=0;j<2;j++){
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[0],&DumbrxData[0],1,100);//�����֣�����ͨ��IN0
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[1], &DumbrxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);													//�����з���EOC
			a=(DumbrxData[0]<<8)|DumbrxData[1];					//���յ�����
			if(j==1){
				data[i]=a/65536.0*4.096;										//REF=4.096
			}
		}
		/*��ȡ8��ͨ������*/	
		for (i=1;i<9;i++){			
			if(i==8){
				i=0;
				b=1;
			}
			CFG=(CFG&0x3C7F)|(i<<7);
			txData[0]=(CFG<<2)>>8;										//�ڲ���׼Դ,REF=4.096,��ֹ������
			txData[1]=(CFG<<2)&0x00FF;		
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[0],&rxData[0],1,100);//�����֣�����ͨ��IN   i+2
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[1], &rxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);											//�����з���EOC
			a=(rxData[0]<<8)|rxData[1];							//���յ�����
			if(b==1){
				i=8;
			}
			data[i-1]=a/65536.0*4.096;							//REF=4.096
		}	
	}
	else if(spiHandle->Instance==SPI1){										//ʹ��SPI1��ȡAD2������
		for(uint8_t j=0;j<2;j++){
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&DumbrxData[0],1,100);//�����֣�����ͨ��IN0
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &DumbrxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);													//�����з���EOC
			a=(DumbrxData[0]<<8)|DumbrxData[1];					//���յ�����
			if(j==1){
				data[i]=a/65536.0*4.096;										//REF=4.096
			}
		}
		/*��ȡ8��ͨ������*/	
		for (i=1;i<9;i++){			
			if(i==8){
				i=0;
				b=1;
			}
			CFG=(CFG&0x3C7F)|(i<<7);
			txData[0]=(CFG<<2)>>8;										//�ڲ���׼Դ,REF=4.096,��ֹ������
			txData[1]=(CFG<<2)&0x00FF;		
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&rxData[0],1,100);//�����֣�����ͨ��IN   i+2
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &rxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);											//�����з���EOC
			a=(rxData[0]<<8)|rxData[1];							//���յ�����
			if(b==1){
				i=8;
			}
			data[i-1]=a/65536.0*4.096;							//REF=4.096
		}			
	 }	
}
	
void ADValueGenericTask(void const * argument)
{
	


}








/*
*****************************************************************************
*@brief		ȡֵ�������������ţ���ѹ����ѹ��ص�ѹ��ת�ǵ�����
*@param		TimGenericData_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		const void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void AD7689_Get_Acc_Brake_Angle_LowVData(AD7689GenericData_n name_num,void * extern_data)
{
	if(name_num==AccPedalOpeningLeft_n){
		*(uint8_t*)extern_data=Acc_Brake_Angle_LowVData.AccPedalOpeningLeft;
	} 	
	else if(name_num==AccPedalOpeningRight_n){
		*(uint8_t*)extern_data=Acc_Brake_Angle_LowVData.AccPedalOpeningRight;
	}
	else if(name_num==BrakePressureFront_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.BrakePressureFront;
	}
	else if(name_num==BrakePressureBack_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.BrakePressureBack;
	}
	else if(name_num==SteeringAngle_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.SteeringAngle;
	}
		else if(name_num==LowVoltageBatV_n){
		*(float*)extern_data=Acc_Brake_Angle_LowVData.LowVoltageBatV;
	}
}


