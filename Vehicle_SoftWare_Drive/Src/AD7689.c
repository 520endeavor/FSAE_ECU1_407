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
*@brief		纳秒延时函数
*@param		uint32_t t，延时大小，步长6纳秒
*@retval	None
*@par
*****************************************************************************
*/
static void AD7689_Delay_6ns(uint32_t t){
	while(t--);
}

/*
*****************************************************************************
*@brief		微秒延时函数
*@param		uint32_t cnt，延时大小，单位微秒
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
*@brief		两个AD7618读取函数，注意SPI1读取的是AD2的数据，即通道AD8-AD15
					SPI2读取的是AD1的数据，即通道AD0-AD7
*@param		SPI_HandleTypeDef *spiHandle：SPI结构体句柄
*@param		float *data：存储AD模拟量数组首地址
*@retval	None
*@par
*****************************************************************************
*/
void Get_AD7689Data(SPI_HandleTypeDef *spiHandle,float *data){
	uint16_t CFG=0x3C49;
	uint8_t txData[2],rxData[2];
	uint8_t DumbrxData[2];					//存储哑读数据
	uint8_t i=0,b=0;
	uint16_t a;
	/*哑读数据*/
	CFG=(CFG&0x3C7F)|(i<<7);
	txData[0]=(CFG<<2)>>8;											//内部基准源,REF=4.096,禁止序列器
	txData[1]=(CFG<<2)&0x00FF;		
	if(spiHandle->Instance==SPI2){										//使用SPI2读取AD1的数据
		for(uint8_t j=0;j<2;j++){
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[0],&DumbrxData[0],1,100);//配置字，输入通道IN0
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[1], &DumbrxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);													//过程中发生EOC
			a=(DumbrxData[0]<<8)|DumbrxData[1];					//接收的数据
			if(j==1){
				data[i]=a/65536.0*4.096;										//REF=4.096
			}
		}
		/*读取8个通道数据*/	
		for (i=1;i<9;i++){			
			if(i==8){
				i=0;
				b=1;
			}
			CFG=(CFG&0x3C7F)|(i<<7);
			txData[0]=(CFG<<2)>>8;										//内部基准源,REF=4.096,禁止序列器
			txData[1]=(CFG<<2)&0x00FF;		
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[0],&rxData[0],1,100);//配置字，输入通道IN   i+2
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi2,&txData[1], &rxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI2_PCS0_GPIO_Port, ECU1_SPI2_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);											//过程中发生EOC
			a=(rxData[0]<<8)|rxData[1];							//接收的数据
			if(b==1){
				i=8;
			}
			data[i-1]=a/65536.0*4.096;							//REF=4.096
		}	
	}
	else if(spiHandle->Instance==SPI1){										//使用SPI1读取AD2的数据
		for(uint8_t j=0;j<2;j++){
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&DumbrxData[0],1,100);//配置字，输入通道IN0
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &DumbrxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);													//过程中发生EOC
			a=(DumbrxData[0]<<8)|DumbrxData[1];					//接收的数据
			if(j==1){
				data[i]=a/65536.0*4.096;										//REF=4.096
			}
		}
		/*读取8个通道数据*/	
		for (i=1;i<9;i++){			
			if(i==8){
				i=0;
				b=1;
			}
			CFG=(CFG&0x3C7F)|(i<<7);
			txData[0]=(CFG<<2)>>8;										//内部基准源,REF=4.096,禁止序列器
			txData[1]=(CFG<<2)&0x00FF;		
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&rxData[0],1,100);//配置字，输入通道IN   i+2
			AD7689_Delay_us(2);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);//SOC
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_RESET);
			AD7689_Delay_6ns(3);	
			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &rxData[1],1,100);
			AD7689_Delay_6ns(3);
			HAL_GPIO_WritePin(ECU1_SPI1_PCS0_GPIO_Port, ECU1_SPI1_PCS0_Pin, GPIO_PIN_SET);
			AD7689_Delay_us(2);											//过程中发生EOC
			a=(rxData[0]<<8)|rxData[1];							//接收的数据
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
*@brief		取值函数，搬运油门，油压，电压电池电压，转角等数据
*@param		TimGenericData_n name_num：索取数据对应的枚举变量
*@param		const void*extern_data：存放传递出来的数据
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


