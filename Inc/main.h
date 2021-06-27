/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ECU1_TemSensor18B20DQ_Pin GPIO_PIN_5
#define ECU1_TemSensor18B20DQ_GPIO_Port GPIOE
#define ECU1_AT24C02_I2C2_SDA_Pin GPIO_PIN_0
#define ECU1_AT24C02_I2C2_SDA_GPIO_Port GPIOF
#define ECU1_AT24C02_I2C2_SCL_Pin GPIO_PIN_1
#define ECU1_AT24C02_I2C2_SCL_GPIO_Port GPIOF
#define IMD_InsulationDetectionIndicator_Pin GPIO_PIN_6
#define IMD_InsulationDetectionIndicator_GPIO_Port GPIOF
#define AMS_AbnormalFeedbackSignalLamp_Pin GPIO_PIN_7
#define AMS_AbnormalFeedbackSignalLamp_GPIO_Port GPIOF
#define ECU1_UART4_TX_Wireless_Pin GPIO_PIN_0
#define ECU1_UART4_TX_Wireless_GPIO_Port GPIOA
#define ECU1_UART4_RX_Wireless_Pin GPIO_PIN_1
#define ECU1_UART4_RX_Wireless_GPIO_Port GPIOA
#define ECU1_SPI1_PCS0_Pin GPIO_PIN_4
#define ECU1_SPI1_PCS0_GPIO_Port GPIOA
#define ECU1_SPI1_CLK_Pin GPIO_PIN_5
#define ECU1_SPI1_CLK_GPIO_Port GPIOA
#define ECU1_SPI1_MISO_Pin GPIO_PIN_6
#define ECU1_SPI1_MISO_GPIO_Port GPIOA
#define ECU1_SPI1_MOSI_Pin GPIO_PIN_7
#define ECU1_SPI1_MOSI_GPIO_Port GPIOA
#define ECU1_Wireless_AUX_Pin GPIO_PIN_4
#define ECU1_Wireless_AUX_GPIO_Port GPIOC
#define ECU1_Wireless_SET_Pin GPIO_PIN_5
#define ECU1_Wireless_SET_GPIO_Port GPIOC
#define RollStabilizerBarSteeringEngine2_Pin GPIO_PIN_0
#define RollStabilizerBarSteeringEngine2_GPIO_Port GPIOB
#define RollStabilizerBarSteeringEngine1_Pin GPIO_PIN_1
#define RollStabilizerBarSteeringEngine1_GPIO_Port GPIOB
#define ECU1_LED1_Pin GPIO_PIN_15
#define ECU1_LED1_GPIO_Port GPIOE
#define GPS_Compass_IO_SCL_Pin GPIO_PIN_10
#define GPS_Compass_IO_SCL_GPIO_Port GPIOB
#define GPS_Compass_IO_SDA_Pin GPIO_PIN_11
#define GPS_Compass_IO_SDA_GPIO_Port GPIOB
#define ECU1_SPI2_PCS0_Pin GPIO_PIN_12
#define ECU1_SPI2_PCS0_GPIO_Port GPIOB
#define ECU1_SPI2_CLK_Pin GPIO_PIN_13
#define ECU1_SPI2_CLK_GPIO_Port GPIOB
#define ECU1_SPI2_MISO_Pin GPIO_PIN_14
#define ECU1_SPI2_MISO_GPIO_Port GPIOB
#define ECU1_SPI2_MOSI_Pin GPIO_PIN_15
#define ECU1_SPI2_MOSI_GPIO_Port GPIOB
#define ECU1_Backup_Gpio2_Pin GPIO_PIN_14
#define ECU1_Backup_Gpio2_GPIO_Port GPIOD
#define ECU1_Backup_Gpio1_Pin GPIO_PIN_15
#define ECU1_Backup_Gpio1_GPIO_Port GPIOD
#define ECU1_SDIO_D0_Pin GPIO_PIN_8
#define ECU1_SDIO_D0_GPIO_Port GPIOC
#define ECU1_SDIO_D1_Pin GPIO_PIN_9
#define ECU1_SDIO_D1_GPIO_Port GPIOC
#define ECU1_USART1_TX_Pin GPIO_PIN_9
#define ECU1_USART1_TX_GPIO_Port GPIOA
#define ECU1_USART1_RX_Pin GPIO_PIN_10
#define ECU1_USART1_RX_GPIO_Port GPIOA
#define ECU1_SDIO_D2_Pin GPIO_PIN_10
#define ECU1_SDIO_D2_GPIO_Port GPIOC
#define ECU1_SDIO_D3_Pin GPIO_PIN_11
#define ECU1_SDIO_D3_GPIO_Port GPIOC
#define ECU1_SDIO_SCK_Pin GPIO_PIN_12
#define ECU1_SDIO_SCK_GPIO_Port GPIOC
#define ECU1_SDIO_CMD_Pin GPIO_PIN_2
#define ECU1_SDIO_CMD_GPIO_Port GPIOD
#define ECU1_USART2_TX_ESP8266_Pin GPIO_PIN_5
#define ECU1_USART2_TX_ESP8266_GPIO_Port GPIOD
#define ECU1_USART2_RX_ESP8266_Pin GPIO_PIN_6
#define ECU1_USART2_RX_ESP8266_GPIO_Port GPIOD
#define ECU1_USART6_RX_GPS_Pin GPIO_PIN_9
#define ECU1_USART6_RX_GPS_GPIO_Port GPIOG
#define ECU1_USART6_TX_GPS_Pin GPIO_PIN_14
#define ECU1_USART6_TX_GPS_GPIO_Port GPIOG
#define ECU1_CAN2RX_Pin GPIO_PIN_5
#define ECU1_CAN2RX_GPIO_Port GPIOB
#define ECU1_CAN2TX_Pin GPIO_PIN_6
#define ECU1_CAN2TX_GPIO_Port GPIOB
#define ECU1_CAN1RX_Pin GPIO_PIN_8
#define ECU1_CAN1RX_GPIO_Port GPIOB
#define ECU1_CAN1TX_Pin GPIO_PIN_9
#define ECU1_CAN1TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
