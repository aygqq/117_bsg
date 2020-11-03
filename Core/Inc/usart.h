/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#include "cmsis_os.h"

#define USART_SZ_BUF_RX_USART2    500
#define USART_SZ_BUF_RX_USART1    1524
#define USART_SZ_BUF_TX_USART1    1024

#define USART_TIMEOUT             15000

typedef UART_HandleTypeDef*	PHuart;

typedef struct{
	IrqFlags irqFlags;
	u8*	pRxBuf;
  u8* pTxBuf;
  u16 szRxBuf;
  u16 szTxBuf;
  PHuart pHuart;
}UartInfo;

extern UartInfo uInfoGnss;
extern UartInfo uInfoSim;

void uartInitInfo();
void uartRxDma(UartInfo* pUInf);
void txUart(char* data, u16 sz, UartInfo* pUInf);
void uartClearInfo(UartInfo* pUinf);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
