/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define UART_FRAME_MAX_DATA  32U
#define UART_CMD_PING        0x01U
#define UART_CMD_SET_SPEED       0x02U
#define UART_CMD_SET_YAW_RATE    0x03U
#define UART_CMD_PING_ACK    0x81U
#define UART_CMD_SET_SPEED_ACK     0x82U
#define UART_CMD_SET_YAW_RATE_ACK  0x83U
#define UART_SPEED_MAX_RPM         150
#define UART_YAW_RATE_MAX_DPS      150

typedef struct
{
  uint32_t rx_bytes;
  uint32_t frames_ok;
  uint32_t checksum_errors;
  uint32_t length_errors;
  uint32_t range_errors;
  uint32_t ore_errors;
  uint32_t fe_errors;
  uint32_t ne_errors;
  uint32_t rearm_errors;
  uint32_t error_callbacks;
} UART_Diagnostics_t;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef UART_Start(void);
HAL_StatusTypeDef UART_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len);
void UART_GetDiagnostics(UART_Diagnostics_t *diag);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

