/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "oled.h"
#include "motor.h"
#include "balance_car.h"
#include "system_init.h"

int main(void)
{
  HAL_StatusTypeDef i2c1_status;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  i2c1_status = MX_I2C1_Init();
  MX_I2C2_Init();
  UART_Start(); /* UART failure must not prevent balance startup. */

  /* PWM starts with both compares zero; STBY stays low throughout startup. */
  if (Motor_Init() != HAL_OK)
  {
    while (1)
    {
    }
  }

  /* I2C1/OLED is optional; a failed I2C init must not access the OLED. */
  if (i2c1_status == HAL_OK) (void)OLED_Init();

  BalanceCar_Init();
  while (1)
  {
    BalanceCar_Task();
  }
}
