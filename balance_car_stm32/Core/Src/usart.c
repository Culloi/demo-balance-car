/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "balance_car.h"

#define UART_HEAD1  0xAAU
#define UART_HEAD2  0x55U

typedef enum
{
  UART_RX_HEAD1,
  UART_RX_HEAD2,
  UART_RX_CMD,
  UART_RX_LEN,
  UART_RX_DATA,
  UART_RX_CHECKSUM
} UART_RxState_t;

static uint8_t uart_rx_byte;
static UART_RxState_t uart_rx_state;
static uint8_t uart_rx_cmd;
static uint8_t uart_rx_len;
static uint8_t uart_rx_count;
static uint8_t uart_rx_checksum;
static uint8_t uart_rx_data[UART_FRAME_MAX_DATA];

static volatile uint32_t uart_rx_byte_count;
static volatile uint32_t uart_frame_ok_count;
static volatile uint32_t uart_checksum_error_count;
static volatile uint32_t uart_length_error_count;
static volatile uint32_t uart_range_error_count;
static volatile uint32_t uart_ore_count;
static volatile uint32_t uart_fe_count;
static volatile uint32_t uart_ne_count;
static volatile uint32_t uart_rearm_error_count;
static volatile uint32_t uart_error_callback_count;

/* HAL transmits from this buffer until TxCpltCallback; never reuse it while busy. */
static uint8_t uart_tx_buffer[UART_FRAME_MAX_DATA + 5U];
static volatile uint8_t uart_tx_busy;
static volatile uint8_t uart_ping_ack_pending;

static void UART_ResetRx(uint8_t byte)
{
  /* A rejected byte may itself be the first byte of the next frame. */
  uart_rx_state = (byte == UART_HEAD1) ? UART_RX_HEAD2 : UART_RX_HEAD1;
  uart_rx_count = 0U;
  uart_rx_checksum = 0U;
}

static void UART_ParseByte(uint8_t byte)
{
  switch (uart_rx_state)
  {
    case UART_RX_HEAD1:
      if (byte == UART_HEAD1) uart_rx_state = UART_RX_HEAD2;
      break;
    case UART_RX_HEAD2:
      if (byte == UART_HEAD2) uart_rx_state = UART_RX_CMD;
      else UART_ResetRx(byte);
      break;
    case UART_RX_CMD:
      uart_rx_cmd = byte;
      uart_rx_checksum = byte;
      uart_rx_state = UART_RX_LEN;
      break;
    case UART_RX_LEN:
      if (byte > UART_FRAME_MAX_DATA)
      {
        ++uart_length_error_count;
        UART_ResetRx(byte);
        break;
      }
      uart_rx_len = byte;
      uart_rx_count = 0U;
      uart_rx_checksum = (uint8_t)(uart_rx_checksum + byte);
      uart_rx_state = (byte == 0U) ? UART_RX_CHECKSUM : UART_RX_DATA;
      break;
    case UART_RX_DATA:
      uart_rx_data[uart_rx_count] = byte;
      ++uart_rx_count;
      uart_rx_checksum = (uint8_t)(uart_rx_checksum + byte);
      if (uart_rx_count == uart_rx_len) uart_rx_state = UART_RX_CHECKSUM;
      break;
    case UART_RX_CHECKSUM:
      if ((byte == uart_rx_checksum) && (uart_rx_cmd == UART_CMD_PING) &&
          (uart_rx_len == 0U))
      {
        ++uart_frame_ok_count;
        /* Coalesce at most one reply while the non-blocking TX is in progress. */
        uart_ping_ack_pending = 1U;
        if (uart_tx_busy == 0U)
        {
          if (UART_SendFrame(UART_CMD_PING_ACK, NULL, 0U) == HAL_OK)
            uart_ping_ack_pending = 0U;
        }
      }
      else if ((byte == uart_rx_checksum) && (uart_rx_len == 2U) &&
               ((uart_rx_cmd == UART_CMD_SET_SPEED) ||
                (uart_rx_cmd == UART_CMD_SET_YAW_RATE)))
      {
        uint16_t raw = (uint16_t)uart_rx_data[0] |
                       ((uint16_t)uart_rx_data[1] << 8);
        int32_t value = (raw < 0x8000U) ? (int32_t)raw :
                        (int32_t)raw - 65536L;

        if (uart_rx_cmd == UART_CMD_SET_SPEED)
        {
          if ((value >= -UART_SPEED_MAX_RPM) &&
              (value <= UART_SPEED_MAX_RPM))
          {
            BalanceCar_SetTargetSpeed((float)value);
            ++uart_frame_ok_count;
            (void)UART_SendFrame(UART_CMD_SET_SPEED_ACK, NULL, 0U);
          }
          else ++uart_range_error_count;
        }
        else if ((value >= -UART_YAW_RATE_MAX_DPS) &&
                 (value <= UART_YAW_RATE_MAX_DPS))
        {
          BalanceCar_SetTargetYawRate((float)value);
          ++uart_frame_ok_count;
          (void)UART_SendFrame(UART_CMD_SET_YAW_RATE_ACK, NULL, 0U);
        }
        else ++uart_range_error_count;
      }
      else if (byte != uart_rx_checksum) ++uart_checksum_error_count;
      else if (((uart_rx_cmd == UART_CMD_PING) && (uart_rx_len != 0U)) ||
               (((uart_rx_cmd == UART_CMD_SET_SPEED) ||
                 (uart_rx_cmd == UART_CMD_SET_YAW_RATE)) &&
                (uart_rx_len != 2U))) ++uart_length_error_count;
      UART_ResetRx(byte);
      break;
    default:
      UART_ResetRx(byte);
      break;
  }
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

 
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    /* UART is optional for balancing; do not halt the car on init failure. */
    return;
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
    /* SysTick=0, TIM1 update=1; USART1 must not preempt balancing. */
    HAL_NVIC_SetPriority(USART1_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(USART1_IRQn);

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef UART_Start(void)
{
  HAL_StatusTypeDef status;

  if (huart1.gState != HAL_UART_STATE_READY) return HAL_ERROR;

  /* This is called once after MX_USART1_UART_Init(), before any UART traffic. */
  UART_ResetRx(0U);
  uart_rx_byte = 0U;
  uart_rx_cmd = 0U;
  uart_rx_len = 0U;
  uart_tx_busy = 0U;
  uart_ping_ack_pending = 0U;
  __HAL_UART_CLEAR_OREFLAG(&huart1); /* SR then DR; discard stale startup data. */
  status = HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
  if (status != HAL_OK) ++uart_rearm_error_count;
  return status;
}

void UART_GetDiagnostics(UART_Diagnostics_t *diag)
{
  if (diag == NULL) return;
  diag->rx_bytes = uart_rx_byte_count;
  diag->frames_ok = uart_frame_ok_count;
  diag->checksum_errors = uart_checksum_error_count;
  diag->length_errors = uart_length_error_count;
  diag->range_errors = uart_range_error_count;
  diag->ore_errors = uart_ore_count;
  diag->fe_errors = uart_fe_count;
  diag->ne_errors = uart_ne_count;
  diag->rearm_errors = uart_rearm_error_count;
  diag->error_callbacks = uart_error_callback_count;
}

HAL_StatusTypeDef UART_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef status;
  uint8_t checksum = (uint8_t)(cmd + len);
  uint8_t i;

  if ((len > UART_FRAME_MAX_DATA) || ((len != 0U) && (data == NULL)))
    return HAL_ERROR;

  /* Mask only USART1: TIM1 retains its priority even for non-ISR callers. */
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  if (uart_tx_busy != 0U)
  {
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return HAL_BUSY;
  }

  uart_tx_busy = 1U;
  uart_tx_buffer[0] = UART_HEAD1;
  uart_tx_buffer[1] = UART_HEAD2;
  uart_tx_buffer[2] = cmd;
  uart_tx_buffer[3] = len;
  for (i = 0U; i < len; ++i)
  {
    uart_tx_buffer[4U + i] = data[i];
    checksum = (uint8_t)(checksum + data[i]);
  }
  uart_tx_buffer[4U + len] = checksum;
  status = HAL_UART_Transmit_IT(&huart1, uart_tx_buffer, (uint16_t)(len + 5U));
  if (status != HAL_OK) uart_tx_busy = 0U;
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  return status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t byte;
  uint32_t rx_error;

  if (huart != &huart1) return;
  byte = uart_rx_byte;
  rx_error = huart->ErrorCode;
  /* Re-arm before parsing so replies never depend on the main loop. */
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
  {
    ++uart_rearm_error_count;
    UART_ResetRx(0U);
    return;
  }
  /* HAL may complete a byte and then report FE/NE/ORE in the same IRQ. */
  if (rx_error == HAL_UART_ERROR_NONE)
  {
    ++uart_rx_byte_count;
    UART_ParseByte(byte);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;
  uart_tx_busy = 0U;
  if (uart_ping_ack_pending != 0U)
  {
    if (UART_SendFrame(UART_CMD_PING_ACK, NULL, 0U) == HAL_OK)
      uart_ping_ack_pending = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;

  ++uart_error_callback_count;
  if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) ++uart_ore_count;
  if ((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U) ++uart_fe_count;
  if ((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U) ++uart_ne_count;

  /* A UART line error invalidates the partial frame, not the car's control.
     AbortReceive is bounded here: USART1 uses IT only, never DMA. */
  UART_ResetRx(0U);
  (void)HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_OREFLAG(&huart1); /* Clears PE/FE/NE/ORE by SR/DR read. */
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
    ++uart_rearm_error_count;
}

/* USER CODE END 1 */

