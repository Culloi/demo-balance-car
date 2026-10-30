#include "motor.h"
#include "main.h"
#include "tim.h"

static uint8_t motor_ready;

static void Motor_CoastPins(void)
{
  HAL_GPIO_WritePin(GPIOB, MOTOR_AIN1_Pin | MOTOR_AIN2_Pin |
                          MOTOR_BIN1_Pin | MOTOR_BIN2_Pin, GPIO_PIN_RESET);
}

void Motor_Stop(void)
{
  /* Zero both duty cycles first, then drop STBY. Never wait for OLED/I2C. */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
  Motor_CoastPins();
}

HAL_StatusTypeDef Motor_Init(void)
{
  motor_ready = 0U;
  Motor_Stop();

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Motor_Stop();
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
  {
    Motor_Stop();
    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    return HAL_ERROR;
  }

  Motor_Stop(); /* Both PWM channels are running, but CCR=0 and STBY stays low. */
  motor_ready = 1U;
  return HAL_OK;
}

static void Motor_Set(int16_t command, uint32_t channel,
                      uint16_t in1, uint16_t in2)
{
  int32_t clamped = (int32_t)command;
  uint32_t magnitude;
  uint32_t compare;
  GPIO_PinState in1_state;

  if (motor_ready == 0U)
  {
    return;
  }

  if (clamped > 1000L)
  {
    clamped = 1000L;
  }
  else if (clamped < -1000L)
  {
    clamped = -1000L;
  }

  if (clamped == 0L)
  {
    __HAL_TIM_SET_COMPARE(&htim4, channel, 0U);
    return;
  }

  /* Verified on both wheels: IN1=1/IN2=0 is forward. */
  in1_state = (clamped > 0L) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  /* Change direction without clearing PWM; lower the opposite pin first. */
  HAL_GPIO_WritePin(GPIOB, (in1_state == GPIO_PIN_SET) ? in2 : in1,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, (in1_state == GPIO_PIN_SET) ? in1 : in2,
                    GPIO_PIN_SET);

  magnitude = (uint32_t)((clamped < 0L) ? -clamped : clamped);
  compare = magnitude * __HAL_TIM_GET_AUTORELOAD(&htim4) / 1000U;
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim4, channel, compare);
}

void Motor_SetLeft(int16_t command)
{
  Motor_Set(command, TIM_CHANNEL_1, MOTOR_AIN1_Pin, MOTOR_AIN2_Pin);
}

void Motor_SetRight(int16_t command)
{
  Motor_Set(command, TIM_CHANNEL_2, MOTOR_BIN1_Pin, MOTOR_BIN2_Pin);
}
