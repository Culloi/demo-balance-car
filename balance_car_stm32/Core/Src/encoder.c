#include "encoder.h"
#include "tim.h"

static uint16_t left_previous;
static uint16_t right_previous;
static Encoder_Data_t encoder_data;

void Encoder_Init(void)
{
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  /* Keep both counters free-running; the first sample starts here. */
  left_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  right_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  encoder_data = (Encoder_Data_t){0};
}

static int16_t Encoder_ReadDelta(TIM_HandleTypeDef *htim, uint16_t *previous)
{
  uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
  uint16_t difference = (uint16_t)(current - *previous);
  *previous = current;

  /* ARR=65535: subtraction wraps modulo 65536 in both directions.
     Direction is unambiguous only if the true count change between
     consecutive reads has absolute value < 32768. */
  return (difference < 32768U) ? (int16_t)difference
                               : (int16_t)((int32_t)difference - 65536L);
}

int16_t Encoder_ReadLeftDelta(void)
{
  int16_t raw_delta = Encoder_ReadDelta(&htim2, &left_previous);

  /* A half-wrap (0x8000) is direction-ambiguous and +32768 cannot fit
     in int16_t. Discard this invalid sample rather than overflowing. */
  if ((int32_t)raw_delta == -32768L)
  {
    return 0;
  }

  /* Single direction-normalization point: forward is positive on both wheels. */
  return (int16_t)(-(int32_t)raw_delta);
}

int16_t Encoder_ReadRightDelta(void)
{
  return Encoder_ReadDelta(&htim3, &right_previous);
}

void Encoder_Update(uint32_t dt_ms)
{
  float dt_seconds;

  if (dt_ms == 0U)
  {
    return; /* No valid time interval: leave both previous CNT bases untouched. */
  }

  encoder_data.left_delta = Encoder_ReadLeftDelta();
  encoder_data.right_delta = Encoder_ReadRightDelta();
  encoder_data.left_total += encoder_data.left_delta;
  encoder_data.right_total += encoder_data.right_delta;
  encoder_data.dt_ms = dt_ms;

  dt_seconds = (float)dt_ms / 1000.0f;
  encoder_data.left_rpm = (float)encoder_data.left_delta * 60.0f /
                          (ENCODER_COUNTS_PER_REV * dt_seconds);
  encoder_data.right_rpm = (float)encoder_data.right_delta * 60.0f /
                           (ENCODER_COUNTS_PER_REV * dt_seconds);
}

const Encoder_Data_t *Encoder_GetData(void)
{
  return &encoder_data;
}
