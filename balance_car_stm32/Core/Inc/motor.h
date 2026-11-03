#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call after MX_GPIO_Init() and MX_TIM4_Init(). Starts both PWM channels
   at zero duty with STBY low; returns HAL_ERROR if either fails to start. */
HAL_StatusTypeDef Motor_Init(void);
/* Open-loop commands are clamped to [-1000, +1000].
   Both motors: positive=IN1 high/IN2 low (forward), negative=reverse.
   Zero sets only that channel's PWM compare to zero; direction pins stay as-is. */
void Motor_SetLeft(int16_t command);
void Motor_SetRight(int16_t command);
/* Immediately set both PWM compares to zero and put TB6612 in standby. */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
