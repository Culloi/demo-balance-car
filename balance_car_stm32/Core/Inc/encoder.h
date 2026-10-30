#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Measured effective counts per revolution of the wheel output shaft on this car. */
#define ENCODER_COUNTS_PER_REV 60000.0f

typedef struct
{
	int16_t left_delta;
	int16_t right_delta;
	int32_t left_total;
	int32_t right_total;
	float left_rpm;
	float right_rpm;
	uint32_t dt_ms;
} Encoder_Data_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Call after MX_TIM2_Init() and MX_TIM3_Init(). */
void Encoder_Init(void);

/* Counts accumulated since the previous call for the corresponding wheel. */
int16_t Encoder_ReadLeftDelta(void);
int16_t Encoder_ReadRightDelta(void);

/* Call once per actual sample interval; dt_ms must match the interval between reads. */
void Encoder_Update(uint32_t dt_ms);
const Encoder_Data_t *Encoder_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
