#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* First-stage measurements; temperature is kept raw, not converted to Celsius. */
typedef struct
{
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
  int16_t temperature_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
} MPU6050_Data_t;

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temperature;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} MPU6050_Raw_t;

/* Call after MX_I2C2_Init(). On failure, no read is allowed until re-init. */
HAL_StatusTypeDef MPU6050_Init(void);
/* Read a single coherent 14-byte sample; outputs change only on success. */
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw_t *raw);
HAL_StatusTypeDef MPU6050_Read(MPU6050_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
