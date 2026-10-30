#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float gyro_x_bias_dps;
  float gyro_y_bias_dps;
  float gyro_z_bias_dps;
  uint8_t valid; /* Set only after all stationary samples have been read. */
} Attitude_Calibration_t;

typedef struct
{
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
} Attitude_Gyro_t;

typedef enum
{
  ATTITUDE_STATUS_ERROR = -1,
  ATTITUDE_STATUS_OK = 0,
  ATTITUDE_STATUS_INIT = 1,
  ATTITUDE_STATUS_HOLD = 2
} Attitude_Status_t;

typedef struct
{
  float filtered_accel_x_g;
  float filtered_accel_y_g;
  float filtered_accel_z_g;
  float filtered_gyro_y_dps;
  float fused_angle_deg;
  uint8_t initialized;
} Attitude_State_t;

typedef struct
{
  float raw_gyro_y_dps;
  float filtered_gyro_y_dps;
  /* accel_pitch_deg is derived from the filtered X/Y/Z accelerations. */
  float accel_pitch_deg;
  float gyro_pitch_rate_dps;
  float corrected_gyro_x_dps;
  float corrected_gyro_y_dps;
  float corrected_gyro_z_dps;
  float gyro_predicted_angle_deg;
  float fused_angle_deg;
  float dt_seconds;
} Attitude_Data_t;

/* Blocking, one-time stationary calibration; requires MPU6050_Init().
   Output is updated only if all 1000 reads succeed. */
HAL_StatusTypeDef Attitude_CalibrateGyro(Attitude_Calibration_t *calibration);
/* Apply a successful calibration to one unmodified MPU6050 measurement. */
HAL_StatusTypeDef Attitude_ApplyGyroBias(const MPU6050_Data_t *measurement,
                                         const Attitude_Calibration_t *calibration,
                                         Attitude_Gyro_t *corrected);
/* Reset both software LPFs and complementary filter after calibration. */
void Attitude_Reset(Attitude_State_t *state);
/* INIT seeds LPFs and fused from the first sample; HOLD preserves filter state
  when dt <= 0 or > 0.5 s. Caller supplies the elapsed successful-read dt. */
Attitude_Status_t Attitude_Update(Attitude_State_t *state,
                       const MPU6050_Data_t *imu,
                       const Attitude_Calibration_t *calibration,
                       float dt_seconds, Attitude_Data_t *output);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
