#include "attitude.h"
#include <math.h>

#define ATTITUDE_GYRO_CAL_SAMPLES 1000U
#define ATTITUDE_DEG_PER_RAD (180.0f / 3.14159265358979323846f)
#define ATTITUDE_PI 3.14159265358979323846f
#define ATTITUDE_ALPHA 0.98f
#define ATTITUDE_MAX_DT_SECONDS 0.5f
/* First-order LPFs at the actual update dt (nominally 10 ms).
  Gyro 30 Hz keeps the control D input responsive; accel 10 Hz rejects
  wheel/motor vibration before atan2. Both are below the 50 Hz Nyquist rate. */
#define ATTITUDE_GYRO_FC_HZ 30.0f
#define ATTITUDE_ACCEL_FC_HZ 10.0f
#define ATTITUDE_GYRO_RC_SECONDS (1.0f / (2.0f * ATTITUDE_PI * ATTITUDE_GYRO_FC_HZ))
#define ATTITUDE_ACCEL_RC_SECONDS (1.0f / (2.0f * ATTITUDE_PI * ATTITUDE_ACCEL_FC_HZ))
/* Physical X points forward and Y lies along the axle: pitch rotates about Y.
  If the installation is reversed, adjust signs here, not in callers. */
#define ATTITUDE_ACCEL_X_PITCH_SIGN (-1.0f)
#define ATTITUDE_GYRO_Y_PITCH_SIGN (1.0f)

HAL_StatusTypeDef Attitude_CalibrateGyro(Attitude_Calibration_t *calibration)
{
  MPU6050_Data_t measurement;
  Attitude_Calibration_t result;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;
  uint32_t sample;
  HAL_StatusTypeDef status;

  if (calibration == NULL)
  {
    return HAL_ERROR;
  }
  calibration->valid = 0U; /* Invalidate an old result before any new attempt. */

  for (sample = 0U; sample < ATTITUDE_GYRO_CAL_SAMPLES; ++sample)
  {
    status = MPU6050_Read(&measurement);
    if (status != HAL_OK)
    {
      return status; /* Never publish a partial calibration. */
    }
    sum_x += measurement.gyro_x_dps;
    sum_y += measurement.gyro_y_dps;
    sum_z += measurement.gyro_z_dps;

    /* DLPF_CFG=0, SMPLRT_DIV=0: gyro output rate 8 kHz, accel 1 kHz.
       Read raw dps here; subtract the stationary mean BEFORE software LPF.
       100 kHz I2C takes >1 ms per 14-byte burst; keep the existing 1 ms
       inter-read gap. Calibration never runs in the control interrupt. */
    if (sample + 1U < ATTITUDE_GYRO_CAL_SAMPLES)
    {
      HAL_Delay(1U);
    }
  }

  result.gyro_x_bias_dps = sum_x / (float)ATTITUDE_GYRO_CAL_SAMPLES;
  result.gyro_y_bias_dps = sum_y / (float)ATTITUDE_GYRO_CAL_SAMPLES;
  result.gyro_z_bias_dps = sum_z / (float)ATTITUDE_GYRO_CAL_SAMPLES;
  result.valid = 1U;
  *calibration = result;
  return HAL_OK;
}

HAL_StatusTypeDef Attitude_ApplyGyroBias(const MPU6050_Data_t *measurement,
                                         const Attitude_Calibration_t *calibration,
                                         Attitude_Gyro_t *corrected)
{
  if ((measurement == NULL) || (calibration == NULL) || (corrected == NULL) ||
      (calibration->valid != 1U))
  {
    return HAL_ERROR;
  }

  corrected->gyro_x_dps = measurement->gyro_x_dps - calibration->gyro_x_bias_dps;
  corrected->gyro_y_dps = measurement->gyro_y_dps - calibration->gyro_y_bias_dps;
  corrected->gyro_z_dps = measurement->gyro_z_dps - calibration->gyro_z_bias_dps;
  return HAL_OK;
}

void Attitude_Reset(Attitude_State_t *state)
{
  if (state != NULL)
  {
    state->filtered_accel_x_g = 0.0f;
    state->filtered_accel_y_g = 0.0f;
    state->filtered_accel_z_g = 0.0f;
    state->filtered_gyro_y_dps = 0.0f;
    state->fused_angle_deg = 0.0f;
    state->initialized = 0U;
  }
}

Attitude_Status_t Attitude_Update(Attitude_State_t *state,
                                  const MPU6050_Data_t *imu,
                                  const Attitude_Calibration_t *calibration,
                                  float dt_seconds, Attitude_Data_t *output)
{
  Attitude_Gyro_t gyro;
  Attitude_Data_t next;
  Attitude_Status_t status;
  float accel_alpha;
  float gyro_alpha;

  if ((state == NULL) || (imu == NULL) || (calibration == NULL) ||
      (output == NULL) ||
      (Attitude_ApplyGyroBias(imu, calibration, &gyro) != HAL_OK) ||
      !isfinite(imu->accel_x_g) || !isfinite(imu->accel_y_g) ||
      !isfinite(imu->accel_z_g) || !isfinite(imu->gyro_y_dps) ||
      !isfinite(gyro.gyro_y_dps))
  {
    return ATTITUDE_STATUS_ERROR;
  }

  /* MPU6050_Read supplies unfiltered g and dps; calibration removes the
     stationary gyro bias before the low-pass, never after it. */
  next.raw_gyro_y_dps = imu->gyro_y_dps;
  next.corrected_gyro_x_dps = gyro.gyro_x_dps;
  next.corrected_gyro_y_dps = gyro.gyro_y_dps;
  next.corrected_gyro_z_dps = gyro.gyro_z_dps;
  next.dt_seconds = dt_seconds;

  if (state->initialized == 0U)
  {
    /* Seed from this first valid measurement: never ramp up from zero. */
    state->filtered_accel_x_g = imu->accel_x_g;
    state->filtered_accel_y_g = imu->accel_y_g;
    state->filtered_accel_z_g = imu->accel_z_g;
    state->filtered_gyro_y_dps = gyro.gyro_y_dps;
    state->fused_angle_deg = atan2f(ATTITUDE_ACCEL_X_PITCH_SIGN * imu->accel_x_g,
                    sqrtf(imu->accel_y_g * imu->accel_y_g +
                        imu->accel_z_g * imu->accel_z_g)) *
                 ATTITUDE_DEG_PER_RAD;
    state->initialized = 1U;
    status = ATTITUDE_STATUS_INIT;
  }
  else if ((dt_seconds > 0.0f) && (dt_seconds <= ATTITUDE_MAX_DT_SECONDS))
  {
    /* alpha = dt / (RC + dt), RC = 1 / (2*pi*fc). No moving window. */
    gyro_alpha = dt_seconds / (ATTITUDE_GYRO_RC_SECONDS + dt_seconds);
    accel_alpha = dt_seconds / (ATTITUDE_ACCEL_RC_SECONDS + dt_seconds);
    state->filtered_gyro_y_dps += gyro_alpha *
                                  (gyro.gyro_y_dps - state->filtered_gyro_y_dps);
    state->filtered_accel_x_g += accel_alpha *
                                 (imu->accel_x_g - state->filtered_accel_x_g);
    state->filtered_accel_y_g += accel_alpha *
                                 (imu->accel_y_g - state->filtered_accel_y_g);
    state->filtered_accel_z_g += accel_alpha *
                                 (imu->accel_z_g - state->filtered_accel_z_g);
    next.gyro_pitch_rate_dps = ATTITUDE_GYRO_Y_PITCH_SIGN * state->filtered_gyro_y_dps;
    next.gyro_predicted_angle_deg = state->fused_angle_deg +
                                    next.gyro_pitch_rate_dps * dt_seconds;
    next.accel_pitch_deg = atan2f(ATTITUDE_ACCEL_X_PITCH_SIGN *
                                   state->filtered_accel_x_g,
                                   sqrtf(state->filtered_accel_y_g *
                                         state->filtered_accel_y_g +
                                         state->filtered_accel_z_g *
                                         state->filtered_accel_z_g)) *
                           ATTITUDE_DEG_PER_RAD;
    state->fused_angle_deg = ATTITUDE_ALPHA * next.gyro_predicted_angle_deg +
                             (1.0f - ATTITUDE_ALPHA) * next.accel_pitch_deg;
    status = ATTITUDE_STATUS_OK;
  }
  else
  {
    /* Skip integration and LPF updates after a stall; next valid interval
       starts from the previous filtered state, not from zero. */
    status = ATTITUDE_STATUS_HOLD;
  }

  if (status == ATTITUDE_STATUS_INIT)
  {
    next.accel_pitch_deg = state->fused_angle_deg;
    next.gyro_pitch_rate_dps = ATTITUDE_GYRO_Y_PITCH_SIGN * state->filtered_gyro_y_dps;
    next.gyro_predicted_angle_deg = state->fused_angle_deg;
  }
  else if (status == ATTITUDE_STATUS_HOLD)
  {
    next.gyro_pitch_rate_dps = ATTITUDE_GYRO_Y_PITCH_SIGN * state->filtered_gyro_y_dps;
    next.accel_pitch_deg = atan2f(ATTITUDE_ACCEL_X_PITCH_SIGN *
                                   state->filtered_accel_x_g,
                                   sqrtf(state->filtered_accel_y_g *
                                         state->filtered_accel_y_g +
                                         state->filtered_accel_z_g *
                                         state->filtered_accel_z_g)) *
                           ATTITUDE_DEG_PER_RAD;
    next.gyro_predicted_angle_deg = state->fused_angle_deg;
  }
  next.filtered_gyro_y_dps = state->filtered_gyro_y_dps;
  next.fused_angle_deg = state->fused_angle_deg;
  *output = next;
  return status;
}
