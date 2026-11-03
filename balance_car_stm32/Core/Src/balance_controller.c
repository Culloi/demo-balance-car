#include "balance_controller.h"
#include "pid.h"
#include <math.h>

#define BALANCE_CONTROLLER_DT_SECONDS 0.010f

/* The only live upright PID: tune gains and controller limits here. */
static PID_t BalancePID = {
  .config = {
    .kp = 149.0f,
    .ki = 0.0f,
    .kd = 3.35f,
    .output_min = -700.0f,
    .output_max = 700.0f,
    .integral_min = -500.0f,
    .integral_max = 500.0f
  }
};

void BalanceController_Init(BalanceController_t *controller, float balance_angle_deg)
{
  BalanceController_Data_t empty_data = {0};

  PID_Init(&BalancePID);
  controller->balance_angle_deg = balance_angle_deg;
  controller->data = empty_data;
}

float BalanceController(BalanceController_t *controller,
                        float fused_angle_deg, float gyro_pitch_rate_dps,
                        float target_angle_deg)
{
  BalanceController_Data_t next;

  if (!isfinite(fused_angle_deg) || !isfinite(gyro_pitch_rate_dps) ||
      !isfinite(target_angle_deg) ||
      fused_angle_deg < -180.0f || fused_angle_deg > 180.0f ||
      gyro_pitch_rate_dps < -2000.0f || gyro_pitch_rate_dps > 2000.0f)
  {
    return NAN;
  }

  next.error_deg = fused_angle_deg - target_angle_deg;
  /* Gyro rate is bias-corrected and software-IIR-filtered by the attitude
     estimator; PID_Update applies the positive Kd to this rate directly. */
  next.output = PID_Update(&BalancePID, next.error_deg, gyro_pitch_rate_dps,
                           BALANCE_CONTROLLER_DT_SECONDS);
  if (!isfinite(next.output)) return NAN;
  next.p_term = BalancePID.p_out;
  next.d_term = BalancePID.d_out;

  controller->data = next;
  return next.output;
}

const BalanceController_Data_t *BalanceController_GetData(const BalanceController_t *controller)
{
  return &controller->data;
}
