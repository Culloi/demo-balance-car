#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid)
{
  PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
  pid->integral = 0.0f;
  pid->p_out = 0.0f;
  pid->i_out = 0.0f;
  pid->d_out = 0.0f;
  pid->output = 0.0f;
}

float PID_Update(PID_t *pid, float error,
                 float derivative_input, float dt_seconds)
{
  float p_out;
  float integral;
  float d_out;
  float result;

  if (!isfinite(error) || !isfinite(derivative_input) ||
      !isfinite(dt_seconds) || dt_seconds <= 0.0f)
  {
    return NAN;
  }

  p_out = pid->config.kp * error;
  integral = pid->integral + pid->config.ki * error * dt_seconds;
  d_out = pid->config.kd * derivative_input;
  if (!isfinite(p_out) || !isfinite(integral) || !isfinite(d_out))
  {
    return NAN;
  }
  if (integral > pid->config.integral_max)
  {
    integral = pid->config.integral_max;
  }
  else if (integral < pid->config.integral_min)
  {
    integral = pid->config.integral_min;
  }

  result = p_out + integral + d_out;
  if (!isfinite(result))
  {
    return NAN;
  }
  if (result > pid->config.output_max)
  {
    result = pid->config.output_max;
  }
  else if (result < pid->config.output_min)
  {
    result = pid->config.output_min;
  }

  pid->integral = integral;
  pid->p_out = p_out;
  pid->i_out = integral;
  pid->d_out = d_out;
  pid->output = result;
  return result;
}
