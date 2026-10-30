#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* The caller defines the signs of error and derivative_input. */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float output_min;
  float output_max;
  float integral_min;
  float integral_max;
} PID_Config_t;

typedef struct
{
  PID_Config_t config;
  float integral;
  float p_out;
  float i_out;
  float d_out;
  float output;
} PID_t;

  /* Fixed gains/limits live in the caller's PID instance. Clear state on re-arm. */
  void PID_Init(PID_t *pid);
/* Clear runtime state without changing gains or limits. */
void PID_Reset(PID_t *pid);
/* P=kp*error, I=clamp(I+ki*error*dt), D=kd*derivative_input.
   Neither error nor derivative_input is calculated or negated here.
    Invalid/non-finite math returns NAN without changing PID state. */
  float PID_Update(PID_t *pid, float error,
              float derivative_input, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
