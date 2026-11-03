#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float error_deg;
  float p_term;
  float d_term;
  float output;
} BalanceController_Data_t;

typedef struct
{
  float balance_angle_deg;
  BalanceController_Data_t data;
} BalanceController_t;

/* Sets the mechanical balance angle and resets PID runtime state on re-arm. */
void BalanceController_Init(BalanceController_t *controller, float balance_angle_deg);
/* Upright control: target_angle_deg is this cycle's mechanical balance angle
  minus the speed-loop correction; the stored base remains unchanged.
  Invalid inputs/results return NAN; the caller must stop the motors. */
float BalanceController(BalanceController_t *controller,
                        float fused_angle_deg, float gyro_pitch_rate_dps,
                        float target_angle_deg);
const BalanceController_Data_t *BalanceController_GetData(const BalanceController_t *controller);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_CONTROLLER_H */
