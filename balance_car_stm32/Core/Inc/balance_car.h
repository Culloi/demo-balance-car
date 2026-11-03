#ifndef BALANCE_CAR_H
#define BALANCE_CAR_H

#ifdef __cplusplus
extern "C" {
#endif

/* One-time safe state and encoder initialization; returns to main. */
void BalanceCar_Init(void);
/* One main-context pass: key/start/stop and 100 ms OLED refresh. */
void BalanceCar_Task(void);
void BalanceCar_SetTargetSpeed(float target_speed_rpm);
void BalanceCar_SetTargetYawRate(float target_yaw_rate_dps);
/* One 10 ms control period; called only from the TIM1 HAL callback. */
void BalanceCar_Control(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_CAR_H */
