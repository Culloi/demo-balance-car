#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* One 20 Hz PI loop. Speeds are signed wheel-output RPM; result is degrees. */
void SpeedController_Init(void);
float SpeedController_Update(float target_rpm, float measured_rpm);
void SpeedController_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_CONTROLLER_H */
