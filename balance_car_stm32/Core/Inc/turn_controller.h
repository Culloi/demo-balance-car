#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Yaw-rate P loop; inputs are deg/s, result is a motor command correction.
   Returns NAN on non-finite input or calculation; caller must stop safely. */
float TurnController_Update(float target_yaw_rate_dps,
                            float actual_yaw_rate_dps);

#ifdef __cplusplus
}
#endif

#endif /* TURN_CONTROLLER_H */
