#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#define CAR_SPEED_MIN_RPM (-150)
#define CAR_SPEED_MAX_RPM 150
#define CAR_SPEED_STEP_RPM 10
#define CAR_TURN_LEFT_DPS 30
#define CAR_TURN_RIGHT_DPS (-30)

typedef struct {
    int uart_fd;
    int target_speed_rpm;
    int target_yaw_rate_dps;
} CarControl;

/* Initialize before use; close before initializing the same object again.
    Sends speed=0 and yaw=0; on failure closes the fd.
   A successful send means bytes were written, not that STM32 acknowledged. */
int CarControl_Init(CarControl *car, const char *uart_device);
/* Try to stop both targets before closing; always invalidate uart_fd. */
void CarControl_Close(CarControl *car);

/* Return 0 after writing the target, or -1 with errno set on failure. */
int CarControl_SetSpeed(CarControl *car, int rpm);
int CarControl_SpeedUp(CarControl *car);
int CarControl_SpeedDown(CarControl *car);
int CarControl_StopSpeed(CarControl *car);
/* Send a signed yaw-rate target; UART rejects values outside -150..150 deg/s. */
int CarControl_SetYawRate(CarControl *car, int dps);
int CarControl_TurnLeft(CarControl *car);
int CarControl_TurnRight(CarControl *car);
int CarControl_StopTurn(CarControl *car);
/* Attempt both zero targets even if one write fails. */
int CarControl_Stop(CarControl *car);

#endif