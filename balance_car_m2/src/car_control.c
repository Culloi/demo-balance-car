#include "car_control.h"

#include "uart.h"

#include <errno.h>
#include <stddef.h>

int CarControl_Init(CarControl *car, const char *uart_device)
{
    int saved_errno;

    if (car == NULL) {
        errno = EINVAL;
        return -1;
    }
    car->uart_fd = -1;
    car->target_speed_rpm = 0;
    car->target_yaw_rate_dps = 0;
    if (uart_device == NULL) {
        errno = EINVAL;
        return -1;
    }
    car->uart_fd = uart_open(uart_device);
    if (car->uart_fd < 0) {
        return -1;
    }
    if (CarControl_Stop(car) < 0) {
        saved_errno = errno;
        uart_close(car->uart_fd);
        car->uart_fd = -1;
        errno = saved_errno;
        return -1;
    }
    return 0;
}

void CarControl_Close(CarControl *car)
{
    if (car != NULL && car->uart_fd >= 0) {
        (void)CarControl_Stop(car);
        uart_close(car->uart_fd);
        car->uart_fd = -1;
    }
}

int CarControl_SetSpeed(CarControl *car, int rpm)
{
    if (car == NULL || car->uart_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    if (rpm < CAR_SPEED_MIN_RPM || rpm > CAR_SPEED_MAX_RPM) {
        errno = ERANGE;
        return -1;
    }
    if (uart_set_speed(car->uart_fd, rpm, NULL) < 0) {
        return -1;
    }
    car->target_speed_rpm = rpm;
    return 0;
}

int CarControl_SpeedUp(CarControl *car)
{
    if (car == NULL || car->uart_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    return CarControl_SetSpeed(car,
        car->target_speed_rpm >= CAR_SPEED_MAX_RPM - CAR_SPEED_STEP_RPM ?
        CAR_SPEED_MAX_RPM : car->target_speed_rpm + CAR_SPEED_STEP_RPM);
}

int CarControl_SpeedDown(CarControl *car)
{
    if (car == NULL || car->uart_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    return CarControl_SetSpeed(car,
        car->target_speed_rpm <= CAR_SPEED_MIN_RPM + CAR_SPEED_STEP_RPM ?
        CAR_SPEED_MIN_RPM : car->target_speed_rpm - CAR_SPEED_STEP_RPM);
}

int CarControl_StopSpeed(CarControl *car)
{
    return CarControl_SetSpeed(car, 0);
}

int CarControl_SetYawRate(CarControl *car, int dps)
{
    if (car == NULL || car->uart_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    if (uart_set_yaw_rate(car->uart_fd, dps, NULL) < 0) {
        return -1;
    }
    car->target_yaw_rate_dps = dps;
    return 0;
}

int CarControl_TurnLeft(CarControl *car)
{
    return CarControl_SetYawRate(car, CAR_TURN_LEFT_DPS);
}

int CarControl_TurnRight(CarControl *car)
{
    return CarControl_SetYawRate(car, CAR_TURN_RIGHT_DPS);
}

int CarControl_StopTurn(CarControl *car)
{
    return CarControl_SetYawRate(car, 0);
}

int CarControl_Stop(CarControl *car)
{
    int saved_errno = 0;

    if (CarControl_StopSpeed(car) < 0) {
        saved_errno = errno;
    }
    /* Always attempt yaw=0 even when speed=0 failed. */
    if (CarControl_StopTurn(car) < 0 && saved_errno == 0) {
        saved_errno = errno;
    }
    if (saved_errno != 0) {
        errno = saved_errno;
        return -1;
    }
    return 0;
}