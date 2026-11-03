#include "speed_controller.h"
#include "pid.h"

#define SPEED_CONTROLLER_DT_SECONDS       0.050f
#define SPEED_CONTROLLER_ANGLE_LIMIT_DEG  3.0f
#define SPEED_CONTROLLER_I_LIMIT_DEG      2.0f

/* Conservative starting gains: degrees/RPM, degrees/(RPM*s). Tune on hardware. */
static PID_t SpeedPID = {
  .config = {
    .kp = 0.065f,//0.055
    .ki = 0.006f,//0.006
    .kd = 0.0f,
    .output_min = -SPEED_CONTROLLER_ANGLE_LIMIT_DEG,
    .output_max = SPEED_CONTROLLER_ANGLE_LIMIT_DEG,
    .integral_min = -SPEED_CONTROLLER_I_LIMIT_DEG,
    .integral_max = SPEED_CONTROLLER_I_LIMIT_DEG
  }
};

void SpeedController_Init(void)
{
  PID_Init(&SpeedPID);
}

float SpeedController_Update(float target_rpm, float measured_rpm)
{
  /* The caller subtracts this speed correction from the base target angle.
     Never send the PI output directly to the motors. */
  return PID_Update(&SpeedPID, measured_rpm - target_rpm, 0.0f,
                    SPEED_CONTROLLER_DT_SECONDS);
}

void SpeedController_Reset(void)
{
  PID_Reset(&SpeedPID);
}
