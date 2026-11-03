#include "turn_controller.h"
#include <math.h>

/* Initial bench-test values: a 20 deg/s error gives 10 motor units;
   at most 40 of the 1000 available units are reserved for turning. */
#define TURN_OUTPUT_LIMIT  180.0f
static float TurnKp = 1.8f;

float TurnController_Update(float target_yaw_rate_dps,
                            float actual_yaw_rate_dps)
{
  float error;
  float output;

  if (!isfinite(target_yaw_rate_dps) || !isfinite(actual_yaw_rate_dps))
    return NAN;

  error = target_yaw_rate_dps - actual_yaw_rate_dps;
  output = TurnKp * error;
  if (!isfinite(error) || !isfinite(output)) return NAN;

  if (output > TURN_OUTPUT_LIMIT) output = TURN_OUTPUT_LIMIT;
  else if (output < -TURN_OUTPUT_LIMIT) output = -TURN_OUTPUT_LIMIT;
  return output;
}
