#include "balance_car.h"
#include "balance_controller.h"
#include "speed_controller.h"
#include "turn_controller.h"
#include "encoder.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "usart.h"
#include "mpu6050.h"
#include "attitude.h"
#include "tim.h"
#include <math.h>

/* Mechanical balance point, separate from the start/fall thresholds. */
#define BALANCE_CAR_FALL_ANGLE_DEG      25.0f
#define BALANCE_CAR_DT_SECONDS          0.010f
#define BALANCE_CAR_SPEED_DIVIDER       5
#define BALANCE_CAR_SPEED_DT_MS         50U
#define SPEED_FILTER_ALPHA              0.611f
#define BALANCE_CAR_OLED_REFRESH_MS     100U
#define BALANCE_CAR_KEY_STABLE_MS       300U
#define BALANCE_START_ANGLE_DEG         10.0f
#define BALANCE_CAR_MOTOR_LIMIT         1000.0f
#define BALANCE_CAR_TURN_TEST_ENABLE    1

#define BALANCE_CAR_BALANCE_ANGLE_DEG   0.69f
static volatile float target_speed_rpm = 0.0f;
static volatile float target_yaw_rate_dps = 0.0f;

/* Preserve signed rounding of the motor command. */
static int32_t BalanceCar_Round(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

/* Single-writer ISR state. 32-bit aligned snapshot fields are atomic on M3;
   the display may mix adjacent cycles but never reads a partially written word. */
static volatile uint8_t balance_running;
static volatile uint8_t balance_fault; /* 0 = none, 1 = fall, 2 = sensor/control */
static volatile float display_angle;
static volatile float display_gyro;
static volatile int32_t display_command;
static volatile float display_left_rpm;
static volatile float display_right_rpm;
static volatile float display_vehicle_rpm;
static volatile float display_speed_correction;
static volatile float display_d_term;
static Attitude_Calibration_t balance_calibration;
static Attitude_State_t balance_filter;
static BalanceController_t balance_controller;
static float speed_angle_correction_deg;
static float filtered_vehicle_speed_rpm = 0.0f;
static uint8_t speed_cycle_count;

/* Main-context key timing, display timing and TIM1 ownership. */
static uint32_t low_since_tick;
static uint32_t last_oled_tick;
static uint8_t low_timing;
static uint8_t release_required;
static uint8_t timer_started;

/* Refresh all dynamic data in main context, never in the TIM1 ISR. */
static void BalanceCar_UpdateDisplay(void)
{
  if (OLED_IsReady() == 0U) return;
  if (OLED_ShowFloat(0, 5, display_angle, 2) != HAL_OK ||
      OLED_ShowFloat(1, 5, balance_controller.balance_angle_deg -
                           display_speed_correction, 2) != HAL_OK ||
      OLED_ShowInt(2, 5, display_command) != HAL_OK ||
      OLED_ShowFloat(3, 5, display_vehicle_rpm, 1) != HAL_OK ||
      OLED_ShowFloat(4, 5, target_speed_rpm, 0) != HAL_OK ||
      OLED_ShowFloat(5, 5, target_yaw_rate_dps, 0) != HAL_OK ||
      OLED_ShowFloat(6, 5, display_left_rpm, 1) != HAL_OK ||
      OLED_ShowFloat(7, 5, display_right_rpm, 1) != HAL_OK)
    return; /* The OLED driver disables itself on failure; control continues. */
}

/* Static display labels are also best-effort, never a start requirement. */
static void BalanceCar_DrawDisplayLabels(void)
{
  if (OLED_IsReady() == 0U) return;
  if (OLED_Clear() != HAL_OK ||
      OLED_ShowString(0, 0, "ANG:") != HAL_OK ||
      OLED_ShowString(1, 0, "TAR:") != HAL_OK ||
      OLED_ShowString(2, 0, "OUT:") != HAL_OK ||
      OLED_ShowString(3, 0, "SPD:") != HAL_OK ||
      OLED_ShowString(4, 0, "SET:") != HAL_OK ||
      OLED_ShowString(5, 0, "YAW:") != HAL_OK ||
      OLED_ShowString(6, 0, "L:") != HAL_OK ||
      OLED_ShowString(7, 0, "R:") != HAL_OK)
    return;
}

void BalanceCar_SetTargetSpeed(float value)
{
  if (isfinite(value)) target_speed_rpm = value;
}

void BalanceCar_SetTargetYawRate(float value)
{
  if (isfinite(value)) target_yaw_rate_dps = value;
}

void BalanceCar_Control(void)
{
  MPU6050_Data_t imu;
  Attitude_Data_t attitude;
  float output;
  float turn_output;
  float left_output;
  float right_output;
  int16_t left_command;
  int16_t right_command;
  uint8_t reason = 2;

  if (balance_running == 0) return;
  if (Key_IsPressed() == 0) goto switch_off;
  if (MPU6050_Read(&imu) != HAL_OK) goto fault;
  if (Attitude_Update(&balance_filter, &imu, &balance_calibration,
                      BALANCE_CAR_DT_SECONDS, &attitude) != ATTITUDE_STATUS_OK ||
      !isfinite(attitude.fused_angle_deg) ||
      !isfinite(attitude.gyro_pitch_rate_dps)) goto fault;

  if (fabsf(attitude.fused_angle_deg - balance_controller.balance_angle_deg) >
      BALANCE_CAR_FALL_ANGLE_DEG)
  {
    reason = 1;
    goto fault;
  }
  if (++speed_cycle_count >= BALANCE_CAR_SPEED_DIVIDER)
  {
    const Encoder_Data_t *speed;
    float vehicle_speed_rpm;

    speed_cycle_count = 0;
    Encoder_Update(BALANCE_CAR_SPEED_DT_MS);
    speed = Encoder_GetData();
    vehicle_speed_rpm = (speed->left_rpm + speed->right_rpm) * 0.5f;
    filtered_vehicle_speed_rpm += SPEED_FILTER_ALPHA *
                    (vehicle_speed_rpm - filtered_vehicle_speed_rpm);
    speed_angle_correction_deg =
      SpeedController_Update(target_speed_rpm, filtered_vehicle_speed_rpm);
    if (!isfinite(speed_angle_correction_deg)) goto fault;
    display_left_rpm = speed->left_rpm;
    display_right_rpm = speed->right_rpm;
    display_vehicle_rpm = vehicle_speed_rpm;
    display_speed_correction = speed_angle_correction_deg;
  }
  output = BalanceController(&balance_controller, attitude.fused_angle_deg,
                             attitude.gyro_pitch_rate_dps,
                             balance_controller.balance_angle_deg -
                             speed_angle_correction_deg);
  if (!isfinite(output)) goto fault;
  turn_output = 0.0f;
  if (BALANCE_CAR_TURN_TEST_ENABLE != 0)
  {
    if (!isfinite(target_yaw_rate_dps) ||
        !isfinite(attitude.corrected_gyro_z_dps)) goto fault;
    turn_output = TurnController_Update(target_yaw_rate_dps,
                                        attitude.corrected_gyro_z_dps);
    if (!isfinite(turn_output)) goto fault;
  }

  /* With turning disabled, both outputs equal the upright output.
     Turning polarity needs real-car verification; swap +/- if feedback is positive. */
  left_output = output - turn_output;
  right_output = output + turn_output;
  if (!isfinite(left_output) || !isfinite(right_output)) goto fault;
  if (left_output > BALANCE_CAR_MOTOR_LIMIT) left_output = BALANCE_CAR_MOTOR_LIMIT;
  else if (left_output < -BALANCE_CAR_MOTOR_LIMIT) left_output = -BALANCE_CAR_MOTOR_LIMIT;
  if (right_output > BALANCE_CAR_MOTOR_LIMIT) right_output = BALANCE_CAR_MOTOR_LIMIT;
  else if (right_output < -BALANCE_CAR_MOTOR_LIMIT) right_output = -BALANCE_CAR_MOTOR_LIMIT;
  left_command = (int16_t)BalanceCar_Round(left_output);
  right_command = (int16_t)BalanceCar_Round(right_output);

  /* No 300 ms debounce in the ISR; a HIGH seen before either motor write
     must not allow the other motor to restart. */
  if (Key_IsPressed() == 0) goto switch_off;
  Motor_SetLeft(left_command);
  if (Key_IsPressed() == 0) goto switch_off;
  Motor_SetRight(right_command);

  display_angle = attitude.fused_angle_deg;
  display_gyro = attitude.gyro_pitch_rate_dps;
  display_command = BalanceCar_Round(output); /* Show the base, not one wheel. */
  display_d_term = BalanceController_GetData(&balance_controller)->d_term;
  return;

switch_off:
  reason = 0;
fault:
  balance_running = 0;
  Motor_Stop();
  SpeedController_Reset();
  filtered_vehicle_speed_rpm = 0.0f;
  speed_angle_correction_deg = 0.0f;
  speed_cycle_count = 0;
  target_speed_rpm = 0.0f;
  target_yaw_rate_dps = 0.0f;
  balance_fault = reason;
}

/* The single HAL timer callback: hardware IRQ remains in stm32f1xx_it.c. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) BalanceCar_Control();
}

void BalanceCar_Init(void)
{
  balance_running = 0;
  target_speed_rpm = 0.0f;
  target_yaw_rate_dps = 0.0f;
  low_since_tick = 0;
  last_oled_tick = 0;
  low_timing = 0;
  release_required = 0;
  timer_started = 0;
  Motor_Stop();
  Encoder_Init(); /* Start both encoder counters once, before arming TIM1. */
}

/* Runs in main context only; calibration and OLED writes must never enter TIM1 ISR. */
static HAL_StatusTypeDef BalanceCar_Start(void)
{
  MPU6050_Data_t imu;
  Attitude_Data_t attitude;

  if (MPU6050_Init() != HAL_OK) return HAL_ERROR;
  if (Attitude_CalibrateGyro(&balance_calibration) != HAL_OK) return HAL_ERROR;
  Attitude_Reset(&balance_filter);
  BalanceController_Init(&balance_controller, BALANCE_CAR_BALANCE_ANGLE_DEG);
  SpeedController_Init();
  if (Key_IsPressed() == 0)
  {
    balance_fault = 0;
    return HAL_ERROR;
  }

  /* Seed the filter from a valid upright sample before arming TIM1. */
  if (MPU6050_Read(&imu) != HAL_OK ||
      Attitude_Update(&balance_filter, &imu, &balance_calibration, 0.0f,
                      &attitude) != ATTITUDE_STATUS_INIT ||
      !isfinite(attitude.fused_angle_deg) ||
      !isfinite(attitude.gyro_pitch_rate_dps) ||
      fabsf(attitude.fused_angle_deg - balance_controller.balance_angle_deg) >
      BALANCE_START_ANGLE_DEG || Key_IsPressed() == 0)
  {
    if (Key_IsPressed() == 0) balance_fault = 0;
    return HAL_ERROR;
  }

  BalanceCar_DrawDisplayLabels();
  if (Key_IsPressed() == 0)
  {
    balance_fault = 0;
    return HAL_ERROR;
  }
  display_angle = attitude.fused_angle_deg;
  display_gyro = attitude.gyro_pitch_rate_dps;
  display_command = 0;
  display_left_rpm = 0.0f;
  display_right_rpm = 0.0f;
  display_vehicle_rpm = 0.0f;
  display_speed_correction = 0.0f;
  display_d_term = 0.0f;
  BalanceCar_UpdateDisplay();
  if (Key_IsPressed() == 0)
  {
    balance_fault = 0;
    return HAL_ERROR;
  }
  balance_fault = 0;
  last_oled_tick = HAL_GetTick();

  /* Discard movement during calibration and start each 50 ms window afresh. */
  Encoder_ReadLeftDelta();
  Encoder_ReadRightDelta();
  SpeedController_Reset();
  filtered_vehicle_speed_rpm = 0.0f;
  speed_angle_correction_deg = 0.0f;
  speed_cycle_count = 0;
  target_speed_rpm = 0.0f;
  target_yaw_rate_dps = 0.0f;

  /* Init generated UIF may be set; discard it before enabling update IRQ. */
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
  balance_running = 1;
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) return HAL_ERROR;
  timer_started = 1;
  return HAL_OK;
}

/* Main-context cleanup; ISR faults already stop both motors immediately. */
static void BalanceCar_Stop(void)
{
  balance_running = 0;
  target_speed_rpm = 0.0f;
  target_yaw_rate_dps = 0.0f;
  Motor_Stop();
  SpeedController_Reset();
  filtered_vehicle_speed_rpm = 0.0f;
  speed_angle_correction_deg = 0.0f;
  speed_cycle_count = 0;
  if (timer_started != 0)
  {
    HAL_TIM_Base_Stop_IT(&htim1);
    timer_started = 0;
  }
  low_timing = 0;
  release_required = 1;
  balance_fault = 0;
}

/* One non-real-time main-loop pass; never waits for a new key press. */
void BalanceCar_Task(void)
{
  if (balance_running != 0)
  {
    if (Key_IsPressed() == 0)
    {
      balance_running = 0; /* Disarm before Motor_Stop: ISR cannot re-enable PWM. */
      BalanceCar_Stop();
      return;
    }
    if ((uint32_t)(HAL_GetTick() - last_oled_tick) >= BALANCE_CAR_OLED_REFRESH_MS)
    {
      last_oled_tick = HAL_GetTick();
      BalanceCar_UpdateDisplay();
    }
    return;
  }

  /* An ISR fault already stopped PWM; the main loop now stops TIM1. */
  if (timer_started != 0)
  {
    BalanceCar_Stop();
    return;
  }

  /* After every stop, require HIGH then 300 ms continuous LOW. */
  if (Key_IsPressed() == 0)
  {
    release_required = 0;
    low_timing = 0;
    return;
  }
  if (release_required != 0) return;
  if (low_timing == 0)
  {
    low_since_tick = HAL_GetTick();
    low_timing = 1;
  }
  if ((uint32_t)(HAL_GetTick() - low_since_tick) < BALANCE_CAR_KEY_STABLE_MS)
    return;

  low_timing = 0;
  release_required = 1;
  balance_fault = 2;
  if (BalanceCar_Start() != HAL_OK)
  {
    BalanceCar_Stop();
    return;
  }
  /* The first TIM1 interrupt may already have stopped the car. */
  if (balance_running == 0)
  {
    BalanceCar_Stop();
  }
}
