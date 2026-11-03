#include "mpu6050.h"
#include "i2c.h"

#define MPU6050_ADDRESS_HAL       (0x68U << 1)
#define MPU6050_WHO_AM_I          0x75U
#define MPU6050_EXPECTED_ID       0x68U
#define MPU6050_PWR_MGMT_1        0x6BU
#define MPU6050_CONFIG            0x1AU
#define MPU6050_SMPLRT_DIV        0x19U
#define MPU6050_ACCEL_CONFIG      0x1CU
#define MPU6050_GYRO_CONFIG       0x1BU
#define MPU6050_ACCEL_XOUT_H      0x3BU

#define MPU6050_PLL_XGYRO         0x01U
/* CONFIG.DLPF_CFG=0: gyro 256 Hz / 8 kHz, accel 260 Hz / 1 kHz.
  No DLPF bandwidth restriction; STM32 filters its 100 Hz readouts. */
#define MPU6050_DLPF_CFG          0x00U
#define MPU6050_SAMPLE_DIV        0x00U
#define MPU6050_ACCEL_16G_BITS    (3U << 3)
#define MPU6050_GYRO_2000_BITS    (3U << 3)
#define MPU6050_ACCEL_LSB_PER_G   2048.0f
#define MPU6050_GYRO_LSB_PER_DPS  16.4f
#define MPU6050_I2C_TIMEOUT_MS    100U
#define MPU6050_STARTUP_MS        30U

static uint8_t mpu6050_ready;

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS_HAL, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6050_ReadRegister(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS_HAL, reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1U,
                          MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6050_VerifyRegister(uint8_t reg, uint8_t expected)
{
  uint8_t actual;
  HAL_StatusTypeDef status = MPU6050_ReadRegister(reg, &actual);
  if (status != HAL_OK)
  {
    return status;
  }
  return (actual == expected) ? HAL_OK : HAL_ERROR;
}

static int16_t MPU6050_DecodeBE(const uint8_t *bytes)
{
  uint16_t value = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
  return (value <= INT16_MAX) ? (int16_t)value
                              : (int16_t)((int32_t)value - 65536L);
}

HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t id;
  HAL_StatusTypeDef status;

  mpu6050_ready = 0U;
  status = HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDRESS_HAL, 2U,
                                 MPU6050_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }
  status = MPU6050_ReadRegister(MPU6050_WHO_AM_I, &id);
  if (status != HAL_OK)
  {
    return status;
  }
  if (id != MPU6050_EXPECTED_ID)
  {
    return HAL_ERROR;
  }

  status = MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, MPU6050_PLL_XGYRO);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(MPU6050_STARTUP_MS);

  status = MPU6050_WriteRegister(MPU6050_CONFIG, MPU6050_DLPF_CFG);
  if (status != HAL_OK) { return status; }
  status = MPU6050_WriteRegister(MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_DIV);
  if (status != HAL_OK) { return status; }
  status = MPU6050_WriteRegister(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_16G_BITS);
  if (status != HAL_OK) { return status; }
  status = MPU6050_WriteRegister(MPU6050_GYRO_CONFIG, MPU6050_GYRO_2000_BITS);
  if (status != HAL_OK) { return status; }

  status = MPU6050_VerifyRegister(MPU6050_PWR_MGMT_1, MPU6050_PLL_XGYRO);
  if (status != HAL_OK) { return status; }
  status = MPU6050_VerifyRegister(MPU6050_CONFIG, MPU6050_DLPF_CFG);
  if (status != HAL_OK) { return status; }
  status = MPU6050_VerifyRegister(MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_DIV);
  if (status != HAL_OK) { return status; }
  status = MPU6050_VerifyRegister(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_16G_BITS);
  if (status != HAL_OK) { return status; }
  status = MPU6050_VerifyRegister(MPU6050_GYRO_CONFIG, MPU6050_GYRO_2000_BITS);
  if (status != HAL_OK) { return status; }

  mpu6050_ready = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw_t *raw)
{
  uint8_t bytes[14];
  MPU6050_Raw_t sample;
  HAL_StatusTypeDef status;

  if ((raw == NULL) || (mpu6050_ready == 0U))
  {
    return HAL_ERROR;
  }
  status = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS_HAL, MPU6050_ACCEL_XOUT_H,
                            I2C_MEMADD_SIZE_8BIT, bytes, sizeof(bytes),
                            MPU6050_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  sample.accel_x = MPU6050_DecodeBE(&bytes[0]);
  sample.accel_y = MPU6050_DecodeBE(&bytes[2]);
  sample.accel_z = MPU6050_DecodeBE(&bytes[4]);
  sample.temperature = MPU6050_DecodeBE(&bytes[6]);
  sample.gyro_x = MPU6050_DecodeBE(&bytes[8]);
  sample.gyro_y = MPU6050_DecodeBE(&bytes[10]);
  sample.gyro_z = MPU6050_DecodeBE(&bytes[12]);
  *raw = sample;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read(MPU6050_Data_t *data)
{
  MPU6050_Raw_t raw;
  MPU6050_Data_t sample;
  HAL_StatusTypeDef status;

  if (data == NULL)
  {
    return HAL_ERROR;
  }
  status = MPU6050_ReadRaw(&raw);
  if (status != HAL_OK)
  {
    return status;
  }

  sample.accel_x_raw = raw.accel_x;
  sample.accel_y_raw = raw.accel_y;
  sample.accel_z_raw = raw.accel_z;
  sample.temperature_raw = raw.temperature;
  sample.gyro_x_raw = raw.gyro_x;
  sample.gyro_y_raw = raw.gyro_y;
  sample.gyro_z_raw = raw.gyro_z;
  sample.accel_x_g = (float)raw.accel_x / MPU6050_ACCEL_LSB_PER_G;
  sample.accel_y_g = (float)raw.accel_y / MPU6050_ACCEL_LSB_PER_G;
  sample.accel_z_g = (float)raw.accel_z / MPU6050_ACCEL_LSB_PER_G;
  sample.gyro_x_dps = (float)raw.gyro_x / MPU6050_GYRO_LSB_PER_DPS;
  sample.gyro_y_dps = (float)raw.gyro_y / MPU6050_GYRO_LSB_PER_DPS;
  sample.gyro_z_dps = (float)raw.gyro_z / MPU6050_GYRO_LSB_PER_DPS;
  *data = sample;
  return HAL_OK;
}
