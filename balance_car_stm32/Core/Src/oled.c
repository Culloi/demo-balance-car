#include "oled.h"
#include "oled_font6x8.h"
#include "i2c.h"
#include <math.h>

#define OLED_FONT_WIDTH   6
#define OLED_FONT_HEIGHT  8
#define OLED_COLUMNS      21
#define OLED_ADDRESS_7BIT  0x3CU
#define OLED_ADDRESS_HAL   (OLED_ADDRESS_7BIT << 1)
#define OLED_I2C_TIMEOUT   100

/* Optional debug display: a failed transaction disables further I2C1 writes. */
static uint8_t oled_ready = 0U;

uint8_t OLED_IsReady(void)
{
  return oled_ready;
}

static HAL_StatusTypeDef OLED_Transmit(uint8_t *packet, uint16_t length)
{
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS_HAL,
                                                     packet, length, OLED_I2C_TIMEOUT);
  if (status != HAL_OK) oled_ready = 0U;
  return status;
}

static HAL_StatusTypeDef OLED_SendCommands(const uint8_t *commands, uint16_t length)
{
  uint8_t packet[32];
  uint16_t i;

  if (length > sizeof(packet) - 1U)
  {
    oled_ready = 0U;
    return HAL_ERROR;
  }
  packet[0] = 0x00U; /* SSD1306: Co=0, D/C#=0: subsequent bytes are commands. */
  for (i = 0; i < length; ++i)
  {
    packet[i + 1U] = commands[i];
  }
  return OLED_Transmit(packet, length + 1U);
}

static HAL_StatusTypeDef OLED_SetPosition(uint8_t page, uint8_t x)
{
  uint8_t commands[3];
  commands[0] = (uint8_t)(0xB0U | page);
  commands[1] = (uint8_t)(0x00U | (x & 0x0FU));
  commands[2] = (uint8_t)(0x10U | (x >> 4));
  return OLED_SendCommands(commands, sizeof(commands));
}

HAL_StatusTypeDef OLED_Init(void)
{
  /* 128x64, internal charge pump, page-addressing mode. */
  static const uint8_t init_commands[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
    0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x30, 0xA4, 0xA6, 0xAF
  };

  HAL_StatusTypeDef status;

  oled_ready = 0U;
  HAL_Delay(100); /* OLED supply settling; only at boot. */
  status = OLED_SendCommands(init_commands, sizeof(init_commands));
  if (status == HAL_OK) oled_ready = 1U;
  return status;
}

HAL_StatusTypeDef OLED_Clear(void)
{
  uint8_t blank[129] = {0};
  uint8_t page;
  if (oled_ready == 0U) return HAL_ERROR;
  blank[0] = 0x40U; /* Co=0, D/C#=1: following bytes are display data. */

  for (page = 0; page < 8; ++page)
  {
    HAL_StatusTypeDef status = OLED_SetPosition(page, 0);
    if (status != HAL_OK)
    {
      return status;
    }
    status = OLED_Transmit(blank, sizeof(blank));
    if (status != HAL_OK)
    {
      return status;
    }
  }
  return HAL_OK;
}

/* Render a cell into six page bytes; the sixth column remains blank. */
static void OLED_EncodeChar(uint8_t *pixels, char character)
{
  uint8_t glyph_index = ((unsigned char)character >= 0x20U &&
                         (unsigned char)character <= 0x7EU) ?
                        (uint8_t)((unsigned char)character - 0x20U) : 0U;
  uint8_t x;
  uint8_t row;

  for (x = 0; x < OLED_FONT_WIDTH; ++x)
  {
    pixels[x] = 0;
    if (x < 5)
    {
      for (row = 0; row < 7; ++row)
      {
        if ((oled_font6x8[glyph_index][row] & (0x10U >> x)) != 0)
          pixels[x] |= (uint8_t)(1U << row);
      }
    }
  }
}

HAL_StatusTypeDef OLED_ShowString(uint8_t line, uint8_t column, const char *text)
{
  uint8_t packet[1U + OLED_COLUMNS * OLED_FONT_WIDTH];
  uint8_t cells = 0;
  HAL_StatusTypeDef status;

  if (oled_ready == 0U) return HAL_ERROR;
  if (line >= 64 / OLED_FONT_HEIGHT || column >= OLED_COLUMNS || text == NULL)
  {
    oled_ready = 0U;
    return HAL_ERROR;
  }

  packet[0] = 0x40U;
  while (*text != '\0' && (uint8_t)(column + cells) < OLED_COLUMNS)
  {
    OLED_EncodeChar(&packet[1U + cells * OLED_FONT_WIDTH], *text++);
    ++cells;
  }
  if (cells == 0) return HAL_OK;

  status = OLED_SetPosition(line, (uint8_t)(column * OLED_FONT_WIDTH));
  if (status != HAL_OK) return status;
  return OLED_Transmit(packet, (uint16_t)(1U + cells * OLED_FONT_WIDTH));
}

HAL_StatusTypeDef OLED_ShowChar(uint8_t line, uint8_t column, char character)
{
  char text[2] = {character, '\0'};
  /* A NUL character is an unsupported glyph, not an empty string. */
  if (character == '\0') text[0] = ' ';
  return OLED_ShowString(line, column, text);
}

HAL_StatusTypeDef OLED_ShowInt(uint8_t line, uint8_t column, int32_t number)
{
  char reversed[11];
  char text[13]; /* 11 digits/sign + one blank to erase a previous value. */
  uint32_t magnitude = (number < 0) ? (uint32_t)(-(number + 1)) + 1U
                                      : (uint32_t)number;
  uint8_t length = 0;
  uint8_t i;

  do
  {
    reversed[length++] = (char)('0' + magnitude % 10U);
    magnitude /= 10U;
  } while (magnitude != 0U);
  if (number < 0) reversed[length++] = '-';

  for (i = 0; i < 12; ++i)
    text[i] = (i < length) ? reversed[length - 1U - i] : ' ';
  text[12] = '\0';
  return OLED_ShowString(line, column, text);
}

HAL_StatusTypeDef OLED_ShowFloat(uint8_t line, uint8_t column,
                                 float number, uint8_t decimals)
{
  char text[9]; /* Always overwrite eight cells, including the old tail. */
  uint32_t scale = 1U;
  uint32_t magnitude;
  uint32_t whole;
  uint32_t fraction;
  uint32_t divisor;
  uint8_t i = 1;
  uint8_t j;
  char reversed[7];
  uint8_t digits = 0;
  float scaled;

  if (decimals > 4)
  {
    oled_ready = 0U;
    return HAL_ERROR;
  }
  for (j = 0; j < decimals; ++j) scale *= 10U;
  scaled = fabsf(number) * (float)scale;
  if (!isfinite(scaled) ||
      scaled >= (float)((decimals == 0) ? 9999999U : 999999U) + 0.5f)
    return OLED_ShowString(line, column, "########");

  magnitude = (uint32_t)(scaled + 0.5f);
  whole = magnitude / scale;
  fraction = magnitude % scale;
  text[0] = (number < 0.0f) ? '-' : '+';
  do
  {
    reversed[digits++] = (char)('0' + whole % 10U);
    whole /= 10U;
  } while (whole != 0U);
  while (digits != 0) text[i++] = reversed[--digits];

  if (decimals != 0)
  {
    text[i++] = '.';
    divisor = scale / 10U;
    for (j = 0; j < decimals; ++j)
    {
      text[i++] = (char)('0' + fraction / divisor);
      fraction %= divisor;
      divisor /= 10U;
    }
  }
  while (i < 8) text[i++] = ' ';
  text[8] = '\0';
  return OLED_ShowString(line, column, text);
}
