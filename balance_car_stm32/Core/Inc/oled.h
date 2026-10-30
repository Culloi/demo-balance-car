#ifndef OLED_H
#define OLED_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 128x64 SSD1306-compatible panel; return HAL_OK only after successful I2C writes. */
HAL_StatusTypeDef OLED_Init(void);
/* Becomes false on any display error; subsequent display calls do not use I2C1. */
uint8_t OLED_IsReady(void);
HAL_StatusTypeDef OLED_Clear(void);
/* Default 6x8 font: line 0..7, column 0..20 (21 cells across 128 pixels).
	Unsupported characters render as spaces; strings stop at the right edge. */
HAL_StatusTypeDef OLED_ShowChar(uint8_t line, uint8_t column, char character);
HAL_StatusTypeDef OLED_ShowString(uint8_t line, uint8_t column, const char *text);
/* Signed int32 occupies 12 cells; unused cells are cleared with spaces. */
HAL_StatusTypeDef OLED_ShowInt(uint8_t line, uint8_t column, int32_t number);
/* Signed float occupies 8 cells including sign; decimals 0..4.
	Unsupported/non-finite values display ########. */
HAL_StatusTypeDef OLED_ShowFloat(uint8_t line, uint8_t column,
											float number, uint8_t decimals);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
