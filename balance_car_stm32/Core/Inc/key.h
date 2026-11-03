#ifndef KEY_H
#define KEY_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO is configured by MX_GPIO_Init(). True = PA12 shorted to GND. */
uint8_t Key_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */
