#include "key.h"
#include "main.h"

uint8_t Key_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(BALANCE_SW_GPIO_Port, BALANCE_SW_Pin) ==
          GPIO_PIN_RESET) ? 1U : 0U;
}
