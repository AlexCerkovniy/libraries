#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "st7565_user_port.h"

void ST7565_PL_Init(void);
void ST7565_PL_RST_Pin(bool state);
void ST7565_PL_CS_Pin(bool state);
void ST7565_PL_DC_Pin(bool state);
void ST7565_PL_Backlight(bool state);
uint8_t ST7565_PL_SPI_Transfer(uint8_t byte);
void ST7565_PL_Delay(uint32_t milliseconds);
void ST7565_PL_DelayUs(uint32_t microseconds);
