#pragma once

#include <stdint.h>
#include <stdbool.h>

void PCD8544_PL_Init(void);
void PCD8544_PL_ResetPin(bool state);
void PCD8544_PL_NssPin(bool state);
void PCD8544_PL_DataCommandPin(bool state);
void PCD8544_PL_BacklightPin(bool state);
uint8_t PCD8544_PL_SPI_Transfer(uint8_t byte);
void PCD8544_PL_Delay(uint16_t milliseconds);
