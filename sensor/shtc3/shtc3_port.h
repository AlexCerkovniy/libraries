#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "shtc3_user_port.h"

void SHTC3_PL_Init(void);
void SHTC3_PL_I2C_Init(void);
void SHTC3_PL_I2C_DeInit(void);
void SHTC3_PL_I2C_Write(uint8_t address, uint8_t *data, uint8_t count);
void SHTC3_PL_I2C_Read(uint8_t address, uint8_t *data, uint8_t count);
void SHTC3_PL_Delay(uint32_t milliseconds);
