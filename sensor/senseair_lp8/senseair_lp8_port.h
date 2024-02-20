#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

#include "senseair_lp8_user_port.h"

void LP8_PL_Init(void);
void LP8_PL_UART_Init(uint32_t baudrate);
void LP8_PL_UART_DeInit(void);
void LP8_PL_UART_Tx(uint8_t *data, uint8_t length);
bool LP8_PL_UART_Rx(uint8_t *data, uint8_t length);
bool LP8_PL_RDY_Read(void);
void LP8_PL_VBB_EN_Set(bool state);
void LP8_PL_Delay(uint32_t milliseconds);
