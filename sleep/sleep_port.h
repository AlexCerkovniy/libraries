#pragma once

#include "stdint.h"
#include "stdbool.h"

void SLEEP_PL_Init(uint32_t wakeup_period);
void SLEEP_PL_Wakeup(void);
void SLEEP_PL_Sleep(void);
