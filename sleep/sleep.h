#pragma once

#include "stdint.h"
#include "stdbool.h"

/* Library configuration file */
#include "sleep_config.h"

typedef enum {
    SLEEP_TOKEN_0       = (1 << 0),
    SLEEP_TOKEN_1       = (1 << 1),
    SLEEP_TOKEN_2       = (1 << 2),
    SLEEP_TOKEN_3       = (1 << 3),
    SLEEP_TOKEN_4       = (1 << 4),
    SLEEP_TOKEN_5       = (1 << 5),
    SLEEP_TOKEN_6       = (1 << 6),
    SLEEP_TOKEN_7       = (1 << 7),
    SLEEP_TOKEN_8       = (1 << 8),
    SLEEP_TOKEN_9       = (1 << 9),
    SLEEP_TOKEN_10       = (1 << 10),
    SLEEP_TOKEN_11       = (1 << 11),
    SLEEP_TOKEN_12       = (1 << 12),
    SLEEP_TOKEN_13       = (1 << 13),
    SLEEP_TOKEN_14       = (1 << 14),
    SLEEP_TOKEN_15       = (1 << 15),
} sleep_token_t;

void SLEEP_Init(uint32_t wakeup_period);
void SLEEP_Main(void);
bool SLEEP_Sleep(void);
void SLEEP_GetToken(sleep_token_t mask);
void SLEEP_ClearToken(sleep_token_t mask);
