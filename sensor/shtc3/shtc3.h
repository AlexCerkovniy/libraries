#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

/* General definitions for SHTC3 sensor */
#include "shtc3_config.h"

typedef enum{
    SHTC3_OFF = 0,
    SHTC3_INIT,
    SHTC3_START,
    SHTC3_MEASURE,
    SHTC3_READ
} shtc3_state_t;

typedef enum{
  SHTC3_SLEEP =                                 ((uint16_t)0xB098),
  SHTC3_WAKEUP =                                ((uint16_t)0x3517),
  SHTC3_CLOCK_STR_DIS_NORMAL_T_FIRST =          ((uint16_t)0x7866),
  SHTC3_CLOCK_STR_DIS_NORMAL_RH_First =         ((uint16_t)0x58E0),
  SHTC3_CLOCK_STR_DIS_LP_T_FIRST =              ((uint16_t)0x609C),
  SHTC3_CLOCK_STR_DIS_LP_RH_First =             ((uint16_t)0x401A),
  SHTC3_CLOCK_STR_EN_NORMAL_T_FIRST =           ((uint16_t)0x7CA2),
  SHTC3_CLOCK_STR_EN_NORMAL_RH_First =          ((uint16_t)0x5C24),
  SHTC3_CLOCK_STR_EN_LP_T_FIRST =               ((uint16_t)0x6458),
  SHTC3_CLOCK_STR_EN_LP_RH_First =              ((uint16_t)0x44DE),
  SHTC3_SOFT_RESET =                            ((uint16_t)0x805D)
} shtc3_command_t;

typedef struct {
    uint8_t temperature_msb;
    uint8_t temperature_lsb;
    uint8_t temperature_crc;
    uint8_t humidity_msb;
    uint8_t humidity_lsb;
    uint8_t humidity_crc;
} shtc3_data_t;

void SHTC3_Init(void);
void SHTC3_Enabled(bool state);
void SHTC3_Main(void);
void SHTC3_Tick(uint32_t period);
void SHTC3_RegisterUpdateEventCallback(void (*callback)(void));
int16_t SHTC3_GetTemperature(void);
int16_t SHTC3_GetHumidity(void);
