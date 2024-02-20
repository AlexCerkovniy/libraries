#pragma once

#include "stdint.h"
#include "stdbool.h"

/* Library configuration file */
#include "st7565_config.h"

#define ST7565_PAGE_SIZE            (128U)
#define ST7565_PAGES_COUNT          (8U)

#define CMD_DISPLAY_OFF   0xAE
#define CMD_DISPLAY_ON    0xAF

#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE  0xB0

#define CMD_SET_COLUMN_UPPER  0x10
#define CMD_SET_COLUMN_LOWER  0x00

#define CMD_SET_ADC_NORMAL  0xA0
#define CMD_SET_ADC_REVERSE 0xA1

#define CMD_SET_DISP_NORMAL 0xA6
#define CMD_SET_DISP_REVERSE 0xA7

#define CMD_SET_ALLPTS_NORMAL 0xA4
#define CMD_SET_ALLPTS_ON  0xA5

#define CMD_RMW  0xE0
#define CMD_RMW_CLEAR 0xEE
#define CMD_INTERNAL_RESET  0xE2
#define CMD_SET_COM_NORMAL  0xC0
#define CMD_SET_COM_REVERSE  0xC8


#define CMD_SET_STATIC_OFF  0xAC
#define  CMD_SET_STATIC_ON  0xAD
#define CMD_SET_STATIC_REG  0x0
#define CMD_SET_BOOSTER_FIRST  0xF8
#define CMD_SET_BOOSTER_234  0
#define  CMD_SET_BOOSTER_5  1
#define  CMD_SET_BOOSTER_6  3
#define CMD_NOP  0xE3
#define CMD_TEST  0xF0

/* Software Reset */
#define CMD_SOFT_RESET                  (0xE2)

/* Bias */
#define CMD_SET_BIAS_9                  (0xA2)
#define CMD_SET_BIAS_7                  (0xA3)

/* Power Controller Set */
#define CMD_SET_POWER_CONTROL           (0x28)  //Default: booster OFF, voltage regulator OFF, voltage follower OFF
#define POWER_CONTROL_BOOSTER           (0x04)  //Booster mask in CMD_SET_POWER_CONTROL command (set - ON, reset - OFF)
#define POWER_CONTROL_VREGULATOR        (0x02)  //Voltage regulator mask in CMD_SET_POWER_CONTROL command (set - ON, reset - OFF)
#define POWER_CONTROL_VFOLLOWER         (0x01)  //Voltage follower mask in CMD_SET_POWER_CONTROL command (set - ON, reset - OFF)

/* V5 Voltage Regulator Internal Resistor Ratio Set  */
#define CMD_SET_RESISTOR_RATIO          (0x20)
#define CMD_SET_RESISTOR_RATIO_MIN_MASK (0)
#define CMD_SET_RESISTOR_RATIO_MAX_MASK (0x07)

/* The Electronic Volume (Double Byte Command) */
#define CMD_SET_VOLUME_FIRST            (0x81)
#define CMD_SET_VOLUME_SECOND           (0)

/* The Booster Ratio (Double Byte Command) */
#define CMD_SET_BOOSTER_RATIO_FIRST     (0xF8)
#define CMD_SET_BOOSTER_RATIO_SECOND    (0)
#define BOOSTER_RATIO_2X_3X_4X_MASK     (0)
#define BOOSTER_RATIO_5X_MASK           (0x01)
#define BOOSTER_RATIO_6X_MASK           (0x03)

void ST7565_Init(void);
uint8_t *ST7565_GetFramebuffer(void);
uint16_t ST7565_GetFramebufferSize(void);
void ST7565_Sleep(bool state);
void ST7565_Backlight(bool state);
bool ST7565_GetBacklightState(void);
void ST7565_SetContrast(uint8_t contrast);
void ST7565_Clear(void);
void ST7565_Update(void);
