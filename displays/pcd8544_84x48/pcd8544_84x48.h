#pragma once

#include "stdint.h"
#include "stdbool.h"

/* Library configuration */
#define PCD8544_RST_DELAY_MS                2
#define PCD8544_SCREEN_WIDTH                84         
#define PCD8544_SCREEN_HEIGTH               48

/* Internal command registers values */
#define PCD8544_DISP_CTRL_BLANK             0x08
#define PCD8544_DISP_CTRL_NORMAL_MODE       0x0C
#define PCD8544_DISP_CTRL_ALL_ON            0x09
#define PCD8544_DISP_CTRL_INVERT            0x0D

#define PCD8544_TEMP_COEFFICIENT_0          0x04
#define PCD8544_TEMP_COEFFICIENT_1          0x05
#define PCD8544_TEMP_COEFFICIENT_2          0x06
#define PCD8544_TEMP_COEFFICIENT_3          0x07

#define PCD8544_BIAS_SYSTEM_7               0x10
#define PCD8544_BIAS_SYSTEM_6               0x11
#define PCD8544_BIAS_SYSTEM_5               0x12
#define PCD8544_BIAS_SYSTEM_4               0x13
#define PCD8544_BIAS_SYSTEM_3               0x14
#define PCD8544_BIAS_SYSTEM_2               0x15
#define PCD8544_BIAS_SYSTEM_1               0x16
#define PCD8544_BIAS_SYSTEM_0               0x17

void PCD8544_Init(void);
uint8_t *PCD8544_GetFramebuffer(void);
uint16_t PCD8544_GetFramebufferSize(void);
void PCD8544_Sleep(bool state);
void PCD8544_Backlight(bool state);
void PCD8544_SetContrast(uint8_t contrast);
void PCD8544_Clear(void);
void PCD8544_Update(void);
