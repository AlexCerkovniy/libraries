#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "adxl345_registers.h"

typedef struct{
	void (*init)(void);
	void (*reset)(bool set);
	void (*read)(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length);
	void (*write)(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length);
	void (*delay_ms)(uint32_t ms);
} adxl345_driver_t;

typedef struct {
	adxl345_driver_t driver;
	uint8_t i2c_address;
} adxl345_t;

void adxl345_init(adxl345_t *adxl);
int16_t adxl345_readx(adxl345_t *adxl);
int16_t adxl345_ready(adxl345_t *adxl);
int16_t adxl345_readz(adxl345_t *adxl);
