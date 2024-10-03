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
	uint8_t device_id;

	adxl345_range_t range;
	adxl345_resolution_t resolution;
	adxl345_data_rate_t data_rate;
} adxl345_t;

typedef struct {
	int16_t x, y, z;
} adxl345_axis_data_t;

void adxl345_init(adxl345_t *adxl);
void adxl345_read_axis_data(adxl345_t *adxl, adxl345_axis_data_t *data);
int16_t adxl345_readx(adxl345_t *adxl);
int16_t adxl345_ready(adxl345_t *adxl);
int16_t adxl345_readz(adxl345_t *adxl);
