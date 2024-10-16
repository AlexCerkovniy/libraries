#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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

	/* Configuration */
	adxl345_range_t range;
	adxl345_resolution_t resolution;
	adxl345_data_rate_t data_rate;
	adxl345_fifo_mode_t fifo_mode;
	uint8_t fifo_samples;

	/* Internal registers cache */
	uint8_t data_format_reg;
	uint8_t power_ctl_reg;
	uint8_t int_map_reg;
	uint8_t int_enable_reg;
	uint8_t fifo_ctl_reg;
} adxl345_t;

typedef struct {
	int16_t x, y, z;
} adxl345_axis_data_t;

void adxl345_init(adxl345_t *adxl);
void adxl345_read_axis_data(adxl345_t *adxl, adxl345_axis_data_t *data);

/* INT_MAP register methods */
uint8_t adxl345_int_map_get(adxl345_t *adxl);
void adxl345_int_map_write(adxl345_t *adxl, uint8_t value);
void adxl345_int_map_set(adxl345_t *adxl, uint8_t mask);
void adxl345_int_map_clear(adxl345_t *adxl, uint8_t mask);

/* INT_ENABLE register methods */
uint8_t adxl345_int_enable_get(adxl345_t *adxl);
void adxl345_int_enable_write(adxl345_t *adxl, uint8_t value);
void adxl345_int_enable_set(adxl345_t *adxl, uint8_t mask);
void adxl345_int_enable_clear(adxl345_t *adxl, uint8_t mask);

/* INT_SOURCE register methods */
uint8_t adxl345_int_source_read(adxl345_t *adxl);

/* FIFO_CTL register methods */
uint8_t adxl345_fifo_ctl_get(adxl345_t *adxl);
void adxl345_fifo_ctl_write(adxl345_t *adxl, uint8_t value);
void adxl345_fifo_ctl_set(adxl345_t *adxl, uint8_t mask);
void adxl345_fifo_ctl_clear(adxl345_t *adxl, uint8_t mask);
void adxl345_fifo_mode_set(adxl345_t *adxl, adxl345_fifo_mode_t mode);
void adxl345_fifo_samlpes_set(adxl345_t *adxl, uint8_t samples);

/* FIFO_STATUS register methods */
uint8_t adxl345_fifo_status_read(adxl345_t *adxl);
