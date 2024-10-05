#include "adxl345.h"

void adxl345_init(adxl345_t *adxl){
	uint8_t tmp = 0;

	/* Initialize driver */
	adxl->driver.init();

	/* Configuration */
	adxl->driver.read(adxl->i2c_address, DEVID, &adxl->device_id, 1);
	tmp = adxl->range | adxl->resolution;
	adxl->driver.write(adxl->i2c_address, DATA_FORMAT, &tmp, 1);
	adxl->driver.write(adxl->i2c_address, BW_RATE, (uint8_t *)&adxl->data_rate, 1);

	/* Start measurement */
	tmp = ADXL_PWR_CTRL_MEASUREMENT_EN | ADXL_PWR_CTRL_SLEEP_SAMPLING_8HZ;
	adxl->driver.write(adxl->i2c_address, POWER_CTL, &tmp, 1);
}

void adxl345_read_axis_data(adxl345_t *adxl, adxl345_axis_data_t *data){
	uint8_t buffer[6];

	adxl->driver.read(adxl->i2c_address, DATAX0, buffer, 6);
	data->x = ((int16_t)buffer[1] << 8) | (int16_t)buffer[0];
	data->y = ((int16_t)buffer[3] << 8) | (int16_t)buffer[2];
	data->z = ((int16_t)buffer[5] << 8) | (int16_t)buffer[4];
}

/**
  * @brief  Get INT_MAP register value
  * @param  adxl - instance [adxl345_t]
  * @retval register value (see adxl345_int_map_flags_t)
  */
uint8_t adxl345_int_map_get(adxl345_t *adxl){
	return adxl->int_map;
}

/**
  * @brief  Write to INT_MAP register
  * @param  adxl - instance [adxl345_t]
  * @param  value - new register value (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_map_write(adxl345_t *adxl, uint8_t value){
	adxl->int_map = value;
	adxl->driver.write(adxl->i2c_address, INT_MAP, &value, 1);
}

/**
  * @brief  Set bits in INT_MAP register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_map_set(adxl345_t *adxl, uint8_t mask){
	adxl->int_map |= mask;
	adxl->driver.write(adxl->i2c_address, INT_MAP, &adxl->int_map, 1);
}

/**
  * @brief  Clear bits in INT_MAP register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_map_clear(adxl345_t *adxl, uint8_t mask){
	adxl->int_map &= ~mask;
	adxl->driver.write(adxl->i2c_address, INT_MAP, &adxl->int_map, 1);
}

/**
  * @brief  Get INT_ENABLE register value
  * @param  adxl - instance [adxl345_t]
  * @retval register value (see adxl345_int_flags_t)
  */
uint8_t adxl345_int_config_get(adxl345_t *adxl){
	return adxl->int_enable;
}

/**
  * @brief  Write to INT_ENABLE register
  * @param  adxl - instance [adxl345_t]
  * @param  value - new register value (see adxl345_int_flags_t)
  * @retval none
  */
void adxl345_int_config_write(adxl345_t *adxl, uint8_t value){
	adxl->int_enable = value;
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &value, 1);
}

/**
  * @brief  Set bits in INT_ENABLE register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_enable_set(adxl345_t *adxl, uint8_t mask){
	adxl->int_enable |= mask;
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &adxl->int_enable, 1);
}

/**
  * @brief  Clear bits in INT_ENABLE register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_enable_clear(adxl345_t *adxl, uint8_t mask){
	adxl->int_enable &= ~mask;
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &adxl->int_enable, 1);
}
