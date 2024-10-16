#include "adxl345.h"

/**
  * @brief  Initialize ADXL345
  * @param  adxl - instance [adxl345_t]
  * @retval none
  */
void adxl345_init(adxl345_t *adxl){
	uint8_t tmp;

	/* Initialize driver */
	adxl->driver.init();

	/* Set defaults */
	tmp = 0;
	adxl->driver.write(adxl->i2c_address, POWER_CTL, &tmp, 1);
	adxl->driver.write(adxl->i2c_address, DATA_FORMAT, &tmp, 1);
	adxl->driver.write(adxl->i2c_address, FIFO_CTL, &tmp, 1);
	adxl->driver.write(adxl->i2c_address, INT_MAP, &tmp, 1);
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &tmp, 1);
	adxl->driver.read(adxl->i2c_address, INT_SOURCE, &tmp, 1);
	adxl345_read_axis_data(adxl, NULL);

	/* Configuration */
	adxl->driver.read(adxl->i2c_address, DEVID, &adxl->device_id, 1);
	adxl->data_format_reg = adxl->range | adxl->resolution;
	adxl->driver.write(adxl->i2c_address, DATA_FORMAT, &adxl->data_format_reg, 1);
	adxl->driver.write(adxl->i2c_address, BW_RATE, (uint8_t *)&adxl->data_rate, 1);
	adxl->fifo_ctl_reg = adxl->fifo_mode | adxl->fifo_samples;
	adxl->driver.write(adxl->i2c_address, FIFO_CTL, &adxl->fifo_ctl_reg, 1);
	adxl->driver.write(adxl->i2c_address, INT_MAP, &adxl->int_map_reg, 1);
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &adxl->int_enable_reg, 1);

	/* Start measurement */
	adxl->power_ctl_reg = ADXL_PWR_CTRL_MEASUREMENT_EN | ADXL_PWR_CTRL_SLEEP_SAMPLING_8HZ;
	adxl->driver.write(adxl->i2c_address, POWER_CTL, &adxl->power_ctl_reg, 1);
}

/**
  * @brief  Get axis data
  * @param  adxl - instance [adxl345_t]
  * @param data - pointer to storage [adxl345_axis_data_t]
  * @retval none
  */
void adxl345_read_axis_data(adxl345_t *adxl, adxl345_axis_data_t *data){
	uint8_t buffer[6];

	adxl->driver.read(adxl->i2c_address, DATAX0, buffer, 6);
	if(data){
		data->x = ((int16_t)buffer[1] << 8) | (int16_t)buffer[0];
		data->y = ((int16_t)buffer[3] << 8) | (int16_t)buffer[2];
		data->z = ((int16_t)buffer[5] << 8) | (int16_t)buffer[4];
	}
}

/**
  * @brief  Get INT_MAP register value
  * @param  adxl - instance [adxl345_t]
  * @retval register value (see adxl345_int_map_flags_t)
  */
uint8_t adxl345_int_map_get(adxl345_t *adxl){
	return adxl->int_map_reg;
}

/**
  * @brief  Write to INT_MAP register
  * @param  adxl - instance [adxl345_t]
  * @param  value - new register value (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_map_write(adxl345_t *adxl, uint8_t value){
	adxl->int_map_reg = value;
	adxl->driver.write(adxl->i2c_address, INT_MAP, &value, 1);
}

/**
  * @brief  Set bits in INT_MAP register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_map_set(adxl345_t *adxl, uint8_t mask){
	adxl->int_map_reg |= mask;
	adxl->driver.write(adxl->i2c_address, INT_MAP, &adxl->int_map_reg, 1);
}

/**
  * @brief  Clear bits in INT_MAP register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_map_clear(adxl345_t *adxl, uint8_t mask){
	adxl->int_map_reg &= ~mask;
	adxl->driver.write(adxl->i2c_address, INT_MAP, &adxl->int_map_reg, 1);
}

/**
  * @brief  Get INT_ENABLE register value
  * @param  adxl - instance [adxl345_t]
  * @retval register value (see adxl345_int_flags_t)
  */
uint8_t adxl345_int_config_get(adxl345_t *adxl){
	return adxl->int_enable_reg;
}

/**
  * @brief  Write to INT_ENABLE register
  * @param  adxl - instance [adxl345_t]
  * @param  value - new register value (see adxl345_int_flags_t)
  * @retval none
  */
void adxl345_int_config_write(adxl345_t *adxl, uint8_t value){
	adxl->int_enable_reg = value;
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &value, 1);
}

/**
  * @brief  Set bits in INT_ENABLE register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_enable_set(adxl345_t *adxl, uint8_t mask){
	adxl->int_enable_reg |= mask;
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &adxl->int_enable_reg, 1);
}

/**
  * @brief  Clear bits in INT_ENABLE register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_int_enable_clear(adxl345_t *adxl, uint8_t mask){
	adxl->int_enable_reg &= ~mask;
	adxl->driver.write(adxl->i2c_address, INT_ENABLE, &adxl->int_enable_reg, 1);
}

/**
  * @brief  Get INT_SOURCE register value
  * @param  adxl - instance [adxl345_t]
  * @retval register value (see adxl345_int_flags_t)
  */
uint8_t adxl345_int_source_read(adxl345_t *adxl){
	uint8_t tmp = 0;
	adxl->driver.read(adxl->i2c_address, INT_SOURCE, &tmp, 1);
	return tmp;
}

/**
  * @brief  Get FIFO_CTL register value
  * @param  adxl - instance [adxl345_t]
  * @retval register value
  */
uint8_t adxl345_fifo_ctl_get(adxl345_t *adxl){
	return adxl->fifo_ctl_reg;
}

/**
  * @brief  Write to FIFO_CTL register
  * @param  adxl - instance [adxl345_t]
  * @param  value - new register value
  * @retval none
  */
void adxl345_fifo_ctl_write(adxl345_t *adxl, uint8_t value){
	adxl->fifo_ctl_reg = value;
	adxl->driver.write(adxl->i2c_address, FIFO_CTL, &value, 1);
}

/**
  * @brief  Set bits in FIFO_CTL register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask
  * @retval none
  */
void adxl345_fifo_ctl_set(adxl345_t *adxl, uint8_t mask){
	adxl->fifo_ctl_reg |= mask;
	adxl->driver.write(adxl->i2c_address, FIFO_CTL, &adxl->fifo_ctl_reg, 1);
}

/**
  * @brief  Clear bits in FIFO_CTL register
  * @param  adxl - instance [adxl345_t]
  * @param  mask - bit mask
  * @retval none
  */
void adxl345_fifo_ctl_clear(adxl345_t *adxl, uint8_t mask){
	adxl->fifo_ctl_reg &= ~mask;
	adxl->driver.write(adxl->i2c_address, FIFO_CTL, &adxl->fifo_ctl_reg, 1);
}

/**
  * @brief  Set FIFO mode in FIFO_CTL register
  * @param  adxl - instance [adxl345_t]
  * @param  mode - FIFO mode (see adxl345_int_map_flags_t)
  * @retval none
  */
void adxl345_fifo_mode_set(adxl345_t *adxl, adxl345_fifo_mode_t mode){
	adxl->fifo_ctl_reg &= ~ADXL_FIFO_CTL_MODE_MASK;
	adxl->fifo_ctl_reg |= mode;
	adxl->driver.write(adxl->i2c_address, FIFO_CTL, &adxl->fifo_ctl_reg, 1);
}

/**
  * @brief  Set FIFO samples count in FIFO_CTL register
  * @param  adxl - instance [adxl345_t]
  * @param  samples - samples count in range 0 ... 31
  * @retval none
  */
void adxl345_fifo_samlpes_set(adxl345_t *adxl, uint8_t samples){
	if(samples > 31){
		samples = 31;
	}

	adxl->fifo_ctl_reg &= ~ADXL_FIFO_CTL_SAMPLES_MASK;
	adxl->fifo_ctl_reg |= samples;
	adxl->driver.write(adxl->i2c_address, FIFO_CTL, &adxl->fifo_ctl_reg, 1);
}

/**
  * @brief  Read FIFO_STATUS register value
  * @param  adxl - instance [adxl345_t]
  * @retval register value (see adxl345_int_flags_t)
  */
uint8_t adxl345_fifo_status_read(adxl345_t *adxl){
	uint8_t tmp = 0;
	adxl->driver.read(adxl->i2c_address, FIFO_STATUS, &tmp, 1);
	return tmp;
}

