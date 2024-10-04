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

int16_t adxl345_readx(adxl345_t *adxl)
{
//	int16_t x;
//	adxl_read_values (0x32);
//	x = ((data_rec[1]<<8)|data_rec[0]);
//	return x;
}

int16_t adxl345_ready(adxl345_t *adxl)
{
//	int16_t y;
//	adxl_read_values (0x32);
//	y = ((data_rec[3]<<8)|data_rec[2]);
//	return y;
}

int16_t adxl345_readz(adxl345_t *adxl)
{
//	int16_t z;
//	adxl_read_values (0x32);
//	z = ((data_rec[5]<<8)|data_rec[4]);
//	return z;
}
