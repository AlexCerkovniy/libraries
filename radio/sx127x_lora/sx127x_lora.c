#include "sx127x_lora.h"

static const uint32_t bw_table_hz[] = {7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};

/*!
 * \brief Initializes the SX1276
 *
 * \param [IN] sx - module object
 */
uint16_t sx127x_lora_init(sx127x_lora_t* sx){
	uint8_t    data;
	uint8_t    read;

	/* Initialize IO */
	sx->driver.init();

	/* Reset chip */
	sx->driver.reset(0);
	sx->driver.delay_ms(3);
	sx->driver.reset(1);
	sx->driver.delay_ms(15);

	/* Read chip version ID */
	sx->driver.read(REG_LR_VERSION, &read, 1);
	if(read != 0x12){
		return LORA_UNAVAILABLE;
	}

	/* Initialize data */
	sx->op_mode = RFLR_OPMODE_SLEEP;
	sx->crc_errors = 0;
	sx->rssi = -255;
	sx->snr = -255;
	sx->freq_error = 0;

	/* Configure chip to LoRa mode */
	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_SLEEP);
	sx->driver.delay_ms(2);
	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_LONGRANGEMODE_ON);
	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_STANDBY);

	/* Set frequency */
	sx127x_lora_set_rf_frequency(sx, sx->frequency);

	/* Set TX power & OCP */
	sx127x_lora_set_pa_output(sx, sx->power_output);
	sx127x_lora_set_rf_power(sx, sx->power);
	if(sx->over_current_protection){
		sx127x_lora_set_ocp(sx, sx->over_current_protection);
	}

	/* Set RX gain */
	if(sx->rx_gain){
		sx127x_lora_set_rx_auto_gain_control(sx, false);
		sx127x_lora_set_rx_gain(sx, sx->rx_gain);
	}
	else{
		sx127x_lora_set_rx_auto_gain_control(sx, true);
	}

	/* Set bandwidth & coding rate */
	sx127x_lora_set_bandwidth(sx, sx->bandwidth);
	sx127x_lora_set_coding_rate(sx, sx->coding_rate);

	/* Set CRC feature */
	sx127x_lora_set_crc(sx, sx->crc);

	/* Set spreading factor (auto set low data rate optimization) */
	sx127x_lora_set_spreading_factor(sx, sx->spreding_factor);

	/* Set symbol timeout */
	sx127x_lora_set_symbol_timeout(sx, 0x3FF);

	/* Set preamble length */
	sx127x_lora_set_preamble_length(sx, sx->preamble_length);

	/* Set syncword */
	sx127x_lora_set_sync_word(sx, sx->syncword);

	/* Configure DIO mapping: --> DIO1: RxDone, other DIO's disabled */
	data = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_11 | RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11;
	sx->driver.write(REG_LR_DIOMAPPING1 | 0x80, &data, 1);
	data = RFLR_DIOMAPPING2_DIO4_11 | RFLR_DIOMAPPING2_DIO5_11;
	sx->driver.write(REG_LR_DIOMAPPING2 | 0x80, &data, 1);

	/* Read temperature */
	//sx127x_lora_get_temperature(sx);

	/* Go to standby */
	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_STANDBY);

	return LORA_OK;
}

/*!
 * \brief Transmit data
 *
 * \param [IN] sx - module object
 * \param [IN] data	- a pointer to the data you wanna send
 * \param [IN] length - size of your data in bytes
 * \param [IN] timeout - timeout in milliseconds
 *
 * \return: 1 in case of success, 0 in case of timeout
 */
uint8_t sx127x_lora_transmit(sx127x_lora_t* sx, uint8_t* data, uint8_t length, uint16_t timeout){
	uint8_t tmp;
	uint8_t mode = sx->op_mode;

	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_STANDBY);

	/* Set hopping period to 0 */
	tmp = 0;
	sx->driver.write(REG_LR_HOPPERIOD | 0x80, &tmp, 1);

	/* Initializes the payload size */
	sx->driver.write(REG_LR_PAYLOADLENGTH | 0x80, &length, 1);

	/* Set full TX FIFO buffer */
	tmp = 0;
	sx->driver.write(REG_LR_FIFOTXBASEADDR | 0x80, &tmp, 1);
	sx->driver.write(REG_LR_FIFOADDRPTR | 0x80, &tmp, 1);

	/* Write TX buffer data in FIFO */
	sx->driver.write(REG_LR_FIFO | 0x80, data, length);

	/* Start TX */
	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_TRANSMITTER);

	while(1){
		sx->driver.delay_ms(2);
		sx->driver.read(REG_LR_IRQFLAGS, &tmp, 1);

		if(tmp & RFLR_IRQFLAGS_TXDONE){
			tmp = RFLR_IRQFLAGS_TXDONE;
			sx->driver.write(REG_LR_IRQFLAGS | 0x80, &tmp, 1);
			sx127x_lora_set_op_mode(sx, mode);
			return LORA_OK;
		}
		else if(tmp & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL){
			tmp = RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL;
			sx->driver.write(REG_LR_IRQFLAGS | 0x80, &tmp, 1);
		}

		/* Check timeout */
		if(timeout > LORA_TX_READY_POLLING_PERIOD_MS){
			timeout -= LORA_TX_READY_POLLING_PERIOD_MS;
		}
		else{
			sx127x_lora_set_op_mode(sx, mode);
			return LORA_TIMEOUT;
		}
	}
}

/*!
 * \brief Start RX. For checking available RX data, call sx127x_lora_available()
 *
 * \param [IN] sx - module object
 */
void sx127x_lora_start_rx(sx127x_lora_t* sx){
	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_RECEIVER);
}

/*!
 * \brief Start RX. For checking available RX data, call sx127x_lora_available()
 *
 * \param [IN] sx - module object
 * \return [IN] available RX data bytes count
 */
uint8_t sx127x_lora_available(sx127x_lora_t* sx){
	uint8_t tmp;

	sx->driver.read(REG_LR_IRQFLAGS, &tmp, 1);
	if((tmp & RFLR_IRQFLAGS_PAYLOADCRCERROR) & sx->crc){
		tmp = RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_RXDONE;
		sx->driver.write(REG_LR_IRQFLAGS | 0x80, &tmp, 1);
		sx->crc_errors++;
	}
	else if(tmp & RFLR_IRQFLAGS_RXDONE){
		sx->driver.read(REG_LR_NBRXBYTES, &tmp, 1);
		return tmp;
	}

	return 0;
}

/*!
 * \brief Start RX. For checking available RX data, call sx127x_lora_available()
 *
 * \param [IN] sx - module object
 * \return [IN] available RX data bytes count
 */
uint8_t sx127x_lora_receive(sx127x_lora_t* sx, uint8_t* data, uint8_t length){
	uint8_t tmp;

	sx->driver.read(REG_LR_IRQFLAGS, &tmp, 1);
	if(tmp & RFLR_IRQFLAGS_RXDONE){
		tmp = RFLR_IRQFLAGS_RXDONE;
		sx->driver.write(REG_LR_IRQFLAGS | 0x80, &tmp, 1);

		/* Read packet SNR & RSSI */
		sx127x_lora_get_packet_snr(sx);
		sx127x_lora_get_packet_rssi(sx);
		sx127x_lora_get_frequency_error(sx);

		/* Get number of actual received bytes */
		sx->driver.read(REG_LR_NBRXBYTES, &tmp, 1);
		if(length > tmp){
			length = tmp;
		}

		/* Read current RX FIFO address */
		sx->driver.read(REG_LR_FIFORXCURRENTADDR, &tmp, 1);
		sx->driver.write(REG_LR_FIFOADDRPTR | 0x80, &tmp, 1);

		/* Read data */
		sx->driver.read(REG_LR_FIFO, data, length);
	}
	else{
		length = 0;
	}

    return length;
}

/*!
 * \brief Sets the SX127x operating mode
 *
 * \param [IN] sx - module object
 * \param [IN] mode - new operating mode
 */
void sx127x_lora_set_op_mode(sx127x_lora_t* sx, uint8_t mode){
	sx->op_mode = (sx->op_mode & RFLR_OPMODE_MASK) | mode;
	sx->driver.write(REG_LR_OPMODE | 0x80, &sx->op_mode, 1);
}

/*!
 * \brief Writes the new RF frequency value
 *
 * \param [IN] sx - module object
 * \param [IN] freq New RF frequency value in [Hz]
 */
void sx127x_lora_set_rf_frequency(sx127x_lora_t* sx, uint32_t freq){
	/* Convert frequency in Hz to synthesizer value: FREQ_STEP is 61.03515625Hz */
	uint64_t synthesizer = ((uint64_t)freq * 100000)/6103515;

	/* Write to chip */
	uint8_t tmp[3];
	tmp[0] = (uint8_t)((synthesizer >> 16) & 0xFF);
	tmp[1] = (uint8_t)((synthesizer >> 8) & 0xFF);
	tmp[2] = (uint8_t)(synthesizer & 0xFF);
	sx->driver.write(REG_LR_FRFMSB | 0x80, tmp, 3);

	/* Save new RF frequency */
	sx->frequency = freq;
}

/*!
 * \brief Writes the new RF frequency value
 *
 * \param [IN] sx - module object
 * \param [IN] bandwidth - one of next values (not all shown, see register list):
 * 				- RFLR_MODEMCONFIG1_BW_7_81_KHZ
 * 				- ...
 * 				- RFLR_MODEMCONFIG1_BW_500_KHZ
 */
void sx127x_lora_set_bandwidth(sx127x_lora_t* sx, uint8_t bandwidth){
	uint8_t data;
	sx->driver.read(REG_LR_MODEMCONFIG1, &data, 1);
	data = (data & RFLR_MODEMCONFIG1_BW_MASK) | (bandwidth & (~RFLR_MODEMCONFIG1_BW_MASK));
	sx->driver.write(REG_LR_MODEMCONFIG1 | 0x80, &data, 1);
	sx->bandwidth = bandwidth;
}

/*!
 * \brief Set coding rate
 *
 * \param [IN] sx - module object
 * \param [IN] coding_rate - one of next values:
 * 				- RFLR_MODEMCONFIG1_CODINGRATE_4_5
 * 				- RFLR_MODEMCONFIG1_CODINGRATE_4_6
 * 				- RFLR_MODEMCONFIG1_CODINGRATE_4_7
 * 				- RFLR_MODEMCONFIG1_CODINGRATE_4_8
 */
void sx127x_lora_set_coding_rate(sx127x_lora_t* sx, uint8_t coding_rate){
	uint8_t data;
	sx->driver.read(REG_LR_MODEMCONFIG1, &data, 1);
	data = (data & RFLR_MODEMCONFIG1_CODINGRATE_MASK) | (coding_rate & (~RFLR_MODEMCONFIG1_CODINGRATE_MASK));
	sx->driver.write(REG_LR_MODEMCONFIG1 | 0x80, &data, 1);
	sx->coding_rate = coding_rate;
}

/*!
 * \brief Set PA output pin configuration
 *
 * \param [IN] sx - module object
 * \param [IN] output - RFLR_PACONFIG_PASELECT_RFO or RFLR_PACONFIG_PASELECT_PABOOST
 */
void sx127x_lora_set_pa_output(sx127x_lora_t* sx, int8_t output){
	uint8_t data;
	sx->driver.read(REG_LR_PACONFIG, &data, 1);
	data = (data & RFLR_PACONFIG_PASELECT_MASK) | (output & (~RFLR_PACONFIG_PASELECT_MASK));
	sx->driver.write(REG_LR_PACONFIG | 0x80, &data, 1);
}

/*!
 * \brief Writes the new RF power value
 *
 * \param [IN] sx - module object
 * \param [IN] freq New RF power value:
 */
void sx127x_lora_set_rf_power(sx127x_lora_t* sx, int8_t power){
	uint8_t pa_config, pa_dac;

	/* Read REG_LR_PACONFIG & REG_LR_PADAC values from chip */
	sx->driver.read(REG_LR_PACONFIG, &pa_config, 1);

	/* Set max power limit to the max */
	pa_config |= 0x70;

	/* Set-up power according to policies */
	if((pa_config & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST){
		sx->driver.read(REG_LR_PADAC, &pa_dac, 1);

		/* +20dBm PA control */
		if(power > 17){
			pa_dac = 0x87; /* On */
			sx->driver.write(REG_LR_PADAC, &pa_dac, 1);
		}
		else if((pa_dac & 0x87) == 0x87){
			pa_dac = 0x84; /* Off */
			sx->driver.write(REG_LR_PADAC, &pa_dac, 1);
		}

		if((pa_dac & 0x87) == 0x87){
			if(power < 5){
				power = 5;
			}
			else if(power > 20){
				power = 20;
			}

			/* Set output power */
			pa_config = (pa_config & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);
		}
		else {
			if(power < 2){
				power = 2;
			}
			if(power > 17){
				power = 17;
			}

			/* Set output power */
			pa_config = (pa_config & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
		}
	}
	else {
		if(power < -1){
			power = -1;
		}
		if(power > 14){
			power = 14;
		}

		/* Set output power */
		pa_config = ( pa_config & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | (uint8_t)((uint16_t)(power + 1) & 0x0F);
	}

	sx->driver.write(REG_LR_PACONFIG | 0x80, &pa_config, 1);
	sx->power = power;
}

/*!
 * \brief Set AGC for receiver
 *
 * \param [IN] sx - module object
 * \param [IN] set - enable (true) or disable (false)
 */
void sx127x_lora_set_rx_auto_gain_control(sx127x_lora_t* sx, bool set){
	uint8_t data;
	sx->driver.read(REG_LR_MODEMCONFIG3, &data, 1);
	if(set && (data & RFLR_MODEMCONFIG3_AGCAUTO_ON)){
		/* Already set */
		return;
	}
	else if(set || (data & RFLR_MODEMCONFIG3_AGCAUTO_ON)){
		data &= RFLR_MODEMCONFIG3_AGCAUTO_MASK;
		if(set){
			data |= RFLR_MODEMCONFIG3_AGCAUTO_ON;
		}
		sx->driver.write(REG_LR_MODEMCONFIG3 | 0x80, &data, 1);
	}
}

/*!
 * \brief Set RX gain
 *
 * \param [IN] sx - module object
 * \param [IN] gain - one of next values (see all values in registers list):
 * 				- RFLR_LNA_GAIN_G1
 * 				- ...
 * 				- RFLR_LNA_GAIN_G6
 */
void sx127x_lora_set_rx_gain(sx127x_lora_t* sx, uint8_t gain){
	uint8_t data;
	sx->driver.read(REG_LR_LNA, &data, 1);
	data = (data & RFLR_LNA_GAIN_MASK) | (gain & (~RFLR_LNA_GAIN_MASK));
	sx->driver.write(REG_LR_LNA | 0x80, &data, 1);
}

/*!
 * \brief Set chip current protection limit
 *
 * \param [IN] sx - module object
 * \param [IN] current - desired OCP value in mA
 */
void sx127x_lora_set_ocp(sx127x_lora_t* sx, uint8_t current){
	uint8_t	ocp_trim;

	if(current < 45)
		current = 45;
	if(current > 240)
		current = 240;

	if(current <= 120)
		ocp_trim = (current - 45)/5;
	else if(current <= 240)
		ocp_trim = (current + 30)/10;

	/* Enable OCP */
	ocp_trim |= RFLR_OCP_ON;

	sx->driver.write(REG_LR_OCP | 0x80, &ocp_trim, 1);
}

/*!
 * \brief Set the LowDataRateOptimization flag. Is is mandated for when the symbol length exceeds 16ms.
 *
 * \param [IN] sx - module object
 * \param [IN] set - state flag (false == off, otherwise to enable)
 */
void sx127x_lora_set_low_data_rate_optimization(sx127x_lora_t* sx, bool set){
	uint8_t	data;

	sx->driver.read(REG_LR_MODEMCONFIG3, &data, 1);

	if(set && (data & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON)){
		/* Already enabled */
		return;
	}
	else if(set || (data & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON)){
		data &= RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK;
		if(set){
			data |= RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON;
		}

		sx->driver.write(REG_LR_MODEMCONFIG3 | 0x80, &data, 1);
	}
}

/*!
 * \brief Set spreading factor.
 *
 * \param [IN] sx - module object
 * \param [IN] spreading_factor - see registers list.
 */
void sx127x_lora_set_spreading_factor(sx127x_lora_t* sx, uint8_t spreading_factor){
	uint8_t	data;

	sx->driver.read(REG_LR_MODEMCONFIG2, &data, 1);
	data &= RFLR_MODEMCONFIG2_SF_MASK;
	data |= spreading_factor;
	sx->driver.write(REG_LR_MODEMCONFIG2 | 0x80, &data, 1);

	/* Update spreading factor parameter */
	sx->spreding_factor = spreading_factor;

	/* Update low data rate optimization option state */
	sx127x_lora_auto_low_data_rate_optimization(sx);
}

/*!
 * \brief Set the LowDataRateOptimization flag automatically based on the symbol length.
 *
 * \param [IN] sx - module object
 */
void sx127x_lora_auto_low_data_rate_optimization(sx127x_lora_t* sx){
	uint32_t symbol_duration = ((1 << (sx->spreding_factor >> 4)) * 1000) / bw_table_hz[sx->bandwidth >> 4];
	sx127x_lora_set_low_data_rate_optimization(sx, (symbol_duration > 16)?(true):(false));
}

/*!
 * \brief Set sync word.
 *
 * \param [IN] sx - module object
 * \param [IN] syncword
 */
void sx127x_lora_set_sync_word(sx127x_lora_t* sx, uint8_t syncword){
	sx->driver.write(REG_LR_SYNCWORD | 0x80, &syncword, 1);
	sx->syncword = syncword;
}

/*!
 * \brief Set symbol timeout.
 *
 * \param [IN] sx - module object
 * \param [IN] timeout - maximum timeout is 0x3FF
 */
void sx127x_lora_set_symbol_timeout(sx127x_lora_t* sx, uint16_t timeout){
	if(timeout > 0x3FF){
		timeout = 0x3FF;
	}

	/* Load timeout LSB to REG_LR_SYMBTIMEOUTLSB */
	uint8_t data = timeout & 0xFF;
	sx->driver.write(REG_LR_SYMBTIMEOUTLSB | 0x80, &data, 1);

	/* Load timeout MSB to REG_LR_MODEMCONFIG2 */
	timeout = timeout >> 8;
	sx->driver.read(REG_LR_MODEMCONFIG2, &data, 1);
	data = (data & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) | ((uint8_t)timeout);
	sx->driver.write(REG_LR_MODEMCONFIG2 | 0x80, &data, 1);
}

/*!
 * \brief Set CRC calculating feature.
 *
 * \param [IN] sx - module object
 * \param [IN] set - state flag (false == off, otherwise to enable)
 */
void sx127x_lora_set_crc(sx127x_lora_t* sx, bool set){
	uint8_t data;
	sx->driver.read(REG_LR_MODEMCONFIG2, &data, 1);
	data &= RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK;
	if(set){
		data |= RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON;
	}
	sx->driver.write(REG_LR_MODEMCONFIG2 | 0x80, &data, 1);

	/* Update CRC feature state */
	sx->crc = set;
}

/*!
 * \brief Set preamble length (in bytes).
 *
 * \param [IN] sx - module object
 * \param [IN] length - preamble bytes count
 */
void sx127x_lora_set_preamble_length(sx127x_lora_t* sx, uint16_t preamble_length){
	uint8_t data = sx->preamble_length >> 8;
	sx->driver.write(REG_LR_PREAMBLEMSB | 0x80, &data, 1);
	data = sx->preamble_length & 0xFF;
	sx->driver.write(REG_LR_PREAMBLELSB | 0x80, &data, 1);
	sx->preamble_length = preamble_length;
}

/*!
 * \brief Get last packet SNR estimate
 *
 * \param [IN] sx - module object
 * \return [IN] last received packet SNR estimate
 */
int16_t sx127x_lora_get_packet_snr(sx127x_lora_t* sx){
	int16_t snr;
	uint8_t data;

	sx->driver.read(REG_LR_PKTSNRVALUE, &data, 1);
	if(data & 0x80){
		snr = ((~data + 1) & 0xFF) >> 2;
		snr = -snr;
	}
	else {
		snr = (data & 0xFF) >> 2;
	}

	sx->snr = snr;
	return snr;
}

/*!
 * \brief Get current RSSI
 *
 * \param [IN] sx - module object
 * \return [OUT] current RSSI value
 */
int16_t sx127x_lora_get_rssi(sx127x_lora_t* sx){
	uint8_t data;

	/* Get RSSI offset value, dependent on RF frequency */
	int16_t	rssi_offset = (sx->frequency < 860000000)?(RSSI_OFFSET_LF):(RSSI_OFFSET_HF);
	sx->driver.read(REG_LR_RSSIVALUE, &data, 1);

	return rssi_offset + data;
}

/*!
 * \brief Get last packet RSSI
 *
 * \param [IN] sx - module object
 * \return [OUT] last received packet RSSI
 */
int16_t sx127x_lora_get_packet_rssi(sx127x_lora_t* sx){
	uint8_t data;
	int16_t rssi, rssi_offset;
	uint32_t tmp;

	sx->driver.read(REG_LR_PKTRSSIVALUE, &data, 1);

	/* Get RSSI offset value, dependent on RF frequency */
	rssi_offset = (sx->frequency < 860000000)?(RSSI_OFFSET_LF):(RSSI_OFFSET_HF);

	if(sx->snr < 0){
		rssi = rssi_offset + data + sx->snr;
	}
	else{
		tmp = ((uint32_t)data * 10666)/10000;	/* data * 1.0666 */
		rssi = rssi_offset + tmp;
	}

	sx->rssi = rssi;
	return rssi;
}

/*!
 * \brief Get frequency error
 *
 * \param [IN] sx - module object
 * \return [OUT] frequency error in Hz
 */
int32_t sx127x_lora_get_frequency_error(sx127x_lora_t* sx){
	uint8_t data[3];
	int32_t freq_error;

	/* Read frequency error registers */
	sx->driver.read(REG_LR_FEIMSB, data, 3);
	freq_error = (((int32_t)(data[0] & 0x07)) << 16) | (((int32_t)data[1]) << 8) | (int32_t)data[2];

	/* Apply sign bit */
	if(data[0] & 0x08){
		freq_error -= 524288;
	}

	sx->freq_error = (int32_t)((((float)freq_error * (1 << 24))/XTAL_FREQ_HZ) * ((float)bw_table_hz[sx->bandwidth >> 4] / 500000.0));
	return sx->freq_error;
}

/*!
 * \brief Get module temperature in C
 *
 * \param [IN] sx - module object
 * \return [OUT] temperature in celsius
 */
int8_t sx127x_lora_get_temperature(sx127x_lora_t* sx){
	uint8_t data = 0;

	uint8_t saved_opmode = sx->op_mode;
	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_ACCESSSHAREDREG_ENABLE);
	if((sx->op_mode & RFLR_OPMODE_MASK) <= RFLR_OPMODE_STANDBY){
		sx127x_lora_set_op_mode(sx, RFLR_OPMODE_SYNTHESIZER_RX);
	}

	sx->driver.read(REG_LR_IMAGECAL, &data, 1);
	data = (data & RFLR_IMAGECAL_TEMP_MONITORING_MASK) | RFLR_IMAGECAL_TEMP_MONITORING_ON;
	sx->driver.write(REG_LR_IMAGECAL | 0x80, &data, 1);
	sx->driver.delay_ms(2);
	data = (data & RFLR_IMAGECAL_TEMP_MONITORING_MASK) | RFLR_IMAGECAL_TEMP_MONITORING_OFF;
	sx->driver.write(REG_LR_IMAGECAL | 0x80, &data, 1);

	sx127x_lora_set_op_mode(sx, RFLR_OPMODE_SLEEP);

	sx->driver.read(REG_LR_TEMP, &data, 1);

	if(data & 0x80){
		sx->temperature = (~data + 1); //_temp = ( ( ~_temp + 1 ) & 0xFF );
	}
	else{
		sx->temperature = data;
	}

	sx127x_lora_set_op_mode(sx, saved_opmode);

	return sx->temperature;
}
