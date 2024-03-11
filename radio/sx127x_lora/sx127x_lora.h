#ifndef RADIO_SX127X_LORA_SX127X_LORA_H_
#define RADIO_SX127X_LORA_SX127X_LORA_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "sx127x_lora_registers.h"

/*!
 * XTAL Frequency
 */
#define XTAL_FREQ_HZ								(32000000)

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Status defines
 */
#define LORA_OK										0
#define LORA_TIMEOUT								1
#define LORA_NOT_FOUND								2
#define LORA_LARGE_PAYLOAD							3
#define LORA_UNAVAILABLE							4

/*!
 * Timings
 */
#define LORA_TX_READY_POLLING_PERIOD_MS				(2)

typedef struct{
	void (*init)(void);
	void (*reset)(bool set);
	void (*read)(uint8_t reg, uint8_t *buffer, uint8_t length);
	void (*write)(uint8_t reg, uint8_t *buffer, uint8_t length);
	void (*delay_ms)(uint32_t ms);
} sx127x_driver_t;

typedef struct{
	/* Driver */
	sx127x_driver_t	driver;

	/* Parameters */
	int 			frequency;					/* Set in Hz */
	uint8_t			spreding_factor;			/* See in registers list, default: RFLR_MODEMCONFIG2_SF_7 */
	uint8_t			bandwidth;					/* See in registers list, default: RFLR_MODEMCONFIG1_BW_125_KHZ */
	uint8_t			coding_rate;				/* See in registers list, default: RFLR_MODEMCONFIG1_CODINGRATE_4_6 */
	uint16_t		preamble_length;			/* Set preamble length, bytes */
	uint8_t 		rx_gain;					/* Set RFLR_LNA_GAIN_G1 ... RFLR_LNA_GAIN_G6. Set 0 for auto gain control (AGC)*/
	int8_t			power;						/* Set in range -1 ... 20dB */
	uint8_t 		power_output;				/* Set TX output: RFLR_PACONFIG_PASELECT_PABOOST or RFLR_PACONFIG_PASELECT_RFO. Depend on your HW design.*/
	uint8_t			over_current_protection;	/* Set in mA in range 45 ... 240mA, OCP limit at TX. Set 0 for disable OCP */
	uint8_t 		syncword;					/* Set syncword, default - 0x12. Value 0x34 reserved for LoRa WAN networks */
	bool			crc;						/* Enable or disable auto CRC calculation & check feature */

	/* States */
	uint8_t 		op_mode;

	/* Statistic */
	int8_t 			temperature;				/* Last packet measured module temperature */
	int16_t 		rssi;						/* Last packet RSSI */
	int16_t 		snr;						/* Last packet SNR */
	int32_t 		freq_error;					/* Frequency error */
	uint32_t 		crc_errors;					/* CRC errors counter */
} sx127x_lora_t;

uint16_t sx127x_lora_init(sx127x_lora_t* _LoRa);
void sx127x_lora_start_rx(sx127x_lora_t* sx);
uint8_t sx127x_lora_available(sx127x_lora_t* sx);
uint8_t sx127x_lora_receive(sx127x_lora_t* sx, uint8_t* data, uint8_t length);
uint8_t sx127x_lora_transmit(sx127x_lora_t* sx, uint8_t* data, uint8_t length, uint16_t timeout);

/* General SX127x functions */
void sx127x_lora_set_op_mode(sx127x_lora_t* sx, uint8_t mode);
void sx127x_lora_set_rf_frequency(sx127x_lora_t* sx, uint32_t freq);
void sx127x_lora_set_pa_output(sx127x_lora_t* sx, int8_t output);
void sx127x_lora_set_rf_power(sx127x_lora_t* sx, int8_t power);
void sx127x_lora_set_rx_auto_gain_control(sx127x_lora_t* sx, bool set);
void sx127x_lora_set_rx_gain(sx127x_lora_t* sx, uint8_t gain);
void sx127x_lora_set_bandwidth(sx127x_lora_t* sx, uint8_t bandwidth);
void sx127x_lora_set_coding_rate(sx127x_lora_t* sx, uint8_t coding_rate);
void sx127x_lora_set_ocp(sx127x_lora_t* sx, uint8_t current);
void sx127x_lora_set_sync_word(sx127x_lora_t* sx, uint8_t syncword);
void sx127x_lora_set_symbol_timeout(sx127x_lora_t* sx, uint16_t timeout);
void sx127x_lora_set_crc(sx127x_lora_t* sx, bool set);
void sx127x_lora_set_preamble_length(sx127x_lora_t* sx, uint16_t preamble_length);
void sx127x_lora_set_low_data_rate_optimization(sx127x_lora_t* sx, bool set);
void sx127x_lora_set_spreading_factor(sx127x_lora_t* sx, uint8_t spreading_factor);
void sx127x_lora_auto_low_data_rate_optimization(sx127x_lora_t* sx);
int16_t sx127x_lora_get_packet_snr(sx127x_lora_t* sx);
int16_t sx127x_lora_get_rssi(sx127x_lora_t* sx);
int16_t sx127x_lora_get_packet_rssi(sx127x_lora_t* sx);
int32_t sx127x_lora_get_frequency_error(sx127x_lora_t* sx);
int8_t sx127x_lora_get_temperature(sx127x_lora_t* sx);

#endif /* RADIO_SX127X_LORA_SX127X_LORA_H_ */
