/*
 * sx1276_lora_misc.h
 *
 *  Created on: Feb 21, 2024
 *      Author: Alex
 */

#ifndef RADIO_SX1276_LORA_SX1276_LORA_MISC_H_
#define RADIO_SX1276_LORA_SX1276_LORA_MISC_H_

#include <stdint.h>
#include <stdbool.h>

/*!
 * \brief Writes the new RF frequency value
 *
 * \param [IN] freq New RF frequency value in [Hz]
 */
void sx1276_lora_set_rf_frequency( uint32_t freq );

/*!
 * \brief Reads the current RF frequency value
 *
 * \retval freq Current RF frequency value in [Hz]
 */
uint32_t sx1276_lora_get_rf_frequency( void );

/*!
 * \brief Writes the new RF output power value
 *
 * \param [IN] power New output power value in [dBm]
 */
void sx1276_lora_set_rf_power( int8_t power );

/*!
 * \brief Reads the current RF output power value
 *
 * \retval power Current output power value in [dBm]
 */
int8_t sx1276_lora_get_rf_power( void );

/*!
 * \brief Writes the new Signal Bandwidth value
 *
 * \remark This function sets the IF frequency according to the datasheet
 *
 * \param [IN] factor New Signal Bandwidth value [0: 125 kHz, 1: 250 kHz, 2: 500 kHz]
 */
void sx1276_lora_set_signal_bandwidth( uint8_t bw );

/*!
 * \brief Reads the current Signal Bandwidth value
 *
 * \retval factor Current Signal Bandwidth value [0: 125 kHz, 1: 250 kHz, 2: 500 kHz]
 */
uint8_t sx1276_lora_get_signal_bandwidth( void );

/*!
 * \brief Writes the new Spreading Factor value
 *
 * \param [IN] factor New Spreading Factor value [7, 8, 9, 10, 11, 12]
 */
void sx1276_lora_set_spreading_factor( uint8_t factor );

/*!
 * \brief Reads the current Spreading Factor value
 *
 * \retval factor Current Spreading Factor value [7, 8, 9, 10, 11, 12]
 */
uint8_t sx1276_lora_get_spreading_factor( void );

/*!
 * \brief Writes the new Error Coding value
 *
 * \param [IN] value New Error Coding value [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 */
void sx1276_lora_set_error_coding( uint8_t value );

/*!
 * \brief Reads the current Error Coding value
 *
 * \retval value Current Error Coding value [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 */
uint8_t sx1276_lora_get_error_coding( void );

/*!
 * \brief Enables/Disables the packet CRC generation
 *
 * \param [IN] enaable [true, false]
 */
void sx1276_lora_set_packet_crc_on( bool enable );

/*!
 * \brief Reads the current packet CRC generation status
 *
 * \retval enable [true, false]
 */
bool sx1276_lora_get_packet_crc_on( void );

/*!
 * \brief Enables/Disables the Implicit Header mode in LoRa
 *
 * \param [IN] enable [true, false]
 */
void sx1276_lora_set_implict_header_on( bool enable );

/*!
 * \brief Check if implicit header mode in LoRa in enabled or disabled
 *
 * \retval enable [true, false]
 */
bool sx1276_lora_get_implict_header_on( void );

/*!
 * \brief Enables/Disables Rx single instead of Rx continuous
 *
 * \param [IN] enable [true, false]
 */
void sx1276_lora_set_rx_single_on( bool enable );

/*!
 * \brief Check if LoRa is in Rx Single mode
 *
 * \retval enable [true, false]
 */
bool sx1276_lora_get_rx_single_on( void );

/*!
 * \brief Enables/Disables the frequency hopping
 *
 * \param [IN] enable [true, false]
 */

void sx1276_lora_set_freq_hop_on( bool enable );

/*!
 * \brief Get the frequency hopping status
 *
 * \param [IN] enable [true, false]
 */
bool sx1276_lora_get_freq_hop_on( void );

/*!
 * \brief Set symbol period between frequency hops
 *
 * \param [IN] value
 */
void sx1276_lora_set_hop_period( uint8_t value );

/*!
 * \brief Get symbol period between frequency hops
 *
 * \retval value symbol period between frequency hops
 */
uint8_t sx1276_lora_get_hop_period( void );

/*!
 * \brief Set timeout Tx packet (based on MCU timer, timeout between Tx Mode entry Tx Done IRQ)
 *
 * \param [IN] value timeout (ms)
 */
void sx1276_lora_set_tx_packet_timeout( uint32_t value );

/*!
 * \brief Get timeout between Tx packet (based on MCU timer, timeout between Tx Mode entry Tx Done IRQ)
 *
 * \retval value timeout (ms)
 */
uint32_t sx1276_lora_get_tx_packet_timeout( void );

/*!
 * \brief Set timeout Rx packet (based on MCU timer, timeout between Rx Mode entry and Rx Done IRQ)
 *
 * \param [IN] value timeout (ms)
 */
void sx1276_lora_set_rx_packet_timeout( uint32_t value );

/*!
 * \brief Get timeout Rx packet (based on MCU timer, timeout between Rx Mode entry and Rx Done IRQ)
 *
 * \retval value timeout (ms)
 */
uint32_t sx1276_lora_get_rx_packet_timeout( void );

/*!
 * \brief Set payload length
 *
 * \param [IN] value payload length
 */
void sx1276_lora_set_payload_length( uint8_t value );

/*!
 * \brief Get payload length
 *
 * \retval value payload length
 */
uint8_t sx1276_lora_get_payload_length( void );

/*!
 * \brief Enables/Disables the 20 dBm PA
 *
 * \param [IN] enable [true, false]
 */
void sx1276_lora_set_pa_20dbm( bool enale );

/*!
 * \brief Gets the current 20 dBm PA status
 *
 * \retval enable [true, false]
 */
bool sx1276_lora_get_pa_20dbm( void );

/*!
 * \brief Set the RF Output pin
 *
 * \param [IN] RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 */
void sx1276_lora_set_pa_output( uint8_t outputPin );

/*!
 * \brief Gets the used RF Ouptut pin
 *
 * \retval RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 */
uint8_t sx1276_lora_get_pa_output( void );

/*!
 * \brief Writes the new PA rise/fall time of ramp up/down value
 *
 * \param [IN] value New PaRamp value
 */
void sx1276_lora_set_pa_ramp( uint8_t value );

/*!
 * \brief Reads the current PA rise/fall time of ramp up/down value
 *
 * \retval freq Current PaRamp value
 */
uint8_t sx1276_lora_get_pa_ramp( void );

/*!
 * \brief Set Symbol Timeout based on symbol length
 *
 * \param [IN] value number of symbol
 */
void sx1276_lora_set_symbol_timeout( uint16_t value );

/*!
 * \brief  Get Symbol Timeout based on symbol length
 *
 * \retval value number of symbol
 */
uint16_t sx1276_lora_get_symbol_timeout( void );

/*!
 * \brief  Configure the device to optimize low datarate transfers
 *
 * \param [IN] enable Enables/Disables the low datarate optimization
 */
void sx1276_lora_set_low_datarate_optimize( bool enable );

/*!
 * \brief  Get the status of optimize low datarate transfers
 *
 * \retval LowDatarateOptimize enable or disable
 */
bool sx1276_lora_get_low_datarate_optimize( void );

/*!
 * \brief Set the preamble length
 *
 * \param [IN] value preamble length
 */
void sx1276_lora_set_preamble_length( uint16_t value );

/*!
 * \brief Get the preamble length
 *
 * \retval value preamble length
 */
uint16_t sx1276_lora_get_preamble_length( void );

/*!
 * \brief Set the number or rolling preamble symbol needed for detection
 *
 * \param [IN] value number of preamble symbol
 */
void sx1276_lora_set_nb_trig_peaks( uint8_t value );

/*!
 * \brief Get the number or rolling preamble symbol needed for detection
 *
 * \retval value number of preamble symbol
 */
uint8_t sx1276_lora_get_nb_trig_peaks( void );

#endif /* RADIO_SX1276_LORA_SX1276_LORA_MISC_H_ */
