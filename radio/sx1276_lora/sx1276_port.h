/*
 * sx1276_port.h
 *
 *  Created on: Feb 21, 2024
 *      Author: user
 */

#ifndef RADIO_SX1276_LORA_SX1276_PORT_H_
#define RADIO_SX1276_LORA_SX1276_PORT_H_

#include <stdint.h>
#include <stdbool.h>

void sx1276_io_init(void);
void sx1276_set_reset(uint8_t state);
void sx1276_write(uint8_t reg, uint8_t data);
void sx1276_read(uint8_t reg, uint8_t *data);
void sx1276_write_burst(uint8_t reg, uint8_t *data, uint8_t count);
void sx1276_read_burst(uint8_t reg, uint8_t *data, uint8_t count);
void sx1276_delay_ms(uint32_t milliseconds);
uint32_t sx1276_get_tick(void);

#endif /* RADIO_SX1276_LORA_SX1276_PORT_H_ */
