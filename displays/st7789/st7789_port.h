#ifndef ST7789_PORT_H
#define ST7789_PORT_H

#include <stdint.h>
#include <stdbool.h>

void st7789_port_init(void);
void st7789_port_delay(uint32_t ms);
void st7789_port_rst_set(void);
void st7789_port_rst_reset(void);
void st7789_port_dc_set(void);
void st7789_port_dc_reset(void);
void st7789_port_select(void);
void st7789_port_deselect(void);
void st7789_port_write(uint8_t *buffer, uint32_t len);

#endif //ST7789_PORT_H
