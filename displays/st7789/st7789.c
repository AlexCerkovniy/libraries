#include "st7789.h"

#include <string.h>

#include "st7789_port.h"

uint16_t disp_buf[ST7789_WIDTH * ST7789_HEIGHT];

/**
 * @brief Write command to ST7789 controller
 * @param cmd -> command to write
 * @return none
 */
static void ST7789_WriteCommand(uint8_t cmd)
{
	st7789_port_dc_reset();
	st7789_port_write(&cmd, sizeof(cmd));
}

/**
 * @brief Write data to ST7789 controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
static void ST7789_WriteData(uint8_t *buff, uint16_t buff_size)
{
	st7789_port_dc_set();
	st7789_port_write(buff, buff_size);
}
/**
 * @brief Write data to ST7789 controller, simplify for 8bit data.
 * data -> data to write
 * @return none
 */
static void ST7789_WriteSmallData(uint8_t data)
{
	st7789_port_dc_set();
	st7789_port_write(&data, sizeof(data));
}

/**
 * @brief Set the rotation direction of the display
 * @param m -> rotation parameter(please refer it in st7789.h)
 * @return none
 */
void ST7789_SetRotation(uint8_t m) {
	const uint8_t presets[4] = {
		ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB,
		ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB,
		ST7789_MADCTL_RGB,
		ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB
	};

	if (m < 4) {
		st7789_port_select();
		ST7789_WriteCommand(ST7789_MADCTL);
		ST7789_WriteSmallData(presets[m]);
		st7789_port_deselect();
	}
}

/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
static void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	st7789_port_select();
	uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;
	
	/* Column Address set */
	ST7789_WriteCommand(ST7789_CASET); 
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
		ST7789_WriteData(data, sizeof(data));
	}

	/* Row Address set */
	ST7789_WriteCommand(ST7789_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		ST7789_WriteData(data, sizeof(data));
	}
	/* Write to RAM */
	ST7789_WriteCommand(ST7789_RAMWR);
	st7789_port_deselect();
}

/**
 * @brief Initialize ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Init(void)
{
	#ifdef USE_DMA
		memset(disp_buf, 0, sizeof(disp_buf));
	#endif

	st7789_port_init();
	st7789_port_delay(10);
    st7789_port_rst_reset();
    st7789_port_delay(10);
	st7789_port_rst_set();
    st7789_port_delay(20);

	st7789_port_select();
    ST7789_WriteCommand(ST7789_COLMOD);		//	Set color mode
    ST7789_WriteSmallData(ST7789_COLOR_MODE_16bit);
  	ST7789_WriteCommand(0xB2);				//	Porch control
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		ST7789_WriteData(data, sizeof(data));
	}
	ST7789_SetRotation(ST7789_ROTATION);	//	MADCTL (Display Rotation)
	
	/* Internal LCD Voltage generator settings */
    ST7789_WriteCommand(0XB7);				//	Gate Control
    ST7789_WriteSmallData(0x35);			//	Default value
    ST7789_WriteCommand(0xBB);				//	VCOM setting
    ST7789_WriteSmallData(0x19);			//	0.725v (default 0.75v for 0x20)
    ST7789_WriteCommand(0xC0);				//	LCMCTRL	
    ST7789_WriteSmallData (0x2C);			//	Default value
    ST7789_WriteCommand (0xC2);				//	VDV and VRH command Enable
    ST7789_WriteSmallData (0x01);			//	Default value
    ST7789_WriteCommand (0xC3);				//	VRH set
    ST7789_WriteSmallData (0x12);			//	+-4.45v (defalut +-4.1v for 0x0B)
    ST7789_WriteCommand (0xC4);				//	VDV set
    ST7789_WriteSmallData (0x20);			//	Default value
    ST7789_WriteCommand (0xC6);				//	Frame rate control in normal mode
    ST7789_WriteSmallData (0x0F);			//	Default value (60HZ)
    ST7789_WriteCommand (0xD0);				//	Power control
    ST7789_WriteSmallData (0xA4);			//	Default value
    ST7789_WriteSmallData (0xA1);			//	Default value
	/**************** Division line ****************/

	ST7789_WriteCommand(0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}

    ST7789_WriteCommand(0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}
    ST7789_WriteCommand (ST7789_INVON);		//	Inversion ON
	ST7789_WriteCommand (ST7789_SLPOUT);	//	Out of sleep mode
  	ST7789_WriteCommand (ST7789_NORON);		//	Normal Display on
  	ST7789_WriteCommand (ST7789_DISPON);	//	Main screen turned on	
	st7789_port_deselect();

	st7789_port_delay(10);

	memset(disp_buf, 0, sizeof(disp_buf));
	ST7789_Draw();
}

void ST7789_Draw(void) {
	ST7789_SetAddressWindow(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);

	//Split data in small chunks because
	uint16_t lines_elapsed = ST7789_HEIGHT;
	uint16_t *disp_buf_tmp = disp_buf;

	st7789_port_select();
	st7789_port_dc_set();
	while (lines_elapsed) {
		st7789_port_write((uint8_t *)disp_buf, ST7789_WIDTH * CHUNK_SIZE_LINES * 2);
		disp_buf_tmp += ST7789_WIDTH * CHUNK_SIZE_LINES;
		lines_elapsed -= CHUNK_SIZE_LINES;
	}
	st7789_port_deselect();
}

/**
 * @brief Fill the DisplayWindow with single color
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill_Color(uint16_t color)
{
	for (uint32_t i = 0; i < ST7789_WIDTH * ST7789_HEIGHT; i++) {
		disp_buf[i] = color >> 8;
		disp_buf[i] |= (color & 0xFF) << 8;
	}
}

// __attribute__((weak)) void st7789_port_init(void){}
// __attribute__((weak)) void st7789_port_delay(uint32_t ms) {
// 	(void)ms;
// }
// __attribute__((weak)) void st7789_port_rst_set(void){}
// __attribute__((weak)) void st7789_port_rst_reset(void){}
// __attribute__((weak)) void st7789_port_dc_set(void){}
// __attribute__((weak)) void st7789_port_dc_reset(void){}
// __attribute__((weak)) void st7789_port_select(void){}
// __attribute__((weak)) void st7789_port_deselect(void){}
// __attribute__((weak)) void st7789_port_write(uint8_t *buffer, uint32_t len) {
// 	(void)buffer;
// 	(void)len;
// }