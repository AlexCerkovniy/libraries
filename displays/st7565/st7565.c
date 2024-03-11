#include "st7565.h"
#include "st7565_port.h"
#include "string.h"

#define ST7565_FRAMEBUFFER_SIZE             ((ST7565_WIDTH * ST7565_HEIGHT) / 8)

static uint8_t framebuffer[ST7565_FRAMEBUFFER_SIZE] = {0};
static bool backlight = false;

void _sendSpiCommand(uint8_t cmd);
void _sendSpiData(uint8_t data);

void ST7565_Init(void){
    ST7565_PL_Init();

    /* Reset sequence */
    ST7565_PL_RST_Pin(false);
    ST7565_PL_Delay(5);
    ST7565_PL_RST_Pin(true);
    ST7565_PL_Delay(2);

    /* Initialize display */
    ST7565_PL_CS_Pin(false);
    _sendSpiCommand(CMD_SOFT_RESET);
    _sendSpiCommand(CMD_DISPLAY_OFF);
    _sendSpiCommand(CMD_SET_DISP_START_LINE);
#if defined(ST7565_FLIP)
    _sendSpiCommand(CMD_SET_ADC_REVERSE);
    _sendSpiCommand(CMD_SET_COM_NORMAL);
#else
    _sendSpiCommand(CMD_SET_ADC_NORMAL);
    _sendSpiCommand(CMD_SET_COM_REVERSE);
#endif
    _sendSpiCommand(CMD_SET_DISP_NORMAL);
    _sendSpiCommand(CMD_SET_BIAS_9);
    _sendSpiCommand(CMD_SET_VOLUME_FIRST);
    _sendSpiCommand(CMD_SET_VOLUME_SECOND | (ST7565_STARTUP_CONTRAST & 0x3F));
    _sendSpiCommand(CMD_SET_RESISTOR_RATIO | 5);
    _sendSpiCommand(CMD_SET_POWER_CONTROL | POWER_CONTROL_BOOSTER | POWER_CONTROL_VREGULATOR | POWER_CONTROL_VFOLLOWER);
    _sendSpiCommand(CMD_SET_BOOSTER_RATIO_FIRST);
    _sendSpiCommand(CMD_SET_BOOSTER_RATIO_SECOND | BOOSTER_RATIO_2X_3X_4X_MASK);

    /* Go to power off (almost zero power consumtion) */
    _sendSpiCommand(CMD_DISPLAY_OFF);
    _sendSpiCommand(CMD_SET_ALLPTS_ON);
    _sendSpiCommand(CMD_SET_POWER_CONTROL); // All flags is reset

    ST7565_PL_CS_Pin(true);
    ST7565_PL_Delay(5);
    ST7565_Update();
}

uint8_t *ST7565_GetFramebuffer(void){
    return framebuffer;
}

uint16_t ST7565_GetFramebufferSize(void){
    return ST7565_FRAMEBUFFER_SIZE;
}

void ST7565_Sleep(bool state){
    ST7565_PL_CS_Pin(false);
    if(state){
        _sendSpiCommand(CMD_DISPLAY_OFF);
        _sendSpiCommand(CMD_SET_ALLPTS_ON);
        _sendSpiCommand(CMD_SET_POWER_CONTROL); // All flags is reset
    }
    else{
        _sendSpiCommand(CMD_SET_ALLPTS_NORMAL);
        _sendSpiCommand(CMD_SET_POWER_CONTROL | POWER_CONTROL_BOOSTER | POWER_CONTROL_VREGULATOR | POWER_CONTROL_VFOLLOWER);
        _sendSpiCommand(CMD_DISPLAY_ON);
    }
    ST7565_PL_CS_Pin(true);
}

void ST7565_Backlight(bool state){
    backlight = state;
    ST7565_PL_Backlight(state);
}

bool ST7565_GetBacklightState(void){
    return backlight;
}

void ST7565_SetContrast(uint8_t contrast){
    ST7565_PL_CS_Pin(false);
    _sendSpiCommand(CMD_SET_VOLUME_FIRST);
    _sendSpiCommand(CMD_SET_VOLUME_SECOND | (contrast & 0x3F));
    ST7565_PL_CS_Pin(true);
}

void ST7565_Clear(void){
    memset(framebuffer, 0, ST7565_FRAMEBUFFER_SIZE);
}

void ST7565_Update(void){
#if defined(ST7565_CUSTOM_PAGE_SEQUENCE)
    const uint8_t pagemap[] = ST7565_CUSTOM_PAGE_SEQUENCE;
#endif

    ST7565_PL_CS_Pin(false);
    for(uint16_t page = 0; page < ST7565_PAGES_COUNT; page++){
#if defined(ST7565_CUSTOM_PAGE_SEQUENCE)
        _sendSpiCommand(CMD_SET_PAGE | pagemap[page]);
#else
        _sendSpiCommand(CMD_SET_PAGE | page);
#endif
        _sendSpiCommand(CMD_SET_COLUMN_UPPER | 0);
#if defined(ST7565_FLIP)
        _sendSpiCommand(CMD_SET_COLUMN_LOWER | 4);  /* X offset need - 4 pixels */
#else
        _sendSpiCommand(CMD_SET_COLUMN_LOWER | 0);
#endif

        /* Send page data */
        ST7565_PL_DC_Pin(true);
        for(uint16_t coulumn = 0; coulumn < ST7565_WIDTH; coulumn++) {
            ST7565_PL_SPI_Transfer(framebuffer[(ST7565_PAGE_SIZE * page) + coulumn]);
        }
    }

    ST7565_PL_CS_Pin(true);
}

void _sendSpiCommand(uint8_t cmd){
    ST7565_PL_DC_Pin(false);
    ST7565_PL_SPI_Transfer(cmd);
}

void _sendSpiData(uint8_t data){
    ST7565_PL_DC_Pin(true);
    ST7565_PL_SPI_Transfer(data);
}
