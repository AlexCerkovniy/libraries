#include "st7565_port.h"

#include "main.h"

void ST7565_PL_Init(void){

}

void ST7565_PL_RST_Pin(bool state){
    GPIO_write(LCD_RST, state);
}

void ST7565_PL_CS_Pin(bool state){
    GPIO_write(LCD_CS, state);
}

void ST7565_PL_DC_Pin(bool state){
    GPIO_write(LCD_DC, state);
}

void ST7565_PL_Backlight(bool state){
    GPIO_write(LCD_BCL, state);
}

uint8_t ST7565_PL_SPI_Transfer(uint8_t byte){
    unsigned char mask = 0x80;

    while(mask){
        if(byte & mask){
            GPIO_write(LCD_MOSI, 1);
        }
        else{
            GPIO_write(LCD_MOSI, 0);
        }
        GPIO_write(LCD_CLK, 1);
        mask >>= 1;
        GPIO_write(LCD_CLK, 0);
    }
    GPIO_write(LCD_MOSI, 0);

    return 0;
}

void ST7565_PL_Delay(uint32_t milliseconds){
    usleep(milliseconds * 1000);
}

void ST7565_PL_DelayUs(uint32_t microseconds){
    usleep(microseconds);
}
