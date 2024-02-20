#include "gfx8lib.h"
#include "gfx8lib_fonts.h"
#include "gfx8lib_models.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

static unsigned char* buffer = NULL;
gfx8_font_t font;
static unsigned char cursor_x = 0;
static unsigned char cursor_y = 0;

/* MACROS FUNCTIONS */
#define G8LIB_PIXEL_MASK(POSITION)              (1 << (POSITION))
#define G8LIB_GET_MIN(A, B)                     (((A) < (B)) ? (A) : (B))

#define G8LIB_SWAP_U8(A, B){                    \
  unsigned char t = A;                          \
  A = B;                                        \
  B = t;                                        \
}

#define G8LIB_SWAP_U16(A, B){                   \
  unsigned short t = A;                         \
  A = B;                                        \
  B = t;                                        \
}

void G8Lib_Init(unsigned char screen_buffer[]){
  buffer = screen_buffer;
  font = GFX8_DEFAULT_FONT;
}

void G8Lib_PutPixel(unsigned char x, unsigned char y, gfx8_color_t color){
    if(x < GFX8_SCREEN_WIDTH && y < GFX8_SCREEN_HEIGHT){
        switch(color){
          case GFX8_RESET:
              buffer[(y>>3)*GFX8_SCREEN_WIDTH + x] &= ~G8LIB_PIXEL_MASK(y&0x07);
              break;
          case GFX8_SET:
              buffer[(y>>3)*GFX8_SCREEN_WIDTH + x] |= G8LIB_PIXEL_MASK(y&0x07);
              break;
          case GFX8_ADAPTIVE:
              buffer[(y>>3)*GFX8_SCREEN_WIDTH + x] ^= G8LIB_PIXEL_MASK(y&0x07);
              break;
        }
    }
}

bool G8Lib_GetPixel(uint8_t x, uint8_t y){
  if(x < GFX8_SCREEN_WIDTH && y < GFX8_SCREEN_HEIGHT){
      if((buffer[(y>>3)*GFX8_SCREEN_WIDTH + x] >> (y&0x07)) & 0x01){
          return true;
      }
  }

  return false;
}

void G8Lib_PutByte(unsigned char x, unsigned char y, unsigned char byte, gfx8_color_t color){
  unsigned short tmp = byte;
  unsigned char offset;
  unsigned short position;
  
  if(x < GFX8_SCREEN_WIDTH && y < GFX8_SCREEN_HEIGHT){
    position = (y >> 3) * GFX8_SCREEN_WIDTH + x;
    offset = y & 0x07;
    
    if(offset){
      tmp <<= offset;
      
      switch(color){
        case GFX8_RESET: buffer[position] &= ~(unsigned char)tmp; break;
        case GFX8_SET: buffer[position] |= (unsigned char)tmp; break;
        case GFX8_ADAPTIVE: buffer[position] ^= (unsigned char)tmp; break;
      }
      
      if(position > SCREEN_BUFFER_SIZE - GFX8_SCREEN_WIDTH){
        return;
      }
      else{
        position += GFX8_SCREEN_WIDTH;
        tmp >>= 8;
        
        switch(color){
          case GFX8_RESET:
            buffer[position] &= ~(unsigned char)tmp;
            break;
          
          case GFX8_SET:
            buffer[position] |= (unsigned char)tmp;
            break;
          
          case GFX8_ADAPTIVE:
            buffer[position] ^= (unsigned char)tmp;
            break;
        }
      }
    } 
    else{
      switch(color){
        case GFX8_RESET:
          buffer[position] &= ~byte;
          break;
        
        case GFX8_SET:
          buffer[position] |= byte;
          break;
        
        case GFX8_ADAPTIVE:
          buffer[position] ^= byte;
          break;
      }
    }
  }
}

void G8Lib_DrawVLine(unsigned char x, unsigned char y0, unsigned char y1){
	if(x > GFX8_SCREEN_WIDTH - 1){
		return;
	}

	if((y0 >= GFX8_SCREEN_HEIGHT) && (y1 >= GFX8_SCREEN_HEIGHT)){
		return;
	}

    uint8_t segment = y0 >> 3;
    uint8_t y_current = y0;
    uint8_t length = y1 - y0 + 1;
    uint8_t mask;
    uint8_t temp;

    if(y0 == y1){
		G8Lib_PutPixel(x, y0, 1);
		return;
	}
    else if(y0 > y1){
    	temp = y1;
    	y1 = y0;
    	y0 = temp;
	}

    if(y1 >= GFX8_SCREEN_HEIGHT){
    	y1 = GFX8_SCREEN_HEIGHT - 1;
    }

    while(length){
    	mask = 0xFF;
    	mask <<= (y_current & 0x07);

    	if((y_current & 0x07) + length < 8){
    		mask &= (0xFF >> (8 - (y_current & 0x07) - length));
    	}

    	/* Draw into segment according to mask & color */
    	buffer[segment * GFX8_SCREEN_WIDTH + x] |= mask;

    	/* Decrease length */
    	if(y_current + length > ((segment + 1) * 8) || length > 8){
    		length -= (8 - (y_current & 0x07));
    	}
    	else{
    		/* Line draw is ended */
    		break;
    	}

    	/* Move to next segment */
		segment++;
		y_current = segment * 8;
    }
}

void G8Lib_DrawDottedVLine(uint8_t y0, uint8_t y1, uint8_t x, uint8_t gap_width, uint8_t dot_width, bool dot_first){
    unsigned char tmp0;

    if(y0 > y1){
      tmp0 = y1;
      y1 = y0;
      y0 = tmp0;
    }

    while(y0 != y1){
        if(dot_first){
            if(y0 + dot_width > y1){
                dot_width = y1 - y0;
            }

            G8Lib_DrawVLine(x, y0, y0 + dot_width);
            y0 += dot_width;
            dot_first = false;
        }
        else{
            if(y0 + gap_width > y1){
                gap_width = y1 - y0;
            }

            y0 += gap_width;
            dot_first = true;
        }
    }
}

void G8Lib_DrawHLine(unsigned char x0, unsigned char x1, unsigned char y, gfx8_color_t color){
    if(x0 < GFX8_SCREEN_WIDTH && y < GFX8_SCREEN_HEIGHT && x1 < GFX8_SCREEN_WIDTH){
        unsigned char tmp0, tmp1;

        if(x0 > x1){
          tmp0 = x1;
          x1 = x0;
          x0 = tmp0;
        }

        /* Get Y row segment (tmp0) and pixel mask (tmp1) */
        tmp0 = y >> 3;
        tmp1 = 1 << (y & 0x07);

        for(; x0 <= x1; x0++){
            switch(color){
                case GFX8_RESET:
                    buffer[tmp0 * GFX8_SCREEN_WIDTH + x0] &= ~tmp1;
                    break;

                case GFX8_SET:
                    buffer[tmp0 * GFX8_SCREEN_WIDTH + x0] |= tmp1;
                    break;

                case GFX8_ADAPTIVE:
                    buffer[tmp0 * GFX8_SCREEN_WIDTH + x0] ^= tmp1;
                    break;
            }
        }
    }
}

void G8Lib_DrawDottedHLine(unsigned char x0, unsigned char x1, unsigned char y, uint8_t step_width_px){
    if(x0 < GFX8_SCREEN_WIDTH && y < GFX8_SCREEN_HEIGHT && x1 < GFX8_SCREEN_WIDTH){
        unsigned char tmp0, tmp1, tmp2;
        uint8_t dotted = 1;

        if(x0 > x1){
          tmp0 = x1;
          x1 = x0;
          x0 = tmp0;
        }

        /* Get Y row segment (tmp0) and pixel mask (tmp1) */
        tmp0 = y >> 3;
        tmp1 = 1 << (y & 0x07);
        tmp2 = step_width_px;

        for(; x0 <= x1; x0++){
            if(tmp2){
                tmp2--;

                if(!tmp2){
                    tmp2 = step_width_px;
                    dotted ^= 1;
                }
            }

            if(dotted){
                buffer[tmp0 * GFX8_SCREEN_WIDTH + x0] |= tmp1;
            }
            else{
                buffer[tmp0 * GFX8_SCREEN_WIDTH + x0] &= ~tmp1;
            }
        }
    }
}

void G8Lib_Rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, gfx8_color_t color){
  if(((x + w) <= GFX8_SCREEN_WIDTH) && ((y + h) <= GFX8_SCREEN_HEIGHT)){
    w = x + w;
    for(uint8_t i = x; i < w; i++){
      G8Lib_DrawVLine(i, y, y + h - 1);
    }
  }
}

void G8Lib_RectNoFill(uint8_t x, uint8_t y, uint8_t w, uint8_t h, gfx8_color_t color){
	if(((x + w) <= GFX8_SCREEN_WIDTH) && ((y + h) <= GFX8_SCREEN_HEIGHT)){
		G8Lib_DrawVLine(x, y, y + h - 1);
		G8Lib_DrawVLine(x + w - 1, y, y + h - 1);
		G8Lib_DrawHLine(x, x + w - 1, y, color);
		G8Lib_DrawHLine(x, x + w - 1, y + h - 1, color);
	}
}

/*************************************************************************************************************************************/
/******************************************************** TEXT PRINTING **************************************************************/
/*************************************************************************************************************************************/
static void gfx8_draw_char(char c, gfx8_color_t color){
  unsigned short temp;
  
  if((c < font.ascii_start) || (c > font.ascii_end)){
    return;
  }
  
  if (GFX8_SCREEN_WIDTH < (cursor_x + font.width) || GFX8_SCREEN_HEIGHT < (cursor_y + font.height)){
    return;
  }
  
  temp = (c - font.ascii_start) * font.width * font.height_pages;
  for(char i = 0; i < font.width; i++){
    for(char j = 0; j < font.height_pages; j++){
      G8Lib_PutByte(cursor_x, cursor_y + (8 * j), font.data[temp + (font.width * j) + i], color);
    }
    cursor_x++;
  }
}

void G8Lib_SetCursor(unsigned char x, unsigned char y){
  cursor_x = x;
  cursor_y = y;
}

unsigned char G8Lib_GetCursorX(void){
	return cursor_x;
}

unsigned char G8Lib_GetCursorY(void){
	return cursor_y;
}

void G8Lib_SetFont(gfx8_font_t new_font){
  font = new_font;
}

gfx8_font_t *G8Lib_GetFont(void){
  return &font;
}

void G8Lib_PrintUnsignedInt(unsigned long number, gfx8_color_t color){
  unsigned char buffer[10];
  unsigned char count = 1;
  
  while(count < 10){
    buffer[count] = number%10;
    number = number/10;
    if(number){
      count++;
    }
    else{
      break;
    }
  }
  
  while(count){
    gfx8_draw_char(0x30 + buffer[count--], color);
  }
}

void G8Lib_PrintSignedInt(long number, gfx8_color_t color){
  
}

void G8Lib_String(char* str, gfx8_color_t color){
  while (*str){
    gfx8_draw_char(*str++, color);
  }
}

void G8Lib_Print(gfx8_color_t color, const char* fmt, ... ){
  char string[GFX8_VA_STRING_BUFFER_SIZE];
  
  va_list args;
  va_start(args, fmt);
  vsnprintf(string, sizeof(string), fmt, args);
  G8Lib_String(string, color);
}

/*************************************************************************************************************************************/
/******************************************************* SPRITES DRAWING *************************************************************/
/*************************************************************************************************************************************/
void G8Lib_DrawBatteryStatus(unsigned char x, unsigned char y, unsigned char length, unsigned char status, gfx8_color_t color){
  G8Lib_PutByte(x++, y, BATTERY_INDICATOR_BODY_FULL, color);
  while(length--){
    if(status){
      G8Lib_PutByte(x++, y, BATTERY_INDICATOR_BODY_FULL, color);
      status--;
    }
    else{
      G8Lib_PutByte(x++, y, BATTERY_INDICATOR_BODY_EMPTY, color);
    }
  }
  G8Lib_PutByte(x++, y, BATTERY_INDICATOR_BODY_FULL, color);
  G8Lib_PutByte(x, y, BATTERY_INDICATOR_BODY_PLUS, color);
}

void G8Lib_DrawBitmap(unsigned char x, unsigned char y, gfx8_bitmap_t bitmap, gfx8_color_t color){
  unsigned char x_tmp = x;
  unsigned char y_tmp = y;
  unsigned short bmp_offset;
  uint8_t bitmap_segments;

  if (x > GFX8_SCREEN_WIDTH || y > GFX8_SCREEN_HEIGHT) {
    return;
  }
  
  if(bitmap.height >= 8){
      bitmap_segments = bitmap.height/8;
  } else{
      bitmap_segments = 1;
  }

  for(unsigned char i = 0; i < bitmap_segments; i++){
    bmp_offset = i * bitmap.width;
    for(unsigned char j = 0; j < bitmap.width; j++){
      G8Lib_PutByte(x_tmp++, y_tmp, bitmap.data[bmp_offset + j], color);
    }
    y_tmp += 8;
    x_tmp = x;
  }
}
