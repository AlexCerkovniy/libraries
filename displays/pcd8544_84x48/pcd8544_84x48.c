#include "pcd8544_84x48.h"
#include "pcd8544_84x48_port.h"

#define PCD8544_SCREEN_BUFFER_SIZE          ((PCD8544_SCREEN_WIDTH * PCD8544_SCREEN_HEIGTH)/8)

static uint8_t framebuffer[PCD8544_SCREEN_BUFFER_SIZE];

void PCD8544_Init(void){
  PCD8544_PL_Init();
  PCD8544_PL_NssPin(false);
  PCD8544_PL_DataCommandPin(false);
  
  /* Reset Display */
  PCD8544_PL_ResetPin(false);
  PCD8544_PL_Delay(PCD8544_RST_DELAY_MS);
  PCD8544_PL_ResetPin(true);                  
  PCD8544_PL_Delay(PCD8544_RST_DELAY_MS);		
  
  /* Upload parameters */
  PCD8544_PL_SpiTransfer(0x21);                               //Function Set: PD = 0, V = 0, H = 1
  PCD8544_PL_SpiTransfer(0x80 | 48);                          //Set Vop
  PCD8544_PL_SpiTransfer(PCD8544_TEMP_COEFFICIENT_2); 
  PCD8544_PL_SpiTransfer(PCD8544_BIAS_SYSTEM_2);  
  PCD8544_PL_SpiTransfer(0x20);                               //Function Set: PD = 0, V = 0, H = 0
  PCD8544_PL_SpiTransfer(PCD8544_DISP_CTRL_NORMAL_MODE);      //Set display configuration
  PCD8544_PL_NssPin(true);
  
  /* Clear Screen */
  PCD8544_Clear();
  PCD8544_Update();
}

uint8_t *PCD8544_GetFramebuffer(void){
  return framebuffer;
}

uint16_t PCD8544_GetFramebufferSize(void){
  return PCD8544_SCREEN_BUFFER_SIZE;
}

void PCD8544_Backlight(bool state){
  PCD8544_PL_BacklightPin((bool)state);
}

void PCD8544_Update(void){
  PCD8544_PL_NssPin(false);
  PCD8544_PL_DataCommandPin(false);            
  PCD8544_PL_SpiTransfer(0x40); 
  PCD8544_PL_SpiTransfer(0x80);
  PCD8544_PL_DataCommandPin(true);  
  
  for(uint16_t x = 0; x < PCD8544_SCREEN_BUFFER_SIZE; x++){
      PCD8544_PL_SpiTransfer(framebuffer[x]);
  }
  
  PCD8544_PL_NssPin(true);		
}

void PCD8544_Clear(void){
  memset(framebuffer, 0, PCD8544_SCREEN_BUFFER_SIZE);
}
