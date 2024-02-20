#include "shtc3.h"
#include "shtc3_port.h"

static struct {
  bool enabled;
  uint32_t timer;
  shtc3_state_t state;
  int16_t temperature;
  int16_t humidity;
  void (*updateEventCallback)(void);
} local;

void SHTC3_Init(void){
  local.state = SHTC3_INIT;
  local.timer = SHTC3_INIT_TDMA_CYCLES;
  SHTC3_PL_Init();
}

void SHTC3_Enabled(bool state){
    local.enabled = state;
}

void SHTC3_Tick(uint32_t period){
    if(local.timer > period){
        local.timer -= period;
    }
    else{
        local.timer = 0;
    }
}

void SHTC3_RegisterUpdateEventCallback(void (*callback)(void)){
    local.updateEventCallback = callback;
}

void SHTC3_Main(){
  static uint8_t buffer[6];
  static shtc3_data_t *data;
  static int32_t t_raw, h_raw;
  
  if(local.timer == 0){
    switch(local.state){
      case SHTC3_OFF:
        if(local.enabled){
          local.timer = SHTC3_INIT_TDMA_CYCLES;
          local.state = SHTC3_START;
        }
        else{
          return;
        }
        break;

      case SHTC3_INIT:
        buffer[0] = (SHTC3_SLEEP >> 8) & 0xFF;
        buffer[1] = SHTC3_SLEEP & 0xFF;
        SHTC3_PL_I2C_Init();
        SHTC3_PL_I2C_Write(SHTC3_ADDRESS, buffer, 2);
        if(local.enabled){
            local.timer = SHTC3_INIT_TDMA_CYCLES;
            local.state = SHTC3_START;
        }
        else{
            SHTC3_PL_I2C_DeInit();
            local.state = SHTC3_OFF;
        }
        break;
      
      case SHTC3_START:
        local.timer = SHTC3_WAKEUP_TDMA_CYCLES;
        local.state = SHTC3_MEASURE;
        buffer[0] = (SHTC3_WAKEUP >> 8) & 0xFF;
        buffer[1] = SHTC3_WAKEUP & 0xFF;
        SHTC3_PL_I2C_Init();
        SHTC3_PL_I2C_Write(SHTC3_ADDRESS, buffer, 2);
        break;
        
      case SHTC3_MEASURE:
        local.timer = SHTC3_MEASURE_TDMA_CYCLES;
        local.state = SHTC3_READ;
        buffer[0] = (SHTC3_CLOCK_STR_DIS_NORMAL_T_FIRST >> 8) & 0xFF;
        buffer[1] = SHTC3_CLOCK_STR_DIS_NORMAL_T_FIRST & 0xFF;
        SHTC3_PL_I2C_Write(SHTC3_ADDRESS, buffer, 2);
        break;
      
      case SHTC3_READ:
        if(local.enabled){
            local.timer = SHTC3_UPDATE_DATA_TDMA_CYCLES - SHTC3_MEASURE_TDMA_CYCLES - SHTC3_WAKEUP_TDMA_CYCLES;
            local.state = SHTC3_START;
        }
        else{
            local.state = SHTC3_OFF;
        }

        /* Read measurement data */
        SHTC3_PL_I2C_Read(SHTC3_ADDRESS, buffer, 6);
        data = (shtc3_data_t *)buffer;
        t_raw = ((uint16_t)data->temperature_msb << 8) | data->temperature_lsb;
        h_raw = ((uint16_t)data->humidity_msb << 8) | data->humidity_msb;
        local.temperature = -450 + (1750 * t_raw)/65536;
        local.humidity = (1000 * h_raw)/65536;

        /* Sleep command */
        buffer[0] = (SHTC3_SLEEP >> 8) & 0xFF;
        buffer[1] = SHTC3_SLEEP & 0xFF;
        SHTC3_PL_I2C_Write(SHTC3_ADDRESS, buffer, 2);
        SHTC3_PL_I2C_DeInit();

        /* Update event callback */
        if(local.updateEventCallback){
            local.updateEventCallback();
        }
        break;
           
      default:
        local.state = SHTC3_INIT;
        local.timer = 0;
        break;
    }
  }
}

int16_t SHTC3_GetTemperature(void){
  return local.temperature;
}

int16_t SHTC3_GetHumidity(void){
  return local.humidity;
}
