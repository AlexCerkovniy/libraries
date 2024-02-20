#include "button_port.h"
#include "button_config.h"

/* Port (platform) includes */
#include "main.h"

void BTN_PL_Init(uint16_t id){

}

bool BTN_PL_Read(uint16_t id){
    return (GPIO_read(BUTTON) > 0)?(true):(false);
}
