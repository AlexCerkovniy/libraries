#include "senseair_lp8_port.h"

__attribute((__weak__))  void LP8_PL_Init(void){

}

__attribute((__weak__))  void LP8_PL_UART_Init(uint32_t baudrate){

}

__attribute((__weak__))  void LP8_PL_UART_DeInit(void){

}

__attribute((__weak__))  void LP8_PL_UART_Tx(uint8_t *data, uint8_t length){

}

__attribute((__weak__))  bool LP8_PL_UART_Rx(uint8_t *data, uint8_t length){
    return false;
}

__attribute((__weak__))  bool LP8_PL_RDY_Read(void){
    return false;
}

__attribute((__weak__))  void LP8_PL_VBB_EN_Set(bool state){

}

__attribute((__weak__)) void LP8_PL_Delay(uint32_t milliseconds){

}
