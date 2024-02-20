#include "sleep.h"
#include "sleep_port.h"

static uint16_t token_state = 0;

void SLEEP_Init(uint32_t wakeup_period){
    token_state = 0;
    SLEEP_PL_Init(wakeup_period);
}

void SLEEP_Main(void){
    if(token_state == 0){
        SLEEP_PL_Sleep();
    }
}

bool SLEEP_Sleep(void){
    if(token_state){
        return false;
    }
    else{
        return true;
    }
}

void SLEEP_Wakeup(void){
    SLEEP_PL_Wakeup();
}

void SLEEP_GetToken(sleep_token_t mask){
    token_state |= mask;
}

void SLEEP_ClearToken(sleep_token_t mask){
    token_state &= ~mask;
}
