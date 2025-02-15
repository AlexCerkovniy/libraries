#include "button.h"
#include "button_port.h"

void BTN_Init(button_t *button, uint16_t id, uint16_t debounce_ms, uint16_t long_press_ms){
    button->id = id;
    button->long_press_ms = long_press_ms;
    button->debounce_ms = debounce_ms;
    button->status = BTN_IDLE;
    button->timer = 0;

    /* Set initial button state */
    button->pressed = BTN_PL_Read(button->id);
}

void BTN_Main(button_t *button){
    bool state;

    switch(button->status){
        case BTN_IDLE:
            if(button->timer){
                return;
            }

            state = BTN_PL_Read(button->id);
            if(state != button->pressed){
                button->status = BTN_DEBOUNCE;
                button->timer = button->debounce_ms;
            }
            break;

        case BTN_DEBOUNCE:
            if(button->timer){
                return;
            }

            state = BTN_PL_Read(button->id);
            if(state != button->pressed){
                button->pressed = state;

                if(state){
                    button->status = BTN_WAIT_LONG_PRESS;
                    button->timer = button->long_press_ms;
                }
                else{
                    button->status = BTN_IDLE;
                    button->timer = button->debounce_ms;
                }

                if(button->callback){
                    button->callback(button->id, BTN_STATE_UPDATED);
                }
            }
            else{
                button->status = BTN_IDLE;
                button->timer = button->debounce_ms;
            }
            break;

        case BTN_WAIT_LONG_PRESS:
            state = BTN_PL_Read(button->id);

            /* Check if button is still pressed */
            if(state != button->pressed){
                button->status = BTN_DEBOUNCE;
                button->timer = button->debounce_ms;

                if(button->callback){
                    button->callback(button->id, BTN_SHORT_PRESS);
                }
                break;
            }

            if(button->timer == 0){
                button->status = BTN_IDLE;

                if(button->callback){
                    button->callback(button->id, BTN_LONG_PRESS);
                }
            }
            break;

        default:
            button->status = BTN_IDLE;
            break;
    }
}

void BTN_Tick(button_t *button, uint32_t period){
    if(button->timer > period){
        button->timer -= period;
    }
    else{
        button->timer = 0;
    }
}

bool BTN_IsPressed(button_t *button){
    return button->pressed;
}

/* Callback register */
void BTN_RegisterCallback(button_t *button, void (*callback)(uint8_t button_id, button_callback_event_t event)){
    button->callback = callback;
}
