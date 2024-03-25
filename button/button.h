#pragma once

#include "stdint.h"
#include "stdbool.h"

#include "button_config.h"

typedef enum{
    BTN_IDLE = 0,
    BTN_DEBOUNCE,
    BTN_WAIT_LONG_PRESS
} button_status_t;

typedef enum{
    BTN_STATE_UPDATED = 0,
    BTN_SHORT_PRESS,
    BTN_LONG_PRESS
} button_callback_event_t;

typedef struct{
    uint16_t id;                            // Unique button ID
    bool pressed;                           // Contains actual filtered button state
    button_status_t status;                 // Internal handle processing state
    uint16_t debounce_ms;                   // Debounce time in milliseconds on press/release button
    uint16_t long_press_ms;                 // Long press time in milliseconds
    uint32_t timer;
    void (*callback)(uint8_t button_id, button_callback_event_t event);
}button_t;

void BTN_Init(button_t *button, uint16_t id, uint16_t debounce_ms, uint16_t long_press_ms);
void BTN_Main(button_t *button);
void BTN_Tick(button_t *button, uint32_t period);
bool BTN_IsPressed(button_t *button);

/* Callback registration */
void BTN_RegisterCallback(button_t *button, void (*callback)(uint8_t button_id, button_callback_event_t event));
