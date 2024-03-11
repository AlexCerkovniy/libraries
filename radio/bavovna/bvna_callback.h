#pragma once

#include <stdint.h>
#include <stdbool.h>

void bvna_error_callback(uint16_t code);
void bvna_registration_success_callback(void);
void bvna_registration_timeout_callback(void);
