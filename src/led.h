#pragma once

#include <stdint.h>

#define LED_STATUS_BLUE 12
#define LED_STATUS_RED 13

void led_init(void);

void set_led(uint8_t idx);
void clear_led(uint8_t idx);
