#pragma once

#include <stdint.h>

void servo_init(void);
void servo_set_pos(uint8_t idx, uint16_t pulse_us);
void servo_disable(uint8_t idx);
uint32_t servo_get_pos(uint8_t idx);

void start_servo_period(void);
