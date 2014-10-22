#pragma once

#include <stdint.h>

void servo_init(void);

void servo_set_pos(uint8_t idx, int16_t pos);

uint8_t servo_read();
