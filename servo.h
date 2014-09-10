#pragma once

#include <stdint.h>

void servo_init(void);

void servo_out(uint16_t);

uint8_t servo_read();
