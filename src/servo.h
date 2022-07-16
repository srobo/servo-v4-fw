#pragma once

#include <stdint.h>

#define NUM_SERVOS 12

// in uS
#define MIN_SERVO_PULSE 200
#define MAX_SERVO_PULSE 10000

void servo_init(void);
void servo_deinit(void);
void servo_set_pos(uint8_t idx, uint16_t pulse_us);
void servo_disable(uint8_t idx);
uint16_t servo_get_pos(uint8_t idx);
void servo_reset(void);

void start_servo_period(void);

extern volatile uint8_t processing_servo_pulses;
