#pragma once

#include <stdint.h>

#define NUM_SERVOS 12
#define NUM_SERVO_PHASES 3
#define SERVOS_PER_PHASE (NUM_SERVOS / NUM_SERVO_PHASES)

// in uS
#define MIN_SERVO_PULSE 500
#define MAX_SERVO_PULSE 4000

void servo_init(void);
void servo_deinit(void);
void servo_set_pos(uint8_t idx, uint16_t pulse_us);
void servo_disable(uint8_t idx);
uint16_t servo_get_pos(uint8_t idx);
void servo_reset(void);

void start_servo_phase(uint8_t phase);
