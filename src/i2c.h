#pragma once

#include <stdint.h>
#include <libopencm3/stm32/i2c.h>

void i2c_init(void);

void i2c_start_message(uint8_t addr, uint8_t recv);
void i2c_stop_message(void);

void i2c_send_byte(char c);
char i2c_recv_byte(void);

// Set expander into byte/bank mode (IOCON.SEQOP=0, IOCON.BANK=0)
// configure pin directions and pullups
void init_expander(uint8_t addr);
