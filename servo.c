#include "servo.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

static uint32_t reg32 __attribute__((unused));
static uint32_t i2c = I2C1;

static void set_reg_pointer(uint8_t reg)
{
	// EV5
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	        & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, 0x21, I2C_WRITE);
	// /EV5


	// EV6
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	// /EV6

	// EV8_1
	i2c_send_data(i2c, reg);
	// /EV8_1
	
	// EV8_2
	while (!(I2C_SR1(i2c) & ( I2C_SR1_BTF | I2C_SR1_TxE )));
	// /EV8_2
}

static void write_reg(uint8_t reg, uint8_t val)
{
	i2c_send_start(i2c);
	set_reg_pointer(reg);

	i2c_send_data(i2c, val);

	while (!(I2C_SR1(i2c) & ( I2C_SR1_BTF | I2C_SR1_TxE )));

	i2c_send_stop(i2c);
}

static uint8_t read_reg(uint8_t reg)
{
	i2c_send_start(i2c);
	set_reg_pointer(reg);

	i2c_send_start(i2c);
	
	// EV5
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	        & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, 0x21, I2C_READ);
	// /EV5

	// EV6_3
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	// Clear ACK
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	reg32 = I2C_SR2(i2c);
	// Set STOP
	i2c_send_stop(i2c);
	// /EV6_3
	
	// EV7
	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));
	// /EV7

	return i2c_get_data(i2c);
}

void servo_init(void) {
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO7);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C1EN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

	i2c_reset(I2C1);
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_14MHZ);
	i2c_set_fast_mode(I2C1);
	i2c_set_ccr(I2C1, 9);
	i2c_set_trise(I2C1, 10);
	i2c_peripheral_enable(I2C1);

	#define OLATA 0x14
	#define GPPUA 0x0C
	write_reg(OLATA, 0x20);
	write_reg(GPPUA, 0x90);

	// Set dir
	write_reg(0x00, 0x90);
	write_reg(0x01, 0x00);
}

void servo_out(uint16_t val)
{
	write_reg(0x14, val & 0xff);
	write_reg(0x15, (val >> 8) & 0xff);
}

uint8_t servo_read()
{
	return read_reg(0x12);
}
