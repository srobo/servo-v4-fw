#include "battery.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "led.h"

void battery_init(void) {
/*	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C1EN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

	i2c_reset(I2C1);
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
	i2c_set_fast_mode(I2C1);
	i2c_set_ccr(I2C1, 0x09);
	i2c_set_trise(I2C1, 0x0b);
	i2c_peripheral_enable(I2C1);*/
}


static uint32_t reg32 __attribute__((unused));
static uint32_t i2c = I2C1;

static void set_reg_pointer(void)
{
	// EV5
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	        & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, 0x40, I2C_WRITE);
	// /EV5


	// EV6
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	// /EV6

	// EV8_1
	i2c_send_data(i2c, 0x01);
	// /EV8_1
	
	// EV8_2
	while (!(I2C_SR1(i2c) & ( I2C_SR1_BTF | I2C_SR1_TxE )));
	// /EV8_2
}

uint16_t battery_voltage(void)
{

	i2c_send_start(i2c);

	set_reg_pointer();

	i2c_send_start(i2c);

	// EV5
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	        & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	// Set POS and ACK
	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

	i2c_send_7bit_address(i2c, 0x40, I2C_READ);
	// /EV5

	// EV6
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	// /EV6

	// Clear ACK
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

	// EV7_3
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_stop(i2c);
	// /EV7_3
	
	uint16_t val;
	val = i2c_get_data(i2c) << 8;
	val |= i2c_get_data(i2c);

	// Clear POS
	I2C_CR1(i2c) &= ~I2C_CR1_POS;


	return val;
}
