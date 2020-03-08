#include "servo.h"
#include "led.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dbgmcu.h>

#include <stdlib.h>

#define NUM_SERVOS 12
#define POS_MAX 100
#define POS_MIN -100

// Cannot be less than 10 due to the time it takes to talk to the GPIO thing
#define START_DELAY 15
#define CENTRE_DELAY 300

static volatile int16_t servo_positions[NUM_SERVOS] = { 0 };

typedef struct
{
	uint8_t idx;
	uint16_t pulse_length;
} indexed_pulse_length_t;

static indexed_pulse_length_t sorted_servo_pulse_lengths[NUM_SERVOS];

typedef struct
{
	uint16_t pin_state;
	uint16_t duration;
} output_command_t;

static volatile output_command_t output_commands[NUM_SERVOS*2] = { { 0, 50 } };

static uint32_t reg32 __attribute__((unused));
static uint32_t i2c = I2C1;

static void setup_next_round(void);
static void stop_timer(void);

#define WAIT_FOR(CONDITION) do { \
	uint32_t timeout = 5000; /* a few ms */ \
	while (!((CONDITION) || timeout == 0)) { timeout--; } \
	if (!timeout) { \
		/* Timeout; typically means that the slave is unresponsive, */ \
	        /* probably because 12V power has been lost */ \
		stop_timer(); \
	} \
} while (0)

static void set_reg_pointer(uint8_t reg)
{
	// EV5
	WAIT_FOR(((I2C_SR1(i2c) & I2C_SR1_SB)
	        & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, 0x21, I2C_WRITE);
	// /EV5


	// EV6
	WAIT_FOR((I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);
	// /EV6

	// EV8_1
	i2c_send_data(i2c, reg);
	// /EV8_1
	
	// EV8_2
	WAIT_FOR((I2C_SR1(i2c) & ( I2C_SR1_BTF | I2C_SR1_TxE )));
	// /EV8_2
}

static void write_reg(uint8_t reg, uint8_t val)
{
	i2c_send_start(i2c);
	set_reg_pointer(reg);

	i2c_send_data(i2c, val);

	WAIT_FOR((I2C_SR1(i2c) & ( I2C_SR1_BTF | I2C_SR1_TxE )));

	i2c_send_stop(i2c);
}

static uint8_t read_reg(uint8_t reg)
{
	i2c_send_start(i2c);
	set_reg_pointer(reg);

	i2c_send_start(i2c);
	
	// EV5
	WAIT_FOR(((I2C_SR1(i2c) & I2C_SR1_SB)
	        & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, 0x21, I2C_READ);
	// /EV5

	// EV6_3
	WAIT_FOR((I2C_SR1(i2c) & I2C_SR1_ADDR));
	// Clear ACK
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	reg32 = I2C_SR2(i2c);
	// Set STOP
	i2c_send_stop(i2c);
	// /EV6_3
	
	// EV7
	WAIT_FOR((I2C_SR1(i2c) & I2C_SR1_RxNE));
	// /EV7

	return i2c_get_data(i2c);
}

static void init_i2c(void)
{
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

static inline void servo_out(uint16_t val)
{
	write_reg(0x14, val & 0xff);
	write_reg(0x15, (val >> 8) & 0xff);
}

static void init_timer(void)
{
	rcc_periph_clock_enable(RCC_TIM1);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_set_priority(NVIC_TIM1_CC_IRQ, 1);

	timer_reset(TIM1);

	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	/* 200kHz */
	timer_set_prescaler(TIM1, 360);
	timer_disable_preload(TIM1);
	timer_continuous_mode(TIM1);
	timer_set_period(TIM1, 50);

	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
	timer_enable_oc_preload(TIM1, TIM_OC1);
	timer_set_oc_value(TIM1, TIM_OC1, 0);

	/* Halt the ADC timer while debugging */
	DBGMCU_CR |= DBGMCU_CR_TIM1_STOP;


	timer_enable_irq(TIM1, TIM_DIER_CC1IE);
}

static void start_timer(void)
{
	setup_next_round();
	timer_set_counter(TIM1, 0);
	timer_enable_counter(TIM1);
}

static void stop_timer(void)
{
        timer_disable_counter(TIM1);
}

void tim1_cc_isr(void)
{
	led_set(LED_STATUS_RED);
	static uint8_t output_command_idx = 0;
	timer_clear_flag(TIM1, TIM_SR_CC1IF);

	// The timer count from 0 to N, inclusive, so we want to count up to N-1 to get N cycles
	timer_set_period(TIM1, output_commands[output_command_idx].duration - 1);
	servo_out(output_commands[output_command_idx].pin_state);

	output_command_idx++;
	if (output_command_idx == NUM_SERVOS*2)
	{
		output_command_idx = 0;
		setup_next_round();
	}
	led_clear(LED_STATUS_RED);
}

static uint16_t current_pin_state = 0x0000;
static inline uint16_t set_output_state(bool highlow, uint8_t output_idx)
{
	if (output_idx < 4)
	{
		if (highlow)
		{
			current_pin_state |= ( 1 << output_idx );
		}
		else
		{
			current_pin_state &= ~( 1 << output_idx );
		}
	}
	else
	{
		if (highlow)
		{
			current_pin_state |= ( 1 << (output_idx + 4) );
		}
		else
		{
			current_pin_state &= ~( 1 << (output_idx + 4) );
		}
	}

	// Always enable link for now
	return current_pin_state | 0x0040;
}

static uint16_t servo_ticks(uint8_t servo_idx)
{
	return CENTRE_DELAY + servo_positions[servo_idx];
}

int compare_servo_pulse_lengths(const void *a, const void *b)
{
	const indexed_pulse_length_t *pla = (const indexed_pulse_length_t *) a;
	const indexed_pulse_length_t *plb = (const indexed_pulse_length_t *) b;

	return (pla->pulse_length > plb->pulse_length) - (pla->pulse_length < plb->pulse_length);
}

static void setup_next_round(void)
{
	/* Copy current servo positions into 'sorted' array, ready for sorting */
	for (int i=0; i < NUM_SERVOS; i++)
	{
		sorted_servo_pulse_lengths[i].idx = i;
		sorted_servo_pulse_lengths[i].pulse_length = servo_ticks(i);
	}

	qsort(sorted_servo_pulse_lengths, NUM_SERVOS, sizeof(indexed_pulse_length_t), compare_servo_pulse_lengths);

	for (int i=0; i < NUM_SERVOS-1; i++)
	{
		output_commands[i].pin_state = set_output_state(true, sorted_servo_pulse_lengths[i].idx);
		output_commands[i].duration = START_DELAY;
	}

	output_commands[11].pin_state = set_output_state(true, sorted_servo_pulse_lengths[11].idx);
	output_commands[11].duration = sorted_servo_pulse_lengths[0].pulse_length - (START_DELAY*(NUM_SERVOS-1));

	for (int i=0; i < NUM_SERVOS-1; i++)
	{
		output_commands[NUM_SERVOS+i].pin_state = set_output_state(false, sorted_servo_pulse_lengths[i].idx);
		output_commands[NUM_SERVOS+i].duration = ( sorted_servo_pulse_lengths[i+1].pulse_length - sorted_servo_pulse_lengths[i].pulse_length + START_DELAY);
	}

	output_commands[23].pin_state = set_output_state(false, sorted_servo_pulse_lengths[11].idx);
	output_commands[23].duration = 3000;
}

void servo_init(void)
{
	init_timer();
	init_i2c();

	start_timer();
}

static const uint8_t actual_output_mapping[] = { 11, 10, 9, 8, 7, 6, 5, 4, 0, 1, 2, 3 };

void servo_set_pos(uint8_t idx, int16_t pos)
{
	if (idx > (NUM_SERVOS - 1 ))
		return;

	if (pos > POS_MAX) pos = POS_MAX;
	if (pos < POS_MIN) pos = POS_MIN;

	servo_positions[actual_output_mapping[idx]] = pos;
}

uint8_t servo_read()
{
	return read_reg(0x12);
}
