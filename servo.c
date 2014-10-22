#include "servo.h"
#include "led.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define NUM_SERVOS 12
#define POS_MAX 100
#define POS_MIN 100

#define START_DELAY 5
#define CENTRE_DELAY 300

static uint16_t servo_positions[NUM_SERVOS] = { 0 };

typedef struct
{
	uint16_t pin_state;
	uint16_t duration;
} output_command_t;

volatile static output_command_t output_commands[NUM_SERVOS*2] = { { 0, 50 } };

static uint32_t reg32 __attribute__((unused));
static uint32_t i2c = I2C1;

static void setup_next_round(void);

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


	timer_enable_irq(TIM1, TIM_DIER_CC1IE);
}

static void start_timer(void)
{
	setup_next_round();
	timer_set_counter(TIM1, 0);
	timer_enable_counter(TIM1);
}

void tim1_cc_isr(void)
{
	led_toggle(LED_STATUS_RED);
	static uint8_t output_command_idx = 0;
	timer_clear_flag(TIM1, TIM_SR_CC1IF);

	timer_set_period(TIM1, output_commands[output_command_idx].duration);
	servo_out(output_commands[output_command_idx].pin_state);

	output_command_idx++;
	if (output_command_idx == NUM_SERVOS*2)
	{
		output_command_idx = 0;
		setup_next_round();
	}
}

static void setup_next_round(void)
{
	output_commands[0].pin_state = 0x0001;
	output_commands[0].duration = 5;

	output_commands[1].pin_state = 0x0003;
	output_commands[1].duration = 5;

	output_commands[2].pin_state = 0x0007;
	output_commands[2].duration = 5;

	output_commands[3].pin_state = 0x000f;
	output_commands[3].duration = 5;

	output_commands[4].pin_state = 0x010f;
	output_commands[4].duration = 5;

	output_commands[5].pin_state = 0x030f;
	output_commands[5].duration = 5;

	output_commands[6].pin_state = 0x070f;
	output_commands[6].duration = 5;

	output_commands[7].pin_state = 0x0f0f;
	output_commands[7].duration = 5;

	output_commands[8].pin_state = 0x1f0f;
	output_commands[8].duration = 5;

	output_commands[9].pin_state = 0x3f0f;
	output_commands[9].duration = 5;

	output_commands[10].pin_state = 0x7f0f;
	output_commands[10].duration = 5;

	output_commands[11].pin_state = 0xff0f;
	output_commands[11].duration = 245;

	output_commands[12].pin_state = 0xff0e;
	output_commands[12].duration = 5;

	output_commands[13].pin_state = 0xff0c;
	output_commands[13].duration = 5;

	output_commands[14].pin_state = 0xff08;
	output_commands[14].duration = 5;

	output_commands[15].pin_state = 0xff00;
	output_commands[15].duration = 5;

	output_commands[16].pin_state = 0xfe00;
	output_commands[16].duration = 5;

	output_commands[17].pin_state = 0xfc00;
	output_commands[17].duration = 5;

	output_commands[18].pin_state = 0xf800;
	output_commands[18].duration = 5;

	output_commands[19].pin_state = 0xf000;
	output_commands[19].duration = 5;

	output_commands[20].pin_state = 0xe000;
	output_commands[20].duration = 5;

	output_commands[21].pin_state = 0xc000;
	output_commands[21].duration = 5;

	output_commands[22].pin_state = 0x8000;
	output_commands[22].duration = 5;

	output_commands[23].pin_state = 0x0000;
	output_commands[23].duration = 3645;
}

#if 0
	typedef enum
	{
		PULSE_START,
		PULSE_END
	} pulse_state;

	static pulse_state state = PULSE_START;
	static uint8_t output_idx = 0;
	static uint16_t next_delay = START_DELAY;

	timer_set_period(TIM1, next_delay);

	switch (state)
	{
		case PULSE_START:
			if (output_idx == NUM_SERVOS)
			{
				state = PULSE_END;
				output_idx = 0;
				next_delay = ( CENTRE_DELAY + servo_positions[output_idx] ) - ( ( NUM_SERVOS-1 ) * START_DELAY );
			}
			else
			{
				// Turn on output_idx
				led_toggle(LED_STATUS_RED);
			}
			break;
		case PULSE_END:
			if (output_idx == NUM_SERVOS)
			{
				state = PULSE_START;
				output_idx = 0;
				next_delay = 10; // Time between rounds
			}
			else
			{
				// Turn off output_idx
				next_delay = ( CENTRE_DELAY + servo_positions[output_idx] );
				led_toggle(LED_STATUS_BLUE);
			}
			break;
	}

	output_idx++;

	if (output_idx == NUM_SERVOS)
	{
		if (state == PULSE_START) state = PULSE_END;
		if (state == PULSE_END) state = PULSE_START;
	}
}

#endif

void servo_init(void)
{
	init_timer();
	init_i2c();

	start_timer();
}

void servo_set_pos(uint8_t idx, int16_t pos)
{
	if (idx > (NUM_SERVOS - 1 ))
		return;

	if (pos > POS_MAX) pos = POS_MAX;
	if (pos < POS_MIN) pos = POS_MIN;

	servo_positions[idx] = pos;
}

uint8_t servo_read()
{
	return read_reg(0x12);
}
