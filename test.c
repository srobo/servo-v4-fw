#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>

#include "led.h"
#include "servo.h"
#include "usart.h"
#include "battery.h"

#define DELAY 4000

#define delay(x) do { for (int i = 0; i < x * 1000; i++) \
                          __asm__("nop"); \
                    } while(0)

void init(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);

	led_init();
	usart_init();
//	servo_init();
//	battery_init();
}

int main(void) {
	init();

	while(1) {
		// Servo status LEDs
		delay(DELAY);
		for (int j=0; j<12; j++)
		{
			led_out(1<<j);
			delay(DELAY);
			led_out(0);
		}

	}
#if 0
		// 5V5 SMPS + I2C + GPIO expander
		servo_out(0x0000);
		delay(DELAY);

		// Link
		servo_out(0x0040);
		delay(DELAY);

		// Turn off SMPS and link
		servo_out(0x0020);
		delay(DELAY);

		servo_out(0xff2f);
		delay(DELAY);
		servo_out(0x0020);
		delay(DELAY);

		// Status LED
		led_set(LED_STATUS_BLUE);
		delay(DELAY);
		led_clear(LED_STATUS_BLUE);
		led_set(LED_STATUS_RED);
		delay(DELAY);
		led_clear(LED_STATUS_RED);
	}
#endif
	return 0;
}

// Configure application start address, put in section that'll be placed at
// the start of the non-bootloader firmware. The actual start address is
// libopencm3's reset handler, seeing how that's what copies .data into sram.
extern void *vector_table;
extern __attribute__((naked)) void reset_handler(void);
uint32_t app_start_address[2] __attribute__((section(".lolstartup"))) =
{
	(uint32_t)&vector_table,
	(uint32_t)&reset_handler,
};
