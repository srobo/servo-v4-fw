#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>

#include "led.h"
#include "servo.h"
#include "usart.h"
#include "battery.h"
#include "cdcacm.h"

#define delay(x) do { for (int i = 0; i < x * 1000; i++) \
                          __asm__("nop"); \
                    } while(0)

usbd_device *usbd_dev;

void timer_init(void) {
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(449);
	systick_interrupt_enable();
}

void init(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);

	led_init();
	usart_init();
	delay(20000);
	servo_init();
	timer_init();
	battery_init();
	usbd_dev = cdcacm_init();
}

void timer_start(void) {
	systick_counter_enable();
}

uint32_t servo_counter=0;
uint32_t pos=19;
uint32_t dir=1;
uint32_t dir_delay=0;

void sys_tick_handler(void) {
	if (servo_counter == 300)
	{
		servo_counter = 0;
		servo_out(0xFF0F);

		if (dir_delay == 30)
		{
			dir_delay = 0;
			pos+=dir;

			if (pos == 38) {
				dir = -1;
			} else if (pos == 19) {
				dir = 1;
			}
		}
		dir_delay++;
	}
	else if (servo_counter == pos)
	{
		servo_out(0x0040);
	}
	servo_counter++;

}

int main(void) {
	init();





	led_set( LED_STATUS_RED );


	//timer_start();

	uint16_t j = 0;

	for (int x = 0; x < 1000000; x++)
		cdcacm_poll(usbd_dev);

	servo_out(0x0040);

	while(1) {
		cdcacm_poll(usbd_dev);
//	uint8_t v = servo_read();
	volatile uint16_t b = battery_voltage();
	char foo[30];
		siprintf( foo, "%d\r\n", (b * 321)-1700 );
		cdcacm_send(usbd_dev, foo);
	/*	if (!(v & 0x10)) {
			led_set(LED_STATUS_RED);
		} else {
			led_clear(LED_STATUS_RED);
		}*/
		led_toggle( LED_STATUS_BLUE );
		led_toggle( LED_STATUS_RED );
		led_out(j);
		j++;
		if (j > 0xfff) j=0;
	//	delay(1000);
	}

	return 0;
}
