#include "led.h"

void led_init(void) {
	led_clear(LED_STATUS_RED);
	led_clear(LED_STATUS_BLUE);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, LED_STATUS_RED);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, LED_STATUS_BLUE);

	led_out(0x000);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, 0x00ff);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, 0xf000);
}

void led_out(uint16_t v) {
	gpio_set(GPIOA, 0xff & v);
	gpio_clear(GPIOA, (0xff & ~v));
	gpio_set(GPIOB, ( (0x0f00 & ~v) >> 8 ) << 12);
	gpio_clear(GPIOB, ( (0x0f00 & v) >> 8 ) << 12);
}
