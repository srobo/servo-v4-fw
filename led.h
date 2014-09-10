#ifndef __LED_H
#define __LED_H

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>

#define LED_STATUS_RED GPIO10
#define LED_STATUS_BLUE GPIO11

void led_init(void);

#define led_set(led) gpio_clear(GPIOB, led)
#define led_clear(led) gpio_set(GPIOB, led)
#define led_toggle(led) gpio_toggle(GPIOB, led)

void led_out(uint16_t);

#endif /* __LED_H */
