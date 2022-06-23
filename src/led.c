#include "led.h"

#include <libopencm3/stm32/gpio.h>

void led_init(void) {
    // set pins A0-7 & B10-15 to outputs
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, 0x00ff);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, 0xfc00);

    // set LEDs to off state
    gpio_clear(GPIOA, 0x00ff);  // active high LEDs
    gpio_set(GPIOB, 0xfc00);  // active low LEDs
}

void set_led(uint8_t idx) {
    if (idx < 8) {
        // active high LEDs
        gpio_set(GPIOA, 1 << idx);
    } else if (idx < 12) {
        // active low LEDs, starting from PB10
        gpio_clear(GPIOB, 1 << (idx + 4));
    } else if (idx == LED_STATUS_RED) {
        gpio_clear(GPIOB, 10);
    } else if (idx == LED_STATUS_BLUE) {
        gpio_clear(GPIOB, 11);
    }
}

void clear_led(uint8_t idx) {
    if (idx < 8) {
        // active high LEDs
        gpio_clear(GPIOA, 1 << idx);
    } else if (idx < 12) {
        // active low LEDs, starting from PB10
        gpio_set(GPIOB, 1 << (idx + 4));
    } else if (idx == LED_STATUS_RED) {
        gpio_set(GPIOB, 10);
    } else if (idx == LED_STATUS_BLUE) {
        gpio_set(GPIOB, 11);
    }
}
