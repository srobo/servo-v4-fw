#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "cdcacm.h"

#define REENTER_BOOTLOADER_RENDEZVOUS	0x08001FFC

void init(void);
void jump_to_bootloader(void);

int main(void)
{
    init();

    // enable status LED
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
    gpio_clear(GPIOB, GPIO11);

    while (1) {
        usb_poll();
        if (re_enter_bootloader) {
            jump_to_bootloader();
        }
    }
}

void init(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);

    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

    usb_init();
}

void jump_to_bootloader(void)
{
    // Actually wait for the usb peripheral to complete
    // it's acknowledgement to dfu_detach
    delay(20);
    // Now reset USB
    usb_deinit();
    // Call back into bootloader
    (*(void (**)())(REENTER_BOOTLOADER_RENDEZVOUS))();
}

// Configure application start address, put in section that'll be placed at
// the start of the non-bootloader firmware. The actual start address is
// libopencm3's reset handler, this copies .data into sram.
extern void *vector_table;
extern __attribute__((naked)) void reset_handler(void);
uint32_t app_start_address[3] __attribute__((section(".startup"))) =
{
    (uint32_t)&vector_table,
    (uint32_t)&reset_handler,
    (uint32_t)0x00000000,  // CRC checksum of the compiled binary
};
