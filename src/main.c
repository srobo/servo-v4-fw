#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "cdcacm.h"
#include "servo.h"
#include "led.h"
#include "systick.h"
#include "global_vars.h"
#include "i2c.h"

#define REENTER_BOOTLOADER_RENDEZVOUS	0x08001FFC

// ## Global variable defines ##
// externs in global_vars.h

bool detected_power_good = false;
int board_voltage_mv = 0;
int board_current_ma = 0;

bool current_sense_updated = true;

// ####

void init(void);
void jump_to_bootloader(void);

int main(void)
{
    init();

    set_led(LED_STATUS_BLUE);

    while (1) {
        usb_poll();
        // measure servo current in dead-time after all pulses
        if (!processing_servo_pulses && !current_sense_updated) {
            get_expander_status(I2C_EXPANDER_ADDR);
            measure_current_sense(CURRENT_SENSE_ADDR);
        }
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

    led_init();
    usb_init();
    servo_init();
    init_current_sense(CURRENT_SENSE_ADDR);
    systick_init();
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
