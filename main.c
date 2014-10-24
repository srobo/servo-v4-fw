#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "usb.h"
#include "led.h"
#include "servo.h"

#include "dfu-bootloader/usbdfu.h"

#define delay(x) do { for (int i = 0; i < x * 1000; i++) \
                          __asm__("nop"); \
                    } while(0)

void
init()
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);

	usb_init();
	led_init();
	// Defer this until insturcted to do so over USB. Prevents the
	// board from hanging if the other side of the isolation barrier
	// is unpowered.
	//servo_init();
}

void
jump_to_bootloader()
{

	// Because spoons can't be used as forks, we have to
	// actually wait for the usb peripheral to complete
	// it's acknowledgement to dfu_detach
	delay(20);
	// Now reset USB
	usb_deinit();
	// Disable any irqs that there are. XXX this is likely
	// to get out of sync.
	nvic_disable_irq(NVIC_ADC1_2_IRQ);
	nvic_disable_irq(NVIC_TIM2_IRQ);
	// Call back into bootloader
	(*(void (**)())(REENTER_BOOTLOADER_RENDEZVOUS))();
}

int main(void)
{
	init();

	led_set(LED_STATUS_BLUE);

	while(1)
	{
		usb_poll();
		if (re_enter_bootloader)
			jump_to_bootloader();
	}

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
