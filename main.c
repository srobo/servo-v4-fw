#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "usb.h"
#include "led.h"

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
