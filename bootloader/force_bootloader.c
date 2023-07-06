#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define delay(x) do { for (int i = 0; i < x * 1000; i++) \
                          __asm__("nop"); \
                    } while(0)

bool force_bootloader(void)
{
	/* Assume that the TX and RX pins are shorted, check otherwise */
	bool enter_bootloader = true;

	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	              GPIO_CNF_INPUT_PULL_UPDOWN, GPIO10);

	/* Pull down */
	gpio_clear(GPIOA, GPIO10);
	
	gpio_clear(GPIOA, GPIO9);
	delay(10);
	if (gpio_get(GPIOA, GPIO10))
	{
		enter_bootloader = false;
		goto cleanup;
	}

	gpio_set(GPIOA, GPIO9);
	delay(10);
	if (!gpio_get(GPIOA, GPIO10))
	{
		enter_bootloader = false;
		goto cleanup;
	}

	/* Pull up */
	gpio_set(GPIOA, GPIO10);

	gpio_clear(GPIOA, GPIO9);
	delay(10);
	if (gpio_get(GPIOA, GPIO10))
	{
		enter_bootloader = false;
		goto cleanup;
	}

	gpio_set(GPIOA, GPIO9);
	delay(10);
	if (!gpio_get(GPIOA, GPIO10))
	{
		enter_bootloader = false;
		goto cleanup;
	}
	

cleanup:
	/* Set pins back to default */
	gpio_clear(GPIOA, GPIO9);
	gpio_clear(GPIOA, GPIO10);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	              GPIO_CNF_INPUT_FLOAT, GPIO9);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	              GPIO_CNF_INPUT_FLOAT, GPIO10);

	return enter_bootloader;
}
