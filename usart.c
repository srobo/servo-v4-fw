#include "usart.h"

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/usart.h"
#include "libopencm3/cm3/nvic.h"
#include "errno.h"

void usart_init() {
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, \
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	usart_set_baudrate(USART1, 19200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	usart_enable(USART1);
}

int usart_get_char(void) {
	return usart_recv_blocking(USART1);
}

int _write(int file, char *ptr, int len) {
	int i;

	if (file == 1) {
		for (i=0; i<len; i++)
			usart_send_blocking(USART1, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}
