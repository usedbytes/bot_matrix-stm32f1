#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>

#include "usb_cdc.h"

static void setup_gpio(void) {
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((13 - 8) * 4) + 2));
	GPIOC_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((13 - 8) * 4));
}

volatile uint32_t msTicks;

void sys_tick_handler(void)
{
	msTicks++;
}

static void setup_systick(void)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(8999);
	systick_interrupt_enable();
	systick_counter_enable();
}

static void delay_ms(uint32_t ms)
{
	uint32_t end = msTicks + ms;
	if (end < msTicks) {
		while (msTicks <= 0xffffffff);
	}
	while (msTicks < end);
}

int main(void)
{
	char buf[16];

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOC);

	setup_gpio();
	setup_systick();

	usb_cdc_init();

	while (!usb_usart_dtr());

	int i, j;

	printf("\r\nStandard I/O Example.\r\n");

	/* Blink the LED (PC12) on the board with every transmitted byte. */
	while (1) {
		int delay = 0;
		char local_buf[32];

		gpio_toggle(GPIOC, GPIO13);	/* LED on/off */
		do {
			printf("Enter the delay constant for blink : ");
			fflush(stdout);
			fgets(local_buf, 32, stdin);
			delay = atoi(local_buf);
			if (delay <= 0) {
				printf("Error: expected a delay > 0\r\n");
			}
		} while (delay <= 0);

		printf("Blinking with a delay of %d\r\n", delay);
		for (j = 0; j < 10; j++) {
			gpio_toggle(GPIOC, GPIO13);
			delay_ms(delay);
		}
	}
	return 0;
}
