#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void gpio_setup(void)
{
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIOC_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((13 - 8) * 4) + 2));
	GPIOC_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((13 - 8) * 4));
}

int main(void)
{
	int i;

	gpio_setup();

	while (1) {
		GPIOC_BSRR = GPIO13;
		for (i = 0; i < 800000; i++)
			__asm__("nop");
		GPIOC_BRR = GPIO13;
		for (i = 0; i < 80000; i++)
			__asm__("nop");
	}

	return 0;
}
