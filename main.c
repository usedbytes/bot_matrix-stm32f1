#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>

#include "pwm.h"
#include "usb_cdc.h"

static void setup_gpio(void) {
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((13 - 8) * 4) + 2));
	GPIOC_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((13 - 8) * 4));
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_AFIO);

	setup_gpio();

	usb_cdc_init();

	pwm_timer_init(TIM2, 200000);
	pwm_timer_enable(TIM2);

	pwm_channel_set_duty(TIM2, TIM_OC1, 0x8000);
	pwm_channel_enable(TIM2, TIM_OC1);
	pwm_channel_set_duty(TIM2, TIM_OC2, 0x4000);
	pwm_channel_enable(TIM2, TIM_OC2);
	pwm_channel_set_duty(TIM2, TIM_OC3, 0x2000);
	pwm_channel_enable(TIM2, TIM_OC3);
	pwm_channel_set_duty(TIM2, TIM_OC4, 0x1000);
	pwm_channel_enable(TIM2, TIM_OC4);

	while (!usb_usart_dtr());

	printf("\r\nPWM Test Application\r\n");

	while (1) {
		int duty = 0;
		char local_buf[32];

		while (1) {
			printf("Enter the duty cycle (0-65535) : ");
			fflush(stdout);
			fgets(local_buf, 32, stdin);
			duty = atoi(local_buf);
			if ((duty < 0) || (duty > 65535)) {
				printf("Error: expected 0 <= duty <= 65535\r\n");
			} else {
				break;
			}
		}

		printf("Setting duty %d\r\n", duty);
		pwm_channel_set_duty(TIM2, TIM_OC1, duty);
	}
	return 0;
}
