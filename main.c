#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <string.h>

#include "hbridge.h"
#include "pwm.h"
#include "usb_cdc.h"
#include "spi.h"

#include "systick.h"

#define TRACE() printf("%s:%d\r\n", __func__, __LINE__)

struct hbridge hb = {
	.timer = TIM2,
	.a = {
		.ch1 = TIM_OC1,
		.ch2 = TIM_OC2,
	},
	.b = {
		.ch1 = TIM_OC3,
		.ch2 = TIM_OC4,
	},
};

static void setup_gpio(void) {
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((13 - 8) * 4) + 2));
	GPIOC_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((13 - 8) * 4));
}

static void ep1_process_packet(struct spi_pl_packet *pkt)
{
	if ((pkt->type != 1) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	if (pkt->data[0])
		gpio_clear(GPIOC, GPIO13);
	else
		gpio_set(GPIOC, GPIO13);
}

static void ep0_process_packet(struct spi_pl_packet *pkt)
{
	static char str[256] = { 0 };

	if ((pkt->type != 0) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	strncat(str, (char *)pkt->data, SPI_PACKET_DATA_LEN);
	if (!pkt->nparts) {
		printf("%s\r\n", str);
		str[0] = '\0';
	}
}

static void ep2_process_packet(struct spi_pl_packet *pkt)
{
	if ((pkt->type != 2) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	uint32_t freq = *((uint32_t*)pkt->data);

	hbridge_set_freq(&hb, freq);
}

static void ep3_process_packet(struct spi_pl_packet *pkt)
{
	if ((pkt->type != 3) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	uint16_t duty = *((uint16_t*)(pkt->data + 2));

	hbridge_set_duty(&hb, pkt->data[0], pkt->data[1], duty);
}

volatile uint32_t tim4_cc1_cnt;
void tim4_isr(void)
{
	static uint16_t cc1_ovf;
	static uint16_t cc1_last;

	if (timer_get_flag(TIM4, TIM_SR_UIF)) {
		cc1_ovf++;
		timer_clear_flag(TIM4, TIM_SR_UIF);
	}

	if (timer_get_flag(TIM4, TIM_SR_CC1IF)) {
		uint16_t cc1 = TIM_CCR1(TIM4);
		tim4_cc1_cnt = ((cc1_ovf << 16) + cc1) - cc1_last;
		cc1_ovf = 0;
		cc1_last = cc1;
		timer_clear_flag(TIM4, TIM_SR_CC1IF);
	}
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM4);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_DMA1);

	systick_init();
	setup_gpio();

	usb_cdc_init();
	spi_init();
	spi_slave_enable(SPI1);

	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      GPIO14 | GPIO15);
	gpio_set(GPIOC, GPIO14);
	gpio_clear(GPIOC, GPIO15);

	gpio_set(GPIOC, GPIO13);

	while (!usb_usart_dtr());
	printf("\r\nStandard I/O Example.\r\n");

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      GPIO_TIM4_CH1 | GPIO_TIM4_CH2);
	timer_reset(TIM4);
	timer_slave_set_mode(TIM4, TIM_SMCR_SMS_OFF);
	timer_set_prescaler(TIM4, 71);
	timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_enable_preload(TIM4);
	timer_update_on_overflow(TIM4);
	timer_enable_update_event(TIM4);
	timer_generate_event(TIM4, TIM_EGR_UG);

	timer_ic_set_input(TIM4, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM4, TIM_IC2, TIM_IC_IN_TI2);

	timer_ic_enable(TIM4, TIM_IC1);

	timer_enable_irq(TIM4, TIM_DIER_UIE | TIM_DIER_CC1IE);

	nvic_enable_irq(NVIC_TIM4_IRQ);
	timer_enable_counter(TIM4);

	hbridge_init(&hb);
	hbridge_set_duty(&hb, HBRIDGE_A, false, 0x1800);

	spi_dump_lists();

	struct spi_pl_packet *pkt;
	uint32_t time = msTicks;
	while (1) {
		delay_ms(10);
		spi_dump_trace();
		while ((pkt = spi_receive_packet())) {
			if (pkt->flags & SPI_FLAG_CRCERR) {
				printf("CRC error in packet id %d\r\n", pkt->id);
			}
			switch (pkt->type) {
				case 0:
					ep0_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				case 1:
					ep1_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				case 2:
					ep2_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				case 3:
					ep3_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				default:
					spi_send_packet(pkt);
			}
		}

		if (msTicks - time >= 500) {
			time = msTicks;

			printf("Count: %lu\r\n", tim4_cc1_cnt);
		}
	}

	return 0;
}
