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
#include "period_counter.h"

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

struct period_counter pc = {
	.timer = TIM4,
};

void tim4_isr(void)
{
	period_counter_update(&pc);
}

/*
#define Kp (0x1)
#define Kd (0x0000)
volatile uint32_t tim3_ticks;
void tim3_isr(void)
{
	static uint16_t duty = 0x2000;
	static uint32_t setpoint = 4000;
	static int32_t err2 = 0;

	if (timer_get_flag(TIM3, TIM_SR_UIF)) {
		tim3_ticks++;
		timer_clear_flag(TIM3, TIM_SR_UIF);
	}

	if (!tim4_cc1_sem)
		return;
	int64_t err = tim4_cc1_cnt - setpoint;

	duty += ((err * Kp) / 1) + (((err - err2) * Kd) / 256);
	err2 = err;
	hbridge_set_duty(&hb, HBRIDGE_A, false, duty);
}
*/

static void pid_timer_init(uint32_t timer)
{
	timer_reset(timer);
	timer_slave_set_mode(timer, TIM_SMCR_SMS_OFF);
	timer_set_prescaler(timer, 71);
	timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_enable_preload(timer);
	timer_update_on_overflow(timer);
	timer_enable_update_event(timer);
	timer_generate_event(timer, TIM_EGR_UG);

	timer_enable_irq(timer, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	timer_set_period(timer, 10000);
	//timer_enable_counter(timer);
}


int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
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

	hbridge_init(&hb);
	hbridge_set_duty(&hb, HBRIDGE_A, false, 0x2000);

	period_counter_init(&pc);
	period_counter_enable(&pc, PC_CH1);
	//pid_timer_init(TIM3);

	spi_dump_lists();

	struct spi_pl_packet *pkt;
	uint32_t time = msTicks;
	while (1) {
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

		if (msTicks - time >= 100) {
			time = msTicks;

			printf("Count: %lu\r\n", period_counter_get(&pc, PC_CH1));
		}
	}

	return 0;
}
