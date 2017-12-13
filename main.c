#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <stdio.h>
#include <string.h>

#include "hbridge.h"
#include "pwm.h"
#include "usb_cdc.h"
#include "spi.h"
#include "period_counter.h"
#include "controller.h"

#include "systick.h"

#define TRACE() printf("%s:%d\r\n", __func__, __LINE__)

const struct gain gains[] = {
	{ FP_VAL(-1), 0, 0 },
	{ FP_VAL(-5), 0, 0 },
	{ FP_VAL(-10), 0, 0 },
	{ FP_VAL(-25), 0, 0 },
	{ FP_VAL(-100), 0, 0 },
	{ FP_VAL(-130), 0, 0 },
	{ FP_VAL(-200), 0, 0 },
};

const uint16_t gs_limits[] = {
	4000,
	3500,
	6000,
	10000,
	18000,
	35000,
	65535,
};

volatile uint16_t duty = 0x4000;

static void setup_irq_priorities(void)
{
	struct map_entry {
		uint32_t irqn;
		uint8_t  prio;
	} map[] = {
		{ NVIC_EXTI4_IRQ,           (0 << 6) | (0 << 4) },
		{ NVIC_TIM4_IRQ,            (1 << 6) | (0 << 4) },
		{ NVIC_USB_LP_CAN_RX0_IRQ,  (2 << 6) | (0 << 4) },
		{ NVIC_USB_WAKEUP_IRQ,      (2 << 6) | (1 << 4) },
		{ NVIC_TIM3_IRQ,            (3 << 6) | (0 << 4) },
		{ 0, 0 }
	}, *p = map;

	/*
	 * Priority ordering to try and make SPI reliable...
	 * stm32f103 only implements 4 bits of priority!
	 * Interrupt priority grouping (2 bits of pre-empt):
	 *   7:6 - pre-emption
	 *   5:4 - priority
	 *   3:0 - unused
	 */
	scb_set_priority_grouping(5 << 8);
	while (p->irqn) {
		nvic_set_priority(p->irqn, p->prio);
		p++;
	};
}

extern char dbg[256];
extern volatile bool cp;

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

struct controller controller = {

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

	return;

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

	uint16_t lduty = *((uint16_t*)(pkt->data + 2));

	printf("Set duty: %x\r\n", lduty);
	duty = lduty;

	hbridge_set_duty(&hb, pkt->data[0], pkt->data[1], lduty);
}

struct gain_set {
	int32_t Kc, Kd, Ki;
};

static void ep4_process_packet(struct spi_pl_packet *pkt)
{
	struct gain_set *gs;
	if ((pkt->type != 4) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	gs = (struct gain_set *)pkt->data;

	printf("Gains: %lx %lx %lx\r\n", gs->Kc, gs->Kd, gs->Ki);

	controller_set_gains(&controller, gs->Kc, gs->Kd, gs->Ki);
}

static void ep5_process_packet(struct spi_pl_packet *pkt)
{
	if ((pkt->type != 5) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	uint32_t set_point = *((uint32_t*)(pkt->data));

	printf("Setpoint: %lu\r\n", set_point);

	controller_set(&controller, set_point);
}

static void ep6_process_packet(struct spi_pl_packet *pkt)
{
	if ((pkt->type != 6) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	int32_t ilimit = *((int32_t*)(pkt->data));

	printf("ilimit: %li\r\n", ilimit);

	controller_set_ilimit(&controller, ilimit);
}

struct period_counter pc = {
	.timer = TIM4,
};

void tim4_isr(void)
{
	period_counter_update(&pc);
}

volatile uint16_t duty = 0x4000;
void tim3_isr(void)
{
	int32_t delta = 0;
	uint8_t gs_idx;
	uint32_t count = period_counter_get(&pc, PC_CH1);

	for (gs_idx = 0; gs_idx < sizeof(gains) / sizeof(gains[0]); gs_idx++) {
		if (duty < gs_limits[gs_idx])
			break;
	}

	timer_clear_flag(TIM3, TIM_SR_UIF);
	delta = controller_tick(&controller, count, gs_idx);
	if (!delta)
		return;

	if (delta > 0 && ((0x10000 - delta) < duty)) {
		duty = 0xffff;
	} else if ((delta < 0) && (-delta > duty)) {
		duty = 0;
	} else {
		duty += delta;
	}

	hbridge_set_duty(&hb, HBRIDGE_A, false, duty);

}

static void pid_timer_init(uint32_t timer)
{
	timer_reset(timer);
	timer_slave_set_mode(timer, TIM_SMCR_SMS_OFF);
	timer_set_prescaler(timer, 7100);
	timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_enable_preload(timer);
	timer_update_on_overflow(timer);
	timer_enable_update_event(timer);
	timer_generate_event(timer, TIM_EGR_UG);

	timer_enable_irq(timer, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	timer_set_period(timer, 500);
	timer_enable_counter(timer);
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

	controller_init(&controller, gains, sizeof(gains) / sizeof(gains[0]));
	controller_set(&controller, 1000);
	//controller_set_gains(&controller, -0x10000, 0, 0);

	period_counter_init(&pc);
	period_counter_enable(&pc, PC_CH1);
	pid_timer_init(TIM3);
	spi_dump_lists();

	setup_irq_priorities();

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
				case 4:
					ep4_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				case 5:
					ep5_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				case 6:
					ep6_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				default:
					spi_send_packet(pkt);
			}
		}

		if (cp) {
			printf("%s", dbg);
			cp = false;
			printf("Duty: %u\r\n", duty);
			printf("Count: %lu\r\n", period_counter_get(&pc, PC_CH1));
		}

		/*
		if (msTicks - time >= 100) {
			time = msTicks;

			printf("Count: %lu\r\n", period_counter_get(&pc, PC_CH1));
			printf("Duty: %u\r\n", duty);
		}
		*/
	}

	return 0;
}
