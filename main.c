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
#include <stdarg.h>

#include "hbridge.h"
#include "motor.h"
#include "pwm.h"
#include "spi.h"
#include "usb_cdc.h"

#include "systick.h"

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

static void setup_gpio(void) {
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((13 - 8) * 4) + 2));
	GPIOC_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((13 - 8) * 4));
}

static void ep0xfe_process_packet(struct spi_pl_packet *pkt)
{
	if ((pkt->type != 0xfe) || (pkt->flags & SPI_FLAG_ERROR))
		return;

	printf("Reset.\r\n");

	scb_reset_system();
}

#define EP_MOTORS 18

#define MESSAGE_PKT_TYPE 0xff
#define MESSAGE_LEVEL_INFO 10
struct message_pkt {
	uint8_t level;
	uint8_t n_args;
	uint8_t pad[2];
	// uint32_t args[]
	// char str[]
};

static void send_message(uint8_t level, const char *str, unsigned int n_args, va_list args)
{
	int ret;
	unsigned int i;
	struct message_pkt *pp;

	struct spi_pl_packet *pkt = spi_alloc_packet();
	if (!pkt) {
		return;
	}

	pkt->type = MESSAGE_PKT_TYPE;
	pp = (struct message_pkt *)pkt->data;
	pp->level = level;
	pp->n_args = n_args;

	/* First offset */
	ret = sizeof(*pp);

	for (i = 0; i < n_args; i++) {
		uint32_t arg = va_arg(args, uint32_t);

		ret = spi_packetise_stream(pkt, ret, (char *)&arg, sizeof(arg));
		if (ret < 0) {
			goto fail;
		}
	}

	ret = spi_packetise_stream(pkt, ret, str, strlen(str) + 1);
	if (ret < 0) {
		goto fail;
	}

	spi_send_packet(pkt);
	return;

fail:
	spi_free_packet(pkt);
}

static void send_printf(const char *str, unsigned int n_args, ...)
{
	va_list args;
	va_start(args, n_args);
	send_message(MESSAGE_LEVEL_INFO, str, n_args, args);
	va_end(args);
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

	motor_init();
	motor_enable_loop();
	motor_set_speed(HBRIDGE_A, DIRECTION_FWD, 0);
	motor_set_speed(HBRIDGE_B, DIRECTION_FWD, 0);

	setup_irq_priorities();

	struct spi_pl_packet *pkt;
	uint32_t time = msTicks;
	while (1) {
		while ((pkt = spi_receive_packet())) {
#ifdef DEBUG
			spi_dump_packet("", pkt);
#endif
			if (pkt->flags & SPI_FLAG_CRCERR) {
				printf("CRC error in packet id %d\r\n", pkt->id);
				spi_free_packet(pkt);
				continue;
			}
			switch (pkt->type) {
				case EP_MOTORS:
					motor_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				case 0xfe:
					ep0xfe_process_packet(pkt);
					spi_free_packet(pkt);
					break;
				default:
					// Bounce anything unknown
					spi_send_packet(pkt);
			}
		}

		if (cp) {
			printf("%s", dbg);
			cp = false;
		}

		if (msTicks - time >= 100) {
			time = msTicks;
			// Do something periodically...
		}
	}

	return 0;
}
