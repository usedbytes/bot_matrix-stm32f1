/*
 * Copyright (C) 2017 Brian Starkey <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"

#define SPI1_RX_DMA 2
#define SPI1_TX_DMA 3

#define SPI_N_PACKETS 3

#ifdef DEBUG
volatile char spi_trace[100];
volatile unsigned int spi_trace_idx;
void spi_dump_trace(void)
{
	unsigned int i;
	if (!spi_trace_idx)
		return;

	printf("S: %s\r\n", spi_trace);
	for (i = 0; i < sizeof(spi_trace); i++) {
		spi_trace[i] = '\0';
	}
	spi_trace_idx = 0;
	fflush(stdout);
}

static inline void __spi_trace(char c)
{
	if (spi_trace_idx < sizeof(spi_trace))
		spi_trace[spi_trace_idx++] = c;
}

static inline void __spi_trace_hex(char c)
{
	static const char *hex = "0123456789abcdef";
	__spi_trace(hex[((c & 0xF0) >> 4)]);
	__spi_trace(hex[(c & 0xF)]);
}
#else
void spi_dump_trace(void) { }
static inline void __spi_trace(char c) { (void)(c); }
static inline void __spi_trace_hex(char c) { (void)(c); }
#endif

struct spi_pl_packet_head {
	/* .next should be the first member so we can cast this to a packet */
	struct spi_pl_packet *next;
	struct spi_pl_packet *last;
	struct spi_pl_packet *current;
	struct spi_pl_packet *done;
	/*
	 * Used as a dummy source/destination for DMA when there's no
	 * packets to process
	 */
	uint8_t zero;
};

volatile bool spi_busy;
volatile uint8_t spi_lock = 0;
#define SPI_TX_LOCK BBIO_SRAM(&spi_lock, 0)
#define SPI_RX_LOCK BBIO_SRAM(&spi_lock, 1)
struct spi_pl_packet packet_pool[SPI_N_PACKETS];
struct spi_pl_packet_head packet_free = {
	.last = (struct spi_pl_packet *)&packet_free,
};
struct spi_pl_packet_head packet_inbox = {
	.last = (struct spi_pl_packet *)&packet_inbox,
};
struct spi_pl_packet_head packet_outbox = {
	.last = (struct spi_pl_packet *)&packet_outbox,
};

#define SPI_PACKET_DMA_SIZE (offsetof(struct spi_pl_packet, crc) - offsetof(struct spi_pl_packet, id))
static inline uint32_t spi_pl_packet_dma_addr(struct spi_pl_packet *pkt);
static void spi_add_last(struct spi_pl_packet_head *list, struct spi_pl_packet *pkt);
static struct spi_pl_packet *
spi_queue_next_packet(struct spi_pl_packet_head *list, uint32_t dma_channel);
static void spi_finish_transaction(void);

/*
 * SPI Transaction sequence.
 *
 * NSS ````\_________________________________________________________________
 *         :
 *         :    +--------+--------+--------+--------+--------+--------+
 * MISO    :    |   0    |   1    |   2    |   3    |  CRC   |   0     ----->
 *         :    +--------+--------+--------+--------+--------+--------+
 * MOSI    :    |   a    |   b    |   c    |   d    |  CRC   |   a     ----->
 *         :    +--------+--------+--------+--------+--------+--------+
 *         :      :     :           :              :        :
 *         :      :     v           :              :        `-----> RXF interrupt.
 *         :      :  RXF, RX DMA    :              :                Timing here is critical.
 *         :      :  reads 'a'      :              :                Before the start of next byte:
 *         :      v                 :              v                 - Drain SPI_DR (this is the CRC)
 *         :  TXE, TX DMA loads     :    RXF, RX DMA reads 'd'       - Enable TX DMA
 *         :  '1' to SPI_DR         :    raises TCI.                 - Read SPI_SR (for CRCERR)
 *         v                        :    Disable DMA, set up         - Disable SPI CRC
 *  NSS goes low, enable            :    next packet, and enable     - Clear SPI_RXCRC/SPI_TXCRC
 *  DMAs, TX DMA loads '0'          :    RXF IRQ, to fire at end     - Re-enable SPI CRC
 *  to SPI_DR                       :    of CRC                     Then, also enable RX DMA, disable
 *                                  v                               the RXF IRQ and process whatever
 *                              TXE, TX DMA loads '3'               packets we just sent/received.
 *                              to SPI_DR and raises TCI
 *                              Disable DMA and set it up
 *                              with the next packet
 *
 * There are three packets lists:
 *  - packet_free: Holds "free" packets, ready to be received into
 *  - packet_outbox: Holds packets which are queued for transmission
 *  - packet_inbox: Holds packets which have been received, ready for processing
 *
 * All manipulations of the packet lists need to hold locks to avoid racing with
 * the IRQ handlers.
 */

void exti4_isr(void)
{
	spi_busy = !gpio_get(GPIOA, GPIO4);

	EXTI_PR |= 1 << 4;

	if (spi_busy) {
		__spi_trace('A');
		__spi_trace_hex(SPI_SR(SPI1));

		spi_enable_tx_dma(SPI1);
		spi_enable_rx_dma(SPI1);

		__spi_trace('B');
		__spi_trace_hex(SPI_SR(SPI1));
	} else {
		__spi_trace('Z');
		__spi_trace_hex(SPI_SR(SPI1));

		SPI_RX_LOCK = 1;

		/* Disable and reset DMAs - keep the same addresses! */
		spi_finish_transaction();

		if (!packet_outbox.current) {
			packet_outbox.current = spi_queue_next_packet(&packet_outbox, SPI1_TX_DMA);
		}

		SPI_RX_LOCK = 0;
	}
}

/* SPI1_RX_DMA */
void dma1_channel2_isr(void)
{
	__spi_trace('R');

	dma_clear_interrupt_flags(DMA1, SPI1_RX_DMA, DMA_TEIF | DMA_HTIF | DMA_TCIF | DMA_GIF);

	spi_disable_rx_dma(SPI1);

	/* RXF will fire at the end of the CRC byte */
	spi_enable_rx_buffer_not_empty_interrupt(SPI1);

	/* Set up the next packet ready to go */
	packet_free.done = (void *)((volatile void*)packet_free.current);
	__spi_trace(packet_free.done ? '1' : '0');
	packet_free.current = spi_queue_next_packet(&packet_free, SPI1_RX_DMA);
	// TODO: Check for overrun here.

	/* Lock everything to protect our critical section in the SPI ISR */
	spi_lock = ~0;
}

/* SPI1_TX_DMA */
void dma1_channel3_isr(void)
{
	__spi_trace('T');

	dma_clear_interrupt_flags(DMA1, SPI1_TX_DMA, DMA_TEIF | DMA_HTIF | DMA_TCIF | DMA_GIF);

	spi_disable_tx_dma(SPI1);

	/* Set up the next packet ready to go */
	packet_outbox.done = (void *)((volatile void*)packet_outbox.current);
	__spi_trace(packet_outbox.done ? '1' : '0');
	packet_outbox.current = spi_queue_next_packet(&packet_outbox, SPI1_TX_DMA);
}

void spi1_isr(void)
{
	struct spi_pl_packet *pkt;
	uint8_t crc = SPI_DR(SPI1);
	uint8_t status = SPI_SR(SPI1);

	/* Critical section */
	spi_enable_tx_dma(SPI1);
	spi_disable_crc(SPI1);
	SPI_RXCRCR(SPI1) = 0;
	SPI_TXCRCR(SPI1) = 0;
	spi_enable_crc(SPI1);
	SPI_SR(SPI1) = 0;
	/* --- */

	__spi_trace('I');

	/* Less critical */
	spi_enable_rx_dma(SPI1);

	/* Relaxed */
	spi_disable_rx_buffer_not_empty_interrupt(SPI1);

	pkt = (void *)((volatile void*)packet_free.done);
	if (pkt) {
		__spi_trace('r');
		packet_free.done = NULL;

		pkt->crc = crc;
		if (status & SPI_SR_CRCERR) {
			pkt->flags |= SPI_FLAG_CRCERR;
		}
		spi_add_last(&packet_inbox, pkt);
	}

	pkt = (void *)((volatile void*)packet_outbox.done);
	if (pkt) {
		__spi_trace('t');
		packet_outbox.done = NULL;

		/*
		 * TODO: Is this too slow for here?
		 * Could defer to a pending "to free" list
		 */
		memset(pkt, 0, sizeof(*pkt));
		spi_add_last(&packet_free, pkt);
		if (!packet_free.current) {
			/* We just freed up a packet we can queue for RX. */
			packet_free.current = spi_queue_next_packet(&packet_free, SPI1_RX_DMA);
		}
	}

	/* Unlock all */
	spi_lock = 0;
}

static void spi_slave_init(uint32_t spidev)
{
	spi_reset(spidev);

	spi_set_dff_8bit(spidev);

	spi_set_clock_phase_0(spidev);
	spi_set_clock_polarity_0(spidev);

	spi_send_msb_first(spidev);

	spi_disable_software_slave_management(spidev);
	spi_disable_ss_output(spidev);

	spi_set_slave_mode(spidev);
}

void spi_slave_enable(uint32_t spidev)
{
	spi_enable(spidev);
}

void spi_slave_disable(uint32_t spidev)
{
	/* Wait until not busy */
	while (SPI_SR(spidev) & SPI_SR_BSY);
	spi_disable(spidev);
}

static inline uint32_t spi_pl_packet_dma_addr(struct spi_pl_packet *pkt)
{
	return (uint32_t)&(pkt->id);
}

static void spi_add_last(struct spi_pl_packet_head *list, struct spi_pl_packet *pkt)
{
	__spi_trace('l');
	list->last->next = pkt;
	list->last = pkt;
}

static struct spi_pl_packet *spi_dequeue_packet(struct spi_pl_packet_head *list)
{
	struct spi_pl_packet *pkt = list->next;
	if (!pkt) {
		return NULL;
	}

	list->next = pkt->next;
	if (!list->next)
		list->last = (struct spi_pl_packet *)list;
	pkt->next = NULL;

	return pkt;
}

static struct spi_pl_packet *
spi_queue_next_packet(struct spi_pl_packet_head *list, uint32_t dma_channel)
{
	struct spi_pl_packet *pkt = spi_dequeue_packet(list);
	dma_disable_channel(DMA1, dma_channel);

	if (pkt) {
		__spi_trace('Q');
		dma_set_memory_address(DMA1, dma_channel, spi_pl_packet_dma_addr(pkt));
		dma_enable_memory_increment_mode(DMA1, dma_channel);
	} else {
		__spi_trace('E');
		dma_set_memory_address(DMA1, dma_channel, (uint32_t)&list->zero);
		dma_disable_memory_increment_mode(DMA1, dma_channel);
	}

	dma_set_number_of_data(DMA1, dma_channel, SPI_PACKET_DMA_SIZE);
	dma_enable_channel(DMA1, dma_channel);

	return pkt;
}

static inline void __spi_lock(void)
{
	nvic_disable_irq(NVIC_DMA1_CHANNEL3_IRQ);
	nvic_disable_irq(NVIC_DMA1_CHANNEL2_IRQ);
	nvic_disable_irq(NVIC_EXTI4_IRQ);

	while (SPI_RX_LOCK);
}

static inline void __spi_unlock(void)
{
	nvic_enable_irq(NVIC_EXTI4_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
}

void spi_free_packet(struct spi_pl_packet *pkt)
{
	memset(pkt, 0, sizeof(*pkt));

	__spi_lock();

	spi_add_last(&packet_free, pkt);

	__spi_unlock();
}

struct spi_pl_packet *spi_alloc_packet(void)
{
	struct spi_pl_packet *pkt;

	__spi_lock();

	pkt = spi_dequeue_packet(&packet_free);

	__spi_unlock();

	return pkt;
}

struct spi_pl_packet *spi_receive_packet(void)
{
	struct spi_pl_packet *pkt;

	__spi_lock();

	pkt = spi_dequeue_packet(&packet_inbox);

	__spi_unlock();

	return pkt;
}

void spi_send_packet(struct spi_pl_packet *pkt)
{
	__spi_lock();

	spi_add_last(&packet_outbox, pkt);
	if (!spi_busy && !packet_outbox.current)
		packet_outbox.current =
			spi_queue_next_packet(&packet_outbox, SPI1_TX_DMA);

	__spi_unlock();
}

static void spi_finish_transaction(void)
{
	dma_disable_channel(DMA1, SPI1_TX_DMA);
	dma_set_number_of_data(DMA1, SPI1_TX_DMA, SPI_PACKET_DMA_SIZE);
	dma_enable_channel(DMA1, SPI1_TX_DMA);

	dma_disable_channel(DMA1, SPI1_RX_DMA);
	dma_set_number_of_data(DMA1, SPI1_RX_DMA, SPI_PACKET_DMA_SIZE);
	dma_enable_channel(DMA1, SPI1_RX_DMA);

	/* Reset the peripheral to discard the TX DR */
	spi_slave_init(SPI1);
	spi_enable_crc(SPI1);
	spi_slave_enable(SPI1);
}

static void spi_init_dma(void)
{
	dma_channel_reset(DMA1, SPI1_RX_DMA);
	dma_disable_channel(DMA1, SPI1_RX_DMA);
	dma_set_read_from_peripheral(DMA1, SPI1_RX_DMA);
	dma_set_memory_size(DMA1, SPI1_RX_DMA, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, SPI1_RX_DMA, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, SPI1_RX_DMA);
	dma_disable_peripheral_increment_mode(DMA1, SPI1_RX_DMA);
	dma_set_peripheral_address(DMA1, SPI1_RX_DMA, (uint32_t)&(SPI_DR(SPI1)));
	dma_enable_transfer_complete_interrupt(DMA1, SPI1_RX_DMA);
	dma_enable_transfer_error_interrupt(DMA1, SPI1_RX_DMA);

	dma_channel_reset(DMA1, SPI1_TX_DMA);
	dma_disable_channel(DMA1, SPI1_TX_DMA);
	dma_set_read_from_memory(DMA1, SPI1_TX_DMA);
	dma_set_memory_size(DMA1, SPI1_TX_DMA, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, SPI1_TX_DMA, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, SPI1_TX_DMA);
	dma_disable_peripheral_increment_mode(DMA1, SPI1_TX_DMA);
	dma_set_peripheral_address(DMA1, SPI1_TX_DMA, (uint32_t)&(SPI_DR(SPI1)));
	dma_enable_transfer_complete_interrupt(DMA1, SPI1_TX_DMA);
	dma_enable_transfer_error_interrupt(DMA1, SPI1_TX_DMA);
}

static void spi_init_packet_pool(void)
{
	unsigned int i;

	for (i = 0; i < (sizeof(packet_pool) / sizeof(*packet_pool)); i++) {
		struct spi_pl_packet *pkt = &packet_pool[i];
		spi_free_packet(pkt);
	}

	packet_outbox.current = spi_queue_next_packet(&packet_outbox, SPI1_TX_DMA);
	packet_free.current = spi_queue_next_packet(&packet_free, SPI1_RX_DMA);
}

void spi_dump_packet(const char *indent, struct spi_pl_packet *pkt)
{
	uint8_t *c = pkt->data;
	if (pkt) {
		printf("%s%p %d %d %d %02x\r\n", indent, pkt, pkt->id, pkt->dst,
				pkt->nparts, pkt->flags);
		printf("%s  ", indent);
		while (*c && (c < pkt->data + sizeof(pkt->data))) {
			printf("%02x ", *c);
			c++;
		}
		printf("\r\n");
		printf("%s crc: %02x\r\n", indent, pkt->crc);
		printf("%s next: %p\r\n", indent, pkt->next);
	} else {
		printf("(nil)\r\n");
	}
}

static void dump_list(struct spi_pl_packet_head *list, const char *name)
{
	struct spi_pl_packet *pkt;

	pkt = list->next;
	printf("%s (%p):\r\n", name, list);
	while (pkt) {
		spi_dump_packet(" ", pkt);
		pkt = pkt->next;
	}
	printf("Current:"); spi_dump_packet("  ", list->current);
	printf("Done:"); spi_dump_packet("  ", list->done);
	printf("Last: %p\r\n", list->last);
	printf("--\r\n");

}

void spi_dump_lists(void)
{
	dump_list(&packet_free, "packet_free");
	dump_list(&packet_outbox, "packet_outbox");
	dump_list(&packet_inbox, "packet_inbox");
}

void spi_init(void)
{
	spi_init_dma();
	spi_init_packet_pool();

	spi_slave_init(SPI1);
	spi_enable_crc(SPI1);

	nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);
	nvic_enable_irq(NVIC_SPI1_IRQ);

	exti_select_source(GPIO4, GPIOA);
	exti_set_trigger(GPIO4, EXTI_TRIGGER_BOTH);
	exti_enable_request(GPIO4);
	nvic_enable_irq(NVIC_EXTI4_IRQ);

	/* SPI1 GPIOs in slave mode */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
	              GPIO4 | GPIO5 | GPIO7);
}
