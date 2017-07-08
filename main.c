#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <string.h>

#include "usb_cdc.h"
#include "spi.h"

#define TRACE() printf("%s:%d\r\n", __func__, __LINE__)

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

static void panic(void)
{
	while (1) {
		delay_ms(500);
		gpio_toggle(GPIOC, GPIO13);
	}
}
void hard_fault_handler(void)
{
	panic();
}
void bus_fault_handler(void)
{
	panic();
}
void usage_fault_handler(void)
{
	panic();
}

static void delay_us(uint32_t us)
{
	while (us--) {
		int i = 72;
		while (i--) {
			asm("nop");
		}
	}
}

static inline void blink(void)
{
	gpio_clear(GPIOC, GPIO13);
	delay_us(1);
	gpio_set(GPIOC, GPIO13);
	delay_us(2);
}

static inline void blink2(uint32_t ontime)
{
	gpio_clear(GPIOC, GPIO13);
	delay_us(ontime);
	gpio_set(GPIOC, GPIO13);
	delay_us(2);
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_DMA1);

	setup_systick();
	setup_gpio();

	usb_cdc_init();
	spi_init();
	spi_slave_enable(SPI1);

	gpio_set(GPIOC, GPIO13);

	while (!usb_usart_dtr());
	printf("\r\nStandard I/O Example.\r\n");

	spi_dump_lists();

	struct spi_pl_packet *pkt;
	while (1) {
		delay_ms(10);
		spi_dump_trace();
		while ((pkt = spi_receive_packet())) {
			spi_send_packet(pkt);
		}
	}

	return 0;
}
