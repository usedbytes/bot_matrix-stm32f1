/*
 * Copyright (C) 2018 Brian Starkey <stark3y@gmail.com>
 *
 * Portions based on libopencm3 core code
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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <errno.h>

#include "i2c.h"
#include "log.h"
#include "util.h"

// FIXME: Horrible mix of generic and non-generic code
static const uint32_t dev = I2C2;

static void i2c_init_dma(void)
{
#define I2C2_RX_DMA 5
	dma_channel_reset(DMA1, I2C2_RX_DMA);
	dma_disable_channel(DMA1, I2C2_RX_DMA);
	dma_set_read_from_peripheral(DMA1, I2C2_RX_DMA);
	dma_set_memory_size(DMA1, I2C2_RX_DMA, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, I2C2_RX_DMA, DMA_CCR_PSIZE_8BIT);
	dma_enable_transfer_complete_interrupt(DMA1, I2C2_RX_DMA);
	dma_enable_memory_increment_mode(DMA1, I2C2_RX_DMA);
	dma_disable_peripheral_increment_mode(DMA1, I2C2_RX_DMA);
	dma_set_peripheral_address(DMA1, I2C2_RX_DMA, (uint32_t)&(I2C_DR(dev)));
}

/*
 * From the libopencm3-examples i2c_stts75_sensor example.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 */
void i2c_init(void) {
	/* Enable clocks for I2C2 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C2); rcc_periph_clock_enable(RCC_AFIO);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C2_SCL | GPIO_I2C2_SDA);

	i2c_reset(I2C2);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C2);

	/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);

	/* 400KHz - I2C Fast Mode */
	i2c_set_fast_mode(I2C2);

	/*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
	i2c_set_ccr(I2C2, 0x1e);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
	 * Incremented by 1 -> 11.
	 */
	i2c_set_trise(I2C2, 0x0b);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C2);

	i2c_init_dma();
}

/*
 * There's a bug with the analogue filters, this is the workaround suggested
 * in the errata document. The i2c_init() at the end is meant to be only
 * a SWRST, but that doesn't seem to work.
 */
static void i2c_errata_2_13_7_reset(void)
{
	uint16_t idr;

	i2c_peripheral_disable(dev);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_OPENDRAIN,
		      GPIO_I2C2_SCL | GPIO_I2C2_SDA);

	GPIO_ODR(GPIOB) |= GPIO_I2C2_SCL | GPIO_I2C2_SDA;
	delay_us(1);
	idr = gpio_get(GPIOB, GPIO_I2C2_SCL | GPIO_I2C2_SDA);
	if ((idr & (GPIO_I2C2_SCL | GPIO_I2C2_SDA)) != (GPIO_I2C2_SCL | GPIO_I2C2_SDA)) {
		log_err("Didn't set high. idr: %08lx\r\n", (uint32_t)idr);
	}

	GPIO_ODR(GPIOB) &= ~(GPIO_I2C2_SDA);
	delay_us(1);
	idr = gpio_get(GPIOB, GPIO_I2C2_SDA);
	if (idr & GPIO_I2C2_SDA) {
		log_err("Didn't set SDA low. idr: %08lx\r\n", (uint32_t)idr);
	}

	GPIO_ODR(GPIOB) &= ~(GPIO_I2C2_SCL);
	delay_us(1);
	idr = gpio_get(GPIOB, GPIO_I2C2_SCL);
	if (idr & GPIO_I2C2_SCL) {
		log_err("Didn't set SCL low. idr: %08lx\r\n", (uint32_t)idr);
	}

	GPIO_ODR(GPIOB) |= GPIO_I2C2_SDA;
	delay_us(1);
	idr = gpio_get(GPIOB, GPIO_I2C2_SDA);
	if (!(idr & GPIO_I2C2_SDA)) {
		log_err("Didn't set SDA high. idr: %08lx\r\n", (uint32_t)idr);
	}

	GPIO_ODR(GPIOB) |= GPIO_I2C2_SCL;
	delay_us(1);
	idr = gpio_get(GPIOB, GPIO_I2C2_SCL);
	if (!(idr & GPIO_I2C2_SCL)) {
		log_err("Didn't set SCL high. idr: %08lx\r\n", (uint32_t)idr);
	}

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C2_SCL | GPIO_I2C2_SDA);

	i2c_init();
}

void i2c_reset_bus(void)
{
	/*
	 * This full reset procedure shouldn't be needed for all bus errors -
	 * but it does seem to be effective at recovering from errors.
	 */
	i2c_errata_2_13_7_reset();

}

static uint32_t i2c_wait_for(uint32_t bit)
{
	int i = 0;
	uint32_t sr;
	while (i++ < 1000) {
		sr = I2C_SR1(dev);
		if (sr & bit) {
			I2C_SR1(dev) = 0;
			return 0;
		}
		if (sr & (I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR)) {
			I2C_SR1(dev) = 0;
			return sr;
		};
		delay_us(1);
	}

	I2C_SR1(dev) = 0;
	return ~bit;
}

static int i2c_wait_idle(void)
{
	int i = 0;
	uint32_t sr;
	while (i++ < 1000) {
		sr = I2C_SR2(dev);
		if (!(sr & I2C_SR2_BUSY)) {
			return 0;
		}
		delay_us(1);
	}

	return -EBUSY;
}

int i2c_write(uint8_t addr_7b, uint8_t reg, uint8_t *data, unsigned int len)
{
	int ret;

	ret = i2c_wait_idle();
	if (ret) {
		return ret;
	}

	i2c_send_start(dev);

	ret = i2c_wait_for(I2C_SR1_SB);
	if (ret)
		goto done;

	//while ((I2C_SR2(dev) & (I2C_SR2_MSL | I2C_SR2_BUSY)));

	/* Send destination address. */
	i2c_send_7bit_address(dev, addr_7b, I2C_WRITE);
	ret = i2c_wait_for(I2C_SR1_ADDR);
	if (ret)
		goto done;
	(void)I2C_SR2(dev);

	i2c_send_data(dev, reg);
	ret = i2c_wait_for(I2C_SR1_BTF);
	if (ret)
		goto done;

	for (unsigned int i = 0; i < len; i++) {
		i2c_send_data(dev, data[i]);
		ret = i2c_wait_for(I2C_SR1_BTF);
		if (ret)
			goto done;
	}

done:
	i2c_send_stop(dev);

	return ret;
}

int i2c_read_byte(uint8_t addr_7b, uint8_t reg, uint8_t *data)
{
	int ret;

	ret = i2c_write(addr_7b, reg, NULL, 0);
	if (ret) {
		return ret;
	}

	ret = i2c_wait_idle();
	if (ret) {
		return ret;
	}

	i2c_send_start(dev);

	ret = i2c_wait_for(I2C_SR1_SB);
	if (ret) {
		return ret;
	}

	/*
	 * The following bizarre series of events is what's specified in
	 * the reference manual (Method 2, single byte)
	 */
	i2c_send_7bit_address(dev, addr_7b, I2C_READ);
	i2c_disable_ack(dev);
	ret = i2c_wait_for(I2C_SR1_ADDR);
	if (ret) {
		i2c_send_stop(dev);
		return ret;
	}
	(void)I2C_SR2(dev);

	i2c_send_stop(dev);
	ret = i2c_wait_for(I2C_SR1_RxNE);
	if (ret)
		return ret;
	*data = i2c_get_data(dev);

	return 0;
}

int i2c_detect(uint8_t addr_7b)
{
	int ret;
	int present = 0;
	uint32_t wait;

	ret = i2c_wait_idle();
	if (ret) {
		return ret;
	}

	i2c_send_start(dev);

	ret = i2c_wait_for(I2C_SR1_SB);
	if (ret) {
		return ret;
	}

	//while ((I2C_SR2(dev) & (I2C_SR2_MSL | I2C_SR2_BUSY)));

	/* Send destination address. */
	i2c_send_7bit_address(dev, addr_7b, I2C_WRITE);
	wait = i2c_wait_for(I2C_SR1_ADDR);
	if (!wait) {
		present = 1;
	} else if (wait == (uint32_t)(~I2C_SR1_ADDR)) {
		present = 0;
	}
	(void)I2C_SR2(dev);

	i2c_send_stop(dev);

	return ret ? ret : (!!present);
}

/*
 * For more than 1 byte, use DMA.
 * Overhead of setup could be a bit much, but it avoids needing to do the
 * craziness specified in the RM, and also is the recommended workaround for
 * limitation 2.13.1
 */
static int i2c_read_dma(uint8_t addr_7b, uint8_t reg, uint8_t *data, unsigned int len)
{
	int ret;

	ret = i2c_write(addr_7b, reg, NULL, 0);
	if (ret) {
		return ret;
	}

	ret = i2c_wait_idle();
	if (ret) {
		return ret;
	}

	i2c_send_start(dev);

	ret = i2c_wait_for(I2C_SR1_SB);
	if (ret) {
		return ret;
	}

	i2c_send_7bit_address(dev, addr_7b, I2C_READ);

	i2c_enable_ack(dev);
	dma_disable_channel(DMA1, I2C2_RX_DMA);
	dma_set_memory_address(DMA1, I2C2_RX_DMA, (uint32_t)data);
	dma_set_number_of_data(DMA1, I2C2_RX_DMA, len);
	dma_enable_channel(DMA1, I2C2_RX_DMA);
	i2c_set_dma_last_transfer(dev);
	i2c_enable_dma(dev);

	ret = i2c_wait_for(I2C_SR1_ADDR);
	if (ret)
		goto finish;
	(void)I2C_SR2(dev);

	while (!dma_get_interrupt_flag(DMA1, I2C2_RX_DMA, DMA_TCIF));
	dma_clear_interrupt_flags(DMA1, I2C2_RX_DMA, DMA_TCIF);

finish:
	i2c_disable_ack(dev);
	i2c_disable_dma(dev);
	dma_disable_channel(DMA1, I2C2_RX_DMA);
	i2c_send_stop(dev);

	return ret;
}

int i2c_read(uint8_t addr_7b, uint8_t reg, uint8_t *data, unsigned int len)
{
	if (!len) {
		return -ENODATA;
	}

	if (len == 1) {
		return i2c_read_byte(addr_7b, reg, data);
	}

	return i2c_read_dma(addr_7b, reg, data, len);
}

int i2c_write_byte(uint8_t addr_7b, uint8_t reg, uint8_t data)
{
	return i2c_write(addr_7b, reg, &data, 1);
}
