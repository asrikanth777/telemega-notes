/*
 * Copyright © 2012 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <ao.h>

struct ao_spi_stm_info {
	uint8_t	miso_dma_index;
	uint8_t mosi_dma_index;
	struct stm_spi *stm_spi;
};

static uint8_t		ao_spi_mutex[STM_NUM_SPI];
static uint8_t		ao_spi_pin_config[STM_NUM_SPI];

static const struct ao_spi_stm_info ao_spi_stm_info[STM_NUM_SPI] = {
	{
		.miso_dma_index = STM_DMA_INDEX(STM_DMA_CHANNEL_SPI1_RX),
		.mosi_dma_index = STM_DMA_INDEX(STM_DMA_CHANNEL_SPI1_TX),
		&stm_spi1
	},
	{
		.miso_dma_index = STM_DMA_INDEX(STM_DMA_CHANNEL_SPI2_RX),
		.mosi_dma_index = STM_DMA_INDEX(STM_DMA_CHANNEL_SPI2_TX),
		&stm_spi2
	}
};

static uint8_t	spi_dev_null;

#if DEBUG
static struct {
	uint8_t	task;
	uint8_t	which;
	AO_TICK_TYPE tick;
	uint16_t len;
} spi_tasks[64];
static uint8_t	spi_task_index;

static void
validate_spi(struct stm_spi *stm_spi, int which, uint16_t len)
{
	uint32_t	sr = stm_spi->sr;

	if (stm_spi != &stm_spi2)
		return;
	spi_tasks[spi_task_index].task = ao_cur_task ? ao_cur_task->task_id : 0;
	spi_tasks[spi_task_index].which = which;
	spi_tasks[spi_task_index].tick = ao_time();
	spi_tasks[spi_task_index].len = len;
	spi_task_index = (spi_task_index + 1) & (63);
	if (sr & (1 << STM_SPI_SR_FRE))
		ao_panic(0x40 | 1);
	if (sr & (1 << STM_SPI_SR_BSY))
		ao_panic(0x40 | 2);
	if (sr & (1 << STM_SPI_SR_OVR))
		ao_panic(0x40 | 3);
	if (sr & (1 << STM_SPI_SR_MODF))
		ao_panic(0x40 | 4);
	if (sr & (1 << STM_SPI_SR_UDR))
		ao_panic(0x40 | 5);
	if ((sr & (1 << STM_SPI_SR_TXE)) == 0)
		ao_panic(0x40 | 6);
	if (sr & (1 << STM_SPI_SR_RXNE))
		ao_panic(0x40 | 7);
	if (which != 5 && which != 6 && which != 13)
		if (ao_cur_task->task_id != ao_spi_mutex[1])
			ao_panic(0x40 | 8);
}
#else
#define validate_spi(stm_spi, which, len) do { (void) (which); (void) (len); } while (0)
#endif

static void
ao_spi_set_dma_mosi(uint8_t id, const void *data, uint16_t len, uint32_t minc)
{
	struct stm_spi *stm_spi = ao_spi_stm_info[id].stm_spi;
	uint8_t	mosi_dma_index = ao_spi_stm_info[id].mosi_dma_index;

	ao_dma_set_transfer(mosi_dma_index,
			    &stm_spi->dr,
			    (void *) data,
			    len,
			    (0 << STM_DMA_CCR_MEM2MEM) |
			    (STM_DMA_CCR_PL_MEDIUM << STM_DMA_CCR_PL) |
			    (STM_DMA_CCR_MSIZE_8 << STM_DMA_CCR_MSIZE) |
			    (STM_DMA_CCR_PSIZE_8 << STM_DMA_CCR_PSIZE) |
			    (minc << STM_DMA_CCR_MINC) |
			    (0 << STM_DMA_CCR_PINC) |
			    (0 << STM_DMA_CCR_CIRC) |
			    (STM_DMA_CCR_DIR_MEM_TO_PER << STM_DMA_CCR_DIR));
}

static void
ao_spi_set_dma_miso(uint8_t id, void *data, uint16_t len, uint32_t minc)
{
	struct stm_spi *stm_spi = ao_spi_stm_info[id].stm_spi;
	uint8_t	miso_dma_index = ao_spi_stm_info[id].miso_dma_index;

	ao_dma_set_transfer(miso_dma_index,
			    &stm_spi->dr,
			    data,
			    len,
			    (0 << STM_DMA_CCR_MEM2MEM) |
			    (STM_DMA_CCR_PL_VERY_HIGH << STM_DMA_CCR_PL) |
			    (STM_DMA_CCR_MSIZE_8 << STM_DMA_CCR_MSIZE) |
			    (STM_DMA_CCR_PSIZE_8 << STM_DMA_CCR_PSIZE) |
			    (minc << STM_DMA_CCR_MINC) |
			    (0 << STM_DMA_CCR_PINC) |
			    (0 << STM_DMA_CCR_CIRC) |
			    (STM_DMA_CCR_DIR_PER_TO_MEM << STM_DMA_CCR_DIR));
}

static void
ao_spi_run(uint8_t id, uint8_t which, uint16_t len)
{
	struct stm_spi *stm_spi = ao_spi_stm_info[id].stm_spi;
	uint8_t	mosi_dma_index = ao_spi_stm_info[id].mosi_dma_index;
	uint8_t	miso_dma_index = ao_spi_stm_info[id].miso_dma_index;

	validate_spi(stm_spi, which, len);

	stm_spi->cr2 = ((0 << STM_SPI_CR2_TXEIE) |
			(0 << STM_SPI_CR2_RXNEIE) |
			(0 << STM_SPI_CR2_ERRIE) |
			(0 << STM_SPI_CR2_SSOE) |
			(1 << STM_SPI_CR2_TXDMAEN) |
			(1 << STM_SPI_CR2_RXDMAEN));

	ao_dma_start(miso_dma_index);
	ao_dma_start(mosi_dma_index);

	ao_arch_critical(
		while (!ao_dma_done[miso_dma_index])
			ao_sleep(&ao_dma_done[miso_dma_index]);
		);

	while ((stm_spi->sr & (1 << STM_SPI_SR_TXE)) == 0);
	while (stm_spi->sr & (1 << STM_SPI_SR_BSY));

	validate_spi(stm_spi, which+1, len);

	stm_spi->cr2 = 0;

	ao_dma_done_transfer(mosi_dma_index);
	ao_dma_done_transfer(miso_dma_index);
}

void
ao_spi_send(const void *block, uint16_t len, uint8_t spi_index)
{
	uint8_t id = AO_SPI_INDEX(spi_index);

	/* Set up the transmit DMA to deliver data */
	ao_spi_set_dma_mosi(id, block, len, 1);

	/* Set up the receive DMA -- when this is done, we know the SPI unit
	 * is idle. Without this, we'd have to poll waiting for the BSY bit to
	 * be cleared
	 */
	ao_spi_set_dma_miso(id, &spi_dev_null, len, 0);

	ao_spi_run(id, 1, len);
}

void
ao_spi_send_fixed(uint8_t value, uint16_t len, uint8_t spi_index)
{
	uint8_t id = AO_SPI_INDEX(spi_index);

	/* Set up the transmit DMA to deliver data */
	ao_spi_set_dma_mosi(id, &value, len, 0);

	/* Set up the receive DMA -- when this is done, we know the SPI unit
	 * is idle. Without this, we'd have to poll waiting for the BSY bit to
	 * be cleared
	 */
	ao_spi_set_dma_miso(id, &spi_dev_null, len, 0);

	ao_spi_run(id, 3, len);
}

void
ao_spi_start_bytes(uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);
	struct stm_spi	*stm_spi = ao_spi_stm_info[id].stm_spi;

	stm_spi->cr2 = ((0 << STM_SPI_CR2_TXEIE) |
			(0 << STM_SPI_CR2_RXNEIE) |
			(0 << STM_SPI_CR2_ERRIE) |
			(0 << STM_SPI_CR2_SSOE) |
			(0 << STM_SPI_CR2_TXDMAEN) |
			(0 << STM_SPI_CR2_RXDMAEN));
	validate_spi(stm_spi, 5, 0xffff);
}

void
ao_spi_stop_bytes(uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);
	struct stm_spi	*stm_spi = ao_spi_stm_info[id].stm_spi;

	while ((stm_spi->sr & (1 << STM_SPI_SR_TXE)) == 0)
		;
	while (stm_spi->sr & (1 << STM_SPI_SR_BSY))
		;
	/* Clear the OVR flag */
	(void) stm_spi->dr;
	(void) stm_spi->sr;
	validate_spi(stm_spi, 6, 0xffff);
	stm_spi->cr2 = 0;
}

void
ao_spi_send_sync(const void *block, uint16_t len, uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);
	const uint8_t	*b = block;
	struct stm_spi	*stm_spi = ao_spi_stm_info[id].stm_spi;

	stm_spi->cr2 = ((0 << STM_SPI_CR2_TXEIE) |
			(0 << STM_SPI_CR2_RXNEIE) |
			(0 << STM_SPI_CR2_ERRIE) |
			(0 << STM_SPI_CR2_SSOE) |
			(0 << STM_SPI_CR2_TXDMAEN) |
			(0 << STM_SPI_CR2_RXDMAEN));
	validate_spi(stm_spi, 7, len);
	while (len--) {
		while (!(stm_spi->sr & (1 << STM_SPI_SR_TXE)));
		stm_spi->dr = *b++;
	}
	while ((stm_spi->sr & (1 << STM_SPI_SR_TXE)) == 0)
		;
	while (stm_spi->sr & (1 << STM_SPI_SR_BSY))
		;
	/* Clear the OVR flag */
	(void) stm_spi->dr;
	(void) stm_spi->sr;
	validate_spi(stm_spi, 8, len);
}

void
ao_spi_recv(void *block, uint16_t len, uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);

	spi_dev_null = 0xff;

	/* Set up transmit DMA to make the SPI hardware actually run */
	ao_spi_set_dma_mosi(id, &spi_dev_null, len, 0);

	/* Set up the receive DMA to capture data */
	ao_spi_set_dma_miso(id, block, len, 1);

	ao_spi_run(id, 9, len);
}

void
ao_spi_duplex(const void *out, void *in, uint16_t len, uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);

	/* Set up transmit DMA to send data */
	ao_spi_set_dma_mosi(id, out, len, 1);

	/* Set up the receive DMA to capture data */
	ao_spi_set_dma_miso(id, in, len, 1);

	ao_spi_run(id, 11, len);
}

#define stm_spi_input_disable(gpio, pin) do {			\
		stm_gpio_conf(gpio, pin,			\
			      STM_GPIO_CR_MODE_INPUT,		\
			      STM_GPIO_CR_CNF_INPUT_FLOATING);	\
	} while(0)

#define stm_spi_output_disable(gpio, pin, mode) do {			\
		ao_gpio_set(gpio, pin, 1);				\
		stm_gpio_conf(gpio, pin,				\
			      mode,					\
			      STM_GPIO_CR_CNF_OUTPUT_PUSH_PULL);	\
	} while(0)

static void
ao_spi_disable_pin_config(uint8_t spi_pin_config)
{
	/* Disable current config
	 */
	switch (spi_pin_config) {
#if SPI_1_PA5_PA6_PA7
	case AO_SPI_1_PA5_PA6_PA7:
		stm_spi_output_disable(&stm_gpioa, 5, SPI_1_MODE_OUTPUT);
#ifndef SPI_1_PA6_DISABLE
		stm_spi_input_disable(&stm_gpioa, 6);
#endif
		stm_spi_output_disable(&stm_gpioa, 7, SPI_1_MODE_OUTPUT);
		break;
#endif
#if SPI_1_PB3_PB4_PB5
	case AO_SPI_1_PB3_PB4_PB5:
		stm_spi_output_disable(&stm_gpiob, 3, SPI_1_MODE_OUTPUT);
		stm_spi_input_disable(&stm_gpiob, 4);
		stm_spi_output_disable(&stm_gpiob, 5, SPI_1_MODE_OUTPUT);
		break;
#endif
#if SPI_2_PB13_PB14_PB15
	case AO_SPI_2_PB13_PB14_PB15:
		stm_spi_output_disable(&stm_gpiob, 13, SPI_2_MODE_OUTPUT);
		stm_spi_input_disable(&stm_gpiob, 14);
		stm_spi_output_disable(&stm_gpiob, 15, SPI_2_MODE_OUTPUT);
		break;
#endif
	}
}

#define stm_spi_input_enable(gpio, pin) do {			\
		stm_gpio_conf(gpio, pin,			\
			      STM_GPIO_CR_MODE_INPUT,		\
			      STM_GPIO_CR_CNF_INPUT_FLOATING);	\
	} while(0)

#define stm_spi_output_enable(gpio, pin, mode) do {			\
		stm_gpio_conf(gpio, pin,				\
			      mode,					\
			      STM_GPIO_CR_CNF_OUTPUT_AF_PUSH_PULL);	\
	} while(0)

static void
ao_spi_enable_pin_config(uint8_t spi_pin_config)
{
	/* Enable new config
	 */
	switch (spi_pin_config) {
#if SPI_1_PA5_PA6_PA7
	case AO_SPI_1_PA5_PA6_PA7:
		stm_set_afio_mapr(STM_AFIO_MAPR_SPI1_REMAP,
				  STM_AFIO_MAPR_SPI1_REMAP_PA4_PA5_PA6_PA7,
				  STM_AFIO_MAPR_SPI1_REMAP_MASK);
		stm_spi_output_enable(&stm_gpioa, 5, SPI_1_MODE_OUTPUT);
#ifndef SPI_1_PA6_DISABLE
		stm_spi_input_enable(&stm_gpioa, 6);
#endif
		stm_spi_output_enable(&stm_gpioa, 7, SPI_1_MODE_OUTPUT);
		break;
#endif
#if SPI_1_PB3_PB4_PB5
	case AO_SPI_1_PB3_PB4_PB5:
		stm_set_afio_mapr(STM_AFIO_MAPR_SPI1_REMAP,
				  STM_AFIO_MAPR_SPI1_REMAP_PA15_PB3_PB4_PB5,
				  STM_AFIO_MAPR_SPI1_REMAP_MASK);
		stm_spi_output_enable(&stm_gpiob, 3, SPI_1_MODE_OUTPUT);
		stm_spi_input_enable(&stm_gpiob, 4);
		stm_spi_output_enable(&stm_gpiob, 5, SPI_1_MODE_OUTPUT);
		break;
#endif
#if SPI_2_PB13_PB14_PB15
	case AO_SPI_2_PB13_PB14_PB15:
		stm_spi_output_enable(&stm_gpiob, 13, SPI_2_MODE_OUTPUT);
		stm_spi_input_enable(&stm_gpiob, 14);
		stm_spi_output_enable(&stm_gpiob, 15, SPI_2_MODE_OUTPUT);
		break;
#endif
	}
}

static void
ao_spi_config(uint8_t spi_index, uint32_t speed)
{
	uint8_t		spi_pin_config = AO_SPI_PIN_CONFIG(spi_index);
	uint8_t		id = AO_SPI_INDEX(spi_index);
	struct stm_spi	*stm_spi = ao_spi_stm_info[id].stm_spi;

	if (spi_pin_config != ao_spi_pin_config[id]) {

		/* Disable old config
		 */
		ao_spi_disable_pin_config(ao_spi_pin_config[id]);

		/* Enable new config
		 */
		ao_spi_enable_pin_config(spi_pin_config);

		/* Remember current config
		 */
		ao_spi_pin_config[id] = spi_pin_config;
	}

	/* Turn the SPI transceiver on and set the mode */
	stm_spi->cr1 = ((0 << STM_SPI_CR1_BIDIMODE) |		/* Three wire mode */
			(0 << STM_SPI_CR1_BIDIOE) |
			(0 << STM_SPI_CR1_CRCEN) |		/* CRC disabled */
			(0 << STM_SPI_CR1_CRCNEXT) |
			(0 << STM_SPI_CR1_DFF) |
			(0 << STM_SPI_CR1_RXONLY) |
			(1 << STM_SPI_CR1_SSM) |        	/* Software SS handling */
			(1 << STM_SPI_CR1_SSI) |		/*  ... */
			(0 << STM_SPI_CR1_LSBFIRST) |		/* Big endian */
			(1 << STM_SPI_CR1_SPE) |		/* Enable SPI unit */
			(speed << STM_SPI_CR1_BR) |		/* baud rate to pclk/4 */
			(1 << STM_SPI_CR1_MSTR) |
			(AO_SPI_CPOL(spi_index) << STM_SPI_CR1_CPOL) |	/* Format */
			(AO_SPI_CPHA(spi_index) << STM_SPI_CR1_CPHA));
	validate_spi(stm_spi, 13, 0);
}

uint8_t
ao_spi_try_get(uint8_t spi_index, uint32_t speed, uint8_t task_id)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);

	if (!ao_mutex_try(&ao_spi_mutex[id], task_id))
		return 0;
	ao_spi_config(spi_index, speed);
	return 1;
}

void
ao_spi_get(uint8_t spi_index, uint32_t speed)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);

	ao_mutex_get(&ao_spi_mutex[id]);
	ao_spi_config(spi_index, speed);
}

void
ao_spi_put(uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);
	struct stm_spi	*stm_spi = ao_spi_stm_info[id].stm_spi;

	stm_spi->cr1 = 0;
	ao_mutex_put(&ao_spi_mutex[id]);
}

void
ao_spi_put_pins(uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);

	ao_spi_disable_pin_config(ao_spi_pin_config[id]);
	ao_spi_pin_config[id] = AO_SPI_CONFIG_NONE;
	ao_spi_put(spi_index);
}

static void
ao_spi_channel_init(uint8_t spi_index)
{
	uint8_t		id = AO_SPI_INDEX(spi_index);
	struct stm_spi	*stm_spi = ao_spi_stm_info[id].stm_spi;

	ao_spi_disable_pin_config(AO_SPI_PIN_CONFIG(spi_index));

	stm_spi->cr1 = 0;
	stm_spi->cr2 = ((0 << STM_SPI_CR2_TXEIE) |
			(0 << STM_SPI_CR2_RXNEIE) |
			(0 << STM_SPI_CR2_ERRIE) |
			(0 << STM_SPI_CR2_SSOE) |
			(0 << STM_SPI_CR2_TXDMAEN) |
			(0 << STM_SPI_CR2_RXDMAEN));

	/* Clear any pending data and error flags */
	(void) stm_spi->dr;
	(void) stm_spi->sr;
}

#if DEBUG
void
ao_spi_dump_cmd(void)
{
	int s;

	for (s = 0; s < 64; s++) {
		int i = (spi_task_index + s) & 63;
		if (spi_tasks[i].which) {
			int t;
			const char *name = "(none)";
			for (t = 0; t < ao_num_tasks; t++)
				if (ao_tasks[t]->task_id == spi_tasks[i].task) {
					name = ao_tasks[t]->name;
					break;
				}
			printf("%2d: %5d task %2d which %2d len %5d %s\n",
			       s,
			       spi_tasks[i].tick,
			       spi_tasks[i].task,
			       spi_tasks[i].which,
			       spi_tasks[i].len,
			       name);
		}
	}
	for (s = 0; s < STM_NUM_SPI; s++) {
		struct stm_spi *spi = ao_spi_stm_info[s].stm_spi;

		printf("%1d: mutex %2d index %3d miso dma %3d mosi dma %3d",
		       s, ao_spi_mutex[s], ao_spi_index[s],
		       ao_spi_stm_info[s].miso_dma_index,
		       ao_spi_stm_info[s].mosi_dma_index);
		printf(" cr1 %04x cr2 %02x sr %03x\n",
		       spi->cr1, spi->cr2, spi->sr);
	}

}

static const struct ao_cmds ao_spi_cmds[] = {
	{ ao_spi_dump_cmd, 	"S\0Dump SPI status" },
	{ 0, NULL }
};
#endif

void
ao_spi_init(void)
{
#if HAS_SPI_1
# if SPI_1_PA5_PA6_PA7
	ao_enable_port(&stm_gpioa);
# endif
# if SPI_1_PB3_PB4_PB5
	ao_enable_port(&stm_gpiob);
# endif
	stm_rcc.apb2enr |= (1 << STM_RCC_APB2ENR_SPI1EN);
	ao_spi_pin_config[0] = AO_SPI_CONFIG_NONE;
	ao_spi_channel_init(0);
#endif

#if HAS_SPI_2
# if SPI_2_PB13_PB14_PB15
	ao_enable_port(&stm_gpiob);
# endif
	stm_rcc.apb1enr |= (1 << STM_RCC_APB1ENR_SPI2EN);
	ao_spi_pin_config[1] = AO_SPI_CONFIG_NONE;
	ao_spi_channel_init(1);
#endif
#if DEBUG
	ao_cmd_register(&ao_spi_cmds[0]);
#endif
}
