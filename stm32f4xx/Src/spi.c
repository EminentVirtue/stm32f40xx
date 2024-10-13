/*
 * spi.c
 *
 *  Created on: Jun 12, 2024
 *      Author: Andrew Streng
 */
#include "spi.h"
#include "rcc.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

/* Private functions */
static void tx_irq_handle(spi_handle *handle);
static int spi_ta_verify(spi_handle *handle, spi_transfer *transfer);
static int spi_set_tx_dff(stm32_spi *spi, u32 format);
void spi_dma_config(spi_handle *handle,struct dma_channel *dma_chan, enum dma_transfer_direction direction);


/* DMA callbacks */
static void spi_dma_tx_cb(void *data);
static void spi_dma_rx_cb(void *data);

/**
 * spi_clock_control - Enables or disabled the SPI peripheral clock according to @enabled
 *
 * @spi: pointer to the SPI device
 * @enabled: true/false for clock enabled/disabled
*/
int spi_clock_control(stm32_spi* spi, bool enabled)
{
	if(spi == SPI1)
	{
		enabled == ENABLE ? SPI1_PCLK_EN : SPI1_PCLK_DN;
	}
	else if(spi == SPI2)
	{
		enabled == ENABLE ? SPI2_PCLK_EN : SPI2_PCLK_DN;
	}
	else if(spi == SPI3)
	{
		enabled == ENABLE ? SPI3_PCLK_EN : SPI3_PCLK_DN;
	}
	else if(spi == SPI4)
	{
		enabled == ENABLE ? SPI4_PCLK_EN : SPI5_PCLK_DN;
	}
	else if(spi == SPI5)
	{
		enabled == ENABLE ? SPI5_PCLK_EN : SPI5_PCLK_DN;
	}
	else if(spi == SPI6)
	{
		enabled == ENABLE ? SPI6_PCLK_EN : SPI6_PCLK_DN;
	}
	else
	{
		printf("%s(): Invalid SPI device given \n", __func__);
		return -EINVAL;
	}

	return 1;
}

/**
 * spi_deinit - De-initializes (resets) the SPI peripheral
 *
 * @spi: pointer to the SPI device
*/
int spi_deinit(stm32_spi *spi)
{
	if(spi == SPI1)
	{
		SPI1_RESET;
	}
	else if(spi == SPI2)
	{
		SPI2_RESET;
	}
	else if(spi == SPI3)
	{
		SPI3_RESET;
	}
	else if(spi == SPI4)
	{
		SPI4_RESET;
	}
	else if(spi == SPI5)
	{
		SPI5_RESET;
	}
	else if(spi == SPI6)
	{
		SPI6_RESET;
	}
	else
	{
		printf("%s(): Invalid SPI device given \n", __func__);
		return -EINVAL;
	}

	return 1;
}

/* An error that would prevent a successful transfer/receive sequence */
static bool spi_has_error(u32 spi_sr)
{
	return ((spi_sr & SPISR_MODF) || (spi_sr & SPISR_OVR) || (spi_sr & SPISR_UDR));
}

static bool spi_rx_required(spi_handle *handle)
{
	u32 curr_comm = SPI_COMM(handle);

	return (curr_comm == SPI_FULL_DUPLEX || curr_comm == SPI_SIMPLEX_RX
				|| curr_comm == SPI_3WIRE_RX);
}

/**
 * spi_init - Initializes the SPI peripheral with the @handle
 *
 * @handle: pointer to the SPI handle to initialize the peripheral with
 * @comm_type: the SPI communication type @SPI comm type
 * @freq_mhz: the desired clock frequency of the SPI
 * @max_div: maximum baud rate divisor
 * @min_div: minimum baud rate divisor
*/
int spi_init(spi_handle *handle,
		     unsigned int comm_type,
			 u32 freq_mhz,
			 u32 max_div,
			 u32 min_div)
{
	SPI_Config_t config = handle->config;
	stm32_spi *spi= handle->spi;
	u32 spi_cr1 = 0;
	int spi_mbr;

	/* Set the mode */
	if(comm_type < SPI_MODE_NONE)
	{
		spi_clock_control(spi, true);
		spi_set_mode(spi, comm_type);
	}
	else
	{
		printf("%s(): Invalid SPI mode given \n", __func__);
		return -EINVAL;
	}

	spi_mbr = spi_prepare_mbr(spi, freq_mhz, max_div, min_div);
	if(spi_mbr < 0)
	{
		printf("%s(): Invalid SPI clock frequency \n", __func__);
		return -EINVAL;
	}

	spi_cr1 = FIELD_PREP(SPICR1_DFF, config.frame_format) |
			  FIELD_PREP(SPICR1_SSM, config.slave_management) |
			  FIELD_PREP(SPICR1_BR_MASK, spi_mbr) |
			  FIELD_PREP(SPICR1_CPOL, config.clock_polarity) |
			  FIELD_PREP(SPICR1_CPHA, config.clock_phase) |
			  FIELD_PREP(SPICR1_MSTR, config.master_selection);

	/* Set the configuration register */
	spi->CR1 |= spi_cr1;
	handle->config.curr_comm = comm_type;

	/* Init the DMA channel */
	memset(&handle->dma_tx_chan, 0, sizeof(struct dma_channel));
	memset(&handle->dma_rx_chan, 0, sizeof(struct dma_channel));

	return 1;
}

/**
 * spi_irq_configure - Enables/Disables @irq_no
 *
 * @irq_no: The irq number to enable/disable
 * @enabled: true/false for irq enabled/disabled
*/
void spi_irq_configure(u8 irq_no, bool enabled)
{
	volatile void *NVIC_Reg;
	u8 mod_amount = 0;

	if(irq_no <= 31)
	{
		NVIC_Reg = (void*)NVIC_ISER0;
		mod_amount = ++irq_no;
	}
	if(irq_no > 31 && irq_no < 64 )
	{
		NVIC_Reg = (void*)NVIC_ISER1;
		mod_amount = 32;
	}
	else if(irq_no >= 64 && irq_no < 96)
	{
		NVIC_Reg = (void*)NVIC_ISER2;
		mod_amount = 64;
	}
	else
		return;

	if(NVIC_Reg)
	{
		if(enabled)
		{
			*(u32*)NVIC_Reg |= (ENABLE << (irq_no % mod_amount));
		}
		else
		{
			*(u32*)NVIC_Reg &= ~(ENABLE << (irq_no % mod_amount));
		}
	}
}

static inline u32 read_sr_bit(stm32_spi *spi, u8 status_bit)
{
	return spi->SR & status_bit;
}

static void init_xfer_params(spi_handle *handle, spi_transfer *xfer)
{
	assert(handle != NULL);
	assert(xfer != NULL);

	handle->rx_buff = xfer->rx_buff;
	handle->tx_buff = xfer->tx_buff;
	handle->tx_len = xfer->tx_len;
	handle->rx_len = xfer->rx_len;
}
/**
 * spi_configure_crc - Configures and enables the CRC based on @enabled
 *
 * @spi: pointer to the SPI device
 * @polynomial: the polynomial selection
 * @enabled: is the CRC enabled/disabled
*/
void spi_configure_crc(spi_handle *handle, u16 polynomial, bool enabled)
{
	u32 spi_cr, irq_config;
	stm32_spi *spi;

	spi = handle->spi;
	spi_cr = spi->CR1;
	irq_config = spi_irq_config(spi);

	spi_irq_disable(spi, SPI_IT_MASK);
	spi_disable(spi, SPI_COMM(handle));

	/* Configure the CRCEN bit*/
	if(enabled)
		spi->CR1 |= SPICR1_CRCEN;
	else
		spi->CR1 &= ~SPICR1_CRCEN;


	/* Configure the polynomial register to be used to calculate the CRC */
	if(polynomial > 0U)
	{
		spi->CRCPR &= ~FIELD_PREP(SPICRC_RXCRCR, 1);
		spi->CRCPR |= FIELD_PREP(SPICRC_RXCRCR, polynomial);
	}

	/* Re-enable any interrupts */
	spi_irq_enable(spi, irq_config);

	/* Re-enable the SPI */
	if(spi_cr & SPICR1_SPE)
		spi->CR1 |= spi_cr;

}

void spi_clk_configure(spi_handle* handle, u8 cpol, u8 cpha)
{
	u32 spi_cr1 = 0;

	if(handle->busy)
		return;

	/* First disable the SPI */
	spi_disable(handle->spi, SPI_COMM(handle));

	if(cpol > BIT_MAX)
		cpol = 0;
	if(cpha > BIT_MAX)
		cpha = 0;

	spi_cr1 |= FIELD_PREP(SPICR1_CPHA, cpha) | FIELD_PREP(SPICR1_CPOL, cpol);

	handle->spi->CR1 |= spi_cr1;
	handle->config.clock_phase = cpha;
	handle->config.clock_polarity = cpol;
}


/* Flush the FIFO before starting a transfer in order to avoid a possible overrun */
static void spi_flush_dr(stm32_spi *spi)
{
	(void)spi->DR;
}

static u8 spi_get_dff(stm32_spi *spi)
{
	return (spi->CR1 & SPICR1_DFF) + 1;
}

/**
 * spi_tx_blocking- Does the transfer in a blocking manner
 *
 * Should only be used with prior handle initialization
 * @spi: pointer to the SPI device
 * @xfer: pointer to the transfer data
*/
int spi_tx_blocking(spi_handle *handle, spi_transfer *xfer)
{


	assert(xfer != NULL);
	assert(handle != NULL);

	stm32_spi *spi = handle->spi;
	u32 irq_config;
	u8 spi_dff;

	handle->rx_buff = xfer->rx_buff;
	handle->tx_buff = xfer->tx_buff;
	handle->tx_len = xfer->tx_len;
	handle->rx_len = xfer->rx_len;


	/* First check to see if the SPI is in an error state. If it is,
	 * then populate the error struct in the handle and return */

	if(spi_has_error(spi->SR)){
		printf("%s(): SPI in error state, cannot Tx \n", __func__);
		return -EINVAL;
	}

	/* Spin until it isn't busy if that's the desired behavior */
	if(xfer->wait_blocking)
	{
		while(read_sr_bit(spi, SPISR_BSY));
	}

	/* Otherwise return busy status */
	else if(read_sr_bit(spi, SPISR_BSY))
	{
		printf("%s(): SPI busy \n", __func__);
		return -EINVAL;
	}

	/* Check to see if the transfer is permissible */
	if(spi_ta_verify(handle, xfer) < 0)
	{
		printf("%s(): Invalid transfer argument \n", __func__);
		return -EINVAL;
	}

	/* Flush the FIFO in order to avoid the possibility of an overrun */
	spi_flush_dr(handle->spi);

	/* The SPI interrupts should be disabled in the case the user did not disable them
	 * and is now sending a blocking transmission so that this process isn't unintentionally
	 * interrupted
	 */

	/* Disable all SPI interrupts */
	irq_config = spi_irq_config(spi);
	spi_irq_disable(spi, SPI_IT_MASK);


	/* Wait until the TX buffer is empty */
#ifdef SPI_RETRY_TIMES
	while( (!read_sr_bit(spi, SPISR_TXE)) && (--xfer->retry_times != 0U));
#else
	while( !read_sr_bit(spi, SPISR_TXE));
#endif



	spi_dff = spi_get_dff(spi);

	/* Load the data into the date register FIFO */

	while(handle->tx_len > 0)
	{

		if(spi_dff == 0)
		{
			spi->DR = *handle->tx_buff;
			handle->tx_buff++;
			handle->tx_len--;
		}
		else
		{
			spi->DR = *((u16*)handle->tx_buff);
			handle->tx_len--;
			handle->tx_len--;
			(u16*)handle->tx_buff++;
		}

		/* Check the mode the SPI is currently in to see if a response is to be expected.
		 * In order to avoid the possibility of an overrun, the data register should be
		 * read before a subsequent transfer is allowed.
		 */

		if(spi_rx_required(handle))
		{
			spi_rx_blocking(handle);
		}
	}

	/* Restore the SPI's interrupt configuration */
	if(spi_irq_config > 0 )
		spi_irq_enable(spi, irq_config);

	spi_tx_end(handle);

	return 1;
}
static int spi_set_tx_dff(stm32_spi *spi, u32 bpf)
{
	if(bpf > 2)
		return -EINVAL;

	spi->CR1 |= FIELD_PREP(SPICR1_DFF, bpf - 1);
	return 1;
}

/* Determine the mode type from the transfer parameters */
static u32 spi_mode_from_transfer(spi_handle *handle, spi_transfer *xfer)
{
	u32 type = SPI_FULL_DUPLEX;
	u32 curr_comm = SPI_COMM(handle);

	/* First check to see if the current mode is full duplex */
	if(curr_comm & type)
		return type;

	/* Otherwise set the mode based on if the 3 wire interface is being
	 * used or not. If it is, then depending on the buffer configuration,
	 * the mode will either be 3 wire Tx or Rx. Otherwise, the simplex configuration
	 * will be used.
	 */
	if( (curr_comm & SPI_3WIRE_RX) || (curr_comm & SPI_3WIRE_RX) )
	{

		if(!xfer->rx_buff)
			type = SPI_3WIRE_TX;
		else
			type = SPI_3WIRE_RX;
	}
	else
	{
		if(!xfer->rx_buff)
			type = SPI_SIMPLEX_TX;
		else
			type = SPI_SIMPLEX_RX;
	}

	return type;
}

static int transfer_one_prepare(spi_handle *handle, spi_transfer *xfer)
{
	u32 maxbr_div, minbr_div;
	u32 speed_hz;
	u32 comm_type;
	u32 bpw;
	int ret;
	stm32_spi *spi;

	maxbr_div = xfer->maxbr_div > 0 ? xfer->maxbr_div : SPI_MIN_BRD;
	minbr_div = xfer->minbr_div > 0 ? xfer->minbr_div : SPI_MIN_BRD;
	speed_hz = xfer->speed_hz;
	bpw = xfer->bpw;
	spi = handle->spi;

	/* Prepare the baud rate */
	ret = spi_prepare_mbr(handle->spi, speed_hz, maxbr_div, minbr_div);

	if(ret < 0)
	{
		printf("%s(): Invalid baud rate determined for transfer \n", __func__);
		return ret;
	}

	/* Set the mode based on the transfer data */
	comm_type = spi_mode_from_transfer(handle, xfer);
	ret = spi_set_mode(spi, comm_type);

	if(ret < 0)
	{
		printf("%s(): Invalid communication mode for SPI determined for transfer \n", __func__);
		return ret;
	}

	/* Set the DFF */
	ret = spi_set_tx_dff(spi, bpw);
	if(ret < 0)
	{
		printf("%s(): Invalid frame format given, setting frame format to 1 byte for transfer \n", __func__);
		spi_set_tx_dff(spi, SPI_DEFAULT_BPF);
	}

	return ret;
}

/**
 * spi_transfer_one - Does a single SPI transfer based on @xfer params without requiring a manual
 * reconfiguration of the handle state
 * @handle: pointer to the SPI handle
 * @xfer: pointer to the transfer data
 * @spi_dma_irq: the SPI DMA irq number if DMA is being used, otherwise value is ignored
 *
 * @return: 1 for success, -1 for failure
*/
int spi_transfer_one(spi_handle *handle,spi_transfer *xfer)
{
	assert(handle != NULL);
	assert(xfer != NULL);

	int ret;

	handle->tx_buff = xfer->tx_buff;
	handle->rx_buff = xfer->rx_buff;
	handle->rx_len = xfer->rx_len;
	handle->tx_len = xfer->tx_len;

	ret = transfer_one_prepare(handle, xfer);

	if(ret < 0)
	{
		printf("%s(), Failed to prepare SPI for single transfer \n", __func__);
		return ret;
	}

	if(handle->use_dma)
	{
		ret = spi_transfer_one_dma(handle, xfer, true);;
	}

	return ret;
}


/**
 * spi_tx_nb- Sets up the supplied SPI handle for TX, which will then be completed upon the interrupt
 * @handle: the SPI handle
 *
 * @xfer: he transfer handle
*/
int spi_tx_nb(spi_handle *handle, spi_transfer *xfer)
{
	assert(handle != NULL);
	assert(xfer != NULL);

	u32 spi_sr;
	stm32_spi *spi;

	spi = handle->spi;
	spi_sr = spi->SR;

	/* First check the handle state, if it is already marked as transmitting,
	 * then do nothing.
	 */
	if(handle->busy)
	{
		printf("%s(): SPI busy \n", __func__);
		return -EINVAL;
	}

	/* Check to see if the transfer arguments are good */
	if(spi_ta_verify(handle, xfer) < 0)
		return -EINVAL;

	/* Check to see if the SPI is in an error state */
	if(spi_has_error(spi_sr))
	{
		printf("%s(): SPI in error state, cannot transfer \n", __func__);
		return -EINVAL;
	}

	/* Disable interrupts so this setup isn't interrupted */
	spi_irq_disable(spi, SPI_IT_MASK);

	/* Transfer the transfer arguments to the handle */
	init_xfer_params(handle, xfer);

	spi_flush_dr(spi);

	/* Enable transfer buffer empty interrupt */
	spi_irq_enable(spi, SPICR2_TXEIE);

	/* Enable the Rxne interrupt */
	if(SPI_COMM(handle) == SPI_FULL_DUPLEX)
		spi_irq_enable(spi, SPICR2_RNEIE);

	/* Enable SPI */
	spi->CR1 |= SPICR1_SPE;

	return 1;
}

/**
 * spi_toggle_ssi - Sets the internal state of the slave select for software managed SS
 * @spi- pointer to the SPI device
 * @enabled - enable/disable the SSI
*/
void spi_toggle_ssi(stm32_spi* spi, bool enabled)
{
	/* TODO First check to see if software slave management is enabled */
	u32 spi_cr1;
	spi_cr1 = spi->CR1;

	if(spi_cr1 & SPICR1_SPE)
	{
		printf("%s(): Disabling SPI to toggle SSI \n", __func__);
		spi_disable(spi, SPI_MODE_NONE);
	}

	if(enabled)
		spi_cr1 |= SPICR1_SSI;
	else
		spi_cr1 &= ~SPICR1_SSI;

	spi->CR1 = spi_cr1;
}

/**
 * spi_toggle_ssoe - Sets the state of the SS output pin
 * @spi- pointer to the SPI device
 * @enabled - 0 to disable the SS output in master mode so the device
 * can work in a multi-master configuration. 1 to enable which means
 * it cannot work with a multi-master configuration
*/
void spi_toggle_ssoe(stm32_spi* spi,u8 enabled)
{
	spi->CR2 |= (enabled << SPICR2_SSOE);
}

int spi_set_mode(stm32_spi *spi, unsigned int comm_type)
{
	u32 spi_cr = 0;

	/* Disable if enabled */
	if(spi->CR1 & SPICR1_SPE)
		spi->CR1 &= ~SPICR1_SPE;

	if(comm_type == SPI_SIMPLEX_TX || comm_type == SPI_3WIRE_TX)
	{
		spi_cr |= SPICR1_BIDIMODE;
		spi_cr |= SPICR1_BIDIOE;
	}
	else if(comm_type == SPI_FULL_DUPLEX || comm_type == SPI_SIMPLEX_RX)
	{
		spi_cr &= ~SPICR1_BIDIMODE;
		spi_cr &= ~SPICR1_BIDIOE;

		if(comm_type == SPI_SIMPLEX_RX)
			spi_cr |= SPICR1_RXONLY;
	}
	else if(comm_type == SPI_3WIRE_RX)
	{
		spi_cr &= ~SPICR1_BIDIOE;
		spi_cr |= SPICR1_BIDIMODE;
	}
	else
	{
		printf("%s(): Invalid SPI mode given \n", __func__);
		return -EINVAL;
	}

	/* Set the configuration */
	spi->CR1 |= spi_cr;

	return 1;
}

/**
 * spi_prepare_mbr - Prepares the SPI baud rate divisor based on desired frequency
 *
 * @spi- pointer to the SPI device
 * @freq_hz - the desired frequency of the SPI in Hz
*/
int spi_prepare_mbr(stm32_spi *spi,
		            u32 freq_hz,
					u32 max_div,
					u32 min_div)
{
	u32 div, mbrdiv;

	/* TODO need to get sys clock in a dynamic way, for now just
	 * use the internal oscillator frequency (16 MHz) for the calulcation
	 */

	/* Round the clock frequency and desired frequency to nearest div */
	div = DIV_ROUND_CLOSEST(HSI_FREQ_HZ, freq_hz);

	if((div < min_div) || (div > max_div))
		return -EINVAL;


	/* Find the nearest power of 2 which is greater than or equal to div */
	mbrdiv = generic_fls((unsigned int)div) - 1;
	return mbrdiv - 1;
}


int spi_rx_blocking(spi_handle *handle)
{
	assert(handle != NULL);
	/* Check the SPI mode - if the mode is unidirectional receive-only
	 * mode, then it may be the case that the SPI isn't enabled, but
	 * the receive sequence automatically starts in this mode once the SPI
	 * is enabled */
	u32 curr_comm;
	stm32_spi *spi;

	curr_comm = SPI_COMM(handle);
	spi = handle->spi;

	if(curr_comm == SPI_SIMPLEX_RX || curr_comm == SPI_3WIRE_RX)
	{
		if( !(handle->spi->CR1 & SPICR1_SPE))
		{
			spi->CR1 |= SPICR1_SPE;
			printf("%s(), Enabling SPI for Rx \n", __func__);
		}
	}

	/* If an RX buffer wasn't provided but it is required to
	 * be read given the mode, then just flush the
	 * data buffer so no overrun happens */

	if(!handle->rx_buff)
	{
		while(!read_sr_bit(spi, SPISR_RXNE));
		spi_flush_dr(spi);

		printf("%s(), SPI Handle Rx buff is null \n", __func__);
		return -EINVAL;;
	}
	else
	{

		while(handle->rx_len > 0 )
		{
			/* Wait for the receive buffer not empty flag to be set */
			while(read_sr_bit(spi, SPISR_RXNE));

			if(spi_get_dff(spi) == 1)
			{
				*((u16*)handle->rx_buff) = (u16)spi->DR;
				(u16*)handle->rx_buff++;
				handle->rx_len--;
				handle->rx_len--;
			}
			else
			{
				*handle->rx_buff = (u8)spi->DR;
				handle->rx_buff++;
				handle->rx_len--;
			}
		}
	}

	return 1;
}

/**
 * spi_dma_config - prepares a SPI dma channel handle
 * @spi: pointer to SPI peripheral
 * @dma_chan: pointer to dma_channel handle
 * @direction: the direction of the dma transfer
*/
void spi_dma_config(spi_handle *handle,
		            struct dma_channel *dma_chan,
					enum dma_transfer_direction direction)
{
	assert(handle != NULL);

	u8 curr_bpw;
	u32 max_burst;
	enum dma_slave_buswidth buswidth;
	stm32_spi *spi;

	curr_bpw = handle->config.frame_format;
	spi = handle->spi;

	if(curr_bpw <= 8)
	{
		buswidth = Byte;
	}
	else if(curr_bpw >= 16)
	{
		buswidth = HalfWord;
	}
	else
		buswidth = Byte;

	/* STM32F40 SPI peripherals don't have FIFOs, so the DMA cannot burst */
	max_burst = DMA_BURST_SINGLE;

	/* DMA request is for RX */
	if(direction == DEV_TO_MEM)
	{
		dma_chan->sconfig.src_addr = SPI_DR_ADDR(spi);
		dma_chan->sconfig.src_addr_width = buswidth;
		dma_chan->threshold = DMA_FIFO_THRESHOLD_NONE;

		/* Set the destination address */
		dma_chan->sconfig.dst_addr = (u32)handle->rx_buff;
		dma_chan->sconfig.dst_addr_width = buswidth;

		/* Set burst */
		dma_chan->sconfig.dst_max_burst = max_burst;
		dma_chan->sconfig.src_max_burst = max_burst;

	}

	/* DMA request is for TX */
	else if(direction == MEM_TO_DEV)
	{
		dma_chan->sconfig.dst_addr = SPI_DR_ADDR(spi);
		dma_chan->sconfig.dst_addr_width = buswidth;
		dma_chan->threshold = DMA_FIFO_THRESHOLD_NONE;

		/* Set the source address */
		dma_chan->sconfig.src_addr = (u32)handle->tx_buff;
		dma_chan->sconfig.src_addr_width = buswidth;

		/* Set burst */
		dma_chan->sconfig.dst_max_burst = max_burst;
		dma_chan->sconfig.src_max_burst = max_burst;

	}
	else
	{
		printf("%s(): SPI DMA direction not supported \n", __func__);
		return;
	}
}

/**
 * spi_dma_chan_config - prepares a DMA slave descriptor based on the transfer parameters @xfer
 *
 * @handle: pointer to the SPI handle
 * @stream: the DMA stream number
 * @channel: the DMA channel number
 * @flags: The stream configuration flags - see @stream_config in dma.h
 *
 * The DMA's clock must be enabled before using this function
*/
void spi_dma_chan_config(struct dma_channel *chan,
		u32 stream,
		u32 channel,
		u32 flags)
{
	struct dma_chan_cfg chan_config;

	/* Init the channel configuration */
	memset(&chan_config, 0, sizeof(struct dma_chan_cfg));
	chan_config.channel_id = channel;
	chan_config.stream_num = stream;
	chan_config.stream_config = flags;

	/* Copy to the SPI handle's DMA channel config */
	memcpy(&chan->channel_config, &chan_config, sizeof(struct dma_chan_cfg));
	dma_set_channel_config(chan);
}

/**
 * spi_transfer_one_dma- prepares a DMA slave descriptor based on the transfer parameters @xfer
 * spi_dma_config should be called before this function
 * @handle: pointer to the SPI handle
 * @xfer: pointer to the transfer parameter data
 * @submit: whether the dma transfer should immediately be submitted after initialization.
 * If false, then spi_dma_transfer_start must be called
*/
int spi_transfer_one_dma(spi_handle *handle,
		                 spi_transfer *xfer,
						 bool submit)
{
	struct dma_desc *tx_desc, *rx_desc;
	struct dma_channel rx_config, tx_config;
	struct dma_sg_req tx_sg, rx_sg;
	stm32_spi *spi;
	u32 curr_comm;
	LIST_CREATE(rx_lst);
	LIST_CREATE(tx_lst);

	rx_desc = NULL;
	tx_desc = NULL;
	curr_comm = SPI_COMM(handle);
	spi = handle->spi;

	memcpy(&tx_config, &handle->dma_tx_chan, sizeof(struct dma_channel));
	memcpy(&rx_config, &handle->dma_rx_chan, sizeof(struct dma_channel));

	init_xfer_params(handle, xfer);

	/* Prepare the Tx descriptor */
	if(xfer->tx_buff)
	{
		spi_dma_config(handle, &tx_config, MEM_TO_DEV);

		/* Init the sg - the DMA address is the source address (the TX buffer) */
		tx_sg.dma_addr = (u32)handle->tx_buff;
		tx_sg.len = handle->tx_len;

		/* Add to sg list */
		list_add((void*)&tx_sg, &tx_lst);

		/* Prep descriptor */
		tx_desc = dma_prep_desc(&tx_config,
				                &tx_lst,
								0,
								MEM_TO_DEV);

		list_free_nodes(&tx_lst);
		handle->dma_tx_chan.desc = tx_desc;
	}

	/* Prepare the Rx descriptor */
	if(xfer->rx_buff)
	{
		spi_dma_config(handle, &rx_config, DEV_TO_MEM);

		/* Init the sg - the DMA address is the source address (the SPI DR) */
		rx_sg.dma_addr = rx_config.sconfig.src_addr;
		//rx_sg.len = rx_config.sconfig.src_addr_width/ 8; /* Width is originally in bits, not bytes */
		rx_sg.len = handle->rx_len;

		/* Add to sg list */
		list_add((void*)&rx_sg, &rx_lst);

		/* Prep descriptor */
		rx_desc = dma_prep_desc(&rx_config,
				                &rx_lst,
								0,
								DEV_TO_MEM);

		list_free_nodes(&rx_lst);
		handle->dma_rx_chan.desc = rx_desc;
	}

	/* Both descriptors are required */
	if(curr_comm == SPI_FULL_DUPLEX && (!tx_desc || !rx_desc))
	{
		printf("%s(): Full Duplex requires an RX and TX descriptor \n", __func__);
		goto err;
	}
	/* Rx descriptor is required */
	if((curr_comm == SPI_3WIRE_RX || curr_comm == SPI_SIMPLEX_RX) && !rx_desc)
	{
		printf("%s(): 3 Wire RX or simplex RX, but no RX descriptor \n", __func__);
		goto err;
	}
	/* Tx descriptor is required */
	else if((curr_comm == SPI_3WIRE_TX || curr_comm == SPI_SIMPLEX_TX) && !tx_desc)
	{
		printf("%s(): 3 Wire TX or simplex TX, but no TX descriptor \n", __func__);
		goto err;
	}

	/* Set the SPI DMA Rx callback */
	if(rx_desc)
	{
		rx_desc->dma_rx_cb = spi_dma_rx_cb;
		rx_desc->callback_param = (void*)spi;

		/* Enable the Rx DMA request */
		spi->CR2 |= SPICR2_RXDMAEN;
	}

	/* Set the SPI DMA TX callback */
	if(tx_desc)
	{
		tx_desc->dma_tx_cb = spi_dma_tx_cb;
		tx_desc->callback_param = (void*)spi;

		/* Enable the Tx DMA request */
		spi->CR2 |= SPICR2_TXDMAEN;
	}

	handle->use_dma = true;

	if(submit)
		spi_transfer_one_dma_start(handle);

	return 1;
err:
	if(tx_desc)
		free(tx_desc);
	if(rx_desc)
		free(tx_desc);
	return -EINVAL;

}

/**
 * spi_dma_transfer_one_start - Enables the DMA configured SPI peripheral to start DMA requests
 * @handle - pointer to the SPI handle
*/
void spi_transfer_one_dma_start(spi_handle *handle)
{
	stm32_spi *spi;
	u8 curr_comm;

	spi = handle->spi;
	curr_comm = SPI_COMM(handle);

	/* Before enabling the SPI, need to set the overrun interrupt flag based on the
	 * mode. If transmit only is being used, then the overrun interrupt flag should be
	 * disabled since the Rx buffer is never read
	 */
	if(curr_comm == SPI_FULL_DUPLEX || curr_comm == SPI_SIMPLEX_RX
			|| curr_comm == SPI_3WIRE_RX)
	{
		/* Enable the overrun interrupt */
		spi->CR2 &= ~SPICR2_ERRIE;
		spi->CR2 |= SPICR2_ERRIE;
	}
	/* Tx only - disable Rx DMA Enable */
	else
	{
		spi->CR2 &= ~SPICR2_RXDMAEN;
	}

	dma_start(&handle->dma_tx_chan);

	if(spi_rx_required(handle))
	{
		dma_start(&handle->dma_rx_chan);
	}

	/* Enable the SPI */
	spi->CR1 |= SPICR1_SPE;
}

static void tx_irq_handle(spi_handle *handle)
{

	/* The SPI peripheral is ready for a data transmission, load
	 * the FIFO according to the DFF, update the handle and return */
	/* 16 bit DFF */
	if(spi_get_dff(handle->spi) == 2)
	{
		handle->spi->DR = *((u16*)handle->tx_buff);
		(u16*)handle->tx_buff++;
		handle->tx_len--;
		handle->tx_len--;
	}
	else
	{
		handle->spi->DR = *handle->tx_buff;
		handle->tx_buff++;
		handle->tx_len--;
	}

	handle->busy = false;
}

static void rxne_irq_handle(spi_handle *handle)
{

	/* If the RX buffer is null, then we're probably dealing with a
	 * transfer only operation in full duplex mode, so just clear the
	 * FIFO
	 */

	if(!handle->rx_buff)
	{
		spi_flush_dr(handle->spi);
		return;
	}

	if(spi_get_dff(handle->spi) == 2)
	{
		*((u16*)handle->rx_buff) = (u16)handle->spi->DR;
		handle->rx_len--;
		handle->rx_len--;
		(u16*)handle->rx_buff++;
	}
	else
	{
		*handle->rx_buff = (u8)handle->spi->DR;
		handle->rx_len--;
		handle->rx_buff++;
	}

	handle->busy = false;
}

/**
 * spi_rx_end - End Rx and clean up handle
 * @handle - pointer to the SPI handle
*/
void spi_rx_end(spi_handle *handle)
{
	handle->spi->CR2 &= ~SPICR2_RNEIE;

	memset(&handle->rx_buff, 0, handle->rx_len);
	handle->rx_len = 0;
	handle->busy = false;
}

/**
 * spi_tx_end - End Tx and clean up handle
 * @handle - pointer to the SPI handle
*/
void spi_tx_end(spi_handle *handle)
{
	handle->spi->CR2 &= ~SPICR2_TXEIE;

	memset(&handle->tx_buff, 0, sizeof(handle->tx_buff));
	handle->tx_len = 0;
	handle->busy = false;
}

/**
 * spi_irq_handled - SPI irq handler to be called within the SPI ISR
 * @handle - pointer to the SPI handle
 * @return - irq_handled indicating if the irq was handled
*/
enum irq_handled spi_irq_handle(spi_handle *handle)
{

	stm32_spi *spi = handle->spi;
	u32 spi_sr, spi_cr2, spi_cr1;
	u32 curr_comm;
	u32 masked = 0;
	bool end;

	spi_sr = spi->SR;
	spi_cr2 = spi->CR2;
	curr_comm = SPI_COMM(handle);
	end = false;
	spi_sr &= ~SPISR_BSY;

	if(!handle->use_dma && ((curr_comm == SPI_SIMPLEX_RX)
			|| (curr_comm == SPI_3WIRE_RX)
			|| (curr_comm == SPI_FULL_DUPLEX)))
	{
		/* Handle the TXE interrupt when handling RXNE interrupt */
		//spi_sr &= ~(SPISR_TXE);
		masked |= SPISR_TXE | SPISR_RXNE | SPISR_OVR;
	}

	if(!handle->use_dma && ((curr_comm == SPI_SIMPLEX_TX)
			|| (curr_comm == SPI_3WIRE_TX)))
	{
		/* Clear the overrun and RXNE flags since receive never happens */
		spi_sr &= ~(SPISR_OVR | SPISR_RXNE);
		masked |= SPISR_TXE;
	}

	if( !(spi_sr & masked))
	{
		printf("%s(), Spurious interrupt \n", __func__);
		return IRQ_NONE;
	}

	if(spi_sr & SPISR_OVR)
	{
		/* With overrun, the hardware does not automatically disable the SPI */

		/* Clear the overrun flag */
		spi_flush_dr(spi);

		end = true;
		goto end_irq;
	}

	/* Check RXNE flag */
	if(spi_sr & SPISR_RXNE)
	{
		if(handle->rx_buff)
			rxne_irq_handle(handle);
		else
			spi_flush_dr(spi);

		if(handle->rx_len == 0)
			end = true;
		else if(handle->tx_buff && handle->tx_len > 0)
		{
			spi_sr &= ~SPISR_TXE;
			tx_irq_handle(handle);
		}
	}

	/* Check TXE flag */
	if(spi_sr & SPISR_TXE)
	{
		if(handle->tx_len == 0)
			end = true;
		if(handle->tx_buff && handle->tx_len > 0)
			tx_irq_handle(handle);
	}

	/* Check ERR flag for any other errors such as MODF, CRC */
	if( (spi_cr2 & SPICR2_ERRIE) && ((spi_sr & SPI_IT_ERR) > 0 ))
	{
		if(spi_sr & SPISR_CRCERR)
			printf("%s(): SPI CRC Error\n", __func__);
		else if(spi_sr & SPISR_MODF)
			printf("%s(): SPI MODF Error\n", __func__);
		else if(spi_sr & SPISR_UDR)
			printf("%s(): SPI Underrun\n", __func__);

		spi_cr1 = spi->CR1;
		if(! (spi_cr1 & SPICR1_SPE))
			printf("%s(), SPI peripheral disabled by HW \n", __func__);
	}

end_irq:
	/* Transfer is over, disabled the interrupts to avoid new interrupts */
	if(end)
	{

		printf("%s(), End of SPI transmission - disabling\n", __func__);
		spi->CR2 &= ~(SPICR2_ERRIE |
				      SPICR2_RNEIE |
					  SPICR2_TXEIE );

		spi->CR1 &= ~SPICR1_SPE;
	}


	return IRQ_HANDLED;
}


int spi_ta_verify(spi_handle *handle, spi_transfer *xfer)
{
	assert(xfer != NULL);
	u32 comm_type;

	comm_type = SPI_COMM(handle);

	if( (NULL == xfer->tx_buff) && (NULL == xfer->rx_buff))
	{
		return -EINVAL;
	}

	/* Check the transfer arguments based on current communication mode */

	/* If full duplex, then both a Tx and Rx buffer are required */
	if(comm_type == SPI_FULL_DUPLEX)
	{
		if(!xfer->tx_buff || !xfer->rx_buff)
		{
			printf("%s(): Tx and Rx buffers required for full duplex mode \n", __func__);
			return -EINVAL;
		}

		/* Set Tx length if not given */
		if(xfer->tx_buff && !xfer->tx_len)
			xfer->tx_len = sizeof(*xfer->tx_buff);

		/* Set Rx length if not given */
		if(xfer->rx_buff && !xfer->rx_len)
			xfer->rx_len = sizeof(*xfer->rx_buff);
	}

	/* If 3 wire Tx or Simplex Tx, then just a Tx buffer is required */
	else if(comm_type == SPI_SIMPLEX_TX || comm_type == SPI_3WIRE_TX)
	{
		if(!xfer->tx_buff)
		{
			printf("%s(): Tx buffer is required for simplex/3 wire Tx \n", __func__);
			return -EINVAL;
		}
		if(xfer->tx_buff && !xfer->tx_len)
			xfer->tx_len = sizeof(*xfer->tx_buff);
	}
	/* If 3 wire Rx or Simplex Rx, then just an Rx buffer is required */
	else
	{
		if(!xfer->tx_buff)
		{
			printf("%s(): Rx buffer is required for simplex/3 wire Rx \n", __func__);
			return -EINVAL;
		}
		if(xfer->rx_buff && !xfer->rx_len)
			xfer->rx_len = sizeof(*xfer->rx_buff);
	}

	u32 spi_dff = spi_get_dff(handle->spi);

	/* If the bytes per frame is equal to 2, then the data size must be a multiple of 2 */
	if(xfer->tx_len % spi_dff > 0)
	{
		return -EINVAL;
	}

	return 1;

}


/**
 * spi_it_config - End Rx and clean up handle
 * @handle - pointer to the SPI handle
*/
u32 spi_irq_config(stm32_spi *spi)
{
	u32 spi_cr2;
	spi_cr2 = spi->CR2;

	return FIELD_GET(SPI_IT_MASK, spi_cr2);
}

/**
 * spi_irq_disable - Disables interrupts indicated by @flags
 * @spi- pointer to the SPI device
 * @flags - the interrupts to disable
*/
void spi_irq_disable(stm32_spi *spi, u32 flags)
{
	spi->CR2 &= ~flags;
}

/**
 * spi_irq_enable - Enables interrupts indicated by @flags
 * @spi- pointer to the SPI device
 * @flags - the interrupts to enable
*/
void spi_irq_enable(stm32_spi *spi, u32 flags)
{
	spi->CR2 |= flags;
}

void spi_tx_abort(spi_handle *handle)
{
	assert(handle != NULL);

	/* Disable SPI interrupts */
	spi_irq_disable(handle->spi, SPICR2_ERRIE |
			                     SPICR2_RNEIE |
								 SPICR2_TXEIE);

	/* Reset the SPI */
	spi_deinit(handle->spi);

	/* Reset handle items */
	handle->rx_len = 0;
	handle->tx_len = 0;
	memset(&handle->rx_buff, 0, sizeof(handle->rx_buff));
	memset(&handle->tx_buff, 0, sizeof(handle->tx_buff));
}


/**
 * spi_disable - Disables the SPI
 * @spi- pointer to the SPI device
*/
void spi_disable(stm32_spi* spi, unsigned int comm_type)
{
	/* Wait for TXE=1 and BSY=0 before disabling the peripheral */
	if(comm_type == SPI_FULL_DUPLEX || comm_type == SPI_3WIRE_TX
			|| comm_type == SPI_SIMPLEX_TX)
	{
		/* We should read the receive buffer before disabling the peripheral, but given
		 * the conditions of these two modes, the receive buffer should always be read.
		 * In the case of a blocking transmission, a receive is always done for each byte transferred
		 * and with a non blocking transfer, the Rx buffer is read with the interrupt */

		while(!read_sr_bit(spi,SPISR_TXE) && read_sr_bit(spi,SPISR_BSY));

	}

	spi->CR1 &= ~SPICR1_SPE;
}

/* Private function definitions */
static void spi_dma_tx_cb(void *data)
{
	printf("%s(): Executing DMA Tx callback \n", __func__);
	stm32_spi *spi = data;

	/* First disable the TX DMA request */
	spi->CR2 &= ~SPICR2_TXDMAEN;

	/* Disable the SPI - will wait for Rx if Rx
	 * is being done with the current communication mode
	 */
	spi_disable(spi, SPI_SIMPLEX_TX);
}
static void spi_dma_rx_cb(void *data)
{
	printf("%s(): Executing DMA Rx callback \n", __func__);
	stm32_spi *spi = data;
	spi_disable(spi, SPI_FULL_DUPLEX);
}
