/*
 * dma.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Andrew Streng
 */

#include "dma.h"
#include <assert.h>


/* Private function declarations  */
static bool dma_is_burst_possible(u32 buf_len, u32 fifo_threshold);
static bool dma_verify_ndt(struct dma_channel *channel, u32 ndt);
static u8 dma_read_status_bit(struct dma_channel *channel, u8 bit);
static u8 dma_get_fifo_thresh(struct dma_channel *channel);
static bool dma_circular_config_valid(struct dma_channel *channel);
static int dma_get_best_burst(u32 mem_size, u32 max_burst, u32 fifo_level);
static u8 dma_fifo_depth_adjusted(u8 fifo_level);
static int dma_get_burst(u32 max_burst);
static int dma_transfer_prep(struct dma_channel *channel, u8 direction);
static void dma_irq_clear(struct dma_channel *channel, u8 flag);
static void dma_irq_clear_all(struct dma_channel *channel);
static void dma_handle_chan_paused(struct dma_channel *channel);
static enum irq_handled_t dma_irq_handle(struct dma_channel *channel);

/* Public function definitions */
void dma_enable_clk(DMA_RegDef_t *dma_controller, bool enabled)
{
	assert(dma_controller);

	u32 bit = 0;

	if(dma_controller == DMA1)
	{
		bit = BIT_DMA1_CLK;
	}
	else
	{
		bit = BIT_DMA2_CLK;
	}

	if(enabled)
	{
		RCC->AHB1ENR |= bit;
	}
	else
	{
		RCC->AHB1ENR &= ~ bit;
	}
}

/**
 * dma_transfer_abort - handles safely aborting the current DMA transfer
 * and returns the DMA channel to its idle state
 *
 * @channel: the DMA channel
*/
void dma_pause(struct dma_channel *channel)
{

	/* Clear and interrupt status */
	dma_irq_clear_all(channel);

	/* Disable the channel */
	int ret = dma_disable_chan(channel);
	if(ret < 0)
		return;

	dma_handle_chan_paused(channel);
}

void dma_stop(struct dma_channel *channel)
{
	const u8 streamno = channel->channel_config.stream_num;

	/* SxCFR interrupts enable values */
	u32 *dma_scr = DMA_REG(channel->base, DMA_SxCFR(streamno));
	*dma_scr &= ~DMA_MASKI;

	/* Clear FIFO error interrupt enable in SxFCR */
	u32 *dma_fcr = DMA_REG(channel->base, DMA_FxCR(streamno));
}

/**
 * dma_read_status - reads the status register of the DMA
 * @channel: the dma channel
*/
u32 dma_read_status(struct dma_channel *channel)
{

	const u8 dma_stream = channel->channel_config.stream_num;
	const DMA_RegDef_t *pBase = channel->base;
	u8 shift_amount = 0;

	/* Select the high/low status register to read from based on the
	 * the current stream number
	 */
	dma_status_reg = dma_stream > 3 ? pBase->HISR : pBase->LISR;

	/* Next we need to shift the status bits if necessary in order to read
	 * the correct bits corresponding to the current stream in the status register
	 */

	if(dma_stream > 0)
	{
		/* Shift the status mask by the stream number
		 * There are 4 indices in each status register and each instance
		 * is 5 bits long */
		shift_amount = (dma_stream % 4) * 6;
	}

	return (dma_status_reg >> shift_amount) & DMA_MASKI;
}

static u8 dma_read_status_bit(struct dma_channel *channel, u8 bit)
{

	u32 dma_status_reg = dma_read_status(channel);

	return dma_status_reg & bit;
}


/**
 * dma_enable_interrupts - enables the given interrupts passed in with @flags
 * @flags: the dma channel
*/
void dma_enable_interrupts(struct dma_channel *channel, u32 flags)
{
	assert(channel);

	if(channel->busy)
		return;

	/* Write the flags to the appropriate DMA_xIFCR register
	 * corresponding to the appropriate channel
	 */

	flags &= DMA_MASKI;
	u32 *xfcr = (u32*)DMA_SCR_REG(channel->channel_config.stream_num);

	*xfcr |= flags;


}
void dma_disable_interrupts(struct dma_channel *channel, u32 flags)
{
	assert(channel);

	if(channel->busy)
		return;

	flags &= DMA_MASKI;
	u32 *xfcr = (u32*)DMA_SCR_REG(channel->channel_config.stream_num);

	*xfcr &= ~flags;

}

/**
 * dma_interrupt_config - reads the current interrupt configuration of the stream
 * @channel: the dma channel
 *
*/
u32 dma_interrupt_config(struct dma_channel *channel)
{
	u32 *xfcr = (u32*)DMA_SCR_REG(channel->channel_config.stream_num);

	if(xfcr)
		return (*xfcr & DMA_MASKI);
	else
		return DMA_MASKI;
}

void dma_flush_fifo(struct dma_channel *channel)
{
	/* Only applies with DMA directions peripheral-to-memory
	 * or memory-to-memory transfers
	 */

	/* Reset the EN bit */

	/* Wait for transfer complete interrupt to be issued */
}

/**
 * dma_transfer_start - starts the transfer given
 * @channel: the dma channel
 *
*/
int dma_transfer_start(struct dma_channel *channel, u32 ndt)
{
	/* First disable the EN bit */

	/* Then clear any interrupts */


	if(dma_verify_ndt(channel, ndt))
	{

	}

	return -EINVAL;
}

/**
 * dma_set_fifo_config - sets the internal FCR register of the dma_reg struct
 * according to the parameters of the channel
 *
 * @channel: the dma channel
 *
*/
void dma_set_fifo_config(struct dma_channel *channel)
{
	/* Clear the FIFO configuration register */
	channel->dma_reg.sfcr = ~DMA_FCR_MASK;

	/* Check to see if the peripheral and memory widths are the same */
	if(channel->periph_mem_width == channel->mem_width)
	{
		channel->dma_reg.scr |= DMA_SCR_DMEIE;
	}
	else
	{
		/* Disable direct mode in the FIFO configuration */
		channel->dma_reg.sfcr |= DMA_FCR_DMDIS;

		/* Set the FIFO threshold and enable FIFO error interrupt */
		channel->dma_reg.sfcr |= (DMA_FCR_FTH & channel->threshold);
		channle->dma_reg.sfcr |= DMA_FCR_FEIE;
	}
}

/**
 * dma_fifo_config_apply - applies the internal FCR register of the dma_reg struct
 * to the current DMA channel specified in @channel
 *
 * @channel: the dma channel
 *
*/
void dma_fifo_config_apply(struct dma_channel *channel)
{

	u32 *dma_fcr = DMA_REG(channel->base, DMA_SxFCR(channel->channel_config.stream_num));
	*dma_fcr = channel->dma_reg.sfcr;
}

/* Private function definitions */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//static void dma_write_reg(DMA_RegDef_t* base, )


/**
 * dma_get_buswidth - Get buswidth of the memory/peripheral in the format to write
 * 					  to the configuration register
 *
 * @width: width of the source/destination in bits
*/
static int dma_get_buswidth(u32 width)
{
	switch(width)
	{
		case Byte:
			return DMA_SLAVE_BUSWIDTH_1_BYTE;
		case HalfWord:
			return DMA_SLAVE_BUSWIDTH_2_BYTES;
		case Word:
			return DMA_SLAVE_BUSWIDTH_4_BYTES;
		default:
			return -EINVAL;
	}
}

static int dma_get_burst(u32 max_burst)
{
	switch(max_burst)
	{
		case 0:
		case 1:
			return DMA_BURST_SINGLE;
		case 4:
			return DMA_BURST_INCR4;
		case 8:
			return DMA_BURST_INCR8;
		case 16:
			return DMA_BURST_INCR16;
		default:
			return -EINVAL;
	}
}

/**
 * dma_get_max_width - Get the max width of the memory/device according to the FIFO threshold
 *
 * @buf_len: the length of the device/memory buffer in bits
 * @threshold: the threshold of the FIFO - @DMA transfer directions
 *
 * Need to check if the device and memory sizes are different before using this function
*/

static enum dma_slave_buswidth dma_get_max_width(u32 buf_len, u32 buf_addr, u32 threshold)
{
	enum dma_slave_buswidth max_width, buf_width, dev_width;
	dev_width = dma_get_buswidth(buf_addr);

	/* If the threshold of the FIFO is Full, that means it fills 4 words before emptying,
	 * which means that the max width of the buffer can be 4 bytes (a word)
	 */
	if(threshold == DMA_FIFO_THRESHOLD_FULL)
	{
		max_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}
	else
	{
		max_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	}

	while((buf_len < max_width) || (buf_len % max_width)
			&& buf_len > DMA_SLAVE_BUSWIDTH_1_BYTE)
	{
		max_width >> 1;
	}

	/* The peripheral and memory width cannot be the same size */
	if(buf_addr & (max_width - 1))
	{
		max_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	}

	return max_width;
}

/**
 * dma_transfer_prep - Initializes the dma_reg configuration register
 *
 * @channel - the DMA channel
*/
static int dma_transfer_prep(struct dma_channel *channel)
{

	u8 dma_direction = channel->channel_config.stream_config & DMA_RESERVED;
	int src_width, dst_width, threshold;
	u32 src_best_burst, dst_best_burst, src_burst_size, dst_burst_size;
	u32 src_addr, dst_addr, src_max_burst, dst_max_burst, irq, dma_scr;

	src_addr = channel->src_addr;
	dst_addr = channel->dst_addr;
	src_max_burst = channel->src_max_burst;
	dst_max_burst = channel->dst_max_burst;
	irq = channel->irq;

	switch(dma_direction)
	{
		case DMA_MEM_TO_PERIPH:

			/* Set the memory's data size */
			dest_width = dma_get_buswidth(channel->periph_mem_width);
			if(dest_width < 0)
			{
				return dest_width;
			}

			/* Get the best burst available for the destination given the
			 * memory width, the FIFO threshold and width of the destination memory
			 */
			dst_best_burst = dma_get_best_burst(dest_width, dst_max_burst, fifo_level);
			dst_burst_size = dma_get_burst(dst_best_burst);

			if(dst_burst_size < 0 )
			{
				return dst_burst_size;
			}

			/* Determine the max width the memory can be */
			src_width = dma_get_max_width(channel->mem_width, src_addr, fifo_threshold);

			/* Need to check if the address is aligned on the address boundary
			 * equal to the size of the transfer - if it isn't, then memory bursting
			 * isn't possible. As in, single transfer must be selected.
			*/
			if(src_addr & (channel->mem_width - 1))
			{
				src_max_burst = 1;
			}
			else
			{
				src_max_burst = DMA_MAX_BURST;
			}

			/* Then determine the best burst for the memory */
			src_best_burst = dma_get_best_burst(src_width, src_max_burst, fifo_level);
			src_burst_size = dma_get_burst(src_best_burst);

			if(src_burst_size < 0)
			{
				return src_burst_size;
			}

			/* Prepare the control register */
			dma_scr = FIELD_PREP(DMA_SCR_DIR_MASK, DMA_MEM_TO_PERIPH) |
					FIELD_PREP(DMA_SCR_PSIZE_MASK, dest_width) |
					FIELD_PREP(DMA_SCR_MSIZE_MASK, src_width) |
					FIELD_PREP(DMA_SCR_PBURST_MASK, dst_burst_size) |
					FIELD_PREP(DMA_SCR_MBURST_MASK, src_burst_size);

			/* Set the FIFO threshold */
			if(channel->threshold != DMA_FIFO_THRESHOLD_NONE)
			{
				channel->dma_reg.sfcr |= FIELD_PREP(DMA_FCR_FTH, channel->threshold);
			}

			/* Set the peripheral address */
			channel->dma_reg.spar = channel->dst_addr;

			/* Set the memory address */
			channel->dma_reg.sm0ar = channel->src_addr;
			break;

		case DMA_PERIPH_TO_MEM:

			/* Set the peripheral's data size */
			src_width = dma_get_buswidth(channel->periph_mem_width);
			if(src_width < 0)
				return src_width;

			/* Set the peripheral burst size */
			src_best_burst = dma_get_best_burst(channel->periph_mem_width, channel->threshold);
			src_burst_size = dma_get_burst(src_best_burst);
			if(src_burst_size < 0)
			{
				return src_burst_size;
			}

			/* Set the memory data size */
			dst_width = dma_get_max_width(channel->mem_width, src_addr, threshold);
			if(dst_width < 0)
			{
				return dst_width;
			}

			/* Set the memory burst size - bursting is not possible
			 * if the address is not aligned on the address boundary equal
			 * to the size of the transfer
			 */
			if(dst_width & (channel->mem_width - 1))
			{
				dst_max_burst = 1;
			}
			else
			{
				dst_max_burst = DMA_MAX_BURST;
			}

			dst_best_burst = dma_get_best_burst(channel->mem_width, channel->dst_max_burst, fifo_level);
			dst_burst_size = dma_get_burst(dst_best_burst);

			if(dst_burst_size < 0)
			{
				return dst_burst_size;
			}

			/* Prepare the control register */
			dma_scr = FIELD_PREP(DMA_SCR_DIR_MASK, DMA_PERIPH_TO_MEM) |
					FIELD_PREP(DMA_SCR_PSIZE_MASK, src_width) |
					FIELD_PREP(DMA_SCR_MSIZE_MASK, dst_width) |
					FIELD_PREP(DMA_SCR_PBURST_MASK, src_burst_size) |
					FIELD_PREP(DMA_SCR_MBURST_MASK, dst_burst_size);

			/* Set the FIFO threshold */
			if(channel->threshold != DMA_FIFO_THRESHOLD_NONE)
			{
				channel->dma_reg.sfcr |= FIELD_PREP(DMA_FCR_FTH, channel->threshold);
			}

			/* Set the peripheral address */
			channel->dma_reg.spar = channel->dst_addr;
			break;

		default:
			return -EINVAL;
	}

	/* Handle the IRQ configuration */
	scr |= FIELD_PREP(DMA_SCR_TCIE, (irq & DMA_SCR_TEIE) ) |
						FIELD_PREP(DMA_SCR_HTIE, (irq & DMA_SCR_HTIE)) |
						FIELD_PREP(DMA_SCR_TEIE, (irq & DMA_SCR_TEIE));

	channel->dma_reg &= ~(DMA_SCR_DIR_MASK | DMA_SCR_PSIZE_MASK |
						DMA_SCR_MSIZE_MASK | DMA_SCR_PBURST_MASK | DMA_MBURST_MASK);

	/* Set the control register */
	channel->dma_reg |= scr;

	/* Set the status to indicate it has been prepared */
	channel->status = DMA_Ready;

	return 1;
}


static bool dma_verify_ndt(struct dma_channel *channel, u32 ndt)
{
	/* The NDT (number of transactions) has restrictions based on what the P and M size are
	 * For example, if PSIZE is 8 bits and MSIZE is 16 bits, then NDT must be a multiple of 2
	 */

	const u32 p_size = channel->periph_mem_width;
	const u32 m_size = channel->mem_width;

	const u16 multiple = p_size / m_size;

	return (ndt % 4) == 0;
}

static int dma_disable_chan(struct dma_channel *channel)
{

	/* Clear the enable bit DMA_SxCR register */
	DMA_RegDef_t *base = channel->base;
	u32 *dma_scr = DMA_SxCFR(channel->channel_config.stream_num);


	if(*dma_src & DMA_SRC_EN)
	{
		dma_scr |= ~DMA_SCR_EN;

		/* Because the stream may take time to actually be disabled
	 	 * i.e if an ongoing transfer is happening and it will wait
	 	 * until it has been completed, poll until the transfer complete
	 	 * interrupt flag (TCIF in DMA_LISR or DMA_HISR) is set
	 	 */

		while(!dma_read_status_bit(channel, DMA_DMA_TCI));
	}

	return 0;
}

static u8 dma_get_fifo_thresh(struct dma_channel *channel)
{
	u32* sfcr = DMA_REG(channel->base, DMA_SxFCR(channel->channel_config.stream_number));

	return *sfcr & DMA_FIFO_THRESHOLD_FULL;
}

static bool dma_circular_config_valid(struct dma_channel *channel)
{
	/* In circular mode, the following formula needs to be
	 * respected, otherwise the circular mode configuration can
	 * lead to undefined behavior and a lack of data integrity
	 *
	 * DMA_SxNDTR = Multiple of Mburst beat * Msize / Psize
	 */

	/* Read the SxNDTR register */
	u32 *ndtr_reg = DMA_REG(channel->base, DMA_SxNDTR(channel->channel_config.stream_num));
	u16 no_items = *ndtr_reg & 0xFFF;

	u32 result = ( channel->mem_width * channel->burst_size ) / channel->periph_mem_width;

	if(no_items % result > 0)
	{
		return false;
	}

	return true;
}

static u8 dma_fifo_depth_adjusted(u8 fifo_level)
{
	return DMA_FIFO_DEPTH  / ( (fifo_level + 1) * 8);
}

static bool dma_is_burst_possible(u32 mem_size, u8 fifo_threshold_selection)
{
	/* If the threshold is none, that means the DMA is operating
	 * in direct mode, which ultimately means bursting is not allowed
	 */
	if(fifo_threshold == FIFO_THRESHOLD_NONE)
	{
		return false;
	}

	/* The size of the burst has to be aligned with the
	 * depth of the FIFO, otherwise problems can arise such as
	 * residue being left in the FIFO
	 *
	 * i.e If the memory size to be transferred in the burst is 1 Byte
	 * and the FIFO threshold level is FULL and increments of 4 is enabled
	 * for the burst size, then it would take 4 bursts of 4 beats (a word) to
	 * reach the FIFO threshold
	 */

	u8 mem_bytes = mem_size / 8;
	u32 fifo_depth_bytes = dma_fifo_depth_adjusted(fifo_threshold_selection);

	/* In order for the burst to be possible, then size of the memory has
	 * to be divisible by the current FIFO depth (accounting for the threshold)
	 */

	return (fifo_depth_bytes % mem_size) == 0;
}

/**
 * dma_get_best_burst - find the best burst size based on the FIFO level and memory size
 *
 * @mem_size: the size of the memory (in bytes)
 * @fifo_level: the FIFO threshold selection
*/
static int dma_get_best_burst(u32 mem_size, u32 max_burst, u8 fifo_level)
{
	u32 best_burst = max_burst;
	enum dma_slave_buswidth bus_width = dma_get_buswidth(mem_size);

	if(best_burst == 1)
		return 0; /* Single Burst */

	/* First check to see if the burst is possible with
	 * the given configuration
	 */
	if(dma_is_burst_possible(mem_size, fifo_level))
	{
		u8 fifo_depth_bytes = dma_fifo_depth_adjusted(fifo_level);
		u8 max_burst = DMA_MAX_BURST;

		/* Then the burst size which is valid is the burst size such that
		 * the burst beats * the size of the beat does not exceed the available
		 * FIFO depth
		 */
		while(fifo_depth_bytes < (max_burst * bus_width) )
		{
			max_burst = max_burst >> 1;
		}

		return max_burst;
	}

	return -EINVAL;
}

/**
 * dma_irq_clear - helper function to clear the irq status in the status register
 *
 * @flag: the interrupt to clear from the status register
*/
static void dma_irq_clear(struct dma_channel *channel, u8 flag)
{
	u32 status_reg, mask;
	DMA_RegDef_t *base = channel->base;


	status_reg = dma_read_status(channel);
	mask = status_reg & flag;

	if(channel->channel_config->stream_config <= DMA_LISR_MAX)
	{
		base->LIFCR |= mask;
	}
	else
	{
		base->HIFCR |= mask;
	}
}

static void dma_irq_clear_all(struct dma_channel *channel)
{
	assert(channel);
	dma_irq_clear(channel, DMA_MASKI);
}


static void dma_handle_chan_paused(struct dma_channel *channel)
{

	/* Read the configuration register of the current transfer */

	/* Read the current number of transfers to be restored upon resume */

	channel->status = DMA_PAUSED;
}

static enum irq_handled_t dma_irq_handle(struct dma_channel *channel)
{

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

