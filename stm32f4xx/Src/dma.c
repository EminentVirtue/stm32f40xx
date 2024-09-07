/*
 * dma.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Andrew Streng
 */

#include "dma.h"
#include <assert.h>
#include <stdio.h>


/* Private function declarations  */
static bool dma_is_burst_possible(u32 buf_len, u32 fifo_threshold);
static u8 dma_read_status_bit(struct dma_channel *channel, u8 bit);
static u8 dma_fifo_depth_adjusted(u8 fifo_level);
static size_t dma_residue(struct dma_channel *chan, u32 next_sg);
struct dma_desc *dma_prep_slave_sg(struct dma_channel *chan, struct list *scatterlst, const size_t d_size, enum dma_transfer_direction direction);
struct dma_desc *dma_prep_memcpy(struct dma_channel *chan, u32 src_addr, u32 dst_addr, u32 len);

/* DMA Burst functions */
static int dma_get_burst(u32 max_burst);
static int dma_get_best_burst(u32 buf_len, u32 max_burst, enum dma_slave_buswidth width, u32 fifo_level);
static bool dma_is_burst_allowed(u32 burst,u32 threshold, enum dma_slave_buswidth width);
static int dma_transfer_prep(struct dma_channel *channel, u32 buf_len, u32 buf_addr, enum dma_transfer_direction direction);

/* DMA IRQ functions */
static void dma_irq_clear(struct dma_channel *channel, u32 flag);
static void dma_irq_clear_all(struct dma_channel *channel);
static void dma_handle_chan_paused(struct dma_channel *channel);
static void dma_handle_chan_done(struct dma_channel *chan, u32 scr);
static u32 dma_read(DMA_RegDef_t *base, u32 *reg);
static void dma_write(DMA_RegDef_t *base, u32 *reg, u32 value);
static u32 dma_get_remaining_bytes(struct dma_channel *chan);
static int dma_get_buswidth(u32 width);
static void dma_sg_inc(struct dma_channel *chan);
static void dma_configure_next_sg(struct dma_channel *chan);
static int dma_get_bw_from_slave_bw(enum dma_slave_buswidth width);
static enum dma_slave_buswidth dma_get_max_width(u32 buf_len, u32 buf_addr, u32 threshold);
static bool dma_is_current_sg(struct dma_channel *chan);
static void dma_resume_reconfigure(struct dma_channel *chan);

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
void dma_pause(struct dma_channel *chan)
{

	if(chan->status == DMA_PAUSED)
		return;

	/* Clear and interrupt status */
	dma_irq_clear_all(chan);

	/* Disable the channel */
	int ret = dma_disable_chan(chan);
	if(ret < 0)
		return;

	dma_handle_chan_paused(chan);
}

/**
 * dma_stop: Stops the DMA channel with no intention of resuming
 * and returns the DMA channel to its idle state
 *
 * @channel: the DMA channel
*/
void dma_stop(struct dma_channel *chan)
{
	assert(chan != NULL);

	u32 status, dma_scr, dma_fcr;

	status = dma_read_status(chan);
	if(status)
		dma_irq_clear(chan, status);

	dma_scr = dma_read(chan->base, DMA_SxCFR(chan));
	dma_fcr = dma_read(chan->base, DMA_SxFCR(chan));

	dma_fcr &= ~DMA_FCR_FEIE;
	dma_scr &= ~DMA_MASK_IE;

	chan->status = DMA_COMPLETE;
	chan->busy = false;
}

/**
 * dma_read_status - reads the status register of the DMA
 * @channel: the DMA channel
*/
u32 dma_read_status(struct dma_channel *chan)
{

	const u8 dma_stream = chan->channel_config.stream_num;
	u8 shift_amount = 0;
	u32 dma_status_reg;

	/* Select the high/low status register to read from based on the
	 * the current stream number
	 */
	dma_status_reg = dma_stream > 3 ? chan->base->HISR : chan->base->LISR;

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
void dma_enable_interrupts(struct dma_channel *chan, u32 flags)
{
	assert(chan != NULL);

	if(chan->busy)
		return;

	/* Write the flags to the appropriate DMA_xIFCR register
	 * corresponding to the appropriate channel
	 */
	dma_write(chan->base, DMA_SxCFR(chan), flags);
}

/**
 * dma_disable_interrupts - disables the given interrupts passed in with @flags
 * @channel: the DMA peripheral
 * @flags: the IRQs to disables
*/
void dma_disable_interrupts(struct dma_channel *chan, u32 flags)
{
	assert(chan != NULL);

	if(chan->busy)
		return;

	flags &= DMA_MASKI;
	dma_write(chan->base, DMA_SxFCR(chan), flags);
}

/**
 * dma_interrupt_config - reads the current interrupt configuration of the stream
 * @channel: the dma channel
 *
*/
u32 dma_interrupt_config(struct dma_channel *chan)
{
	u32 reg;
	reg = dma_read(chan->base, DMA_SxCFR(chan));
	return reg & DMA_MASKI;
}

void dma_flush_fifo(struct dma_channel *chan)
{
	/* Only applies with DMA directions peripheral-to-memory
	 * or memory-to-memory transfers
	 */

	/* Reset the EN bit */

	/* Wait for transfer complete interrupt to be issued */
}

/**
 * dma_start - starts the transfer given
 * @channel: DMA Channel handle
 * @return: 1 for success, -1 for failure
 *
*/
int dma_start(struct dma_channel *chan)
{
	struct dma_sg_req *next_sg_req;
	struct dma_chan_reg sg_chan_reg;
	int ret;
	u32 status;

	if(chan->busy)
	{
		printf("%s(): Cannot start transfer - DMA is busy\n", __func__);
		return -EINVAL;
	}
	if(!chan->desc)
	{
		printf("%s(): No descriptor has been prepared - cannot start transfer\n", __func__);
		return -EINVAL;
	}
	if(chan->next_sg == chan->desc->num_sgs
			|| chan->next_sg > chan->desc->num_sgs)
	{
		printf("%s(): Resetting next_sg \n", __func__);
		chan->next_sg = 0;
	}

	ret = dma_disable_chan(chan);
	next_sg_req = &chan->desc->sg_req[chan->next_sg];

	/* Disable the channel first */
	if(ret < 0)
	{
		printf("%s(): Could not disable the channel\n", __func__);
		return ret;
	}
	next_sg_req = &chan->desc->sg_req[chan->next_sg];
	sg_chan_reg = next_sg_req->chan_reg;

	sg_chan_reg.scr &= ~DMA_SCR_EN;

	/* Write the prepared registers to the controller's registers */
	dma_write(chan->base, DMA_SxCFR(chan), sg_chan_reg.scr);
	dma_write(chan->base, DMA_SxSPAR(chan), sg_chan_reg.spar);
	dma_write(chan->base, DMA_SxM0AR(chan), sg_chan_reg.sm0ar);
	dma_write(chan->base, DMA_SxM1AR(chan), sg_chan_reg.sm1ar);
	dma_write(chan->base, DMA_SxNDTR(chan), sg_chan_reg.sndtr);
	dma_write(chan->base, DMA_SxFCR(chan), sg_chan_reg.sfcr);

	/* Clear any interrupts */
	status = dma_read_status(chan);

	if(status)
		dma_irq_clear(chan, status);

	/* Enable the transfer */
	chan->status = DMA_IN_PROGRESS;
	chan->busy = true;
	sg_chan_reg.scr |= DMA_SCR_EN;
	dma_write(chan->base, DMA_SxCFR(chan), sg_chan_reg.scr);

	return 1;
}

/**
 * dma_prep_desc - Prepares a descriptor to be used for the DMA transfers based on @dma_direction
 *
 * A dma_channel handle should first be configured before initializing the descriptor.
 * The following should first be configured : irq, threshold, dst_max_burst, src_max_burst,
 * src_addr, dst_addr otherwise initial value is assumed
 *
 * @channel: DMA channel handle
 * @scatterlst: The scatterlst of sgs - only applies if the direction MEM-TO-DEV/DEV-TO-MEM,
 * otherwise null can be provided and it is ignored
 * @buf_len: The length of the copy buffer - only applies to MEM-MEM direction modes,
 * with all other direction, this argument is ignored and 0 can be provided
 * @dma_direction: the direction the transactions in the descriptor will be
 * @return: pointer to new descriptor if success, null if failed
 *
*/
struct dma_desc *dma_prep_desc(struct dma_channel *chan,
							   struct list *scatterlst,
							   const size_t buf_len,
							   enum dma_transfer_direction direction)
{
	struct dma_desc *dma_new_desc;
	u32 src_addr, dst_addr;
	src_addr = chan->sconfig.dst_addr;
	dst_addr = chan->sconfig.src_addr;

	switch(direction)
	{
		case MEM_TO_DEV:
		case DEV_TO_MEM:
		{
			dma_new_desc = dma_prep_slave_sg(chan, scatterlst, 22, direction);
			assert(dma_new_desc != NULL);
			return dma_new_desc;
		}
		case MEM_TO_MEM:
		{
			dma_new_desc = dma_prep_memcpy(chan, src_addr, dst_addr, buf_len);
			assert(dma_new_desc != NULL);
			return dma_new_desc;
		}
		default:
			return NULL;
	}
}


/**
 * dma_resume - Resume and reconfigure (if applicable) the paused channel
 * @channel: DMA channel handle
 * @return: 1 for success, -1 for failure
 *
*/
int dma_resume(struct dma_channel *chan)
{
	u32 scr, ndtr,spar,sm0ar,sm1ar, offset;
	struct dma_chan_reg *reg;

	reg = &chan->dma_reg;

	if(chan->status != DMA_PAUSED)
		return -EINVAL;

	scr = dma_read(chan->base, DMA_SxCFR(chan));

	/* Can only be done if the channel is disabled */
	if(scr & DMA_SCR_EN)
		return -EINVAL;

	/* Restore values store in the dma_reg saved off when pausing */
	scr = reg->scr;
	ndtr = reg->sndtr;
	spar = reg->spar;
	sm0ar = reg->sm0ar;
	sm1ar = reg->sm1ar;
	offset = ndtr;

	/* If PSIZE is 1 byte (00), then the increment number is just the number of
	 * transfers (1 byte per transfer). However, if it is 10 (i.e) (a half word),
	 * then the offset is NDT * 2 (bytes)
	 */
	offset <<= FIELD_GET(DMA_SCR_PSIZE_MASK,dma_read(chan->base, DMA_SxCFR(chan)));

	/* The peripheral and/or memory addresses need to be updated if
	 * the pointer incrementing mode is enabled for the channel
	 */
	if(scr & DMA_SCR_PINC)
	{
		dma_write(chan->base, DMA_SxSPAR(chan), spar + offset);
	}
	else
	{
		dma_write(chan->base, DMA_SxSPAR(chan), spar);
	}

	/* Do not increment the memory address if it is not enabled */
	if(!(scr & DMA_SCR_MINC))
		offset = 0U;

	/* If double buffer mode is enabled and CT=1, then the sm1ar
	 * address needs to be updated by the offset since it is the current
	 * target
	 */
	if( (scr & DMA_SCR_DBM) & (scr & DMA_SCR_CT))
	{
		dma_write(chan->base, DMA_SxM1AR(chan), sm1ar + offset);
	}
	else
	{
		dma_write(chan->base, DMA_SxM0AR(chan), sm0ar + offset);
	}

	/* The NDTR value has to be reset otherwise the internal counter
	 * will not be accurate
	 */
	dma_write(chan->base, DMA_SxNDTR(chan), ndtr);

	/* Set the enable bit and write configuration register to the DMA */
	chan->status = DMA_IN_PROGRESS;
	scr |= DMA_SCR_EN;
	dma_write(chan->base, DMA_SxCFR(chan), scr);

	return 1;
}

/**
 * dma_prep_slave_sg - prepares a descriptor with a list of DMA slave transactions
 * @chan: DMA channel handle
 * @scatterlst: The list of the DMA transactions
 * @data_len: The size of list data item (i.e for a u32 is is 4 bytes)
 * @return - pointer to new dma_desc
*/
struct dma_desc* dma_prep_slave_sg(struct dma_channel *chan,
								   struct list *scatterlst,
								   const size_t d_size,
								   enum dma_transfer_direction direction)
{
	assert(chan != NULL);
	assert(scatterlst != NULL);

	u32 sg_len,s_config;
	struct dma_desc *desc;

	sg_len = list_size(scatterlst);
	//s_config = chan->channel_config->stream_config;
	desc = NULL;

	if(sg_len < 1)
	{
		return NULL;
	}

	/* Allocate the descriptor */
	desc = (struct dma_desc*)malloc(sizeof(struct dma_desc) + (d_size * sg_len));
	if(!desc)
	{
		return NULL;
	}

	/* Set the peripheral flow controller */
	if(s_config & DMA_SCR_PFCTRL)
	{
		chan->dma_reg.scr |= DMA_SCR_PFCTRL;
	}
	else
	{
		chan->dma_reg.scr &= ~DMA_SCR_PFCTRL;
	}

	/* Construct the DMA control registers for each transaction */
	LIST_ITEM curr;
	u32 i;

	list_for_each(&curr, scatterlst->head)
	{
		struct dma_sg_req *curr_dma_sg_reg = (struct dma_sg_req*)(&curr.data);
		if(!curr_dma_sg_reg)
			goto err;

		int ret = dma_transfer_prep(chan, curr_dma_sg_reg->len, curr_dma_sg_reg->dma_addr, direction);
		if(ret < 0)
			goto err;

		u32 dma_len, nb_data_items, buswidth;
		dma_len = curr_dma_sg_reg->len;

		/* Get the buswidth of the transaction */
		buswidth = dma_get_max_width(dma_len,curr_dma_sg_reg->dma_addr,chan->threshold);
		buswidth = dma_get_bw_from_slave_bw(buswidth);
		if(buswidth < 0)
			goto err;

		//reg = curr_dma_sg_reg->chan_reg;
		nb_data_items = dma_len / buswidth;

		if(nb_data_items > DMA_MAX_ITEMS)
			goto err;

		/* Initialize the chan_reg address targets with the current list item */
		desc->sg_req[i].chan_reg.scr = chan->dma_reg.scr;
		desc->sg_req[i].chan_reg.sfcr = chan->dma_reg.sfcr;
		desc->sg_req[i].chan_reg.sm0ar = curr_dma_sg_reg->chan_reg.sm0ar;
		desc->sg_req[i].chan_reg.sm1ar = curr_dma_sg_reg->chan_reg.sm1ar;
		desc->sg_req[i].chan_reg.sndtr = nb_data_items;

		i++;
	}

	desc->num_sgs = sg_len;
	desc->cyclic = false;
	return desc;

err:
	free(desc);
	return NULL;

}

/**
 * dma_prep_cyclic - prepares a cyclic descriptor
 * @chan: DMA channel handle
 * @dma_direction: The direction of the cyclic transfer
 * @buf_addr: The address of the transaction buffer
 * @buf_len: The length of the buffer
 * @period_len: The length of the period - the length of each transfer
 * @d_size: The size of
 * @return - pointer to new dma_desc
*/
struct dma_desc *dma_prep_cyclic(struct dma_channel *chan, u32 dma_direction, u32 buf_addr, size_t buf_len,
		size_t period_len)
{
	u32 nb_data_items, buswidth, num_periods, i;
	struct dma_desc *desc;

	if(!buf_len || !period_len)
		return NULL;

	/* The buffer length and size of the transfer cannot be the same
	 * size in circular mode
	 */
	if(buf_len % period_len)
		return NULL;

	int ret = dma_transfer_prep(chan,
								buf_len,
								buf_addr,
								dma_direction);
	if(ret < 0)
		return NULL;

	/* Determine the buswidth */
	buswidth = dma_get_max_width(buf_len, buf_addr, chan->threshold);
	buswidth = dma_get_bw_from_slave_bw(buswidth);

	if(buswidth < 0)
		return NULL;

	/* Determine the number of data items */
	nb_data_items = period_len / buswidth;
	if(nb_data_items > DMA_MAX_ITEMS)
		return NULL;

	/* The number of periods is the number of transfers it takes
	 * to completely transfer the up to the buffer length given
	 * a period length
	 */
	num_periods = buf_len / period_len;

	/* If the total buffer can be transferred in one period, then circular mode should be
	 * enabled as the hardware will just start from the beginning. Otherwise DBM should be enabled
	 * so the software is interrupted when a transfer complete happens and the sg internals can then be updated
	 */
	if(num_periods == 1)
	{
		chan->dma_reg.scr |= DMA_SCR_CIRC;
	}
	else
	{
		chan->dma_reg.scr |= DMA_SCR_DBM;
		/* Set the target memory as SM0AR */
		chan->dma_reg.scr &= ~DMA_SCR_CT;
	}

	/* Allocate the descriptor */
	desc = DMA_MALLOC_DESC(DMA_SGREQ_SIZE, num_periods);

	if(!desc)
	{
		return NULL;
	}

	for(i = 0; i < num_periods; i++)
	{
		desc->sg_req[i].chan_reg.scr = chan->dma_reg.scr;
		desc->sg_req[i].chan_reg.sfcr = chan->dma_reg.sfcr;
		desc->sg_req[i].chan_reg.spar = chan->dma_reg.spar;
		desc->sg_req[i].chan_reg.sm0ar = buf_addr;
		desc->sg_req[i].chan_reg.sm1ar = buf_addr;
		desc->sg_req[i].chan_reg.sndtr = nb_data_items;
	}

	chan->desc->cyclic = true;
	chan->desc->num_sgs = num_periods;

	/* Clear peripheral flow controller */
	chan->dma_reg.scr &= ~DMA_SCR_PFCTRL;

	return desc;
}

/**
 * dma_prep_memcpy - prepares a descriptor with for DMA direction MEM-MEM
 *
 * Function should be used for DMA transfers when the direction is MEM-MEM
 * Given the length (@len) of the transfer, a descriptor will be generated whose
 * length is determined by the length of the transfer / buswidth. The memory offsets
 * are updated at each index of the descriptor so that the subsequent transfers start
 * where the previous one left off.
 * For example, if the length of the transfer where 128 bits in length and size of the destination
 * address
 * @chan: DMA channel handle
 * @src_addr: The address of the source
 * @dst_addr: The address of the destination
 * @len: The length of the transfer (bytes)
 * @return - pointer to new dma_desc
*/
struct dma_desc *dma_prep_memcpy(struct dma_channel *chan,
								u32 src_addr,
								u32 dst_addr,
								u32 len)
{
	struct dma_desc *desc;
	u32 threshold, i, num_sgs, offset, best_burst,max_transfer_len;
	int dma_burst;
	enum dma_slave_buswidth max_buswidth;

	threshold = chan->threshold;

	/* Any length under the boundary end address can be completed with one transfer */
	num_sgs = DIV_ROUND_UP(len, DMA_MEM_END);

	if(!threshold)
	{
		printf("%s(): Direct mode not possible for MEM-MEM transfers \n", __func__);
		return NULL;
	}
	/* Only DMA2 controller can perform memory to memory transfers */
	if(chan->base == DMA1)
	{
		printf("%s(): DMA1 controller selected for MEM-MEM transfer, but only DMA2 can handle this transfer type \n", __func__);
		return NULL;
	}

	desc = DMA_MALLOC_DESC(DMA_SGREQ_SIZE, num_sgs);
	if(!desc)
	{
		return NULL;
	}

	/* The max memory region size of the DMA controller is 0xffff
	 * Valid transfers start from 0 and end at the boundary above.
	 * So it is best to round the the max boundary address down to the closest
	 * multiple of the full FIFO threshold (16 bytes) in order to avoid run-off.
	 *
	 * If the buffer length exceeds this boundary, then the transfer needs to be split
	 * up into multiple transfers. Enabling the MINC bit will automatically handle
	 * adjusting the buffer offset after each successive FIFO drain. Given that MINC is enabled,
	 * it's extremely important to make sure the buffer length aligns with the destination length,
	 * otherwise the buffer can be copied into unintended memory regions.
	 */

	/* The max size of the transfer is the nearest multiple of the boundary end point and
	 * the full FIFO size
	 */
	max_transfer_len = ALIGN_DOWN(DMA_MEM_END,DMA_FIFO_DEPTH);
	u32 xfer_count;

	for(i = 0; offset < len; i++)
	{
		/* Since each transfer is totaling 1 byte,
		 * the number of transfers required is the minimum between the
		 * max transfer length the DMA supports and the length of the supplied buffer
		 */
		xfer_count = min_t(size_t, len - offset, max_transfer_len);

		/* Adjust the offset to add to the base memory address */
		offset += xfer_count;
		max_buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;

		/* Get the best burst */
		best_burst = dma_get_best_burst(len, DMA_MAX_BURST, max_buswidth, threshold);
		dma_burst = dma_get_burst(best_burst);
		if(dma_burst < 0)
		{
			free(desc);
			return NULL;
		}

		/* Prepare the configuration register */
		desc->sg_req[i].chan_reg.scr = (FIELD_PREP(DMA_SCR_CHSEL_MASK, DMA_MEM_TO_MEM) |
										FIELD_PREP(DMA_SCR_MBURST_MASK, dma_burst) |
										FIELD_PREP(DMA_SCR_MSIZE_MASK, max_buswidth) |
										DMA_SCR_MINC |
										DMA_SCR_PINC |
										DMA_SCR_TCIE |
										DMA_SCR_TEIE);

		/* Disable direct mode in FIFO configuration register
		 * and enable the error interrupt
		 */
		desc->sg_req[i].chan_reg.sfcr = FIELD_PREP(DMA_FCR_FTH, threshold) |
										 DMA_FCR_DMDIS |
										 DMA_FCR_FEIE;

		desc->sg_req[i].chan_reg.sm0ar = src_addr + offset;
		desc->sg_req[i].chan_reg.sm1ar = src_addr + offset;
		desc->sg_req[i].chan_reg.spar = dst_addr;
		desc->sg_req[i].chan_reg.sndtr = xfer_count;
	}

	chan->desc->num_sgs = num_sgs;
	chan->desc->cyclic = false;

	return desc;
}

/**
 * dma_irq_handle - Handles the the DMA interrupt
 * @channel: the DMA peripheral
 * @return - irq_handle_t value representing the IRQ handled
 *
 * Function should be called when the DMA ISR is executed
 * to ensure the handle has the proper state
*/
void dma_irq_handle(struct dma_channel *chan)
{
	u32 scr, sfcr, status;

	status = dma_read_status(chan);
	scr = dma_read(chan->base, DMA_SxCFR(chan));
	sfcr = dma_read(chan->base, DMA_SxFCR(chan));

	if(status & DMA_FEI)
	{
		dma_irq_clear(chan, DMA_FEI);
		status &= ~DMA_FEI;

		if(sfcr & DMA_FCR_FEIE)
		{
			u32 width;
			u32 fifo_depth;

			fifo_depth = dma_fifo_depth_adjusted(chan->threshold);

			/* Check to see if a faulty FIFO configuration was the cause */
			if(chan->channel_config.stream_config & DMA_PERIPH_TO_MEM)
			{
				width = FIELD_GET(DMA_SCR_PSIZE_MASK, scr);
			}
			else if(chan->channel_config.stream_config & DMA_MEM_TO_PERIPH)
			{
				width = FIELD_GET(DMA_SCR_MSIZE_MASK, scr);
			}

			width = dma_get_buswidth(width);
			if(width > 0)
			{
				if(fifo_depth % width)
				{
					printf("%s(): FIFO Error: bad configuration \n", __func__);
					goto err;
				}
			}
			else
			{
				printf("%s(): FIFO Error: bad configuration \n", __func__);
				goto err;

			}

			/* If the transfer hasn't been completed, meaning it has been started,
			 * then it is most likely an overrun error
			 */
			if( !(status & DMA_SCR_TCIE))
			{
				printf("%s(): FIFO error: overrun has occurred \n", __func__);
			}

			/* Otherwise, the transfer hasn't started and the error may have been
			 * brought about by a peripheral request before the memory bus has been
			 * granted
			 */
			else
			{
				printf("%s(): FIFO error: underrun has occurred \n", __func__);
			}
		}
	}

	if(status & DMA_DMEIF)
	{
		dma_irq_clear(chan, DMA_SCR_DMEIE);
		status &= ~DMA_DMEIF;

		if(scr & DMA_SCR_DMEIE)
		{
			printf("%s(): DMA direct mode error \n", __func__);
		}

	}

	if(status & DMA_HTI)
	{
		dma_irq_clear(chan, DMA_HTI);
		status &= ~DMA_SCR_HTIE;
	}
	if(status & DMA_TCI)
	{
		if(scr & DMA_SCR_TCIE)
		{
			if(chan->status != DMA_PAUSED)
			{
				dma_handle_chan_done(chan, scr);
			}
		}
	}

err:
	if(status)
	{
		dma_irq_clear(chan, status);

		/* Then a transfer error interrupt most likely occurred
		 * and the hardware automatically disables the stream with
		 * this interrupt
		 */
		if(!(scr & DMA_SCR_EN))
		{
			printf("%s(): DMA stream disabled by HW\n", __func__);
		}
	}
}
/**
 * dma_set_fifo_config - sets the internal FCR register of the dma_reg struct
 * according to the parameters of the channel
 *
 * @channel: the dma channel
 *
*/
void dma_set_fifo_config(struct dma_channel *chan,
						 u32 src_burst,
						 u32 dst_burst)
{
	/* Clear the FIFO configuration register */
	chan->dma_reg.sfcr = ~DMA_FCR_MASK;

	/* Bursting is not possible with direct mode */
	if(!src_burst && !dst_burst)
	{
		chan->dma_reg.scr |= DMA_SCR_DMEIE;
	}
	else
	{
		/* Disable direct mode in the FIFO configuration */
		chan->dma_reg.sfcr |= DMA_FCR_DMDIS;

		/* Set the FIFO threshold and enable FIFO error interrupt */
		chan->dma_reg.sfcr |= (DMA_FCR_FTH & chan->threshold);
		chan->dma_reg.sfcr |= DMA_FCR_FEIE;
	}
}

/**
 * dma_tx_status - Read the remaining length to be transferred by the descriptor
 *
 * @channel: the dma channel
 * @return: the number of remaining bytes
 *
*/
u32 dma_tx_status(struct dma_channel *chan)
{
	assert(chan != NULL);
	u32 remaining = 0;

	if(chan->status == DMA_COMPLETE)
		return remaining;

	remaining = dma_residue(chan, chan->next_sg);
	return remaining;
}


/* Private function definitions */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

static int dma_get_bw_from_slave_bw(enum dma_slave_buswidth width)
{
	switch(width)
	{
		case DMA_SLAVE_BUSWIDTH_1_BYTE:
			return Byte;
		case DMA_SLAVE_BUSWIDTH_2_BYTES:
			return HalfWord;
		case DMA_SLAVE_BUSWIDTH_4_BYTES:
			return Word;
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

static enum dma_slave_buswidth dma_get_max_width(u32 buf_len,
												 u32 buf_addr,
												 u32 threshold)
{
	enum dma_slave_buswidth max_width;

	/* For bursting with FIFO thresholds, the burst size multiplied by the data size must not exceed the FIFO size.
	 * If the threshold of the FIFO is Full, that means the max possible width of the bus
	 * while being able to burst would be 4 bytes (a word), otherwise the max width the bus can
	 * be in order to possibly burst is 2 bytes. For example, if MSIZE were 4 bytes and the FIFO threshold were FULL,
	 * then the only burst possible would be INCR4 - because MSIZE(4) * burst size (4) <= FIFO size (16)
	 * If the FIFO threshold were 3/4, the total size of the FIFO is 12 bytes and MSIZE(4) * burst size(4) > FIFO size (16)
	 * By setting the max width, a burst configuration can be guaranteed.
	 */
	if(threshold == DMA_FIFO_THRESHOLD_FULL)
	{
		max_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}
	else
	{
		max_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	}

	while( ((buf_len < max_width) || (buf_len % max_width))
			&& buf_len > DMA_SLAVE_BUSWIDTH_1_BYTE)
	{
		max_width >>= 1;
	}

	/* The buswidth and memory width is not aligned, set max width to 1 byte,
	 * the address size will at least by 1 byte.
	 */
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
 * @buf_len - Length of the source buffer in bytes
 * @buf_addr - The address of the source buffer
 * @width: The buswidth
 * @direction: Direction of the DMA transfer
*/
static int dma_transfer_prep(struct dma_channel *chan,
							 u32 buf_len, u32 buf_addr,
							 enum dma_transfer_direction direction)
{

	int src_width, dst_width, threshold;
	int dst_buswidth, src_buswidth;
	int src_burst_size, dst_burst_size;
	u32 src_best_burst, dst_best_burst;
	u32 src_addr, dst_addr, src_max_burst, dst_max_burst, irq, dma_scr;

	src_max_burst = chan->sconfig.src_max_burst;
	dst_max_burst = chan->sconfig.dst_max_burst;
	dst_width = chan->sconfig.dst_addr_width;
	src_width = chan->sconfig.src_addr_width;
	irq = chan->irq;
	threshold = chan->threshold;

	switch(direction)
	{
		case DMA_MEM_TO_PERIPH:

			/* Set the dev's data size */
			dst_buswidth = dma_get_buswidth(dst_width);
			if(dst_buswidth < 0)
			{
				return dst_buswidth;
			}

			/* Get the best burst available for the destination given the
			 * memory width, the FIFO threshold and width of the destination memory
			 */
			dst_best_burst = dma_get_best_burst(dst_width,
												dst_max_burst,
												dst_buswidth,
												threshold);

			dst_burst_size = dma_get_burst(dst_best_burst);

			if(dst_burst_size < 0 )
			{
				return dst_burst_size;
			}

			/* Determine the max width the memory can be */
			src_buswidth = dma_get_max_width(buf_len,
											 buf_addr,
											 threshold);

			/* Need to check if the address is aligned on the address boundary
			 * equal to the size of the transfer - if it isn't, then memory bursting
			 * isn't possible. As in, single transfer must be selected.
			*/
			if(src_addr & (src_width - 1))
			{
				src_max_burst = 1;
			}
			else
			{
				src_max_burst = DMA_MAX_BURST;
			}

			/* Then determine the best burst for the memory */
			src_best_burst = dma_get_best_burst(buf_len,
												src_max_burst,
												src_buswidth,
												threshold);

			src_burst_size = dma_get_burst(src_best_burst);

			if(src_burst_size < 0)
			{
				return src_burst_size;
			}

			/* Prepare the control register */
			dma_scr = FIELD_PREP(DMA_SCR_DIR_MASK, DMA_MEM_TO_PERIPH) |
					  FIELD_PREP(DMA_SCR_PSIZE_MASK, dst_buswidth) |
					  FIELD_PREP(DMA_SCR_MSIZE_MASK, src_buswidth) |
					  FIELD_PREP(DMA_SCR_PBURST_MASK, dst_burst_size) |
					  FIELD_PREP(DMA_SCR_MBURST_MASK, src_burst_size);

			/* Set the FIFO threshold */
			if(chan->threshold != DMA_FIFO_THRESHOLD_NONE)
			{
				chan->dma_reg.sfcr |= FIELD_PREP(DMA_FCR_FTH, threshold);
			}

			/* Set the peripheral address */
			chan->dma_reg.spar = dst_addr;

			/* Set the memory address */
			chan->dma_reg.sm0ar = src_addr;
			break;

		case DMA_PERIPH_TO_MEM:

			/* Set the dev's data size */
			src_buswidth = dma_get_buswidth(src_width);
			if(src_buswidth < 0)
				return src_buswidth;

			/* Set the peripheral burst size */
			src_best_burst = dma_get_best_burst(buf_len,
												src_max_burst,
												src_buswidth,
												chan->threshold);

			src_burst_size = dma_get_burst(src_best_burst);
			if(src_burst_size < 0)
			{
				return src_burst_size;
			}

			/* Set the memory data size */
			dst_buswidth = dma_get_max_width(buf_len,
										  	 buf_addr,
											 threshold);
			if(dst_width < 0)
			{
				return dst_width;
			}

			/* Set the memory burst size - bursting is not possible
			 * unless the memory address boundary is aligned on the size of
			 * the transfer
			 */
			if(buf_addr & (buf_len - 1))
			{
				dst_max_burst = 1;
			}
			else
			{
				dst_max_burst = DMA_MAX_BURST;
			}

			dst_best_burst = dma_get_best_burst(buf_len,
												dst_max_burst,
												dst_buswidth,
												threshold);

			dst_burst_size = dma_get_burst(dst_best_burst);

			if(dst_burst_size < 0)
			{
				return dst_burst_size;
			}

			/* Prepare the control register */
			dma_scr = FIELD_PREP(DMA_SCR_DIR_MASK, DMA_PERIPH_TO_MEM) |
					  FIELD_PREP(DMA_SCR_PSIZE_MASK, src_buswidth) |
					  FIELD_PREP(DMA_SCR_MSIZE_MASK, dst_buswidth) |
					  FIELD_PREP(DMA_SCR_PBURST_MASK, src_burst_size) |
					  FIELD_PREP(DMA_SCR_MBURST_MASK, dst_burst_size);

			/* Set the FIFO threshold */
			if(threshold != DMA_FIFO_THRESHOLD_NONE)
			{
				chan->dma_reg.sfcr |= FIELD_PREP(DMA_FCR_FTH, threshold);
			}

			/* Set the peripheral address */
			chan->dma_reg.spar = chan->sconfig.src_addr;
			break;

		default:
			return -EINVAL;
	}

	/* Handle the IRQ configuration */
	dma_scr |= FIELD_PREP(DMA_SCR_TCIE, (irq & DMA_SCR_TEIE) ) |
						FIELD_PREP(DMA_SCR_HTIE, (irq & DMA_SCR_HTIE)) |
						FIELD_PREP(DMA_SCR_TEIE, (irq & DMA_SCR_TEIE));

	dma_scr &= ~(DMA_SCR_DIR_MASK |
					   DMA_SCR_PSIZE_MASK |
					   DMA_SCR_MSIZE_MASK |
					   DMA_SCR_PBURST_MASK |
					   DMA_SCR_MBURST_MASK);

	/* Set the control register */
	chan->dma_reg.scr |= dma_scr;

	/* Set the status to indicate it has been prepared */
	chan->status = DMA_READY;
	chan->busy = false;

	return 1;
}

static void dma_configure_next_sg(struct dma_channel *chan)
{
	struct dma_sg_req *next_sg_req;
	u32 dma_scr;
	u32 dma_sm0ar, dma_sm1ar;

	/* Get the next SG from the list */
	next_sg_req = &chan->desc->sg_req[chan->next_sg];
	dma_scr = dma_read(chan->base, DMA_SxCFR(chan));

	/* Set the sg's SM0AR as the DMA's new SM0AR */
	if(dma_scr & DMA_SCR_CT)
	{
		dma_sm0ar = next_sg_req->chan_reg.sm0ar;
		dma_write(chan->base, DMA_SxM0AR(chan), dma_sm0ar);
	}
	else
	{
		dma_sm1ar = next_sg_req->chan_reg.sm1ar;
		dma_write(chan->base, DMA_SxM0AR(chan), dma_sm1ar);
	}
}

int dma_disable_chan(struct dma_channel *chan)
{

	/* Clear the enable bit DMA_SxCR register */
	u32 dma_scr;
	dma_scr = dma_read(chan->base, DMA_SxCFR(chan));


	if(dma_scr & DMA_SCR_EN)
	{
		dma_scr |= ~DMA_SCR_EN;
		dma_write(chan->base, DMA_SxCFR(chan), dma_scr);

		/* Because the stream may take time to actually be disabled
	 	 * i.e if an ongoing transfer is happening and it will wait
	 	 * until it has been completed, poll until the transfer complete
	 	 * interrupt flag (TCIF in DMA_LISR or DMA_HISR) is set
	 	 */

		while(!dma_read_status_bit(chan, DMA_TEI));
	}

	return 0;
}

static u8 dma_fifo_depth_adjusted(u8 fifo_level)
{
	return (fifo_level + 1) * 4;
}

static bool dma_is_burst_possible(u32 buf_len, u32 threshold)
{
	/* If the threshold is none, that means the DMA is operating
	 * in direct mode, which ultimately means bursting is not allowed
	 */
	if(threshold == FIFO_THRESHOLD_NONE)
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
	u32 fifo_depth_adjusted = dma_fifo_depth_adjusted(threshold);

	/* In order for the burst to be possible, then size of the buffer has
	 * to be divisible by the FIFO depth with the given threshold
	 *
	 * i.e if the threshold selection was 3/4 full, then the buffer length has to be
	 * divisible by 12, since 3/4 of the DMA FIFO is 12 bytes
	 */

	return (buf_len % fifo_depth_adjusted) == 0;
}

static bool dma_is_burst_allowed(u32 burst,
								 u32 threshold,
								 enum dma_slave_buswidth width)
{
	/* Bursting not possible in direct mode */
	if(threshold == DMA_FIFO_THRESHOLD_NONE)
		return false;

	u32 fifo_adjusted;
	fifo_adjusted = dma_fifo_depth_adjusted(threshold);

	if(width < DMA_SLAVE_BUSWIDTH_UNDEFINED)
	{
		if(burst > 0 )
		{

			/* In order for the burst to be valid, the burst size multiplied
			 * by the data size must not exceed the FIFO size
			 */

			if(fifo_adjusted < (width * burst))
				return false;
		}
		return true;
	}

	return false;
}


/**
 * dma_get_best_burst - find the best burst size based on the FIFO level and memory size
 *
 * @buf_len: the length of the buffer (in bytes)
 * @fifo_level: the FIFO threshold selection
*/
static int dma_get_best_burst(u32 buf_len,
							  u32 max_burst,
							  enum dma_slave_buswidth width,
							  u32 threshold)
{
	u32 best_burst = max_burst;

	/* No burst will be found with the given threshold */
	if(dma_is_burst_possible(buf_len, threshold))
		return -EINVAL;

	/* Single Burst */
	if(best_burst == 1)
		return -EINVAL;

	/* First check to see if the burst is possible with
	 * the given configuration
	 */

	max_burst = DMA_MAX_BURST;

	/* Then the burst size which is valid is the burst size such that
	 * the burst beats * the size of the beat does not exceed the available
	 * FIFO depth
	 */
	while(buf_len < (max_burst * width) ||
			dma_is_burst_allowed(max_burst, threshold, width ))
	{
		max_burst = max_burst >> 1;
	}

	return max_burst;
}

/**
 * dma_irq_clear - helper function to clear the irq status in the status register
 *
 * @flag: the interrupt to clear from the status register
*/
static void dma_irq_clear(struct dma_channel *chan, u32 flag)
{
	assert(chan != NULL);
	u32 status_reg, mask;

	status_reg = dma_read_status(chan);
	mask = status_reg & flag;

	if(DMA_STREAM(chan) <= DMA_LISR_MAX)
	{
		chan->base->LIFCR |= mask;
	}
	else
	{
		chan->base->HIFCR |= mask;
	}
}

static void dma_irq_clear_all(struct dma_channel *chan)
{
	assert(chan != NULL);
	dma_irq_clear(chan, DMA_MASKI);
}


static void dma_handle_chan_paused(struct dma_channel *chan)
{

	/* Read the configuration register of the current transfer
	 * to be restored upon resuming */
	u32 dma_scr, dma_ndtr;
	dma_scr = dma_read(chan->base, DMA_SxCFR(chan));

	/* In the case that the channel's DBM or circular mode has been disabled
	 */
	if(chan->desc && chan->desc->cyclic)
	{
		if(chan->desc->num_sgs == 1)
		{
			dma_scr |= DMA_SCR_CIRC;
		}
		else
		{
			dma_scr |= DMA_SCR_DBM;
		}
	}

	/* Save off the configuration register */
	chan->dma_reg.scr = dma_scr;

	/* Before reading the NDTR register, double buffer/circular mode needs to be disabled
	 * Otherwise, the hardware can possibly update the current transferring item and thus influence
	 * the true NDTR when the channel may be reconfigured
	 */
	if(chan->desc && chan->desc->cyclic)
	{
		dma_scr &= ~(DMA_SCR_CIRC | DMA_SCR_DBM);
		dma_write(chan->base, DMA_SxCFR(chan), dma_scr);
	}

	/* Read the current number of transfers to be
	 * restored upon resuming */
	dma_ndtr = dma_read(chan->base, DMA_SxNDTR(chan));
	chan->dma_reg.sndtr = dma_ndtr;

	chan->status = DMA_PAUSED;
}

/**
 * dma_resume_reconfigure - Restarts a DMA (labeled) cyclic descriptor
 *
 * This will happen upon a transfer complete interrupt if a circular mode enabled stream in paused then
 * resumed. Since circular/double buffer is temporarily disabled in order to read the NDTR register accurately, the
 * stream needs to reconfigured with it's configurations at the time of the pause as well re-enabling DBM or CIRC
 *
 * @chan: DMA peripheral
*/
static void dma_resume_reconfigure(struct dma_channel *chan)
{
	struct dma_sg_req *next_sg_req;
	u32 dma_scr, status;

	/* Clear interrupts */
	status = dma_read_status(chan);

	if(status)
		dma_irq_clear(chan, status);

	/* Read SCR */
	dma_scr = dma_read(chan->base, DMA_SxCFR(chan));

	/* In the case the paused transfer was the last transfer */
	if(!chan->next_sg)
	{
		next_sg_req = &chan->desc->sg_req[chan->desc->num_sgs - 1];
	}
	else
	{
		next_sg_req = &chan->desc->sg_req[chan->next_sg - 1];
	}

	/* Reconfigure the memory targets */
	dma_write(chan->base, DMA_SxM0AR(chan), next_sg_req->chan_reg.sm0ar);
	dma_write(chan->base, DMA_SxM1AR(chan), next_sg_req->chan_reg.sm1ar);

	/* Reconfigure the SPAR */
	dma_write(chan->base, DMA_SxSPAR(chan), next_sg_req->chan_reg.spar);

	/* Reconfigure the NDTR - with cyclic mode NDTR doesn't change between transfers */
	dma_write(chan->base, DMA_SxNDTR(chan), next_sg_req->chan_reg.sndtr);

	/* Reconfigure circular/double buffer mode if needed */
	if(next_sg_req->chan_reg.scr & DMA_SCR_CIRC)
	{
		dma_scr |= DMA_SCR_CIRC;
	}
	if(next_sg_req->chan_reg.scr & DMA_SCR_DBM)
	{
		dma_scr |= DMA_SCR_DBM;
		if(next_sg_req->chan_reg.scr & DMA_SCR_CT)
		{
			dma_scr &= ~DMA_SCR_CT;
		}
		else
			dma_scr |= DMA_SCR_CT;
	}

	/* Load configuration register */
	dma_write(chan->base, DMA_SxCFR(chan), dma_scr);
	dma_configure_next_sg(chan);

	/* Enable the stream */
	dma_scr |= DMA_SCR_EN;
	dma_write(chan->base, DMA_SxCFR(chan), dma_scr);
}

/**
 * dma_handle_chan_done - Handle stream done DMA interrupt
 *
 * @base: DMA peripheral
 * @reg: pointer to the register to read from
*/
static void dma_handle_chan_done(struct dma_channel *chan, u32 scr)
{
	if(!chan->desc)
		return;

	/* Check cyclic mode and update anything as needed
	 * The hardware handles starting the next transfer after the ISR */
	if(chan->desc->cyclic)
	{
		/* If circular or double buffer mode is not enabled
		 * then the channel has been paused and needs to be reconfigured
		 */
		dma_sg_inc(chan);

		if( !(scr & DMA_SCR_CIRC) && !(scr & DMA_SCR_DBM))
		{
			dma_resume_reconfigure(chan);
			printf("%s(): Resume/Reconfigure for cyclic transfer\n", __func__);
		}

		/* Otherwise, just update the memory addresses with
		 * next sg transfer
		 */
		else
		{
			dma_configure_next_sg(chan);
		}
	}

	/* If the transfer is not cyclic, then submit the next
	 * transfer in the list
	 */
	else
	{
		chan->status = DMA_COMPLETE;
		if(chan->next_sg == chan->desc->num_sgs)
		{
			if(!chan->desc->persistent)
			{
				printf("%s(): Max sgs reached for non-cyclic transfer - deleting descriptor", __func__);
				free(&chan->desc);
				chan->desc = NULL;
			}
		}
		dma_start(chan);
	}
}


/**
 * dma_read - read from a register of the DMA peripheral
 *
 * @base: DMA peripheral
 * @reg: pointer to the register to read from
*/
static u32 dma_read(DMA_RegDef_t *base, u32 *reg)
{
	assert(base != NULL);
	assert(reg != NULL);

	u32 *offset = ((u32*)base) + *reg;
	if(offset)
	{
		return (u32)(*offset);
	}

	return -EINVAL;
}

/**
 * dma_write - write to a register of the DMA peripheral
 *
 * @base: DMA peripheral
 * @reg: pointer to the register to write to
 * @value: the value to write to @reg
*/
static void dma_write(DMA_RegDef_t *base,
					  u32 *reg,
					  u32 value)
{
	assert(base != NULL);
	assert(reg != NULL);

	u32 *offset = ((u32*)base) + *reg;
	if(offset)
	{
		*offset = value;
	}
}

/**
 * dma_sg_inc - increment the sg transfer index
*/
static void dma_sg_inc(struct dma_channel *chan)
{
	chan->next_sg++;

	/* If circular mode, start from the beginning again */
	if(chan->desc->cyclic && (chan->next_sg == chan->desc->num_sgs))
		chan->next_sg = 0;
}

static u32 dma_get_remaining_bytes(struct dma_channel *chan)
{
	u32 dma_scr,width,ndtr;

	dma_scr = dma_read(chan->base, DMA_SxCFR(chan));
	width = FIELD_GET(DMA_SCR_PSIZE_MASK, dma_scr);
	ndtr = dma_read(chan->base, DMA_SxNDTR(chan));

	return ndtr << width;
}

static bool dma_is_current_sg(struct dma_channel *chan)
{
	struct dma_sg_req *sg_req;
	u32 dma_scr,dma_smar,period_len,mem_addr;
	bool in_range;
	dma_scr = dma_read(chan->base, DMA_SxCFR(chan));

	/* If circular mode is enabled, then there is no possibility
	 * the hardware has updated the any of the memory address targets
	 */
	if( !(dma_scr & DMA_SCR_DBM))
		return true;

	sg_req = &chan->desc->sg_req[chan->next_sg];
	period_len = sg_req->len;

	/* If the current target is SM10AR, then check to see if
	 * SM0AR is in range. If it is, then we would expect to see that the
	 * dma_smar (the currently loaded address) is less than or equal to the
	 * configuration address and no greater than base address + the period length. The
	 * latter would happen if the transaction was paused and will later be resumed.
	 */
	if(dma_scr & DMA_SCR_CT)
	{
		dma_smar = dma_read(chan->base, DMA_SxM0AR(chan));
		mem_addr = sg_req->chan_reg.sm0ar;
		return dma_smar >= sg_req->chan_reg.sm0ar;
	}
	else
	{
		dma_smar = dma_read(chan->base, DMA_SxM1AR(chan));
		mem_addr = sg_req->chan_reg.sm1ar;

	}

	in_range = dma_smar >= mem_addr && dma_smar <= mem_addr + period_len;
	return in_range;
}

static size_t dma_residue(struct dma_channel *chan, u32 next_sg)
{
	u32 residue, num_sgs, curr_sg, burst_size;
	u32 mem_burst, mem_size;
	u32 rem;
	int i, bus_width;

	num_sgs = chan->desc->num_sgs;
	curr_sg = next_sg;

	residue = dma_get_remaining_bytes(chan);

	/* The residue is initialized with the current sg's ndtr * the busdwith
	 * With double buffer mode the two memory addresses SM0AR and SM1AR are swapped
	 * at the end of the transaction. Meaning, that when the remaining bytes are read
	 * the hardware could have switched the target which means the previous target is no longer
	 * pending and thus should not be taken into account for the residue calculation. Therefore,
	 * is_current_sg is checked and if it results in false, the curr_sg should be incremented as to
	 * skip the sg that was just completed.
	 * */

	if(chan->desc->cyclic && !dma_is_current_sg(chan))
	{
		curr_sg++;

		if(curr_sg == chan->desc->num_sgs)
			curr_sg = 0;
	}

	/* Then we aren't at the last transaction */
	if(curr_sg != 0)
	{
		for(i = curr_sg; i < num_sgs; i++)
		{
			residue += chan->desc->sg_req[i].len;
		}
	}

	mem_burst = FIELD_GET(DMA_SCR_MBURST_MASK, chan->dma_reg.scr) * 2;
	mem_size = FIELD_GET(DMA_SCR_MSIZE_MASK, chan->dma_reg.scr);
	bus_width = dma_get_buswidth((mem_size + 1) * 8);

	if(bus_width < 0)
		return residue;

	burst_size = mem_burst * bus_width;
	rem = residue % burst_size;

	/* Remove the exceeding bursts from the residue */
	if(rem)
		residue = residue - (rem + burst_size);

	return residue;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

