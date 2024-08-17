/*
 * dma.h
 *
 *  Created on: Aug 6, 2024
 *      Author: Andrew Streng
 */

#ifndef DMA_H_
#define DMA_H_

#include "stm32f4xx.h"
#include <stdbool.h>

/* Enums related to DMA configuration */

/* The burst size of the DMA transfer */
enum dma_burst_size
{
	DMA_BURST_SINGLE,
	DMA_BURST_INCR4,
	DMA_BURST_INCR8,
	DMA_BURST_INCR16
};

/* The priority of the DMA stream */
enum dma_priority_level
{
	DMA_PRIORITY_LOW,
	DMA_PRIORITY_MEDIUM,
	DMA_PRIORITY_HIGH,
	DMA_PRIORITY_VHIGH
};


/* The width of the transfer */
enum dma_width {
	DMA_BYTE,
	DMA_HALF_WORD,
	DMA_WORD
};

enum dma_slave_buswidth
{
	DMA_SLAVE_BUSWIDTH_1_BYTE,
	DMA_SLAVE_BUSWIDTH_2_BYTES,
	DMA_SLAVE_BUSWIDTH_4_BYTES,
};

enum dma_status
{
	DMA_BUSY,
	DMA_READY,
	DMA_PAUSED
};

enum irq_handled_t
{

};


/* DMA related data structures */


/* The configurable items of the DMA channel */

/**
 * struct dma_chan_cfg - DMA channel configuration
 * @channel_id: channel ID
 * @stream_num: DMA request stream
 * @stream_config: 32bit mask specifying the DMA channel configuration
 *
 * Used to set the following bits (of the configuration register)
 * 1. CT
 * 2. DBM
 * 3. PL
 * 4. MINC
 * 5. PINC
 * 6. CIRC
 * 7. DIR
 * 8. PFCTRL
 *
 * Used in the following manner : stream_config = DMA_SCR_CT | ~DMA_SCR_DBM .. etc;
 *
 * @interrupt_config: 16 bit mask specifying the DMA stream's interrupt configuration
 * Used in the following manner : interrupt_config = DMA_SCR_TCIE | DMA_SCR_HTIE
 *
 * Any of these values not specified in the configuration are assumed to be the reset value
 * of the bit in the register
 */

struct dma_chan_cfg
{
	u32 channel_id;
	u8 stream_num;
	u32 stream_config;
	u16 interrupt_config;
};

/**
 * struct dma_chan_reg - DMA register map struct mainly for internal use to store
 * the state of the DMA channel when paused to restore at a later point
 */
struct dma_chan_reg
{
	u32 lisr;
	u32 hisr;
	u32 lifcr;
	u32 hicfr;
	u32 scr;
	u32	sndtr;
	u32 spar;
	u32 sm0ar;
	u32 s1mar;
	u32 sfcr;
};

/**
 * struct dma_channel - DMA channel handle
 * @base: pointer to the DMA controller
 * @dma_chan_cfg: channel configuration
 * @dma_reg: the register of the handle to be initialized when @dma_transfer_start is called
 * @status: the status of the handle
 * @irq: the irq mask used to initialize the interrupt enabled flags in the configuration register
 * @threshold: the threshold value of the FIFO
 * @src_max_burst: the max burst size the DMA source should use, default is DMA max burst
 * @dst_max_burst: the max burst size the DMA destination should use, default is DMA max burst
 * @mem_width: memory data size
 * @periph_mem_width: peripheral data size
 * @src_addr: the address of the source
 * @dst_addr: the address of the destination
 *
 */
struct dma_channel
{

	DMA_RegDef_t* base;

	struct dma_chan_cfg channel_config;
	struct dma_chan_reg dma_reg;
	enum dma_status status;
	u32 irq;
	u32 threshold;
	u32 dst_max_burst;
	u32 src_max_burst;
	u32 mem_width;
	u32 periph_mem_width;
	u32 src_addr;
	u32 dst_addr;
};


/* Macros related to DMA configuration */

#define DMA_MAX_BURST						16
#define DMA_BURST							4 			/* The default burst size */

/* @DMA transfer directions */
#define DMA_PERIPH_TO_MEM					0x00
#define DMA_MEM_TO_PERIPH					0x01
#define DMA_MEM_TO_MEM						0x10
#define DMA_RESERVED						0x11

/* DMA FIFO threshold selections */
#define DMA_FIFO_THRESHOLD_QUARTERFULL		0x00
#define DMA_FIFO_THRESHOLD_HALFFULL			0x01
#define DMA_FIFO_THRESHOLD_3QUARTFULL		0x10
#define DMA_FIFO_THRESHOLD_FULL				0x11
#define DMA_FIFO_THRESHOLD_NONE				-1

/* Bit definitions for DMA configuration register */
#define	DMA_SCR_CHSEL_MASK		BITMASK(27,25)				/* Channel select */
#define DMA_SCR_MBURST_MASK		BITMASK(24,23)				/* Memory burst transfer config */
#define DMA_SCR_PBURST_MASK		BITMASK(22,21)				/* Peripheral burst transfer config */
#define DMA_SCR_PL_MASK			BITMASK(17,16)				/* Priority level */
#define DMA_SCR_MSIZE_MASK		BITMASK(14,13)				/* Memory data size */
#define DMA_SCR_PSIZE_MASK		BITMASK(12,11)				/* Peripheral data size */
#define DMA_SCR_DIR_MASK		BITMASK(7,6)				/* Data transfer direction */
#define DMA_SCR_CT				BIT(19)						/* Current target */
#define DMA_SCR_DBM				BIT(18)						/* Double buffer mode */
#define DMA_SCR_PINCOS			BIT(15)						/* Peripheral increment offset size */
#define DMA_SCR_MINC			BIT(10)						/* Memory increment mode */
#define DMA_SCR_PINC			BIT(9)						/* Peripheral increment mode */
#define DMA_SCR_CIRC			BIT(8)						/* Circular mode */
#define DMA_SCR_PFCTRL			BIT(5)						/* Peripheral flow controller */
#define DMA_SCR_TCIE			BIT(4)						/* Transfer complete interrupt enable */
#define DMA_SCR_HTIE			BIT(3)						/* Half transfer interrupt enable */
#define DMA_SCR_TEIE			BIT(2)						/* Transfer error interrupt enable */
#define DMA_SCR_DMEIE			BIT(1)						/* Direct mode error interrupt enable */
#define DMA_SCR_EN				BIT(0)						/* Stream enable */
#define DMA_MASK_IE				(DMA_SCR_TCIE | DMA_HTIE \
								DMA_SCR_TEIE)

/* DMA interrupts from the interrupt status register */
#define DMA_TCI					BIT(5)						/* Transfer complete interrupt flag */
#define DMA_HTI					BIT(4)						/* Half transfer complete interrupt flag */
#define DMA_TEI					BIT(3)						/* Stream transfer error interrupt flag */
#define DMA_EIF					BIT(2)						/* Stream direct mode error interrupt flag */
#define DMA_FEI					BIT(0)						/* Stream FIFO error interrupt flag */
#define DMA_MASKI				(DMA_TCI | DMA_HTI \
								| DMA_TEI | DMA_EIF \
								| DMA_FEI ) 				/* DMA interrupts mask */

/* FIFO configuration register bits */
#define DMA_FCR_FEIE			BIT(7)						/* FIFO error interrupt enable */
#define DMA_FCR_FS				BITMASK(5,3)				/* FIFO status */
#define DMA_FCR_DMDIS			BIT(2)						/* Direct mode disable */
#define DMA_FCR_FTH				BITMASK(1,0)				/* FIFO threshold selection */
#define DMA_FCR_MASK			(DMA_FCR_FTH | DMA_FCR_DMDIS | DMA_FCR_FEIE)


/* Generic DMA macros */
#define FIFO_THRESHOLD_NONE				0
#define DMA_SCR_REG(n)					(0x10 + (0x18 * n))
#define DMA_NDTR_REG(n)					(0x14 + (0x18 * n))
#define DMA_FIFO_CFG_REG				(0x24 + (0x18 * n))
#define DMA_SxCFR(n)					((u32*)(DMA_SCR_REG(n)))
#define DMA_SxFCR(n)					((u32*)DMA_FIFO_CFG_REG(n))
#define DMA_SxNDTR(n)					((u32*)DMA_NDTR_REG(n))
#define DMA_REG(base,offset)			(base + offset)
#define DMA_FIFO_DEPTH					((4 * 32) / 8)
#define DMA_LISR_MAX					3							/* The max stream number contained in the low status register */

/* Function Declarations */

///////////////////////////////////////////////////////////////////////////////////////

void dma_initialize(struct dma_channel *channel);
void dma_enable_interrupts(struct dma_channel *channel, u32 flags);
void dma_disable_interrupts(struct dma_channel *channel, u32 flags);
u32 dma_interrupt_config(struct dma_channel *channel);
void dma_configure_channel(void);
void dma_disable_chan(struct dma_channel *channel);
void dma_pause(struct dma_channel *channel);
void dma_stop(struct dma_channel *channel);
void dma_transfer_resume(void);
u32 dma_read_status(struct dma_channel *channel);
void dma_set_fifo_config(struct dma_channel *channel);
void dma_fifo_config_apply(struct dma_channel *channel);
void dma_prepare_transfer(struct dma_channel *channel, const u32 number_of_items);
void dma_enable_clk(DMA_RegDef_t *dma_controller, bool enabled);
void dma_fifo_flush(struct dma_channel *channel);
int dma_transfer_start(struct dma_channel *channel, u32 ndt);

///////////////////////////////////////////////////////////////////////////////////////

#endif /* DMA_H_ */

