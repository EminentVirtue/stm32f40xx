/*
 * spi.h
 *
 *  Created on: Jun 12, 2024
 *      Author: Andrew Streng
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"
#include "gpio.h"
#include "dma.h"
#include <stdbool.h>
#include <stddef.h>


/* Bit definitions for common bit in various SPI registers (may not be complete) */

/* Control register 1 */
#define SPICR1_BIDIMODE		BIT(15)			/* Bidirectional data mode enable */
#define SPICR1_BIDIOE		BIT(14) 		/* Output enable in bidirectional mode */
#define SPICR1_CRCEN		BIT(13)			/* Hardware CRC calculation enable */
#define SPICR1_CRCNEXT		BIT(12)			/* CRC transfer next */
#define SPICR1_DFF			BIT(11)			/* Data frame format */
#define SPICR1_RXONLY		BIT(10)			/* Receive only */
#define SPICR1_SSM			BIT(9)			/* Software slave management */
#define SPICR1_SSI			BIT(8) 			/* Internal slave select */
#define SPICR1_LSBFIRST		BIT(7)
#define SPICR1_SPE			BIT(6)			/* SPI enable */
#define SPICR1_BR_MASK		BITMASK(5,3)	/* Baud rate control (3 bits long) */
#define SPICR1_MSTR			BIT(2) 			/* Master selection */
#define SPICR1_CPOL			BIT(1)			/* Clock polarity */
#define SPICR1_CPHA			BIT(0)			/* Clock phase */

/* Control register 2 */
#define SPICR2_TXEIE		BIT(7)			/* Buffer empty interrupt enable */
#define SPICR2_RNEIE		BIT(6)			/* RX buffer not empty interrupt enable */
#define SPICR2_ERRIE		BIT(5)			/* Error interrupt enable */
#define SPICR2_FRF			BIT(4)			/* Frame format */
#define SPICR2_SSOE			BIT(2)			/* SS output enable */
#define SPICR2_TXDMAEN		BIT(1)			/* TX buffer DMA enable */
#define SPICR2_RXDMAEN		BIT(0)			/* RX buffer DMA enable */

#define SPI_IT_MASK			(SPICR2_TXEIE | SPICR2_RNEIE | SPICR2_ERRIE )

/* SPI status register */
#define SPISR_BSY			BIT(7)			/* Busy flag */
#define SPISR_OVR			BIT(6)			/* Overrun flag */
#define SPISR_MODF			BIT(5)			/* Mode fault */
#define SPISR_CRCERR		BIT(4)			/* CRC error flag */
#define SPISR_UDR			BIT(3)			/* Underrun flag */
#define SPISR_CHSIDE		BIT(2)			/* Channel side */
#define SPISR_TXE			BIT(1)			/* Transmit buffer empty */
#define SPISR_RXNE			BIT(0)			/* Receive buffer not empty */

/* SPI CRC register */
#define SPICRC_RXCRCR		BITMASK(15,0)

#define SPI_RETRY_TIMES			10
#define SPI_COMM(b)				(b->config.curr_comm)
#define SPI_IT_ERR				(SPISR_CRCERR | SPISR_MODF | SPISR_OVR | SPISR_UDR)
#define SPI_MIN_BRD				2			/* The minimum baud rate divisor */
#define SPI_DEFAULT_BPF			1			/* The default bytes per transfer frame */
#define SPI_DR_OFFSET			((u32)0x0C)
#define SPI_SSM_ENABLE			1
#define SPI_SSM_DISABLED		~SPI_SSM_ENABLE
#define SPI_MASTER_ENABLED		1
#define SPI_MASTER_DISABLED		~SPI_MASTER_ENABLED
#define SPI_BR_DIV_MAX			256
#define SPI_BR_DIV_MIN			2
#define SPI_DR_ADDR(base)		((u32)((uintptr_t)((uint8_t*)base + SPI_DR_OFFSET)))

/* @SPI comm type */
#define SPI_FULL_DUPLEX			0
#define SPI_SIMPLEX_TX			1			/* 1 clock and 1 unidirectional data wire (BIDIMODE=0, RXONLY=0) */
#define SPI_SIMPLEX_RX			2			/* 1 clock and 1 unidirectional data wire (BIDIMODE=0, RXONLY=1) */
#define SPI_3WIRE_TX			3
#define SPI_3WIRE_RX			4
#define SPI_MODE_NONE			5

typedef struct {
	u8 frame_format;		/* 8-bit or 16-bit data frames */
	u8 slave_management;	/* Hardware or software slave management */
	u8 spi_speed;			/* The speed of the SPI master clock */
	u8 clock_polarity;		/* Clock polarity - default is 0 */
	u8 clock_phase;			/* Clock phase - default is 0 */
	u8 master_selection;	/* Master/slave configuration */
	u8 use_delay;			/* Specifies whether the send should happen after a delay
								 * 1 for delay enabled, 0 for delay disabled */
	u32 curr_comm;
}SPI_Config_t;

enum spi_br_divisor
{
	PCLK_DIV_2,
	PCLK_DIV_4,
	PCLK_DIV_8,
	PCLK_DIV_16,
	PCLK_DIV_32,
	PCLK_DIV_64,
	PCLK_DIV_128,
	PCLK_DIV_256
};


/* Data type used to define a transaction delay */
typedef struct {
	#define SPI_DELAY_UNIT_USECS	0
	#define SPI_DELAY_UNIT_NSECS	1
	#define SPI_DELAY_UNIT_SEC		2
	uint16_t	value;
	u8 	unit;
}SPI_Delay_t;


/**
 * @SPI_Transfer_t- SPI transfer handle
 * @tx_buff: pointer to the transfer buffer
 * @rx_buff: pointer to the receive buffer
 * @tx_len: the length of the transfer buffer
 * @rx_len: the length of the receive buffer
 * @wait_blocking: Pointer to the RX buffer
 * @retryTimes: Pointer to the TX buffer
 * @use_dma: whether the single transfer should use DMA
 * @speed_hz: the clock speed of of the transfer - see @SPI_Transfer_t Note
 * @bpw: bytes per transfer frame - see @SPI_Transfer_t Note
 * @maxbr_div: the maximum baud rate divisor for the transfer clock speed - see @SPI_Transfer_t Note
 * @minbr_div: the maximum baud rate divisor for the transfer clock speed - see @SPI_Transfer_t Note
 * SPI frame formats
 *
 * @SPI_Transfer_t Note - only applies when using spi_transfer_one - otherwise the handle configuration
 * is used over the values set in the transfer argument
 */

typedef struct {
	u8 *tx_buff;
	u8 *rx_buff;
	u32 tx_len;
	u32 rx_len;
	u8 wait_blocking;
	u8 retry_times;
	bool use_dma;
	u32 speed_hz;
	u32 bpw;
	u32 maxbr_div;
	u32 minbr_div;
}SPI_Transfer_t;

typedef SPI_Transfer_t spi_transfer;


/**
 * @spi_handle- SPI Handle structure
 * @spi: Base pointer to the SPI peripheral
 * @dma_chan: SPI DMA handle
 * @rx_state: The state of the RX transfer
 * @tx_state: The state of the TX transfer
 * @rxBuffer: Pointer to the RX buffer
 * @tx_buff: Pointer to the TX buffer
 * @rx_len: Length of the RX buffer
 * @tx_len: Length of the TX buffer
 * @error_handle: Error handle storing error status register
 * @*rxISRCallback: RX callback
 * @*txISRCallback: TX callback
 */
typedef struct {
	stm32_spi *spi;
	SPI_Config_t config;
	struct dma_channel dma_tx_chan;
	struct dma_channel dma_rx_chan;

	volatile u8 *rx_buff;
	volatile u8 *tx_buff;
	volatile size_t rx_len;
	volatile size_t tx_len;
	bool use_dma;
	bool busy;

}SPI_Handle_t;

typedef SPI_Handle_t spi_handle;

/* Function declarations */

/* SPI Initialization functions */
int spi_init(spi_handle *handle, unsigned int comm_type, u32 freq_mhz, u32 max_div, u32 min_div);
int spi_deinit(stm32_spi *spi);
void spi_disable(stm32_spi* spi, unsigned int comm_type);

/* SPI configuration functions */
int spi_clock_control(stm32_spi *spi, bool enabled);
void spi_toggle_ssi(stm32_spi *spi, bool enabled);
void spi_toggle_ssoe(stm32_spi* spi,u8 enabled);
int spi_set_mode(stm32_spi *spi, unsigned int comm_type);
int spi_prepare_mbr(stm32_spi *spi, u32 freq_hz, u32 max_div, u32 min_div);
void spi_clk_configure(spi_handle *handle, u8 cpol, u8 cpha);
void spi_configure_crc(spi_handle *handle, u16 polynomial, bool enabled);

/* SPI Rx,Tx functions */
int spi_tx_blocking(spi_handle *handle, spi_transfer *transfer);
int spi_transfer_one(spi_handle *handle, spi_transfer *xfer);
int spi_tx_nb(spi_handle *handle, spi_transfer *pTransfer);
int spi_rx_blocking(spi_handle *handle);
void spi_rx_end(spi_handle *handle);
void spi_tx_end(spi_handle *handle);
void spi_tx_abort(spi_handle *handle);

/* SPI IRQ functions */
void spi_irq_configure(u8 irq_no, bool enabled);
enum irq_handled spi_irq_handle(spi_handle*handle);
void spi_irq_disable(stm32_spi *spi, u32 flags);
void spi_irq_enable(stm32_spi *spi, u32 flags);
u32 spi_irq_config(stm32_spi *spi);

/* SPI DMA functions */
void spi_dma_chan_config(struct dma_channel *chan,u32 stream, u32 channel, u32 flags);
int spi_transfer_one_dma(spi_handle *handle, spi_transfer *xfer, bool submit);
void spi_transfer_one_dma_start(spi_handle *handle);

#endif /* SPI_H_ */
