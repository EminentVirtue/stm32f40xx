/*
 * spi.h
 *
 *  Created on: Jun 12, 2024
 *      Author: Andrew Streng
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stddef.h>


/* Bit definitions for common bit in various SPI registers (may not be complete) */

/* Control register 1 */
#define SPICR1_BIT_BIDIMODE		15		/* Bidirectional data mode enable */
#define SPICR1_BIT_BIDIOE		14 		/* Output enable in bidirectional mode */
#define SPICR1_BIT_CRCEN		13		/* Hardware CRC calculation enable */
#define SPICR1_BIT_CRCNEXT		12		/* CRC transfer next */
#define SPICR1_BIT_DFF			11		/* Data frame format */
#define SPICR1_BIT_RXONLY		10		/* Receive only */
#define SPICR1_BIT_SSM			9		/* Software slave management */
#define SPICR1_BIT_SSI			8 		/* Internal slave select */
#define SPICR1_BIT_SPE			6		/* SPI enable */
#define SPICR1_BIT_BR			3		/* Baud rate control (3 bits long) */
#define SPICR1_BIT_MSTR			2 		/* Master selection */
#define SPICR1_BIT_CPOL			1		/* Clock polarity */
#define SPICR1_BIT_CPHA			0		/* Clock phase */

/* Control register 2 */
#define SPICR2_BIT_TXEIE		7		/* Buffer empty interrupt enable */
#define SPICR2_BIT_RNEIE		6		/* RX buffer not empty interrupt enable */
#define SPICR2_BIT_ERRIE		5		/* Error interrupt enable */
#define SPICR2_BIT_FRF			4		/* Frame format */
#define SPICR2_BIT_SSOE			3		/* SS output enable */
#define SPICR2_BIT_TXDMAEN		2		/* TX buffer DMA enable */
#define SPICR2_BIT_RXDMAEN		1		/* RX buffer DMA enable */

/* SPI status register */
#define SPISTAT_BIT_BSY			7		/* Busy flag */
#define SPISTAT_OVR				6		/* Overrun flag */
#define SPISTAT_MODF			5		/* Mode fault */
#define SPISTAT_CRCERR			4		/* CRC error flag */
#define SPISTAT_UDR				3		/* Underrun flag */
#define SPISTAT_CHSIDE			2		/* Channel side */
#define SPISTAT_TXE				1		/* Transmit buffer empty */
#define SPISTAT_RXNE			0		/* Receive buffer not empty */

/* SPI status bit masks */
#define BUSY_BITMASK			0x40
#define OVR_BITMASK				0x20
#define MODF_BITMASK			0x10
#define CRCERR_BITMASK			0x8
#define UDR_BITMASK				0x4
#define CHSIDE_BITMASK			0x3
#define TXE_STATUS				(1 << SPISTAT_TXE)
#define RXNE_BITMASK			0x0

/* SPI serial clock selection macros where PCLK_DIV_X is the SPI peripheral clock divided by X */
#define PCLK_DIV_2				0
#define PCLK_DIV_4				1
#define PCLK_DIV_8				2
#define PCLK_DIV_16				3
#define PCLK_DIV_32				4
#define PCLK_DIV_64				5
#define PCLK_DIV_128			6
#define PCLK_DIV_256			7

/* Other generic macros related to SPI */
#define BUSY			1		/* SPI peripheral busy */
#define NOT_BUSY		0		/* SPI peripheral not busy */
#define OVERRUN			1		/* Overrun occurred */
#define NO_OVERRUN		0		/* No ovverun occured */
#define MODE_FAULT		1		/* Mode fault occurred */
#define NO_MODE_FAULT	0		/* No mode fault occurred */
#define UNDERRUN		1		/* Underrun occurred */
#define NO_UNDERRUN		0		/* No underrun occurred */
#define TX_EMPTY		1		/* Transmit buffer empty */
#define TX_NOT_EMPTY	0		/* Transmit buffer not empty */
#define RX_NOT_EMPTY	1		/* Receive buffer not empty */
#define RX_EMPTY		0		/* Receive buffer empty */
#define SPI_STATUS_UNKNOWN	-1

#define SPI_ENABLED_FLAG		(1 << SPICR1_BIT_SPE)
#define SPI_DFF_8BIT			0
#define SPI_DFF_16BIT			1
#define SPI_BIDI_ON				1			/* SPI communication uses a bidirectional line */
#define	SPI_BIDI_OFF			0			/* SPI communication uses multiple unidrectional lines */
#define SPI_OUTPUT_DISABLE		DISABLE		/* SPI operates in receive only mode (only applies when BIDI mode is enabled) */
#define SPI_OUTPUT_ENABLE		ENABLE		/* SPI operates in transmit only mode (only applies when BIDI mode is enabled) */
#define SPI_SW_SLAVE_ENABLE		ENABLE
#define SPI_HW_SLAVE_ENABLE		DISABLE
#define SPI_MASTER_ENABLE		1
#define SPI_SLAVE_ENABLE		0
#define SPI_CS_SET              1
#define SPI_CS_UNSET			0
#define SPI_CR2_READ			1
#define SPI_CR1_READ			0

#define SPI_RETRY_TIMES			10

#define SPI_IT_TXEIE(val)		(val << 2)
#define SPI_IT_RXNEIE(val)		(val << 1)
#define SPI_IT_ERRIE(val)		(val << 0)

#define SPI_IT_ALL				0x7
#define SPI_REG(reg)			((void*)reg)
#define CR1_REG(pSPIx)			SPI_REG(pSPIx->CR1)

typedef struct {
	uint8_t bidi_mode;			/* Bidirectional data mode enabled/disabled */
	uint8_t bidi_output_mode;	/* Bidirectional data mode receive/send only mode - only applies to half duplex */
	uint8_t frame_format;		/* 8-bit or 16-bit data frames */
	uint8_t slave_management;	/* Hardware or software slave management */
	uint8_t spi_speed;			/* The speed of the SPI master clock */
	uint8_t clock_polarity;		/* Clock polarity - default is 0 */
	uint8_t clock_phase;		/* Clock phase - default is 0 */
	uint8_t master_selection;	/* Master/slave configuration */
	uint8_t use_delay;			/* Specifies whether the send should happen after a delay
								 * 1 for delay enabled, 0 for delay disabled */
}SPI_Config_t;

typedef enum {
	Busy,
	Overrun,
	Success,
	ModeFault,
	Underrun,
	InvalidArgument,
	ErrorState,
	BusyInTx,
	BusyInRx,
	FrequencyLowered,
	FrequencyIncreased,
	StatusUnknown
}SPI_Status_t;

typedef enum {
	ModeFullDuplex,
	ModeUnidirectionalTxOnly,
	ModeUnidirectionalRxOnly,
	ModeBidirectionalRxOnly,
	ModeBidirectionalTxOnly,
	ModeSlaveRxOnly,
	ModeSlaveBidirectionalRx,
	ModeUnknown
}SPI_Mode_t;

/* Data type used to define a transaction delay */
typedef struct {
	#define SPI_DELAY_UNIT_USECS	0
	#define SPI_DELAY_UNIT_NSECS	1
	#define SPI_DELAY_UNIT_SEC		2
	uint16_t	value;
	uint8_t 	unit;
}SPI_Delay_t;

typedef struct {
	uint8_t MODF;			/* The MODF error flag is currently set if the value = 1 */
	uint8_t OVR;			/* The OVR error flag is currently set if the value = 1 */
	uint8_t CRCERR;			/* The CRC error flag is currently set if the value = 1 */
}SPI_Err_Handle_t;

typedef struct {
	uint8_t *txBuffer;
	uint8_t *rxBuffer;

	volatile size_t txSize;
	volatile size_t rxSize;
	uint8_t wait_blocking;
	uint8_t retryTimes;
	bool useDMA;
	bool doCRC;
}SPI_Transfer_t;

typedef enum _lpspi_delay_type
{
    kLPSPI_PcsToSck = 1U,  /*!< PCS-to-SCK delay. */
    kLPSPI_LastSckToPcs,   /*!< Last SCK edge to PCS delay. */
    kLPSPI_BetweenTransfer /*!< Delay between transfers. */
} SPI_DelayType_t;

typedef struct {
	u16 adjustedFrequency;
	u8 baudSetting;
	SPI_Status_t status;
}SPI_ClockStatus_t;

typedef struct {
	SPI_RegDef_t *pSPIx;	/* Holds the base address of the SPI peripheral */
	SPI_Config_t config;	/* Holds the SPI Configuration */

	uint8_t rxState;		/* The state of the RX buffer */
	uint8_t txState;		/* The state of the TX buffer */
	uint8_t *rxBuffer;		/* To store the RX buffer address */
	uint8_t *txBuffer;		/* To store the TX buffer address */
	volatile size_t rxSize;			/* The length of the RX buffer */
	volatile size_t txSize;			/* The length of the TX buffer */
	bool txMarked;


	SPI_Err_Handle_t error_handle;		/* Stores any SPI error states in this item - this item is updated after every SPI
										 * interrupt is generated - the status register is read */

	void (*rxISRCallback)();
	void (*txISRCallback)();
	void (*CRCErrCallback)();

}SPI_Handle_t;

/* Function declarations */
void SPI_Init(SPI_Handle_t *pHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t enabled);
void SPI_Prepare(SPI_RegDef_t *pSPIx);
void SPI_Toggle_SSI(SPI_RegDef_t *pSPIx, uint8_t enabled);
SPI_Status_t SPI_MasterTransferBlocking(SPI_Handle_t *pSPIx, SPI_Transfer_t *transfer);
SPI_Status_t SPI_MasterTransferNonBlocking(SPI_Handle_t *pHandle, SPI_Transfer_t *pTransfer);
void SPI_ReceiveBlocking(SPI_RegDef_t *pSPIx, SPI_Transfer_t *pTransfer);
void SPI_ReceiveNonBlocking(SPI_Handle_t *pHandle, SPI_Transfer_t *pTransfer);
void SPI_IRQ_Config(uint8_t IRQ_Number, uint8_t enabled);
void SPI_Disable(SPI_RegDef_t* pSPIx);
void SPI_SSOE_Configure(SPI_RegDef_t* pSPIx,uint8_t enabled);
void SPI_IRQ_Handle(SPI_Handle_t *pHandle);
void SPI_Close_Reception(SPI_Handle_t *pHandle);
void SPI_Close_Transmission(SPI_Handle_t *pHandle);
void SPI_TransferAbort(SPI_Handle_t *pHandle);
void SPI_DisableInterrupts(SPI_RegDef_t *pSPIx, const uint8_t interruptConfig);
void SPI_EnableInterrupts(SPI_RegDef_t *pSPIx, const uint8_t interruptConfig);
uint8_t SPI_InterruptConfig(SPI_RegDef_t *pSPIx);
SPI_ClockStatus_t SPI_ChooseBestClockFrequency(SPI_RegDef_t *pSPIx, const unsigned char freqMHz);
uint32_t SPI_SetDelayTimes(SPI_Handle_t *pHandle, uint32_t delayTimeInNanoSec, SPI_DelayType_t whichDelay, uint32_t srcClock_Hz);
SPI_Status_t SPI_UpdateClockPolarity(SPI_Handle_t *pHandle, uint8_t CPOL, uint8_t CPHA);
uint8_t SPI_CheckTransferArgument(SPI_Handle_t *pHandle, SPI_Transfer_t *transfer, bool isDMA);
SPI_Mode_t SPI_GetCurrentMode(SPI_RegDef_t *pSPIx);
SPI_Status_t SPI_ConfigureCRC(SPI_RegDef_t *pSPIx, bool shouldEnable, u16 polynomial);

//__weak void SPI_ISR();

#endif /* SPI_H_ */
