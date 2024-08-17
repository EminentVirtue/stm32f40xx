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


static u8 handle_err_interrupt(SPI_Handle_t *pHandle);
static void handle_tx_interrupt(SPI_Handle_t *pHandle);
static bool isRxRequired(SPI_RegDef_t* pSPIx);
static uint8_t SPI_CheckTransferArgument(SPI_Handle_t *pHandle, SPI_Transfer_t *transfer);

/*********************************************************************
 * @fn      		  - SPI_PeripheralClockControl
 * @brief             - Enables or disables the SPI peripheral clock based
 * 						on supplied flag
 *
 * @param[in]         - SPI_RegDef_t *pSPIx - ptr to SPI peripheral
 * @param[in]		  - u8 enabled - 1 for enabled, 0 for disabled
 ********************************************************************/
void SPI_PeripheralClockControl(SPI_RegDef_t* pSPIx, u8 enabled)
{
	if(pSPIx == SPI1)
	{
		enabled == ENABLE ? SPI1_PCLK_EN : SPI1_PCLK_DN;
	}
	else if(pSPIx == SPI2)
	{
		enabled == ENABLE ? SPI2_PCLK_EN : SPI2_PCLK_DN;
	}
	else if(pSPIx == SPI3)
	{
		enabled == ENABLE ? SPI3_PCLK_EN : SPI3_PCLK_DN;
	}
	else if(pSPIx == SPI4)
	{
		enabled == ENABLE ? SPI4_PCLK_EN : SPI5_PCLK_DN;
	}
	else if(pSPIx == SPI5)
	{
		enabled == ENABLE ? SPI5_PCLK_EN : SPI5_PCLK_DN;
	}
	else if(pSPIx == SPI6)
	{
		enabled == ENABLE ? SPI6_PCLK_EN : SPI6_PCLK_DN;
	}
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 * @brief             - Resets the SPI peripheral

 * @param[in]         - SPI_RegDef_t *pSPIx - the SPI peripheral
 ********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_RESET;
	}
	else if(pSPIx == SPI2)
	{
		SPI2_RESET;
	}
	else if(pSPIx == SPI3)
	{
		SPI3_RESET;
	}
	else if(pSPIx == SPI4)
	{
		SPI4_RESET;
	}
	else if(pSPIx == SPI5)
	{
		SPI5_RESET;
	}
	else if(pSPIx == SPI6)
	{
		SPI6_RESET;
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 * @brief             - Initializes the SPI peripheral given user requirements

 * @param[in]         - SPI_Handle_t *pHandle
 ********************************************************************/
void SPI_Init(SPI_Handle_t *pHandle)
{
	SPI_Config_t config = pHandle->config;
	SPI_RegDef_t* pSPI = pHandle->pSPIx;

	SPI_PeripheralClockControl(pSPI, ENABLE);

	/* Check to see if if the given configuration is valid
	 * Mainly, check to see if the given configuration would result
	 * in a MODF error and check to see if the given clock prescaler
	 * works
	 */

	/* BIDI mode */
	pSPI->CR1 |= ( config.bidi_mode << SPICR1_BIT_BIDIMODE );

	/* Output enable in BIDI mode */
	pSPI->CR1 |= (config.bidi_output_mode << SPICR1_BIT_BIDIOE);

	/* Frame format */
	pSPI->CR1 |= (config.frame_format << SPICR1_BIT_DFF);

	/* Slave management */
	pSPI->CR1 |= (config.slave_management << SPICR1_BIT_SSM);

	/* Clock speed */
	pSPI->CR1 |= (config.spi_speed << SPICR1_BIT_BR);

	/* Clock phase */
	pSPI->CR1 |= (config.clock_phase << SPICR1_BIT_CPOL);

	/* Clock polarity */
	pSPI->CR1 |= (config.clock_phase << SPICR1_BIT_CPHA);

	/* Master selection */
	pSPI->CR1 |= (config.master_selection << SPICR1_BIT_MSTR);
}

/*********************************************************************
 * @fn      		  - SPI_IRQ_Config
 * @brief             - Enables or disables the SPI peripheral's IRQ

 * @param[in]         - IRQ_Number - the IRQ number of the SPI
 * @param[in]		  - enabled - the enabled/disabled flag
 ********************************************************************/
void SPI_IRQ_Config(u8 IRQ_Number, u8 enabled)
{
	volatile void *NVIC_Reg;
	u8 mod_amount = 0;

	if(IRQ_Number <= 31)
	{
		NVIC_Reg = (void*)NVIC_ISER0;
		mod_amount = ++IRQ_Number;
	}
	if(IRQ_Number > 31 && IRQ_Number < 64 )
	{
		NVIC_Reg = (void*)NVIC_ISER1;
		mod_amount = 32;
	}
	else if(IRQ_Number >= 64 && IRQ_Number < 96)
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
			*(u32*)NVIC_Reg |= (ENABLE << (IRQ_Number % mod_amount));
		}
		else
		{
			*(u32*)NVIC_Reg &= ~(ENABLE << (IRQ_Number % mod_amount));
		}
	}
}
static u8 SPI_ReadBit(void *reg, u8 bit)
{
	/* All SPI regs are 4 bytes, so this can safely be casted */
	return *((u32*)reg) >> bit & 0x1;
}

static u8 read_status_bit(SPI_RegDef_t *pSPIx, u8 status_bit)
{
	return ( pSPIx->SR  >> status_bit) & 0x1;
}

SPI_Status_t SPI_ToggleCS(const SPI_Handle_t *pHandle)
{
	/* First check the state of the pin, if it's already enabled,
	 * don't do anything
	 */
	const GPIO_Handle_t *hChipSelect = pHandle->pChipSelect;

	if(GPIO_ReadFromPin(hChipSelect->pGPIOPad, hChipSelect->config.pin_number)
			== pHandle->chipSelectEnabledLevel)
	{
		return StatusEnabled;
	}

	/* Enable the chip select before the transaction has started */
	if(pHandle->pChipSelect)
	{
		const GPIO_Handle_t *hChipSelect = pHandle->pChipSelect;
		GPIO_WriteToPin(hChipSelect->pGPIOPad,hChipSelect->config.pin_number, pHandle->chipSelectEnabledLevel);
	}

	return Success;
}


/*********************************************************************
 * @fn      		  - read_control_bit
 * @brief             - Reads control bit from CR1 or CR2 register - for
 	 	 	 	 	 	internal use only

 * @param[in]         - pSPIx - SPI peripheral
 * @param[in]		  - controlBit - The bit of the address to read
 * @param[in]		  - controlReg - The control register to read the
 	 	 	 	 	 	bit from

 ********************************************************************/
static u8 read_control_bit(SPI_RegDef_t *pSPIx, u8 controlBit, u8 controlReg)
{
	return controlReg == SPI_CR1_READ ? ((pSPIx->CR1 >> controlBit) & 0x1) :
			((pSPIx->CR2 >> controlBit) & 0x1);
}


/*********************************************************************
 * @fn      		  - SPI_ConfigureCRC
 * @brief             - Handles disabling/enabling the CRC check for a
 	 	 	 	 	 	transfer whether the CRC is desired

 * @param[in]         - shouldEnable - True/False for enabling the CRC
 * @param[in]		  - polynomial - The polynomial used for the
 * 						CRC calculation. If the reset polynomial value
 * 						is desired, 0 should be passed in for this param
 ********************************************************************/
SPI_Status_t SPI_ConfigureCRC(SPI_RegDef_t *pSPIx, bool shouldEnable, u16 polynomial)
{
	const u8 interruptConfig = SPI_InterruptConfig(pSPIx);
	SPI_DisableInterrupts(pSPIx, SPI_IT_ALL);

	/* First the SPI should be disabled */
	while(read_status_bit(pSPIx, SPISTAT_BIT_BSY));

	if(shouldEnable)
	{
		/* Check to see if it is already enabled */
		if( (pSPIx->CR1 & 0x1000) == 0x1000)
		{
			SPI_EnableInterrupts(pSPIx, interruptConfig);
			return Success;
		}
	}

	SPI_Disable(pSPIx);

	/* Configure the CRCEN bit*/
	pSPIx->CR1 |= (shouldEnable << SPICR1_BIT_CRCEN);

	/* Configure the polynomial register to be used to calculate the CRC */
	if(polynomial > 0U)
	{
		pSPIx->CRCPR &= ~polynomial;
		pSPIx->CRCPR = polynomial;
	}

	/* Re-enable any interrupts */
	SPI_EnableInterrupts(pSPIx, interruptConfig);

	/* Re-enable the SPI */
	pSPIx->CR1 |= (ENABLE << SPICR1_BIT_SPE);

	return Success;
}


static bool isRxRequired(SPI_RegDef_t* pSPIx)
{
	SPI_Mode_t currentSPIMode = SPI_GetCurrentMode(pSPIx);
	bool rxRequired = false;

	switch(currentSPIMode)
	{
		case ModeUnidirectionalRxOnly:
			rxRequired = true;
			break;
		case ModeBidirectionalRxOnly:
			rxRequired = true;
			break;
		case ModeBidirectionalTxOnly:
			rxRequired = false;
			break;
		case ModeFullDuplex:
			rxRequired = true;
			break;
		case ModeUnidirectionalTxOnly:
			rxRequired = false;
			break;
		default:
			rxRequired = true;
			break;
	}

	return rxRequired;
}
/*********************************************************************
 * @function      	  - SPI_Prepare
 * @brief             - Enables the SPI peripheral and waits until the
  						busy flag is cleared

 * @param[in]         - SPI_RegDef_t *pSPIx
 * @Note              - This should be called before send, receive
 	 	 	 	 	 	assuming the peripheral is disabled, especially
 	 	 	 	 	 	if a hardware CS is being used.
 ********************************************************************/
void SPI_Prepare(SPI_RegDef_t* pSPIx)
{
	pSPIx->CR1 |= (ENABLE << SPICR1_BIT_SPE);

	/* Wait until the peripheral is ready for transmission */
	while( ((pSPIx->SR >> SPISTAT_BIT_BSY) & 0x1) == 0x1);
}


SPI_Status_t SPI_UpdateClockPolarity(SPI_Handle_t* pHandle,u8 CPOL, u8 CPHA)
{
	if(pHandle->rxState == BUSY || pHandle->txState == BUSY)
	{
		return Busy;
	}

	/* First disable the SPI */

	return Success;
}


/* Flush the FIFO before starting a transfer in order to avoid a possible overrun */
static void SPI_FlushFifo(SPI_RegDef_t *pSPIx)
{
	(void)pSPIx->DR;
}

/* Is the SPI BIDI mode */
static u8 isBIDI(SPI_RegDef_t *pSPIx)
{
	return (pSPIx->CR1 >> SPICR1_BIT_BIDIMODE) & 0x1;
}

/* Is the SPI enabled for RX only */
static u8 isRxOnly(SPI_RegDef_t *pSPIx)
{
	return (pSPIx->CR1 >> SPICR1_BIT_RXONLY) & 0x1;
}

/* Is the SPI configured for BIDI mode with only RX */
static u8 isBIDIRx(SPI_RegDef_t *pSPIx)
{
	return isBIDI(pSPIx) && isRxOnly(pSPIx);
}

static u8 bytes_per_frame(SPI_RegDef_t *pSPIx)
{
	return ((((pSPIx->CR1 >> SPICR1_BIT_DFF) & 0x1) * 8U) + 8U) / 8U;
}

SPI_Status_t SPI_MasterTransferBlocking(SPI_Handle_t *pHandle, SPI_Transfer_t *pTransfer)
{


	assert(pTransfer != NULL);
	assert(pHandle != NULL);

	SPI_RegDef_t *pSPIx = pHandle->pSPIx;
	assert(pSPIx != NULL);


	/* First check to see if the SPI is in an error state. If it is,
	 * then populate the error struct in the handle and return */

	if(handle_err_interrupt(pHandle)){
		return ErrorState;
	}

	/* Spin until it isn't busy if that's the desired behavior */
	if(pTransfer->wait_blocking)
	{
		while(read_status_bit(pSPIx, SPISTAT_BIT_BSY) == BUSY);
	}

	/* Otherwise return busy status */
	else if(read_status_bit(pSPIx, SPISTAT_BIT_BSY) == BUSY)
	{
		return Busy;
	}

	/* Check to see if the transfer is permissible */
	if(!SPI_CheckTransferArgument(pHandle, pTransfer))
	{
		return InvalidArgument;
	}

	/* Flush the FIFO in order to avoid the possibility of an overrun */
	SPI_FlushFifo(pHandle->pSPIx);

	/* The SPI interrupts should be disabled in the case the user did not disable them
	 * and is now sending a blocking transmission so that this process isn't unintentionally
	 * interrupted
	 */

	/* Disable all SPI interrupts */
	const u8 interruptConfig = SPI_InterruptConfig(pSPIx);
	SPI_DisableInterrupts(pSPIx, SPI_IT_ALL);


	/* Wait until the TX buffer is empty */
#ifdef SPI_RETRY_TIMES
	while( (read_status_bit(pSPIx, SPISTAT_TXE) == TX_NOT_EMPTY) && (--pTransfer->retryTimes != 0U));
#else
	while( read_status_bit(pSPIx, SPISTAT_TXE) == TX_NOT_EMPTY);
#endif


	u8 bytesPerFrame = ((((pSPIx->CR1 >> SPICR1_BIT_DFF) & 0x1) * 8U) + 8U) / 8U;
	volatile size_t txSize = pTransfer->txSize;
	volatile u8 *txBuffer = pTransfer->txBuffer;

	SPI_ToggleCS(pHandle);

	/* Load the data into the date register FIFO */

	//for(u8 i = 0;i < 3; i++)
	while(txSize > 0)
	{

		if(bytesPerFrame == 1)
		{
			pSPIx->DR = *txBuffer;
			txBuffer++;
			txSize--;
		}
		else
		{
			pSPIx->DR = *((u16*)txBuffer);
			txSize--;
			txSize--;
			(u16*)txBuffer++;
		}

		/* Check the mode the SPI is currently in to see if a response is to be expected.
		 * In order to avoid the possibility of an overrun, the data register should be
		 * read before a subsequent transfer is allowed.
		 */

		if(isRxRequired(pSPIx))
		{
			SPI_ReceiveBlocking(pSPIx, pTransfer);
		}
	}

	/* Restore the SPI's interrupt configuration */
	if(interruptConfig > 0 )
		SPI_EnableInterrupts(pSPIx, interruptConfig);


	return Success;
}

/**
 * SPI_MasterTransferNonBlocking- Sets up the supplied SPI handle for TX, which will then be completed upon the interrupt
 * @pHandle: the SPI handle
 *
 * @pTransfer: he transfer handle
*/
SPI_Status_t SPI_MasterTransferNonBlocking(SPI_Handle_t *pHandle, SPI_Transfer_t *pTransfer)
{
	/* First check the handle state, if it is already marked as transmitting,
	 * then do nothing.
	 */
	if(pHandle->rxState == BUSY)
		return BusyInTx;

	assert(pTransfer != NULL);

	/* Check to see if the SPI is in an error state */
	if(handle_err_interrupt(pHandle))
	{
		return ErrorState;
	}

	/* Disable interrupts so this setup isn't interrupted */
	SPI_DisableInterrupts(pHandle->pSPIx, SPI_IT_ALL);

	/* Check to see if the transfer arguments are good */
	if(!SPI_CheckTransferArgument(pHandle, pTransfer))
	{
		return InvalidArgument;
	}

	/* Transfer the transfer arguments to the handle */
	pHandle->txSize = pTransfer->txSize;
	pHandle->txBuffer = pTransfer->txBuffer;

	/* Handle is setup, re-enable the interrupts
	 * The RXNE interrupt should be enabled in the case
	 * that a response is received and handled in order to avoid an overrun
	 * One particular instance that this can happen is if the user is in full duplex mode
	 * but is transmitting only.
	 */


	/* In transmit-only mode, the OVR flag is set in the SR register
			 * since the received data are never read. Therefore set OVR
			 * interrupt only when rx buffer is available.
			 */
	SPI_FlushFifo(pHandle->pSPIx);
	SPI_EnableInterrupts(pHandle->pSPIx, SPI_IT_ALL);

	return Success;
}

SPI_Status_t SPI_MasterTransferDMA(SPI_Handle_t *pHandle, SPI_Transfer_t *pTransfer)
{

	return Success;
}
SPI_Status_t SPI_MasterReceiveDMA(SPI_Handle_t *pHandle, SPI_Transfer_t *pTransfer)
{


	return Success;
}

/*********************************************************************
 * @function      	  - SPI_Toggle_SSI
 * @brief             - Toggles the slave select if it is software managed
 *
 * @param[in]         - SPI_RegDef_t *pSPIx - the SPI peripheral
 * @param[in]		  - enabled - the enabled/disabled flag
 ********************************************************************/
void SPI_Toggle_SSI(SPI_RegDef_t* pSPIx, u8 enabled)
{
	/* TODO First check to see if software slave management is enabled */

	pSPIx->CR1 |= (enabled << SPICR1_BIT_SSI);
}

void SPI_SSOE_Configure(SPI_RegDef_t* pSPIx,u8 enabled)
{
	pSPIx->CR2 |= (enabled << SPICR2_BIT_SSOE);
}

void SPI_ReceiveBlocking(SPI_RegDef_t* pSPIx, SPI_Transfer_t* pTransfer)
{
	/* Check the SPI mode - if the mode is unidirectional receivev-only
	 * mode, then it may be the case that the SPI isn't enabled, but
	 * the receive sequence automatically starts in this mode once the SPI
	 * is enabled */

	SPI_Mode_t currentSPIMode = SPI_GetCurrentMode(pSPIx);

	if(currentSPIMode == ModeUnidirectionalRxOnly)
	{
		if( (pSPIx->CR1 & 0x20) != 0x20)
		{
			pSPIx->CR1 |= (ENABLE << SPICR1_BIT_SPE);
		}
	}

	/* If an RX buffer wasn't provided but it is required to
	 * be read given the mode, then just flush the
	 * data buffer so no overrun happens */

	if(!pTransfer->rxBuffer)
	{
		while(read_status_bit(pSPIx, RXNE_BITMASK) == RX_EMPTY);
		SPI_FlushFifo(pSPIx);
	}
	else
	{

		while(pTransfer->rxSize > 0 )
		{
			/* Wait for the receive buffer not empty flag to be set */
			while(read_status_bit(pSPIx, RXNE_BITMASK) == RX_EMPTY);

			if(bytes_per_frame(pSPIx) == 2)
			{
				*((u16*)pTransfer->rxBuffer) = (u16)pSPIx->DR;
				(u16*)pTransfer->rxBuffer++;
				pTransfer->rxSize--;
				pTransfer->rxSize--;
			}
			else
			{
				*pTransfer->rxBuffer = (u8)pSPIx->DR;
				pTransfer->rxBuffer++;
				pTransfer->rxSize--;
			}
		}
	}

}

void SPI_ReceiveNonBlocking(SPI_Handle_t *pHandle, SPI_Transfer_t *pTransfer)
{
	/* Check the state of handle. If it is already marked as
	 * receiving, then there is nothing to do. Otherwise, mark
	 * the handle as receiving and save off the receive buffer
	 * so that the when the interrupt is generated, the appropriate action is taken
	 */

	if(pHandle->rxState == BUSY)
		return;


	pHandle->rxBuffer = pTransfer->rxBuffer;
	pHandle->rxSize = sizeof(pTransfer->rxBuffer);
	pHandle->rxState = BUSY;

	/* Enable the RXEIE (RX buffer empty interrupt */
	if(((pHandle->pSPIx->CR2 >> SPICR2_BIT_RNEIE) & 0x1) == DISABLE)
		pHandle->pSPIx->CR2 |= (ENABLE << SPICR2_BIT_RNEIE);
}


static void handle_tx_interrupt(SPI_Handle_t *pHandle)
{

	/* First check to see if the transfer buffer length is 0, if so
	 * blank the handle and execute callback if one */

	if(pHandle->txSize <= 0)
	{
		if(pHandle->txISRCallback)
		{
			/* Disable the TXE interrupt before executing the callback */
			pHandle->pSPIx->CR2 &= ~(ENABLE << SPICR2_BIT_TXEIE);
			pHandle->txISRCallback();
		}

		SPI_CloseTransmission(pHandle);
	}

	/* The SPI peripheral is ready for a data transmission, load
	 * the FIFO according to the DFF, update the handle and return */


	/* 16 bit DFF */
	if(bytes_per_frame(pHandle->pSPIx) == 2)
	{
		pHandle->pSPIx->DR = *((u16*)pHandle->txBuffer);
		(u16*)pHandle->txBuffer++;
		pHandle->txSize--;
		pHandle->txSize--;
	}
	else
	{
		printf("Loading buffer\n");
		if(pHandle->txSize == 3)
			SPI_ToggleCS(pHandle);
		pHandle->pSPIx->DR = *pHandle->txBuffer;
		pHandle->txBuffer++;
		pHandle->txSize--;
	}

}

static void handle_rxne_interrupt(SPI_Handle_t *pHandle)
{

	/* If the RX buffer is null, then we're probably dealing with a
	 * transfer only operation in full duplex mode, so just clear the
	 * FIFO
	 */

	if(!pHandle->rxBuffer)
	{
		SPI_FlushFifo(pHandle->pSPIx);
		return;
	}

	/* First check to see if the receive buffer length is 0, if so
	 * blank the handle and execute callback if one */

	if(pHandle->rxSize <= 0)
	{
		memset(pHandle, 0, sizeof(SPI_Handle_t));
		SPI_Close_Reception(pHandle);

		if(pHandle->rxISRCallback)
		{
			pHandle->rxISRCallback();
		}
	}


	if(bytes_per_frame(pHandle->pSPIx) == 2)
	{
		*((u16*)pHandle->rxBuffer) = (u16)pHandle->pSPIx->DR;
		pHandle->rxSize--;
		pHandle->rxSize--;
		(u16*)pHandle->rxBuffer++;
	}
	else
	{
		*pHandle->rxBuffer = (u8)pHandle->pSPIx->DR;
		pHandle->rxSize--;
		pHandle->rxBuffer++;
	}

}

static u8 handle_err_interrupt(SPI_Handle_t *pHandle)
{
	/* Read the most significant 8 bits of the status register and
	 * pack those results into the error struct of the handle */

	u8 *err_handle = (u8*)&pHandle->error_handle;
	u8 hasErrorState = 0;

	static const u8 err_bits[3] = {SPISTAT_MODF, SPISTAT_OVR, SPISTAT_CRCERR};

	for(u8 i = 0; i < sizeof(SPI_Err_Handle_t); i++)
	{
		u8 *member = (err_handle + i);
		*member = read_status_bit(pHandle->pSPIx, err_bits[i]);

		if(*member == 1)
			hasErrorState = 1;
	}

	return hasErrorState;
}

/*********************************************************************************************
 * @function      	  - SPI_Close_Reception
 * @brief             - Clears the handle transmission details and disables
 	 	 	 	 	    the RX buffer not empty interrupt

 * @param[in]         - SPI_Handle_t *pHandle - pointer to the SPI handle.
 *
 * @Note              - See SPI_Close_Transmission for use explanations
 ********************************************************************************************/
void SPI_Close_Reception(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~(ENABLE << SPICR2_BIT_RNEIE);

	memset(&pHandle->rxBuffer, 0, sizeof(pHandle->rxBuffer));
	pHandle->rxSize = 0;
	pHandle->rxState = NOT_BUSY;
}

/*********************************************************************************************
 * @function      	  - SPI_Close_Transmission
 * @brief             - Clears the handle transmission details and disables
 	 	 	 	 	    the TX buffer empty interrupt

 * @param[in]         - SPI_Handle_t *pHandle - pointer to the SPI handle.
 *
 * @Note              - This is primarily used internally. It can be used in a sense
 *					    to cancel the current transaction before a TX buffer empty interrupt
 *					    is generated. This function only applies if the send method is thru
 *					    interrupts, with synchronous, this function call has no effect.
 ********************************************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~(ENABLE << SPICR2_BIT_TXEIE);

	memset(&pHandle->txBuffer, 0, sizeof(pHandle->txBuffer));
	pHandle->txSize = 0;
	pHandle->txState = NOT_BUSY;
}

/*********************************************************************************************
 * @function      	  - SPI_IRQ_Handle
 * @brief             - Responsible for handling interrupts generated by the SPI peripheral

 * @param[in]         - SPI_Handle_t *pHandle - pointer to the SPI handle.
 * @Note              - Should execute this function from within the redefined SPIx ISR
 	 	 	 	 	 	to properly handle receive,transfer and error interrupts from the
 	 	 	 	 	 	SPI peripheral
 ********************************************************************************************/
void SPI_IRQ_Handle(SPI_Handle_t *pHandle)
{

	SPI_RegDef_t *pSPIx = pHandle->pSPIx;

	/* Check TXE flag */
	if(read_control_bit(pSPIx, SPICR2_BIT_TXEIE,SPI_CR2_READ) &&
			read_status_bit(pSPIx, SPISTAT_TXE))
	{
		handle_tx_interrupt(pHandle);
	}

	/* Check RXNE flag */
	if(read_control_bit(pSPIx, SPICR2_BIT_RNEIE,SPI_CR2_READ) &&
			read_status_bit(pSPIx, SPISTAT_RXNE))
	{
		handle_rxne_interrupt(pHandle);
	}

	/* Check ERR flag */
	if(read_control_bit(pSPIx, SPICR2_BIT_ERRIE, SPI_CR2_READ))
	{
		handle_err_interrupt(pHandle);
	}
}


u8 SPI_CheckTransferArgument(SPI_Handle_t *pHandle, SPI_Transfer_t *transfer)
{
	assert(transfer != NULL);

	if(transfer->txSize == 0U)
	{
		return INVALID;
	}

	if( (NULL == transfer->txBuffer) && (NULL == transfer->rxBuffer))
	{
		return INVALID;
	}

	/* If bidi mode is enabled, then only only TX or RX can happen at once, not
	 * simultaneously */
	if(pHandle->config.bidi_mode == ENABLE)
	{
		/* If RX only is enabled, check to make sure TX buffer is empty */
		if( ((pHandle->pSPIx->CR1 >> SPICR1_BIT_RXONLY ) & 0x1) == ENABLE)
		{
			if(transfer->txBuffer != NULL)
			{
				return INVALID;
			}
		}

		/* Otherwise, check to make sure txBuffer and rxBuffer both aren't null */
		else
		{
			if(transfer->rxBuffer != NULL && transfer->txBuffer != NULL)
			{
				return INVALID;
			}
		}
	}

	u32 bitsPerFrame = (((pHandle->pSPIx->CR1 >> SPICR1_BIT_DFF) & 0x1) * 8U) + 8U;
	u32 bytesPerFrame = bitsPerFrame / 8U;

	/* If the bytes per frame is equal to 2, then the data size must be a multiple of 2 */
	if(transfer->txSize % bytesPerFrame > 0)
	{
		return INVALID;
	}

	return VALID;

}


/*********************************************************************************************
 * @function      	  - SPI_InterruptConfig
 * @brief             - Reads the SPI's current interrupt configuration

 * @param[in]         - pSPIx - SPI peripheral.
 *
 * @return 			  - u8 where 3 MSBs (from first to last) are TXEIE, RXNEIE, ERRIE
 ********************************************************************************************/
u8 SPI_InterruptConfig(SPI_RegDef_t *pSPIx)
{
	u32 reg = pSPIx->CR2;
	u8 TXEIE = reg >> SPICR2_BIT_TXEIE & 0x1;
	u8 RXNEIE = reg >> SPICR2_BIT_RNEIE & 0x1;
	u8 ERRIE = reg >> SPICR2_BIT_ERRIE & 0x1;

	return SPI_IT_TXEIE(TXEIE) | SPI_IT_RXNEIE(RXNEIE) | SPI_IT_ERRIE(ERRIE);
}

/*********************************************************************************************
 * @function      	  - SPI_DisableInterrupts
 * @brief             - Disables all interrupts on the SPI peripheral

 * @param[in]         - pSPIx - SPI peripheral.
 ********************************************************************************************/
void SPI_DisableInterrupts(SPI_RegDef_t *pSPIx, const u8 interruptConfig)
{
	if(interruptConfig <= SPI_IT_ALL)
		pSPIx->CR2 &= ~(interruptConfig << SPICR2_BIT_ERRIE);
}

/*********************************************************************************************
 * @function      	  - SPI_DisableInterrupts
 * @brief             - Enables all interrupts on the SPI peripheral

 * @param[in]         - pSPIx - SPI peripheral.
 ********************************************************************************************/
void SPI_EnableInterrupts(SPI_RegDef_t *pSPIx, const u8 interruptConfig)
{
	if(interruptConfig <= SPI_IT_ALL)
		pSPIx->CR2 |= (interruptConfig << SPICR2_BIT_ERRIE);
}

void SPI_TransferAbort(SPI_Handle_t *pHandle)
{
	assert(pHandle != NULL);

	/* Disable SPI interrupts */
	SPI_DisableInterrupts(pHandle->pSPIx, SPI_IT_ALL);

	/* Reset the SPI */
	SPI_DeInit(pHandle->pSPIx);

	/* Reset handle items */
	pHandle->rxSize = 0;
	pHandle->txSize = 0;
	memset(&pHandle->rxBuffer, 0, sizeof(pHandle->rxBuffer));
	memset(&pHandle->txBuffer, 0, sizeof(pHandle->txBuffer));
}

/*********************************************************************************************
 * @function      	  - SPI_GetCurrentMode
 * @brief             - Determines the mode the SPI is currently in based on its configuruations
 * 						for the transmission and reception procedures. Particularly important as
 * 						each mode requires different steps to be taken when the SPI is disabled

 * @param[in]         - pSPIx - SPI peripheral.
 * @return			  - the SPI mode
 ********************************************************************************************/
SPI_Mode_t SPI_GetCurrentMode(SPI_RegDef_t *pSPIx)
{
	u8 bidiModeEnable = (SPI_ReadBit(CR1_REG(pSPIx), SPICR1_BIT_BIDIMODE) == 1);
	u8 bidiOEDirection = SPI_ReadBit(CR1_REG(pSPIx), SPICR1_BIT_BIDIOE);

	SPI_Mode_t mode = ModeUnknown;

	/* Full-duplex */
	if(!bidiModeEnable)
	{
		/* Unidirectional receive-only mode (BIDIMODE=0 and RXONLY=1)
		* Full duplex (BIDIMODE=0 and RXONLY=0)
		*/

		mode = isRxOnly(pSPIx) ? ModeUnidirectionalRxOnly : ModeFullDuplex;
	}
	/* Half-duplex */
	else
	{
		/* Bidirectional transmit mode (BIDIMODE=1 and BIDIOE=1)
		* Bidirectional receive mode (BIDIMODE=1 and BIDIOE=0) */
		mode = bidiOEDirection == 1 ? ModeBidirectionalTxOnly : ModeBidirectionalRxOnly;
	}

	return mode;
}


/*********************************************************************
 * @function      	  - SPI_Disable
 * @brief             - Disables the SPI peripheral when it is ready

 * @param[in]         - SPI_RegDef_t *pSPIx
 *
 * @return            - none
 ********************************************************************/
void SPI_Disable(SPI_RegDef_t* pSPIx)
{
	u8 currentSPIMode = SPI_GetCurrentMode(pSPIx);

	printf("CURRENT SPI MODE %d\n", currentSPIMode);

	/* Wait for TXE=1 and BSY=0 before disabling the peripheral */
	if(currentSPIMode == ModeFullDuplex || currentSPIMode == ModeBidirectionalTxOnly)
	{
		/* We should read the receive buffer before disabling the peripheral, but given
		 * the conditions of these two modes, the receive buffer should always be read.
		 * In the case of a blocking transmission, a receive is always done for each byte transferred
		 * and with a non blocking transfer, the Rx buffer is read with the interrupt */

		while(true)
		{
			u8 TXE_flag = read_status_bit(pSPIx, SPISTAT_TXE);
			u8 busy_flag = read_status_bit(pSPIx, SPISTAT_BIT_BSY);

			if(TXE_flag == TX_EMPTY && busy_flag == NOT_BUSY)
				break;
		}
	}
	else if(currentSPIMode == ModeUnidirectionalTxOnly || currentSPIMode == ModeBidirectionalTxOnly)
	{
		while(read_status_bit(pSPIx, SPISTAT_TXE) == TX_NOT_EMPTY);
		while(read_status_bit(pSPIx, SPISTAT_BIT_BSY));
	}
	else if(currentSPIMode == ModeUnidirectionalRxOnly || currentSPIMode == ModeBidirectionalRxOnly)
	{
		u8 FRFBit = (pSPIx->CR2 >> SPICR2_BIT_FRF ) & 0x1;

		/* For SPI TI configuration (FRF bit set to 1), the following
		 * procedure has to be taken in order to avoid generating an undesired
		 * pulse on the NSS when the SPI is disabled
		 */
		if(FRFBit)
		{

		}

		/* For SPI Motorola configuration (FRF bit set to 0),
		 * ensure a new transfer is not initiated
		 */
		else
		{

		}

	}

	pSPIx->CR1 &= ~(ENABLE << SPICR1_BIT_SPE);
}

SPI_ClockStatus_t SPI_ChooseBestClockFrequency(SPI_RegDef_t *pSPIx, const unsigned char freqMHz)
{

	SPI_ClockStatus_t clockStatus;
	static const u16 baudRates[8] = {2,4,8,16,32,64,128,256};
	static const u8 baudSettings[8] = {0,1,2,3,4,5,6,7};

	/* For right now, assume we are running the HSI clock, which has a max of only 16 MHz
	 * TODO if the sysclock is not being driven by HSI, measure the HSE/PLL frequency */
	u16 peripheralFrequency = 0U;
	u16 sysClkFrequency = get_sysclk_frequency();

	if(pSPIx == SPI1)
	{
		peripheralFrequency = sysClkFrequency / get_apb_prescaler(APB1_PRESCALER_READ);
	}
	else
	{
		peripheralFrequency = sysClkFrequency / get_apb_prescaler(APB2_PRESCALER_READ);
	}

	/* First check to see if the desired frequency is greater than the current bus speed.
	 * If it is, then change the desired frequency to match the max speed of the bus
	 */

	if(freqMHz > peripheralFrequency)
	{
		clockStatus.adjustedFrequency = peripheralFrequency;
		clockStatus.status = FrequencyLowered;
		clockStatus.baudSetting = baudSettings[0];
	}
	else
	{
		u16 frequencies[9];
		u8 matchIndex = 0U;
		u16 freqResult = 999;

		for(unsigned char i = 0; i < 8; i++)
		{
			u8 freq = freqMHz / baudRates[i];

			if(freq > 0)
			{
				frequencies[i] = freq;
				u8 distance = freqMHz - freq;

				if(distance < freqResult)
				{
					freqResult = freq;
					matchIndex = i;
				}
			}

		}

		clockStatus.adjustedFrequency =  freqResult;
		clockStatus.status = FrequencyIncreased;

		if(matchIndex <= sizeof(baudSettings) / sizeof(u8))
		{
			clockStatus.baudSetting = baudSettings[matchIndex];
		}

		(void)frequencies;
	}

	return clockStatus;
}
