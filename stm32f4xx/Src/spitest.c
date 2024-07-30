/*
 * spitest.c
 *
 *  Created on: Jun 18, 2024
 *      Author: Andrew
 *
 *  Description: SPI driver demo - communicate with an RPI slave
 */

#include "stm32f4xx.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);
int main(void)
{
	/* For printf */
	initialise_monitor_handles();


	/* SPI MOSI pin - PB5 */
	GPIO_Handle_t spi_mosi;
	memset(&spi_mosi, 0, sizeof(spi_mosi));
	spi_mosi.pGPIOPad = GPIOB;
	spi_mosi.config.mode_type = GPIO_MODE_ALTERNATE;
	spi_mosi.config.output_type = GPIO_PUSH_PULL;
	spi_mosi.config.alternate_mode_type = GPIO_AF_5;
	spi_mosi.config.pin_number = GPIO_PIN_FIVE;
	spi_mosi.config.PUPD = GPIO_NPUPD;

	GPIO_Init(&spi_mosi);

	/* SPI Chip Select pin - PC3, keep idle high */
	GPIO_Handle_t spi_cs;
	memset(&spi_cs, 0, sizeof(spi_cs));
	spi_cs.pGPIOPad = GPIOC;
	spi_cs.config.mode_type = GPIO_MODE_OUTPUT;
	spi_cs.config.output_type = GPIO_OPEN_DRAIN;
	spi_cs.config.pin_number = GPIO_PIN_THREE;
	spi_cs.config.od_state_init = GPIO_UNSET;
	spi_cs.config.PUPD = GPIO_PU;

	GPIO_Init(&spi_cs);

	/* Level shifter output enable pin */
	GPIO_Handle_t level_shifter_oe;
	memset(&level_shifter_oe, 0, sizeof(level_shifter_oe));
	level_shifter_oe.pGPIOPad = GPIOB;
	level_shifter_oe.config.mode_type = GPIO_MODE_OUTPUT;
	level_shifter_oe.config.output_speed =GPIO_SPEED_HIGH;
	level_shifter_oe.config.output_type = GPIO_PUSH_PULL;
	level_shifter_oe.config.pin_number = GPIO_PIN_SEVEN;
	level_shifter_oe.config.PUPD = GPIO_NPUPD;

	GPIO_Init(&level_shifter_oe);

	/* SPI Serial Clock pin - PA5 */
	GPIO_Handle_t spi_sck;
	memset(&spi_sck, 0, sizeof(spi_sck));
	spi_sck.pGPIOPad = GPIOA;
	spi_sck.config.mode_type = GPIO_MODE_ALTERNATE;
	spi_sck.config.output_speed =GPIO_SPEED_HIGH;
	spi_sck.config.output_type = GPIO_PUSH_PULL;
	spi_sck.config.alternate_mode_type = GPIO_AF_5;
	spi_sck.config.pin_number = GPIO_PIN_FIVE;
	spi_sck.config.PUPD = GPIO_NPUPD;

	GPIO_Init(&spi_sck);

	/* Initialize the SPI peripheral */
	SPI_Handle_t spi_handle;
	memset(&spi_handle, 0, sizeof(spi_handle));

	spi_handle.pSPIx = SPI1;
	spi_handle.config.bidi_mode = SPI_BIDI_OFF;
	spi_handle.config.bidi_output_mode = SPI_OUTPUT_DISABLE;
	spi_handle.config.frame_format = SPI_DFF_8BIT;
	spi_handle.config.slave_management = SPI_SW_SLAVE_ENABLE;
	spi_handle.config.master_selection = SPI_MASTER_ENABLE;

	/* Anything higher and the scope may not capture the frame correctly */
	spi_handle.config.spi_speed = PCLK_DIV_8;

	SPI_Init(&spi_handle);

	/* Ground the SPI's slave select internally - the MCU is always in master mode */
	SPI_Toggle_SSI(spi_handle.pSPIx, GPIO_SET);
	SPI_SSOE_Configure(spi_handle.pSPIx, DISABLE);

	SPI_Prepare(SPI1);

	//uint8_t buff[3] = {69,68,67};
	uint8_t buff[1] = {69};

	SPI_Transfer_t transfer;
	memset(&transfer,0,sizeof(transfer));
	transfer.txBuffer = buff;
	transfer.txSize = sizeof(buff);

	GPIO_WriteToPin(spi_cs.pGPIOPad, spi_cs.config.pin_number, GPIO_SET);
	GPIO_WriteToPin(level_shifter_oe.pGPIOPad, level_shifter_oe.config.pin_number, GPIO_SET);
	SPI_MasterTransferBlocking(&spi_handle, &transfer);

	//SPI_Disable(SPI1);
	//GPIO_WriteToPin(spi_cs.pGPIOPad, spi_cs.config.pin_number, GPIO_SET);
	printf("Made it here\n");

	/* Loop forever */
	for(;;);
}
