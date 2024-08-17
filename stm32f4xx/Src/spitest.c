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

#define CONTINUOUS_SEND 	ENABLE
//#define BLOCKING_TEST		ENABLE
static SPI_Handle_t sHandle;

void test()
{
	printf("It worked \n");
}

int main(void)
{
	/* For printf */
	initialise_monitor_handles();


	/* SPI MOSI pin - PB5 */
	GPIO_Handle_t sMosi;
	memset(&sMosi, 0, sizeof(sMosi));
	sMosi.pGPIOPad = GPIOB;
	sMosi.config.mode_type = GPIO_MODE_ALTERNATE;
	sMosi.config.output_type = GPIO_PUSH_PULL;
	sMosi.config.alternate_mode_type = GPIO_AF_5;
	sMosi.config.pin_number = GPIO_PIN_FIVE;
	sMosi.config.output_speed = GPIO_SPEED_VHIGH;
	sMosi.config.PUPD = GPIO_NPUPD;

	GPIO_Init(&sMosi);

	/* SPI Chip Select pin - PC3, keep idle high */
	GPIO_Handle_t sChipSelect;
	memset(&sChipSelect, 0, sizeof(sChipSelect));
	sChipSelect.pGPIOPad = GPIOC;
	sChipSelect.config.mode_type = GPIO_MODE_OUTPUT;
	sChipSelect.config.output_speed = GPIO_SPEED_HIGH;
	sChipSelect.config.output_type = GPIO_PUSH_PULL;
	sChipSelect.config.pin_number = GPIO_PIN_THREE;
	sChipSelect.config.od_state_init = GPIO_SET;
	sChipSelect.config.PUPD = GPIO_NPUPD;

	GPIO_Init(&sChipSelect);

	/* Initial state of chip select is high */
	GPIO_WriteToPin(sChipSelect.pGPIOPad, sChipSelect.config.pin_number, GPIO_SET);

	/* Level shifter output enable pin */
	GPIO_Handle_t levelShifterOE;
	memset(&levelShifterOE, 0, sizeof(levelShifterOE));
	levelShifterOE.pGPIOPad = GPIOB;
	levelShifterOE.config.mode_type = GPIO_MODE_OUTPUT;
	levelShifterOE.config.output_speed = GPIO_SPEED_HIGH;
	levelShifterOE.config.output_type = GPIO_PUSH_PULL;
	levelShifterOE.config.pin_number = GPIO_PIN_SEVEN;
	levelShifterOE.config.PUPD = GPIO_NPUPD;

	GPIO_Init(&levelShifterOE);

	/* SPI Serial Clock pin - PA5 */
	GPIO_Handle_t sSCK;
	memset(&sSCK, 0, sizeof(sSCK));
	sSCK.pGPIOPad = GPIOA;
	sSCK.config.mode_type = GPIO_MODE_ALTERNATE;
	sSCK.config.output_speed =GPIO_SPEED_HIGH;
	sSCK.config.output_type = GPIO_PUSH_PULL;
	sSCK.config.alternate_mode_type = GPIO_AF_5;
	sSCK.config.pin_number = GPIO_PIN_FIVE;
	sSCK.config.PUPD = GPIO_PU;

	GPIO_Init(&sSCK);

	/* Initialize the SPI peripheral */
	memset(&sHandle, 0, sizeof(sHandle));

	sHandle.pSPIx = SPI1;
	sHandle.config.bidi_mode = SPI_BIDI_OFF;
	sHandle.config.bidi_output_mode = SPI_OUTPUT_DISABLE;
	sHandle.config.frame_format = SPI_DFF_8BIT;
	sHandle.config.slave_management = SPI_SW_SLAVE_ENABLE;
	sHandle.config.master_selection = SPI_MASTER_ENABLE;
	sHandle.pChipSelect = &sChipSelect;
	sHandle.txISRCallback = test;
	sHandle.chipSelectEnabledLevel = GPIO_UNSET;

	/* Note - Anything higher and the signal starts distorting on the scope */
	sHandle.config.spi_speed = PCLK_DIV_8;

	SPI_Init(&sHandle);

#ifndef BLOCKING_TEST
	/* Enable the SPI IRQ */
	SPI_IRQ_Config(SPI1IRQ_NO, ENABLE);
#endif

	/* Ground the SPI's slave select internally - the MCU is always in master mode */
	SPI_Toggle_SSI(sHandle.pSPIx, GPIO_SET);
	SPI_SSOE_Configure(sHandle.pSPIx, DISABLE);

	SPI_Prepare(SPI1);

	/* Enable the B bank on level shifter */
	GPIO_WriteToPin(levelShifterOE.pGPIOPad, levelShifterOE.config.pin_number, GPIO_SET);

	/** Note
	 * The threshold voltage on the scope for the MOSI trigger
	 * at 1.0V decodes fine with signals produced from a 3.3V
	 * system
	 * Needs to be raised higher(around 2v) to decode signals correctly
	 * on a 5V system or the 5V side of the level shifter
	 */


	while(true)
	{
		uint8_t buff[3] = {45,67,45};
		//uint8_t buff[1] = {45};

		printf("Sending a message \n");
		SPI_Transfer_t transfer;
		memset(&transfer,0,sizeof(transfer));
		transfer.txBuffer = buff;
		transfer.txSize = sizeof(buff);

		SPI_Prepare(SPI1);
		printf("YO\n");
	//	GPIO_WriteToPin(sChipSelect.pGPIOPad, sChipSelect.config.pin_number, GPIO_UNSET);


		// 67 == 97
		// 45 == 90
#ifdef BLOCKING_TEST
		SPI_MasterTransferBlocking(&sHandle, &transfer);
#else
		//SPI_ToggleCS(&sHandle);
		SPI_Status_t ret = SPI_MasterTransferNonBlocking(&sHandle, &transfer);
		printf("Status %d\n", ret);
#endif

		SPI_Disable(SPI1);
		GPIO_WriteToPin(sChipSelect.pGPIOPad, sChipSelect.config.pin_number, GPIO_SET);


#ifdef CONTINUOUS_SEND
	for(u64 i = 0; i < 1000000; i++);
#else
	break;
#endif
	}

}

void SPI1_IRQHandler()
{
	SPI_IRQ_Handle(&sHandle);
}
