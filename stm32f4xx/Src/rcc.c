/*
 * rcc.c
 *
 *  Created on: Jul 13, 2024
 *      Author: Andrew Streng
 */

#include "rcc.h"

/*********************************************************************
 * @function      	  - get_apb_prescaler
 * @brief             - Returns the current configured prescaler values
 	 	 	 	 	 	of the APB1 or APB2 bus

 * @param[in]         - prescalerSelection - APB1_PRESCALER_READ or
 * 						APB2_PRESCALER_READ
 *
 * @return            - none
 ********************************************************************/
u8 get_apb_prescaler(const unsigned char prescalerSelection)
{
	uint8_t prescaler = 0U;
	static const u8 apb_prescalers[4] = {2,4,6,8};

	if(prescalerSelection == APB1_PRESCALER_READ)
	{
		prescaler = ((RCC->CFGR >> 13) & 0x7);
	}
	else if(prescalerSelection == APB2_PRESCALER_READ)
	{
		prescaler = ((RCC->CFGR >> 10) & 0x7);
	}
	else
		return prescaler;

	if(prescaler < 0)
	{
		prescaler = 1;
	}
	else
	{
		prescaler = apb_prescalers[prescaler - 3];
	}

	return get_ahb_prescaler() / prescaler;
}

u16 get_ahb_prescaler()
{
	u8 prescaler = (RCC->CFGR >> 4) & 0xF;
	static const u16 ahb_prescalers[9] = {2,4,6,8,16,64,128,256,512};

	if(prescaler < 8)
	{
		prescaler = 1;
	}
	else
	{
		prescaler = ahb_prescalers[prescaler - 8];
	}

	return prescaler;
}

/*********************************************************************
 * @function      	  - choose_best_peripheral_frequency
 * @brief             - Attempts to determine the best possible frequency

 * @param[in]         - prescalerSelection - APB1_PRESCALER_READ or
 * 						APB2_PRESCALER_READ
 *
 * @return            - none
 ********************************************************************/
u16 choose_best_peripheral_frequency(FrequencyConfig_t *pFreqConfig)
{
/*	u8 peripheralBus = pFreqConfig->peripheralBus;
	u8 peripheralSelection = 0U;

	if(peripheralBus == APB1_PERIPHERAL)
		peripheralSelection = APB1_PRESCALER_READ;
	else
		peripheralSelection = APB2_PRESCALER_READ;

	u16 initialPeriphalSpeed = get_ahb_prescaler(peripheralSelection);

	if(directive ==  Highest)
	{

	}
	else if(directive == Lowest)
	{

	}
	else
	{

	}*/

	return 0;
}


