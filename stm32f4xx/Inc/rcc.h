/*
 * rcc.h
 *
 *  Created on: Jul 13, 2024
 *      Author: Andrew Streng
 */

#ifndef RCC_H_
#define RCC_H_


#include "stm32f4xx.h"
static inline u16 get_sysclk_frequency()
{
	// TODO
	return 16;
}

typedef enum {
	Highest,
	Lowest,
	Desired
}PeripheralFrequencySelection_t;


typedef PeripheralFrequencySelection_t Frequency;

typedef struct {
	Frequency frequencySelection;
	u16 desiredFrequencyMHz;
	u8 peripheralBus;
}FrequencyConfig_t;

PrescalerSelection_t get_apb_prescaler(const unsigned char prescalerSelection);
u16 get_ahb_prescaler();
u16 choose_best_peripheral_frequency(FrequencyConfig_t *pFreqConfig);
#endif /* RCC_H_ */
