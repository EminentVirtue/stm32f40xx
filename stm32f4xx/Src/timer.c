/*
 * timer.c
 *
 *  Created on: Jul 6, 2024
 *      Author: Andrew Streng
 */


#include "timer.h"

void select_clock_source(uint8_t source)
{

}
void Timer_Init(Timer_Handle_t *pHandle)
{

}
void Timer_DeInit(Timer_Handle_t *pHandle)
{

}
void Timer_PeripheralClockControl(GPTimer_RegDef_t *pTimer, uint8_t enabled)
{

}
void read_timer_counter(TimerInputCapture_t *pHandle)
{

}
void toggle_counter(GPTimer_RegDef_t *pTimer, uint8_t enabled)
{
	uint8_t enabled_status = (pTimer->CR2 >> TIMCR1_BIT_CEN) & 0x1;

	if(enabled)
	{
		if(!enabled_status)
			pTimer->CR2 |= ENABLED << TIMCR1_BIT_CEN;
	}
	else
	{
		pTimer->CR2 &= ~(ENABLED << TIMCR2_BIT_CEN);
	}
}

void update_timer_config(TimerConfigUpdate_t *pConfig)
{
	/* First disable the UEV so no pre-loading occurs during the update */
	pTimer->CR1 |= (ENABLE << TIMCR1_BIT_USID);
}

static void Handle_HSIMeasurement()
{
	/* See pg. 161 of the reference manual for full analysis
	 * The following steps are needed to perform the measurement
	 * 1. Enable the TIM5 timer and configure channel4 in Input capture mode by
	 * setting the TI4_RMP bits in the TIM5_OR register to 0x01, which then connects
	 * the LSI clock internally to the input capture channel
	 * 2. Use the TIM5 capture/compare 4 event, so enable the interrupt
	 *
	 */

	/* Enable Timer5 peripheral clock */
	Timer_PeripheralClockControl(TIMER5, ENABLE);

	/* Configure the option register (TIM5_OR) to make the TIM5_CH4
	 * take the LSI internal clock as the input source
	 */

}
/*********************************************************************************************
 * @function      	  - Timer_ConfigureExternalMode1
 * @brief             - Configures the timer for external mode one, which ticks the counter
 * 						at each rising/falling edge of a selected input

 * @param[in]         - pConfig - The external mode one configuration handle
 * @param[in]		  - pTimer - A pointer to some timer
 ********************************************************************************************/

void Timer_ConfigureExternalMode1(ExtModeOne_Config_t* pConfig, void *pTimer)
{
	if(pConfig->timerType == GeneralPurpose)
	{
		GPTimer_RegDef_t *gpTimer = (GPTimer_RegDef_t*)pTimer;
		memset(gpTimer, 0, sizeof(GPTimer_RegDef_t));

		if(pConfig->edgeSelection == RISING_EDGE)
		{

		}
		else
		{

		}
	}
}

/*********************************************************************************************
 * @function      	  - Timer_ConfigureExternalMode2
 * @brief             - Configures the timer for external mode two, which ticks the counter
 * 						at each any active edge on the ETRF signal

 * @param[in]         - pConfig - The external mode two configuration handle
 * @param[in]		  - pTimer - A pointer to some timer
 ********************************************************************************************/
void Timer_ConfigureExternalMode2(ExtModeTwo_Config_t *pConfig, void *pTimer)
{
	if(pConfig->timerType == GeneralPurpose)
	{
		GPTimer_RegDef_t *gpTimer = (GPTimer_RegDef_t*)pTimer;
		memset(gpTimer, 0, sizeof(GPTimer_RegDef_t));

		/* 1. Configure the external trigger prescaler
		 * 2. Configure the external trigger filter
		 * 3. Configure the mode as external mode 2
		 */

		gpTimer->SMCR |= (pConfig->ETPS_Prescaler << TIMSMCR_BIT_ETPS);
		gpTimer->SMCR |= (0x0 << TIMSMCR_BIT_ETF);
		gpTimer->SMCR |= (1 << TIMSMCR_BIT_ECE);

	}
}



static void Handle_HSEMeasurement()
{

}
unsigned short measure_sysclk_frequency(void)
{
	/* Timers can be used to give a rough estimation of a given
	 * internal/external clock frequency. This is done by configuring
	 * the timer's input multiplexer which allows choosing whether the input capture is triggered
	 * by I/O or by an internal/external clock signal. The number of clock counts between consecutive
	 * edges of the signal provides a measurement of the clock period, which can then be used to
	 * calculate the frequency
	 */

	ExtModeTwo_Config_t config;
	config.timerType = GeneralPurpose;
	config.ETPS_Prescaler = ETRP_FreqDiv2;

	Timer_ConfigureExternalMode2(&config, (void*)TIMER5);


}


