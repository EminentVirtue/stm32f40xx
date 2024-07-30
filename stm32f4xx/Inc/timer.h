/*
 * timer.h
 *
 *  Created on: Jul 6, 2024
 *      Author: Andrew Streng
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f4xx.h"

#define CK_INT		1		/* Internal clock */
#define CK_EXT		2		/* External input pin */
#define CK_ETR		3		/* External trigger input */

#define CAPTURE_SUCCESS		1
#define CAPTURE_FAIL		0

/* Timer Bit Definitions */
#define TIMCR1_BIT_CEN		0
#define TIMCR1_BIT_USID		1
#define TIMCR1_BIT_URS		2
#define TIMCR1_BIT_OPM		3
#define TIMCR1_BIT_DIR		4
#define TIMCR1_BIT_CMS		5
#define TIMCR1_BIT_ARPE		7
#define TIMCR1_CKD			8

/* Control Register 2 */
#define TIMCR2_BIT_CCDS		3
#define TIMCR2_BIT_MMS		4
#define TIMCR2_BIT_TI1S		7

/* Slave Mode Control Register */
#define TIMSMCR_BIT_SMS		0
#define TIMSMCR_BIT_TS		4
#define TIMSMCR_BIT_MSM		7
#define TIMSMCR_BIT_ETF		8
#define TIMSMCR_BIT_ETPS	12
#define TIMSMCR_BIT_ECE		14
#define TIMSMCR_BIT_ETP		15

/* DMA/Interrupt Enable Register */
#define TIMDEIR_BIT_UIE		0
#define TIMDEIR_BIT_CC1IE	1
#define TIMDEIR_BIT_CC2IE	2
#define TIMDEIR_BIT_CC3IE	3
#define TIMDEIR_BIT_CC4IE	4
#define TIMDEIR_BIT_TIE		6
#define TIMDEIR_BIT_UDE		8
#define TIMDEIR_BIT_CC1DE	9
#define TIMDEIR_BIT_CC2DE	10
#define TIMDEIR_BIT_CC3DE	11
#define TIMDEIR_BIT_CC4DE	12
#define TIMDEIR_BIT_TDE		14

/* Status Register */
#define TIMSTAT_BIT_UIF		0
#define TIMSTAT_BIT_CC1F	1
#define TIMSTAT_BIT_CC2IF	2
#define TIMSTAT_BIT_CC3IF	3
#define TIMSTAT_BIT_CC4IF	4
#define TIMSTAT_BIT_TIF		6
#define TIMSTAT_BIT_CC1OF	9
#define TIMSTAT_BIT_CC2OF	10
#define TIMSTAT_BIT_CC3OF	11
#define TIMSTAT_BIT_CC4OF	12

/* Event Generation Register */
#define TIMEGR_BIT_UG		0
#define TIMEGR_BIT_CC1G		1
#define TIMEGR_BIT_CC2G		2
#define TIMEGR_BIT_CC3G		3
#define TIMEGR_BIT_CC4G		4
#define TIMEGR_BIT_TG		6

/* Capture/Compare Mode Register 1 */
#define TIMCCRMX_BIT_CCXS			0
#define TIMCCMRX_BIT_OCXFE			2
#define TIMCCMRX_BIT_OCXPE			3
#define TIMCCMRX_BIT_OCXM			4
#define TIMCCMRX_BIT_OCXCE			7
#define TIMCCMRX_BIT_CCXS			8
#define TIMCCMRX_BIT_OCXFE			10
#define TIMCCMRX_BIT_OCXPE			11
#define TIMCCMRX_BIT_OCXM			12
#define TIMCCMRX_BIT_OCXCE			15

#define TIMCCMRX_BIT_ICXPSC			2
#define TIMCCMRX_BIT_ICXF			4
#define TIMCCMRX_BIT_CCXS			8
#define TIMCCMRX_BIT_ICXPSC			10
#define TIMCCMRX_BIT_ICXF			12

#define TIMCNT_CNT_LOW				0
#define TIMCNT_CNT_HIGH				16


/* Generic Timer Macros */
#define TIM_ARPE_PRELOAD	1
#define TIM_DIRECTION_UP	0
#define TIM_DIRECTION_DOWN	1
#define TIM_OPM_ENABLE		1

typedef struct {
	uint8_t input_source;
	uint8_t prescaler;
	uint8_t filter;
	uint8_t status;
	uint32_t counter_value;
}TimerInputCapture_t;

typedef struct {
	uint32_t 	counter_value;
	uint32_t 	timeout_value;
	uint8_t  	ar_preload; 	/* Auto-reload preload enabled */
	uint8_t 	clock_division;
	uint8_t 	direction;		/* The counter counts up/down */
	uint8_t 	opm_enabled;	/* One pulse mode enabled? 0 is default, the counter is not stopped
								   after an update event */
}Timer_Handle_t;

typedef struct {
	uint32_t timeout_value;
	uint16_t prescaler_value;

}TimerConfigUpdate_t;

typedef enum {
	ExternalMode1,			/* The counter of the timer counts at each rising/falling edge on selected input */
	ExternalMode2
}TimerMode_t;

typedef enum {
	GeneralPurpose,
	Advanced
}TimerType_t;

typedef enum {
	PrescalerOff = 0x0,
	ETRP_FreqDiv2,
	ETRP_FreqDiv4,
	ETRP_FreqDiv8
}ETPS_Prescaler_t;

typedef struct {
	unsigned char inputFilterDuration;
	u8 edgeSelection;
	TimerType_t timerType;
}ExtModeOne_Config_t;

typedef struct {
	u8 inputSelection;
	TimerType_t timerType;
	ETPS_Prescaler_t ETPS_Prescaler;
}ExtModeTwo_Config_t;

void select_clock_source(uint8_t source);
void Timer_Init(Timer_Handle_t *pHandle);
void Timer_DeInit(Timer_Handle_t *pHandle);
void Timer_PeripheralClockControl(GPTimer_RegDef_t *pTimer, uint8_t enabled );
void update_timer_config(TimerConfigUpdate_t *pConfig);
void read_timer_counter(TimerInputCapture_t *pHandle);
void toggle_counter(GPTimer_RegDef_t *pTimer, uint8_t enabled);
unsigned short measure_sysclk_frequency(void);
void Timer_ConfigureExternalMode1(ExtModeOne_Config_t* pConfig, void *pTimer);
void Timer_ConfigureExternalMode2(ExtModeTwo_Config_t *pConfig, void *pTimer);

void Timer_IRQConfig(uint8_t IRQ_Number, uint8_t enabled);
void Timer_IRQHandle();

#endif /* TIMER_H_ */
