/*
 * gpio.h
 *
 *  Created on: May 13, 2024
 *      Author: Andrew
 */

#ifndef GPIO_H_
#define GPIO_H_


#include "stm32f4xx.h"

/* MODER register configuration values */
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTERNATE 2
#define GPIO_ANALOG 		3

#define GPIO_PUSH_PULL 		0
#define GPIO_OPEN_DRAIN		1

/* OSPEEDR register configuration values */
#define GPIO_SPEED_LOW		1
#define GPIO_SPEED_MEDIUM	2
#define GPIO_SPEED_HIGH		3
#define GPIO_SPEED_VHIGH	4

/* PUPDR register configuration values */
#define GPIO_NPUPD 			1 	/* No pull-up, no pull-down */
#define GPIO_PU				2
#define GPIO_PUPD			3
#define GPIO_RESERVED		4

/* Edge detection values for interrupt */
#define GPIO_RT			1		/* Rising edge */
#define GPIO_FT			2		/* Falling edge */

/* GPIO alternate functionality macros */
#define GPIO_AF_0 		0
#define GPIO_AF_1		1
#define GPIO_AF_2		2
#define GPIO_AF_3 		3
#define GPIO_AF_4		4
#define GPIO_AF_5		5
#define GPIO_AF_6		6
#define GPIO_AF_7		7
#define GPIO_AF_8		8
#define GPIO_AF_9		9
#define GPIO_AF_10		10
#define GPIO_AF_11		11
#define GPIO_AF_12		12
#define GPIO_AF_13		13
#define GPIO_AF_14		14
#define GPIO_AF_15		15

#define GPIO_PA 		0
#define GPIO_PB			1
#define GPIO_PC			2
#define GPIO_PD			3
#define GPIO_PE			4
#define GPIO_PF			5
#define GPIO_PG			6
#define GPIO_PH			7
#define GPIO_PI			8
#define GPIO_PJ			9
#define GPIO_PK			10

/* GPIO pin number macros - each pad has 16 pins */
#define GPIO_PIN_ONE				1
#define GPIO_PIN_TWO				2
#define GPIO_PIN_THREE				3
#define GPIO_PIN_FOUR				4
#define GPIO_PIN_FIVE				5
#define GPIO_PIN_SIX				6
#define GPIO_PIN_SEVEN				7
#define GPIO_PIN_EIGHT				8
#define GPIO_PIN_NINE				9
#define GPIO_PIN_TEN				10
#define GPIO_PIN_ELEVEN				11
#define GPIO_PIN_TWELVE				12
#define GPIO_PIN_THIRTEEN			13
#define GPIO_PIN_FOURTEEN			14
#define GPIO_PIN_FIFTEEN			15
#define GPIO_PIN_SIXTEEN			16

/* The configurable GPIO parameters  */
typedef struct
{
	uint8_t output_speed;				/* GPIO output speed */
	uint8_t mode_type;					/* GPIO mode, input/output, alternate */
	uint8_t pin_number;					/* GPIO pin to configure */
	uint8_t alternate_mode_type;		/* GPIO alternate mode type (0-15) */
	uint8_t output_type;				/* GPIO output type - open drain, push/pull, etc */
	uint8_t od_state_init;				/* For open drain configuration - set as 0 if the pin is desired
										 * to be low upon initialization, 1 otherwise */
	uint8_t PUPD; 						/* Pull-up pull-down resistor configuration */
	uint8_t edge_detection;				/* Rising/Falling edge for the pin interrupt - default is rising*/

}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t* pGPIOPad;
	GPIO_PinConfig_t config;
}GPIO_Handle_t;

void GPIO_Init(GPIO_Handle_t *pHandle);
void GPIO_DeInit(GPIO_Handle_t *pHandle);
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint32_t, uint8_t value);
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t status);
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint32_t pin);
uint32_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint32_t value);
void GPIO_ISR(const uint8_t pin_number);

void GPIO_IRQConfig(GPIO_Handle_t *pHandle, uint8_t IRQ_Number, uint8_t enabled);
void GPIO_IRQPriority(void);

#endif /* GPIO_H_ */
