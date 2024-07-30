/*
 * gpio.c
 *
 *  Created on: May 22, 2024
 *      Author: Andrew Streng
 */



#include "gpio.h"

/*
 * @fn 				- GPIO_Init
 * @brief 			- Initializes the GPIO port
 * @param[in] 		- GPIO_Handle_t*
 *
 * @return 			- none
 */


void GPIO_Init(GPIO_Handle_t *pHandle)
{
	GPIO_RegDef_t* pGPIOx = pHandle->pGPIOPad;
	uint8_t pin_number = pHandle->config.pin_number;

	GPIO_PeripheralClockControl(pGPIOx, ENABLE);

	/* Configure mode */
	if(pHandle->config.mode_type ==  GPIO_MODE_ALTERNATE)
	{
		pGPIOx->MODER |= ( pHandle->config.mode_type << ( 2 * pin_number) );

		/* Determine if the alternate low or high register should be configured */
		if (pHandle->config.pin_number < 8)
		{
			uint32_t temp_reg = ( pHandle->config.alternate_mode_type << (4 * pin_number) );
			pGPIOx->AFRL |= temp_reg;
		}
		else
		{
			uint32_t temp_reg = (pHandle->config.alternate_mode_type << (4 * pin_number ) );
			pGPIOx->AFRH |= temp_reg;

		}
	}
	else
	{
		pGPIOx->MODER |= ( pHandle->config.mode_type << (2 * pin_number));
	}

	/* Configure speed */
	pGPIOx->OSPEEDR |= ( pHandle->config.output_speed << pin_number);

	/* Configure output type */
	if(pHandle->config.mode_type != GPIO_MODE_INPUT)
	{
		pGPIOx->OTYPER |= (pHandle->config.output_type << pin_number);
	}

	/* Configure pull-up/down register */
	pGPIOx->PUPDR |= (pHandle->config.PUPD << pin_number);

	/* If open drain is enabled, configure the initial state of the pin */
	if(pHandle->config.mode_type == GPIO_OPEN_DRAIN)
	{
		GPIO_WriteToPin(pGPIOx, pin_number, pHandle->config.od_state_init);
	}
}

void GPIO_IRQConfig(GPIO_Handle_t *pHandle,uint8_t IRQ_Number, uint8_t enabled)
{

	if(pHandle->config.pin_number > GPIO_PIN_MAX)
		return;

	SYSCFG_CLK_EN;

	/* Configure the SYSCFG multiplexer for EXTI input*/
	uint8_t gpio_pad = 0;
	uint8_t pin_number = pHandle->config.pin_number;
	GPIO_RegDef_t* pGPIOx = pHandle->pGPIOPad;

	if(pGPIOx == GPIOA)
		gpio_pad = GPIO_PA;
	else if(pGPIOx == GPIOB)
		gpio_pad = GPIO_PB;
	else if(pGPIOx == GPIOC)
		gpio_pad = GPIO_PC;
	else if(pGPIOx == GPIOD)
		gpio_pad = GPIO_PD;
	else if(pGPIOx == GPIOE)
		gpio_pad = GPIO_PE;
	else if(pGPIOx == GPIOF)
		gpio_pad = GPIO_PF;
	else if(pGPIOx == GPIOG)
		gpio_pad = GPIO_PG;

	uint8_t bit_pos = pin_number % 4;

	if(enabled == ENABLE)
	{
		SYSCFG->EXTICR[pin_number / EXTI_REG_NUM] |= (gpio_pad << (4 * bit_pos));

		/* Configure the EXTI peripheral */
		EXTI->IMR |= (1 << pin_number);

		/* Configure the edge detection of the interrupt */
		if(pHandle->config.edge_detection == GPIO_RT)
		{
			/* Disable the FT and enable the RT */
			EXTI->FTSR &= ~(1 << pin_number);
			EXTI->RTSR |= (1 << pin_number);
		}
		else
		{
			/* Disable the RT and enable the FT */
			EXTI->RTSR &= ~(1 << pin_number);
			EXTI->FTSR |= (1 << pin_number);
		}

		/* Enable the interrupts in the NVIC - STM32F407xx only has 81 total interrupts */

		if(IRQ_Number > 0 && IRQ_Number < 32)
		{
			*NVIC_ISER0 |= (1 << IRQ_Number);
		}
		else if (IRQ_Number > 32 && IRQ_Number < 64)
		{
			*NVIC_ISER1 |= (1 << IRQ_Number);
		}
		else
		{
			*NVIC_ISER2 |= (1 << IRQ_Number);
		}
	}
	else
	{
		/* Clear EXTI, SYSCFG instances of IRQ and clear edge detection configuration registers */
		SYSCFG->EXTICR[pin_number / EXTI_REG_NUM] &= ~(0xF << (4 * bit_pos));
		EXTI->IMR &= ~(1 << pin_number);
		EXTI->FTSR &= ~(1 << pin_number);
		EXTI->RTSR &= ~(1 << pin_number);

		/* Set NVIC interrupt clear-enable register */
		if(IRQ_Number > 0 && IRQ_Number < 32)
		{
			*NVIC_ICER0 |= (1 << IRQ_Number);
		}
		else if (IRQ_Number > 32 && IRQ_Number < 64)
		{
			*NVIC_ICER0 |= (1 << IRQ_Number);
		}
		else
		{
			*NVIC_ICER0 |= (1 << IRQ_Number);
		}

	}
}

void GPIO_DeInit(GPIO_Handle_t *pHandle)
{
	GPIO_RegDef_t* pGPIOx = pHandle->pGPIOPad;

	if(pGPIOx == GPIOA)
	{
		GPIOA_RESET;
		GPIOA_PCLK_DN;
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_RESET;
		GPIOB_PCLK_DN;
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_RESET;
		GPIOC_PCLK_DN;
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_RESET;
		GPIOD_PCLK_DN;
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_RESET;
		GPIOE_PCLK_DN;
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_RESET;
		GPIOF_PCLK_DN;
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_RESET;
		GPIOG_PCLK_DN;
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_RESET;
		GPIOH_PCLK_DN;
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_RESET;
		GPIOI_PCLK_DN;
	}

}
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx, uint8_t status)
{
	if(pGPIOx == GPIOA)
	{
		status == ENABLE ? GPIOA_PCLK_EN : GPIOA_PCLK_DN;
	}
	else if(pGPIOx == GPIOB)
	{
		status == ENABLE ? GPIOB_PCLK_EN : GPIOB_PCLK_DN;
	}
	else if(pGPIOx == GPIOC)
	{
		status == ENABLE ? GPIOC_PCLK_EN : GPIOC_PCLK_DN;
	}
	else if(pGPIOx == GPIOD)
	{
		status == ENABLE ? GPIOD_PCLK_EN : GPIOD_PCLK_DN;
	}
	else if(pGPIOx == GPIOE)
	{
		status == ENABLE ? GPIOE_PCLK_EN : GPIOE_PCLK_DN;
	}
	else if(pGPIOx == GPIOF)
	{
		status == ENABLE ? GPIOF_PCLK_EN : GPIOF_PCLK_DN;
	}
	else if(pGPIOx == GPIOG)
	{
		status == ENABLE ? GPIOG_PCLK_EN : GPIOG_PCLK_DN;
	}
	else if(pGPIOx == GPIOH)
	{
		status == ENABLE ? GPIOH_PCLK_EN : GPIOH_PCLK_DN;
	}
	else if(pGPIOx == GPIOI)
	{
		status == ENABLE ? GPIOI_PCLK_EN : GPIOI_PCLK_DN;
	}
}

void GPIO_WriteToPin(GPIO_RegDef_t* pGPIOx, uint32_t pin, uint8_t value)
{
	if(value == GPIO_SET)
	{
		pGPIOx->ODR |= (1 << pin);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pin);
	}
}


uint8_t GPIO_ReadFromPin(GPIO_RegDef_t* pGPIOx, uint32_t pin)
{
	uint8_t value;

	/* Read the least significant bit of the register */
	value = (uint8_t)( ( pGPIOx->IDR >> pin) & 0x00000001 );
	return value;
}

uint32_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx)
{
	uint32_t value;

	value = pGPIOx->IDR;
	return value;
}
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint32_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ISR(const uint8_t pin_number)
{
	/* Clear pending register on EXTI line (set it to 1) */
	EXTI->PR |= (1 << pin_number);
}









