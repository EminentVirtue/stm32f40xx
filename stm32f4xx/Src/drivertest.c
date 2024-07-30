/*
 * wtf.c
 *
 *  Created on: May 22, 2024
 *      Author: Andrew
 */


#include "gpio.h"
#include <stdbool.h>
#include <string.h>

void toggle_leds(uint8_t value)
{
	if(value == 1 )
	{
		GPIO_WriteToPin(GPIOD, GPIO_PIN_TWELVE, GPIO_SET);
		GPIO_WriteToPin(GPIOD, GPIO_PIN_FIFTEEN, GPIO_SET);
		GPIO_WriteToPin(GPIOC, GPIO_PIN_SEVEN, GPIO_SET);

	}
	else
	{
		GPIO_WriteToPin(GPIOD, GPIO_PIN_TWELVE, GPIO_UNSET);
		GPIO_WriteToPin(GPIOD, GPIO_PIN_FIFTEEN, GPIO_UNSET);
		GPIO_WriteToPin(GPIOC, GPIO_PIN_SEVEN, GPIO_UNSET);
	}
}

int main(void)
{
	GPIO_Handle_t handle;
	memset(&handle, 0, sizeof(handle));

	handle.config.output_speed = GPIO_SPEED_LOW;
	handle.config.PUPD = GPIO_PU;
	handle.config.mode_type = GPIO_MODE_INPUT;
	//handle.config.output_type = GPIO_PUSH_PULL;
	handle.config.pin_number = GPIO_PIN_FIVE;

	handle.pGPIOPad = GPIOB;
	GPIO_Init(&handle);

	GPIO_IRQConfig(&handle, EXTI9_5_IRQ_NUM, ENABLE);

	/* Board LEDs */
	GPIO_Handle_t LD4_handle;
	memset(&handle, 0, sizeof(LD4_handle));

	LD4_handle.config.output_speed = GPIO_SPEED_LOW;
	LD4_handle.config.PUPD = GPIO_NPUPD;
	LD4_handle.config.mode_type = GPIO_MODE_OUTPUT;
	LD4_handle.config.output_type = GPIO_PUSH_PULL;
	LD4_handle.config.pin_number = GPIO_PIN_TWELVE;
	LD4_handle.pGPIOPad = GPIOD;

	GPIO_Init(&LD4_handle);

	GPIO_Handle_t LD3_handle;
	LD3_handle.config.output_speed = GPIO_SPEED_LOW;
	LD3_handle.config.PUPD = GPIO_NPUPD;
	LD3_handle.config.mode_type = GPIO_MODE_OUTPUT;
	LD3_handle.config.output_type = GPIO_PUSH_PULL;
	LD3_handle.config.pin_number = GPIO_PIN_THIRTEEN;
	LD3_handle.pGPIOPad = GPIOD;

	GPIO_Init(&LD3_handle);

	GPIO_Handle_t LD5_handle;
	LD5_handle.config.output_speed = GPIO_SPEED_LOW;
	LD5_handle.config.PUPD = GPIO_NPUPD;
	LD5_handle.config.mode_type = GPIO_MODE_OUTPUT;
	LD5_handle.config.output_type = GPIO_PUSH_PULL;
	LD5_handle.config.pin_number = GPIO_PIN_FOURTEEN;
	LD5_handle.pGPIOPad = GPIOD;

	GPIO_Init(&LD5_handle);

	GPIO_Handle_t LD6_handle;
	LD6_handle.config.output_speed = GPIO_SPEED_LOW;
	LD6_handle.config.PUPD = GPIO_NPUPD;
	LD6_handle.config.mode_type = GPIO_MODE_OUTPUT;
	LD6_handle.config.output_type = GPIO_PUSH_PULL;
	LD6_handle.config.pin_number = GPIO_PIN_FIFTEEN;
	LD6_handle.pGPIOPad = GPIOD;

	GPIO_Init(&LD6_handle);

	GPIO_Handle_t blue_handle;
	blue_handle.config.output_speed = GPIO_SPEED_HIGH;
	blue_handle.config.PUPD = GPIO_NPUPD;
	blue_handle.config.mode_type = GPIO_MODE_OUTPUT;
	blue_handle.config.output_type = GPIO_PUSH_PULL;
	blue_handle.config.pin_number = GPIO_PIN_SEVEN;
	blue_handle.pGPIOPad = GPIOC;

	GPIO_Init(&blue_handle);

	/* Output HSI signal onto GPIO pin */
	GPIO_Handle_t gpio_clock;
	gpio_clock.config.mode_type = GPIO_MODE_ALTERNATE;
	gpio_clock.config.PUPD = GPIO_NPUPD;
	gpio_clock.config.pin_number = GPIO_PIN_EIGHT;
	gpio_clock.config.alternate_mode_type = GPIO_AF_0;
	gpio_clock.pGPIOPad = GPIOA;

	RCC_MCO1_SOURCE(MCO1_HSI);

	RCC->CFGR |= ( 0x6 << 24);

	GPIO_Init(&gpio_clock);


	while(true)
	{
		//uint8_t value = GPIO_ReadFromPin(GPIOA, GPIO_PIN_FIVE);
		//toggle_leds(value);
	}
}



void EXTI0_IRQHandler(void)
{
	uint32_t x;
	(void)x;

	toggle_leds(1);

}
void EXTI1_IRQHandler(void)
{
	uint32_t x;
	(void)x;
	toggle_leds(1);


}
void EXTI2_IRQHandler(void)
{
	uint32_t x;
	(void)x;
	toggle_leds(1);

}

void EXTI9_5_IRQHandler(void)
{
	uint32_t x;
	(void)x;
	toggle_leds(1);
	GPIO_ISR(GPIO_PIN_FIVE);
}

