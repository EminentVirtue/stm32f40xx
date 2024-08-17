/*
 * stm32f4xx.h
 *
 *  Created on: May 13, 2024
 *      Author: Andrew Streng
 */


#include <stdint.h>

#ifndef STM32F4XX_H_
#define STM32F4XX_H_

#define ENABLE 1
#define DISABLE 0
#define INVALID 0
#define VALID 	1

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#define BIT(n) (1 << n)
#define BITMASK(end, start) (((1U << ((end) - (start) + 1)) - 1) << (start))
#define EINVAL				1

#define fbs(mask)(__builtin_ffs(mask) - 1)											/* First bit set */
#define FIELD_PREP(MASK, VALUE) ((typeof(MASK))(VALUE) << fbs(MASK)) & (MASK)		/* Shift value to first bit set in mask */

enum byte_def
{
	Byte = 8,
	HalfWord = 16,
	Word = 32
};

#define NOT_NULL(expr) expr != NULL
#define RISING_EDGE			1
#define FALLING_EDGE		2

/* Base addresses for various buses in the stm32f4xx MCU*/
#define APB1_BASE_ADDR 0x40000000U
#define APB2_BASE_ADDR 0x40010000U
#define AHB1_BASE_ADDR 0x40020000U
#define AHB2_BASE_ADDR 0x50000000U

#define APB1_PERIPHERAL		1
#define APB2_PERIPHERAL		2

#define __weak __attribute__((weak))

/* Bus speeds (in MHz) */
#define APB2_MAX_SPEED 84
#define APB1_MAX_SPEED 42
#define AHB1_MAX_SPEED 168
#define AHB2_MAX_SPEED AHB1_MAX_SPEED
#define SYSCLK_MAX_SPEED 	168

/* Base addresses for flash items */
#define SRAM1_SIZE 1C000
#define SRAM1_BASE_ADDR 0x2000
#define SRAM2_BASE_ADDR ( SRAM1_BASE_ADDR + SRAM1_SIZE )
#define FLASH_BASE_ADDR 0x0800

#define APB2_PRESCALER_READ			1
#define APB1_PRESCALER_READ			0

/* NVIC Macros */
#define NVIC_ISER0		((volatile uint32_t*)0xE000E100)		/* Interrupt Set-Enable Register 1 */
#define NVIC_ISER1		((volatile uint32_t*)0xE000E104)		/* Interrupt Set-Enable Register 2 */
#define NVIC_ISER2		((volatile uint32_t*)0xE000E108)		/* Interrupt Set-Enable Register 3 */
#define NVIC_ISER3		((volatile uint32_t*)0xE000E10c)		/* Interrupt Set-Enable Register 4 */
#define NVIC_ICER0		((volatile uint32_t*)0xE000E180)		/* Interrupt Clear-Enable Register 1 */
#define NVIC_ICER1		((volatile uint32_t*)0xE000E184)		/* Interrupt Clear-Enable Register1 */
#define NVIC_ICER2		((volatile uint32_t*)0xE000E188)		/* Interrupt Clear-Enable Register1 */
#define NVIC_ICER3		((volatile uint32_t*)0xE000E18c)		/* Interrupt Clear-Enable Register1 */
#define NVIC_ISPR		((volatile uint32_t*)0xE000E200)		/* Interrupt Set-Pending Registers */
#define NVIC_ICPR		((volatile uint32_t*)0xE000E280)		/* Interrupt Clear-Pending Registers */
#define NVIC_IABR		((volatile uint32_t*)0xE000E300)		/* Interrupt Active Bit Register (RO) */
#define NVIC_IPR		((volatile uint32_t*)0xE000E400)		/* Interrupt Priority Register */

#define NVIC_ENABLE_IRQ		ENABLE
#define NVIC_DISABLE_IRQ	DISABLE

/* STM32F405xx vector table */
#define EXTI0_IRQ_NUM	6
#define EXTI1_IRQ_NUM	7
#define EXTI2_IRQ_NUM	8
#define EXTI3_IRQ_NUM	9
#define EXTI4_IRQ_NUM	10
#define EXTI9_5_IRQ_NUM 23 		/* This IRQ number must be used for interrupts which are on EXTI lines 5-9 - do not use EXTI_2 IRQ number */

/* Register maps of the various peripherals  */

/* Register Map of the RCC Peripheral */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint16_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR; //68
	uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED5;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
}RCC_RegDef_t;

#define RCC_ADDR_OFFSET 0x3800
#define RCC_BASE_ADDR ( AHB1_BASE_ADDR + RCC_ADDR_OFFSET )
#define RCC ( (RCC_RegDef_t*)RCC_BASE_ADDR )

typedef enum {
	PrescalerDiv2,
	PrescalerDiv4,
	PrescalerDiv8,
	PrescalerDiv16,
	PrescalerNone
}PrescalerSelection_t;

typedef enum {
	SourceHSI = 0x0,
	SourceHSE,
	SourcePLL,
	SourceNotAllowed
}SYSCLK_Source_t;

/* MCO1 clock source selection macros */
#define MCO1_HSI		0
#define MCO1_LSE		1
#define MCO1_HSE		2
#define MCO1_PLL		3

/* MCO2 clock source selection macros */
#define MCO2_SYSCLK		0
#define MCO2_PLLI2S		1
#define MCO2_HSE		2
#define MCO2_PLL		3

/* RCC Configuration Register (RCC_CFGR) bit definitions */
#define CFGR_MCO2		31
#define CFGR_MCO1		21

#define RCC_MCO1_SOURCE(source)  (RCC->CFGR |= ( source << CFGR_MCO1) )
#define RCC_MCO2_SOURCE(source)  (RCC->CFGR |= (source & CFGR_MCO2) )



/* RCC Bit Definitions */
#define BIT_SYSCFGEN	14

typedef struct {
	volatile uint32_t MODER;  	/* GPIO port mode register */
	volatile uint32_t OTYPER; 	/* GPIO port output type register */
	volatile uint32_t OSPEEDR; 	/* GPIO port output speed register */
	volatile uint32_t PUPDR; 	/* GPIO port pull-up/pull-down register */
	volatile uint32_t IDR;		/* GPIO port input data register */
	volatile uint32_t ODR;		/* GPIO port output data register */
	volatile uint32_t BSSR; 	/* GPIO port bit set/reset register */
	volatile uint32_t LCKR; 	/* GPIO port configuration lock register */
	volatile uint32_t AFRL; 	/* GPIO port alternate function low register pins 0-7 */
	volatile uint32_t AFRH; 	/* GPIO port alternate function high register pins 8-15*/

}GPIO_RegDef_t;

typedef struct {
	volatile uint32_t MACCR;		/* MAC configuration register */
	volatile uint32_t MACFFR;		/* MAC frame filter register */
	volatile uint32_t MACHTHR;		/* MAC hash table high register */
	volatile uint32_t MACHTLR;		/* MAC hash table low register */
	volatile uint32_t MACMIIAR;		/* MAC MII address register */
	volatile uint32_t MACMIIDR;		/* MAC MII data register */
	volatile uint32_t MACFCR;		/* MAC flow control register */
	volatile uint32_t MACVLANTR;	/* MAC VLAN tag register */
	volatile uint32_t MACRWUFFR;	/* MAC remove wake-up frame filter register */
	volatile uint32_t MACPMTCSR;	/* MAC PMT control and status register */
	volatile uint32_t MACDBGR;		/* MAC debug register */
	volatile uint32_t MACSR;		/* MAC interrupt status register */
	volatile uint32_t MACIMR;		/* MAC interrupt mask register */
	volatile uint32_t MACA0HR;		/* MAC address 0 high register */
	volatile uint32_t MACA0LR;		/* MAC address 0 low register */
	volatile uint32_t MACA1HR;		/* MAC address 1 high register */
	volatile uint32_t MACA1LR;		/* MAC address 1 low register */
	volatile uint32_t MACA2HR;		/* MAC address 2 high register */
	volatile uint32_t MACA2LR;		/* MAC address 2 low register */
	volatile uint32_t MACA3HR;		/* MAC address 3 high register */
	volatile uint32_t MACA3LR;		/* MAC address 3 low register */
	volatile uint32_t MMCCR;		/* MMC control register */
	volatile uint32_t MMCRIR;		/* MMC receive interrupt register */
	volatile uint32_t MMCTIR;		/* MMC transmit interrupt register */
	volatile uint32_t MMCRIMR;		/* MMC receive interrupt mask register */

}ETH_RegDef_t;

/* RTC Register Map */
typedef struct {
	uint32_t TR; 				/* Time register */
    uint32_t DR; 				/* Date register */
	volatile uint32_t CR; 		/* Control register */
	volatile uint32_t ISR; 		/* Initialization and status register */
	volatile uint32_t PRER; 	/* Prescaler register */
	volatile uint32_t WUTR; 	/* Wake-up register */
	volatile uint32_t CALIBR; 	/* Calibration register */
	uint32_t ALRMAR;			/* Alarm A register */
	uint32_t ALRMBR; 			/* Alarm B register */
	uint32_t WPR;				/* Write protection register */
	volatile uint32_t SSR; 		/* Sub second register */
	volatile uint32_t SHIFTR; 	/* Shift control register */
	volatile uint32_t TSTR; 	/* Time stamp time register */
	volatile uint32_t TSDR; 	/* Time stamp date register */
	volatile uint32_t TSSSR; 	/* Time stamp sub second register */
	volatile uint32_t CALR; 	/* Calibration register */
	volatile uint32_t TAFCR; 	/* Tamper and alternate function configuration register */
	uint32_t ALRMASSR; 			/* Alarm A sub second register */
	uint32_t ALRMBSSR; 			/* Alarm B sub second register */
	volatile uint32_t BKP0R; 	/* Backup registers */
}RTC_RegDef_t;

/* Register Map of the EXTI peripheral */
typedef struct {
	volatile uint32_t IMR;		/* Interrupt mask register */
	volatile uint32_t EMR;		/* Event mask register */
	volatile uint32_t RTSR;		/* Rising trigger selection register */
	volatile uint32_t FTSR;		/* Falling trigger selection register */
	volatile uint32_t SWIER;	/* Software interrupt event register */
	volatile uint32_t PR;		/* Pending register */

}EXTI_RegDef_t;


/* Register Map of the SYSCFG peripheral */
typedef struct {
	uint32_t MEMRMP;
	uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;
}SYSCFG_RegDef_t;

/* SYSCFG Clock Enable/Disable Macros */
#define SYSCFG_CLK_EN ( RCC->APB2ENR |= ( 1 << BIT_SYSCFGEN) )
#define SYSCFG_CLK_DN ( RCC->APB2ENR &= ~( 1 << BIT_SYSCFGEN) )

#define EXTI_REG_NUM 4

/* Register map of the SPI peripherals */
typedef struct {
	volatile uint32_t CR1;			/* SPI control register 1 */
	volatile uint32_t CR2;			/* SPI control register 2 */
	volatile uint32_t SR;			/* SPI status register */
	volatile uint32_t DR;			/* SPI data register */
	volatile uint32_t CRCPR;		/* SPI polynomial register */
	volatile uint32_t RXCRCR;		/* SPI RX CRC register */
	volatile uint32_t TXCRCR;		/* SPI TX CRC register */
	volatile uint32_t I2SCFGR;		/* SPI I2S configuration register */
	volatile uint32_t I2SPR;		/* SPI prescaler register */
}SPI_RegDef_t;

/* AHB1 Peripherals */

/* GPIO */
#define GPIOA_BASE_ADDR (AHB1_BASE_ADDR + 0x000)
#define GPIOB_BASE_ADDR (AHB1_BASE_ADDR + 0x400)
#define GPIOC_BASE_ADDR (AHB1_BASE_ADDR	+ 0x800)
#define GPIOD_BASE_ADDR (AHB1_BASE_ADDR + 0xC00)
#define GPIOE_BASE_ADDR (AHB1_BASE_ADDR + 0x100)
#define GPIOF_BASE_ADDR (AHB1_BASE_ADDR + 0x140)
#define GPIOG_BASE_ADDR (AHB1_BASE_ADDR + 0x180)
#define GPIOH_BASE_ADDR (AHB1_BASE_ADDR + 0x1C0)
#define GPIOI_BASE_ADDR (AHB1_BASE_ADDR + 0x200)
#define GPIOJ_BASE_ADDR (AHB1_BASE_ADDR + 0x240)
#define GPIOK_BASE_ADDR (AHB1_BASE_ADDR + 0x280)

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASE_ADDR)
#define GPIOJ ((GPIO_RegDef_t*)GPIOJ_BASE_ADDR)
#define GPIOK ((GPIO_RegDef_t*)GPIOK_BASE_ADDR)

/* GPIO Peripheral Clock Enable Macros */
#define GPIOA_PCLK_EN 	(RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN 	(RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN	(RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN 	(RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN	(RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN	(RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN	(RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN	(RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN	(RCC->AHB1ENR |= ( 1 << 8 ) )

/* GPIO Peripheral Clock Disable Macros */
#define GPIOA_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DN	(RCC->AHB1ENR &= ~(1 << 7) )

/* GPIO Reset Macros */
#define GPIOA_RESET	do{RCC->AHB1RSTR |= (0 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOB_RESET	do{RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOC_RESET	do{RCC->AHB1RSTR |= (2 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOD_RESET	do{RCC->AHB1RSTR |= (3 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOE_RESET	do{RCC->AHB1RSTR |= (4 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOF_RESET	do{RCC->AHB1RSTR |= (5 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOG_RESET	do{RCC->AHB1RSTR |= (6 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOH_RESET	do{RCC->AHB1RSTR |= (7 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)
#define GPIOI_RESET	do{RCC->AHB1RSTR |= (8 << 0); RCC->AHB1RSTR &= ~ (1 << 0);}while(0)


/* Miscellaneous GPIO Macros */
#define GPIO_SET 			ENABLE
#define GPIO_UNSET 			DISABLE
#define IRQ_FT_ENABLED 		1		/* Falling trigger interrupt enabled */
#define IRQ_RT_ENABLED		2		/* Rising trigger interrupt enabled */
#define GPIO_PIN_MAX		16		/* The max pin number of the GPIO pad */

/* @SPI Peripherals
 * SPI peripherals and clock enable/disable macros and reset macros */

#define SPI1 ((SPI_RegDef_t*)(APB2_BASE_ADDR + 0x3000))		/* SPI1 */
#define SPI2 ((SPI_RegDef_t*)(APB1_BASE_ADDR +0x3800))		/* SPI2 */
#define SPI3 ((SPI_RegDef_t*)(APB1_BASE_ADDR + 0x3C00))		/* SPI3 */
#define SPI4 ((SPI_RegDef_t*)(APB2_BASE_ADDR + 0x3400))		/* SPI4 */
#define SPI5 ((SPI_RegDef_t*)(APB2_BASE_ADDR + 0x5000))		/* SPI5 */
#define SPI6 ((SPI_RegDef_t*)(APB2_BASE_ADDR + 0x5400))		/* SPI6 */

/* SPI reset macros */
#define SPI1_RESET		do { RCC->APB2RSTR |= (1 << 14); RCC->APB2RSTR &= ~(1 << 15);}while(0)
#define SPI2_RESET 		do { RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 15);}while(0)
#define SPI3_RESET 		do { RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15);}while(0)
#define SPI4_RESET 		do { RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13);}while(0)
#define SPI5_RESET 		do { RCC->APB2RSTR |= (1 << 20); RCC->APB2RSTR &= ~(1 << 20);}while(0)
#define SPI6_RESET 		do { RCC->APB2RSTR |= (1 << 21); RCC->APB2RSTR &= ~(1 << 21);}while(0)

/* SPI clock enable macros */
#define SPI1_PCLK_EN		(RCC->APB2ENR |= 1 << 12)
#define SPI2_PCLK_EN		(RCC->APB1ENR |= 1 << 14)
#define SPI3_PCLK_EN		(RCC->APB1ENR |= 1 << 15)
#define SPI4_PCLK_EN		(RCC->APB2ENR |= 1 << 13)
#define SPI5_PCLK_EN		(RCC->APB2ENR |= 1 << 20)
#define SPI6_PCLK_EN		(RCC->APB2ENR |= 1 << 21)

/* SPI clock disable macros */
#define SPI1_PCLK_DN		(RCC->APB2ENR &= ~1 << 12)
#define SPI2_PCLK_DN		(RCC->APB1ENR &= ~1 << 14)
#define SPI3_PCLK_DN		(RCC->APB1ENR &= ~1 << 15)
#define SPI4_PCLK_DN		(RCC->APB2ENR &= ~1 << 13)
#define SPI5_PCLK_DN		(RCC->APB2ENR &= ~1 << 20)
#define SPI6_PCLK_DN		(RCC->APB2ENR &= ~1 << 21)

/* SPI IRQ Macros */
#define SPI1IRQ_NO		35
#define SPI2IRQ_NO		36
#define SPI3IRQ_NO		51
#define SPI4IRQ_NO		84
#define SPI5IRQ_NO		85
#define SPI6IRQ_NO		86


/* Flash Interface Register */
#define FLASH_IR_ADDR_OFFSET 0x3C00
#define FLASH_IR (AHB1_BASE_ADDR + FLASH_IR_ADDR_OFFSET)

/* BKPSRAM */
#define BKPSRAM_ADDR_OFFSET 0x4000
#define BKPSRAM (AHB1_BASE_ADDR + BKPSRAM_ADDR_OFFSET )

/* DMA1/DMA2 */
#define DMA1_ADDR_OFFSET 0x6000
#define DMA2_ADDR_OFFSET 0x6400

#define DMA1 (AHB1_BASE_ADDR + DMA1_ADDR_OFFSET)
#define DMA2 (AHB1_BASE_ADDR + DMA2_ADDR_OFFSET)

/* Ethernet MAC */
#define ETH_BASE_ADDR ( AHB1_BASE_ADDR + 0x8000 )
#define ETH_MAC ((ETH_RegDef_t*)ETH_BASE_ADDR)

/* DMA2D */
#define DMA2D_ADDR_OFFSET 0xB000
#define DMA2D (AHB1_BASE_ADDR + DMA2D_ADDR_OFFSET)

/* USB OTG HS */
#define OTG_HS_ADDR_OFFSET
#define USB_OTG_HS (AHB1_BASE_ADDR + OTG_HS_ADDR_OFFSET)

/* AHB2 Peripherals */

/* Digital Camera Interface (DCMI) */
#define DCMI_ADDR_OFFSET
#define DCMI (AHB2_BASE_ADDR + DCMI_ADDR_OFFSET)

/* Cryptographic Processor (CRYP) */
#define CRYP_ADDR_OFFSET

/* Hash Processor (HASH) */

/* Random Number Generator (RNG) */

/* APB1 Peripherals */

#define APB1_PERIPH(offset) (APB1_BASE_ADDR + offset)

/* General-Purpose Timer Register Map */
typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	uint32_t RESERVED;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	uint32_t RESERVED2;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t TIM2_OR;
	volatile uint32_t TIM5_OR;
}GPTimer_RegDef_t;

/* Timers */
#define TIM2_BASE_ADDR 		(APB1_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR 		(APB1_BASE_ADDR + 0X400)
#define TIM4_BASE_ADDR 		(APB1_BASE_ADDR + 0x800)
#define TIM5_BASE_ADDR 		(APB1_BASE_ADDR + 0xC00)
#define TIM6_BASE_ADDR 		(APB1_BASE_ADDR + 0x1000)
#define TIM7_BASE_ADDR 		(APB1_BASE_ADDR + 0x1400)
#define TIM12_BASE_ADDR 	(APB1_BASE_ADDR + 0x1800)
#define TIM13_BASE_ADDR 	(APB1_BASE_ADDR + 0x1C00)
#define TIM14_BASE_ADDR 	(APB1_BASE_ADDR + 0x2000)

#define TIMER(offset) (APB1_BASE_ADDR + offset)

#define TIMER2 		((GPTimer_RegDef_t*)TIM2_BASE_ADDR)
#define TIMER3 		(TIM3_BASE_ADDR)
#define TIMER4 		(TIM4_BASE_ADDR)
#define TIMER5 		((GPTimer_RegDeft_t*)TIM5_BASE_ADDR)
#define TIMER6 		(TIM6_BASE_ADDR)
#define TIMER7 		(TIM7_BASE_ADDR)
#define TIMER12 	(TIM12_BASE_ADDR)
#define TIMER13 	(TIM13_BASE_ADDR)
#define TIMER14 	(TIM14_BASE_ADDR)

/* Timer Enable Macros */
#define TIM2_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 0))
#define TIM3_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 1))
#define TIM4_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 2))
#define TIM5_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 3))
#define TIM6_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 4))
#define TIM7_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 5))
#define TIM12_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 6))
#define TIM13_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 7))
#define TIM14_PCLK_EN		(RCC->APB1ENR |= (ENABLE << 8))

#define TIM2_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 0))
#define TIM3_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 1))
#define TIM4_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 2))
#define TIM5_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 3))
#define TIM6_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 4))
#define TIM7_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 5))
#define TIM12_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 6))
#define TIM13_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 7))
#define TIM14_PCLK_DN		(RCC->APB1ENR &= ~(ENABLE << 8))

/* Timer Rest Macros */
#define TIM2_RESET		do { RCC->APB1RSTR |= (ENABLE << 0); RCC->APB1RSTR &= ~(ENABLE << 0);}while(0)
#define TIM3_RESET		do { RCC->APB1RSTR |= (ENABLE << 1); RCC->APB1RSTR &= ~(ENABLE << 1);}while(0)
#define TIM4_RESET		do { RCC->APB1RSTR |= (ENABLE << 2); RCC->APB1RSTR &= ~(ENABLE << 2);}while(0)
#define TIM5_RESET		do { RCC->APB1RSTR |= (ENABLE << 3); RCC->APB1RSTR &= ~(ENABLE << 3);}while(0)
#define TIM6_RESET		do { RCC->APB1RSTR |= (ENABLE << 4); RCC->APB1RSTR &= ~(ENABLE << 4);}while(0)
#define TIM7_RESET		do { RCC->APB1RSTR |= (ENABLE << 5); RCC->APB1RSTR &= ~(ENABLE << 5);}while(0)
#define TIM12_RESET		do { RCC->APB1RSTR |= (ENABLE << 6); RCC->APB1RSTR &= ~(ENABLE << 6);}while(0)
#define TIM13_RESET		do { RCC->APB1RSTR |= (ENABLE << 7); RCC->APB1RSTR &= ~(ENABLE << 7);}while(0)
#define TIM14_RESET		do { RCC->APB1RSTR |= (ENABLE << 8); RCC->APB1RSTR &= ~(ENABLE << 8);}while(0)

typedef struct {
	volatile u32 LISR;
	volatile u32 HISR;
	volatile u32 LIFCR;
	volatile u32 HIFCR;
	volatile u32 S0CR;
	volatile u32 S0NDTR;
	volatile u32 S0PAR;
	volatile u32 S0M0AR;
	volatile u32 S0M1AR;
	volatile u32 S0FCR;
	volatile u32 S1CR;
	volatile u32 S1NDTR;
	volatile u32 S1PAR;
	volatile u32 S1M0AR;
	volatile u32 S1M1AR;
	volatile u32 S1FCR;
	volatile u32 S2CR;
	volatile u32 S2NDTR;
	volatile u32 S2PAR;
	volatile u32 S2M0AR;
	volatile u32 S2M1AR;
	volatile u32 S2FCR;
	volatile u32 S3CR;
	volatile u32 S3NDTR;
	volatile u32 S3PAR;
	volatile u32 S3M0AR;
	volatile u32 S3M1AR;
	volatile u32 S3FCR;
	volatile u32 S4CR;
	volatile u32 S4NDTR;
	volatile u32 S4PAR;
	volatile u32 S4M0AR;
	volatile u32 S4M1AR;
	volatile u32 S4FCR;
	volatile u32 S5CR;
	volatile u32 S5NDTR;
	volatile u32 S5PAR;
	volatile u32 S5M0AR;
	volatile u32 S5M1AR;
	volatile u32 S5FCR;
	volatile u32 S6CR;
	volatile u32 S6NDTR;
	volatile u32 S6PAR;
	volatile u32 S6M0AR;
	volatile u32 S6M1AR;
	volatile u32 S6FCR;
	volatile u32 S7CR;
	volatile u32 S7NDTR;
	volatile u32 S7PAR;
	volatile u32 S7M0AR;
	volatile u32 S7M1AR;
	volatile u32 S7FCR;

}DMA_RegDef_t;


#define DMA1 ((DMA_RegDef_t*)(AHB1_BASE_ADDR + 0x6000))
#define DMA2 ((DMA_RegDef_t*)(AHB1_BASE_ADDR + 0x6400))

#define DMA1_CLK_EN		(RCC->AHB1ENR |= (1 << 21))
#define DMA2_CLK_EN		(RCC->AHB1ENR |= (1 << 22))

#define BIT_DMA1_CLK	(1 << 21)
#define BIT_DMA2_CLK	(1 << 22)

#define DMA1_CLK_DN		(RCC->AHB1ENR &= ~(1 << 21))
#define DMA2_CLK_DN		(RCC->AHB2ENR &= ~(1 << 22))

#define RTC ( (RTC_RegDef_t*)RTC_BASE_ADDR)

/* Window Watchdog (WWDG) */
#define WWDG_ADDR_OFFSET 0x2C00
#define WWDG APB1_PERIPH(WWDG_ADDR_OFFSET)

/* Independent Watchdog (IWDG) */
#define IWDG_ADDR_OFFSET 0x3000
#define IWDG APB1_PERIPH(IWDG_ADDR_OFFSET)

/* I2S2ext */
#define I2S2EXT_ADDR_OFFSET 0x3400
#define I2S2ext APB1_PERIPH(I2S2EXT_ADDR_OFFSET)

/* I2S3ext */
#define I2S3ext_ADDR_OFFSET 0x4000
#define I2S3ext APB1_PERIPH(I2S3ext_ADDR_OFFSET)

/* USART2 */
#define USART2_ADDR_OFFSET 0x4400
#define USART2 APB1_PERIPH(USART2_ADDR_OFFSET)

/* USART3 */
#define USART3_ADDR_OFFSET 0x4800
#define USART3 APB1_PERIPH(USART3_ADDR_OFFSET)

/* UART4 */
#define UART4_ADDR_OFFSET 0x4C00
#define UART4 APB1_PERIPH(UART4_ADDR_OFFSET)

/* UART5 */
#define UART5_ADDR_OFFSET 0x5000
#define UART5 APB1_PERIPH(UART5_ADDR_OFFSET)

/* I2C1 */
#define I2C1_ADDR_OFFSET 0x5400
#define I2C1 APB1_PERIPH(I2C1_ADDR_OFFSET)

/* I2C2 */
#define I2C2_ADDR_OFFSET 0x5800
#define I2C2 APB1_PERIPH(I2C2_ADDR_OFFSET)

/* I2C3 */
#define I2C3_ADDR_OFFSET 0x5800
#define I2C3 APB1_PERIPH(I2C3_ADDR_OFFSET)

/* CAN1 */
#define CAN1_ADDR_OFFSET 0x6400
#define CAN1 APB1_PERIPH(CAN1_ADDR_OFFSET)

/* CAN2 */
#define CAN2_ADDR_OFFSET 0x6800
#define CAN2 APB1_PERIPH(CAN2_ADDR_OFFSET)

/* PWR */
#define PWR_ADDR_OFFSET 0x7000
#define PWR APB1_PERIPH(PWR_ADDR_OFFSET)

/* DAC */
#define DAC_ADDR_OFFSET 0x7400
#define DAC APB1_PERIPH(DAC_ADDR_OFFSET)

/* UART7 */
#define UART7_ADDR_OFFSET 0x7800
#define UART7 APB1_PERIPH(UART7_ADDR_OFFSET)

/* UART8 */
#define UART8_ADDR_OFFSET 0x7C00
#define UART8 APB1_PERIPH(UART8_ADDR_OFFSET)

/* APB2 Peripherals */

/* TIM1 */
#define TIM1 (APB2_BASE_ADDR + 0x0000)

/* TIM8 */
#define TIM8 (APB2_BASE_ADDR + 0x400)

/* USART1 */
#define USART1 (APB2_BASE_ADDR + 0x1000)

/* UART6 */
#define UART6 (APB2_BASE_ADDR + 0x1400)

/* ADC */
#define ADC  (APB2_BASE_ADDR + 0x2000)

/* SDIO */
#define SDIO (APB2_BASE_ADDR + 0x2C00)



/* SYSCFG */
#define SYSCFG ( (SYSCFG_RegDef_t*)(APB2_BASE_ADDR + 0x3800))

/* EXTI */
#define EXTI  ( (EXTI_RegDef_t*) (APB2_BASE_ADDR + 0x3C00) )

/* TIM9 */
#define TIM9  (APB2_BASE_ADDR + 0x4000)

/* TIM10 */
#define TIM10 (APB2_BASE_ADDR + 0x4400)

/* TIM11 */
#define TIM11 (APB2_BASE_ADDR + 0x4800)


/* SAI1 */
#define SAI1  (APB2_BASE_ADDR + 0x5800)

/* LCD-TFT */
#define LCD_TFT (APB2_BASE_ADDR + 0x6800)

#endif /* STM32F4XX_H_ */
