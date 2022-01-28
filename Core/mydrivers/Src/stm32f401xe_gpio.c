/*
 * stm32f401xe_GPIO.c
 *
 *  Created on: Jan 12, 2022
 *      Author: ROJEK
 */

#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"

/*
 * Start clock for GPIO
 *
 * @param[*GPIO] - base address of gpiox peripheral
 * @return - void
 */
static void GPIO_ClockEnable(GPIO_TypeDef *GPIO)
{
	if (GPIO == GPIOA)
	{
		RCC_CLOCK_GPIOA_ENABLE();
	}
	else if (GPIO == GPIOB)
	{
		RCC_CLOCK_GPIOB_ENABLE();
	}
	else if (GPIO == GPIOC)
	{
		RCC_CLOCK_GPIOC_ENABLE();
	}
	else if (GPIO == GPIOD)
	{
		RCC_CLOCK_GPIOD_ENABLE();
	}
	else if (GPIO == GPIOE)
	{
		RCC_CLOCK_GPIOE_ENABLE();
	}
	else if (GPIO == GPIOH)
	{
		RCC_CLOCK_GPIOH_ENABLE();
	}

	//this operation is unnecessary here because configuration library is taking more than 2 clock cycles
	//between clock enable and configuring register, i leave it here to remind myself that stmf401x has
	//a limitation that is described in errata point 2.1.6
	__DSB();
}
/*
 * Initialize GPIO pin
 *
 * @param[*hGPIO] - handler to GPIO structure that contains peripheral base address and pin configuration
 * @return - void
 */
void GPIO_InitPin(GPIO_Handle_t *hGPIO)
{

	//enable clock
	GPIO_ClockEnable(hGPIO->pGPIOx);

	// mode selection
	if (hGPIO->PinConfig.Mode < GPIO_PIN_MODE_EXTI_FT)
	{
		// non IRQ mode
	hGPIO->pGPIOx->MODER &= ~(0b11 << (hGPIO->PinConfig.PinNumber * 2));
	hGPIO->pGPIOx->MODER |= hGPIO->PinConfig.Mode << (hGPIO->PinConfig.PinNumber * 2);
	}
	else
	{
		//IRQ mode

		//set as input
		hGPIO->pGPIOx->MODER &= ~(0b11 << (hGPIO->PinConfig.PinNumber * 2));
		//interrupt mask
		EXTI->IMR |= (0b1 << hGPIO->PinConfig.PinNumber);

		//rising/falling trigger
		if ((hGPIO->PinConfig.Mode == GPIO_PIN_MODE_EXTI_FT) || (hGPIO->PinConfig.Mode == GPIO_PIN_MODE_EXTI_FTRT))
		{
			EXTI->FTSR |= (0b1 << hGPIO->PinConfig.PinNumber);
		}

		if ((hGPIO->PinConfig.Mode == GPIO_PIN_MODE_EXTI_RT) || (hGPIO->PinConfig.Mode == GPIO_PIN_MODE_EXTI_FTRT))
		{
			EXTI->RTSR |= (0b1 << hGPIO->PinConfig.PinNumber);
		}

		//enable NVIC interrupt
		if (hGPIO->PinConfig.PinNumber < GPIO_PIN_5)
		{
			// positions for EXTI interrupts in NVIC vector table are 6-10
			NVIC->ISER[0] |= (0b1 << (hGPIO->PinConfig.PinNumber + 6));
		}
		else if (hGPIO->PinConfig.PinNumber < GPIO_PIN_10)
		{
			// position for EXTI9_5 is 23
			NVIC->ISER[0] |= (0b1 << 23);
		}
		else
		{
			// position for EXTI15_10 is 40
			NVIC->ISER[1] |= (0b1 << 8);
		}

		// set SYSCFG for external IRQ
		// enable clock
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		// get 4 bits code for certain port
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(hGPIO->pGPIOx);
		// put it in syscfg register
		SYSCFG->EXTICR[hGPIO->PinConfig.PinNumber / 4] |= (portcode << ((hGPIO->PinConfig.PinNumber % 4) * 4));
	}

	// set speed and output type for mode output or AF
	if ((hGPIO->PinConfig.Mode == GPIO_PIN_MODE_OUTPUT) || (hGPIO->PinConfig.Mode == GPIO_PIN_MODE_AF))
	{
		//speed selection
		hGPIO->pGPIOx->OSPEEDR &= ~(0b11 << (hGPIO->PinConfig.PinNumber * 2));
		hGPIO->pGPIOx->OSPEEDR |= hGPIO->PinConfig.OutputSpeed << (hGPIO->PinConfig.PinNumber * 2);

		//output type selection
		hGPIO->pGPIOx->OTYPER &= ~(0b1 << (hGPIO->PinConfig.PinNumber));
		hGPIO->pGPIOx->OTYPER |= hGPIO->PinConfig.OutputType << (hGPIO->PinConfig.PinNumber);
	}

	// set alternate function bits
	if (hGPIO->PinConfig.Mode == GPIO_PIN_MODE_AF)
	{
		// clear 4 AF bits and set new value
		hGPIO->pGPIOx->AFR[(hGPIO->PinConfig.PinNumber) / 8] &= ~(15UL << ((hGPIO->PinConfig.PinNumber % 8) * 4));
		hGPIO->pGPIOx->AFR[(hGPIO->PinConfig.PinNumber) / 8] |= (hGPIO->PinConfig.AF << ((hGPIO->PinConfig.PinNumber % 8) * 4));
	}

	// pull ups pull downs
	hGPIO->pGPIOx->PUPDR &= ~(0b11 << (hGPIO->PinConfig.PinNumber * 2));
	hGPIO->pGPIOx->PUPDR |= (hGPIO->PinConfig.PullUpPullDown << (hGPIO->PinConfig.PinNumber * 2));

}

/*
 * Write GPIO pin
 *
 * @param[*GPIO] - base address of gpiox peripheral
 * @param[PinNumber] - GPIO_PIN_x @PinNumber
 * @param[PinState]- GPIO_PIN_RESET/GPIO_PIN_SET @GPIOPinState
 * @return - void
 */
void GPIO_WritePin(GPIO_TypeDef *GPIO, uint8_t PinNumber, uint8_t PinState)
{
	GPIO->ODR &= ~(0b1 << PinNumber);
	GPIO->ODR |= PinState << PinNumber;
}

/*
 * Toggle GPIO pin
 *
 * @param[*GPIO] - base address of gpiox peripheral
 * @param[PinNumber] - GPIO_PIN_x @PinNumber
 * @return - void
 */
void GPIO_TogglePin(GPIO_TypeDef *GPIO, uint8_t PinNumber)
{
	GPIO->ODR ^= 0b1 << PinNumber;
}

/*
 * Clear pending flag
 *
 * @param[PinNumber] - GPIO_PIN_x @PinNumber
 * @return - void
 */
void GPIO_ClearPendingEXTIFlag(uint8_t PinNumber)
{
	EXTI->PR |= (0b1 << PinNumber);
}
