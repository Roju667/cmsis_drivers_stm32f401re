/*
 * stm32f401xe_GPIO.c
 *
 *  Created on: Jan 12, 2022
 *      Author: ROJEK
 */

#include "stm32f401xe_gpio.h"

#include "stm32f401xe_rcc.h"

/*
 * Starts clock for GPIO and resets the peripheral
 * @param[*p_GPIOx] - gpiox base address
 * @return - void
 */
void GPIO_InitClock(GPIO_TypeDef *GPIO)
{
	if (GPIO == GPIOA)
	{
		RCC_CLOCK_GPIOA_ENABLE();
		RCC_RESET_GPIOA();
	}
	else if (GPIO == GPIOB)
	{
		RCC_CLOCK_GPIOB_ENABLE();
		RCC_RESET_GPIOB();
	}
	else if (GPIO == GPIOC)
	{
		RCC_CLOCK_GPIOC_ENABLE();
		RCC_RESET_GPIOC();
	}
	else if (GPIO == GPIOD)
	{
		RCC_CLOCK_GPIOD_ENABLE();
		RCC_RESET_GPIOD();
	}
	else if (GPIO == GPIOE)
	{
		RCC_CLOCK_GPIOE_ENABLE();
		RCC_RESET_GPIOE();
	}
	else if (GPIO == GPIOH)
	{
		RCC_CLOCK_GPIOH_ENABLE();
		RCC_RESET_GPIOH();
	}

	// this operation is unnecessary here because configuration library is taking
	// more than 2 clock cycles between clock enable and configuring register, i
	// leave it here to remind myself that stmf401x has a limitation that is
	// described in errata point 2.1.6
	__DSB();
}

/*
 * Init basic gpio parameters -> enable clock , set mode and PUPD
 * for standrad input mode this is enough
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_flags] - (GPIO_FLAG_PINx | GPIO_FLAG_PINy)
 * @param[mode] - input/output/af/analog
 * @param[PUPD] - nopull/pullup/pulldown
 * @return - void
 */
void GPIO_ConfigBasic(GPIO_TypeDef *p_GPIOx, uint16_t pin_flags,
		GpioMode_t mode, GpioPUPD_t PUPD)
{
	for (uint16_t pin_count = 0; pin_count < 16; pin_count++)
	{
		if (pin_flags >> pin_count & 1U)
		{
			// mode
			p_GPIOx->MODER &= ~(0x03U << (pin_count * 2));
			p_GPIOx->MODER |= mode << (pin_count * 2);

			// PUPD
			p_GPIOx->PUPDR &= ~(0x03U << (pin_count * 2));
			p_GPIOx->PUPDR |= (PUPD << (pin_count * 2));
		}
	}

	return;
}

/*
 * Init configuration for output pin
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_flags] - (GPIO_FLAG_PINx | GPIO_FLAG_PINy)
 * @param[output_type] - open drain/pushpull
 * @param[speed] - slow/medium/fast/veryfast
 * @return - void
 */
void GPIO_ConfigOutput(GPIO_TypeDef *p_GPIOx, uint16_t pin_flags,
		GpioOutputType_t output_type, GpioSpeed_t speed)
{
	for (uint16_t pin_count = 0; pin_count < 16; pin_count++)
	{
		if (pin_flags >> pin_count & 1U)
		{
			// speed selection
			p_GPIOx->OSPEEDR &= ~(0x03U << (pin_count * 2));
			p_GPIOx->OSPEEDR |= (speed << (pin_count * 2));

			// output type selection
			p_GPIOx->OTYPER &= ~(0x01U << pin_count);
			p_GPIOx->OTYPER |= (output_type << pin_count);
		}
	}
	return;
}

/*
 * Init configuration for alternate function pin
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_flags] - (GPIO_FLAG_PINx | GPIO_FLAG_PINy)
 * @param[af] - alternate function number
 * @return - void
 */
void GPIO_ConfigAF(GPIO_TypeDef *p_GPIOx, uint16_t pin_flags, GpioAF_t af)
{
	for (uint16_t pin_count = 0; pin_count < 16; pin_count++)
	{
		if (pin_flags >> pin_count & 1U)
		{
			// clear 4 AF bits and set new value
			p_GPIOx->AFR[pin_count / 8] &= ~(15UL << ((pin_count) * 4));
			p_GPIOx->AFR[pin_count / 8] |= (af << ((pin_count % 8) * 4));
		}
	}
	return;
}

/*
 * Init configuration for exti pin
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin] - pin number
 * @param[trigger] - rising/falling/both
 * @return - void
 */
void GPIO_ConfigEXTI(GPIO_TypeDef *p_GPIOx, GpioPinNumber_t pin,
		GpioEXTITrigger_t trigger)
{
	// set as input
	p_GPIOx->MODER &= ~(0x3U << (pin * 2));
	// interrupt mask
	EXTI->IMR |= (0x1U << pin);

	// rising/falling trigger
	if ((trigger == kGpioFallingTrig) || (trigger == kGpioFallingRisingTrig))
	{
		EXTI->FTSR |= (0x01U << pin);
	}

	if ((trigger == kGpioRisingTrig) || (trigger == kGpioFallingRisingTrig))
	{
		EXTI->RTSR |= (0x01U << pin);
	}

	// enable NVIC interrupt
	if (pin < kGpioPin5)
	{
		// positions for EXTI interrupts in NVIC vector table are 6-10
		NVIC->ISER[0] |= (0x01U << (pin + 6));
	}
	else if (pin < kGpioPin10)
	{
		// position for EXTI9_5 is 23
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	else
	{
		// position for EXTI15_10 is 40
		NVIC_EnableIRQ(EXTI15_10_IRQn);
	}

	// set SYSCFG for external IRQ
	// enable clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// get 4 bits code for certain port
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(p_GPIOx);
	// put it in syscfg register
	SYSCFG->EXTICR[pin / 4] |= (portcode << ((pin % 4) * 4));

	return;
}

/*
 * Write GPIO pin
 * @param[*p_GPIOx] - base address of gpiox peripheral
 * @param[pin] - pin number
 * @param[pin_state]- GPIO_PIN_RESET/GPIO_PIN_SET
 * @return - void
 */
void GPIO_WritePin(GPIO_TypeDef *p_GPIOx, GpioPinNumber_t pin,
		uint8_t pin_state)
{
	p_GPIOx->ODR &= ~(0x01U << pin);
	p_GPIOx->ODR |= pin_state << pin;
}

/*
 * Read GPIO pin
 * @param[*p_GPIOx] - base address of gpiox peripheral
 * @param[pin] - pin number
 * @return - 0 or 1
 */
uint8_t GPIO_ReadPin(GPIO_TypeDef *p_GPIOx, GpioPinNumber_t pin)
{
	return ((p_GPIOx->IDR >> pin) & 0x01U);
}

/*
 * Toggle GPIO pin
 * @param[*p_GPIOx] - base address of gpiox peripheral
 * @param[pin] - pin number
 * @return - void
 */
void GPIO_TogglePin(GPIO_TypeDef *p_GPIOx, GpioPinNumber_t pin)
{
	p_GPIOx->ODR ^= 0x01U << pin;
}

/*
 * Clear pending flag
 * @param[PinNumber] - GPIO_PIN_x @PinNumber
 * @return - void
 */
void GPIO_ClearPendingEXTIFlag(GpioPinNumber_t pin)
{
	EXTI->PR |= (0b1 << pin);
}
