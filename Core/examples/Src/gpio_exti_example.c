/*
 * gpio_exti_example.c
 *
 *  Created on: Feb 9, 2022
 *      Author: pawel
 */

#include "gpio_example.h"
#include "stm32f401xe_gpio.h"

// UART examples
// - configure input button exti
// - configure output led
// - when button is pressed then start/stop blinking

volatile exti_flag;
void GPIOConfig(void);

void gpio_exti_example(void)
{

	// config peripherals
	GPIOConfig();

	while (1)
	{

		if (exti_flag == 1)
		{
			for (uint32_t i = 0; i < 100000; i++)
			{
			}
			GPIO_TogglePin(GPIOA, kGpioPin5);
		}

	}
}

void GPIOConfig(void)
{
	// PA5 Led as output
	GPIO_InitClock(GPIOA);
	GPIO_ConfigBasic(GPIOA, kGpioPin5, kGpioModeOutput, kGpioPUPDNoPull);
	GPIO_ConfigOutput(GPIOA, kGpioPin5, kGpioOTPushPull, kGpioSpeedHigh);

	// PC13 Button as EXTI
	GPIO_InitClock(GPIOC);
	GPIO_ConfigBasic(GPIOC, kGpioPin13, kGpioModeInput, kGpioPUPDNoPull);
	GPIO_ConfigEXTI(GPIOC, kGpioPin13, kGpioFallingTrig);

	return;
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_ClearPendingEXTIFlag(kGpioPin13);
	exti_flag ^= 0x01U;

}
