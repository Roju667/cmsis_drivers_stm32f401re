/*
 * gpio_example.c
 *
 *  Created on: Feb 9, 2022
 *      Author: pawel
 */

#include "gpio_example.h"
#include "stm32f401xe_gpio.h"

// UART examples
// - configure input button
// - configure output led
// - when button is pressed led is not blinking

void GPIOConfig(void);

void gpio_example(void)
{

	// config peripherals
	GPIOConfig();

	while (1)
	{

		if (GPIO_ReadPin(GPIOC, kGpioPin13) == 1)
		{
			for (uint32_t i = 0; i < 100000; i++)
			{
			}
			GPIO_TogglePin(GPIOA, kGpioPin5);
		}

	}
}

/*
 * configure LED on the board
 */
void GPIOConfig(void)
{
	// PA5 Led
	GPIO_InitClock(GPIOA);
	GPIO_ConfigBasic(GPIOA, GPIO_FLAG_PIN_5, kGpioModeOutput, kGpioPUPDNoPull);
	GPIO_ConfigOutput(GPIOA, GPIO_FLAG_PIN_5, kGpioOTPushPull, kGpioSpeedHigh);

	// PC13 Button
	GPIO_InitClock(GPIOC);
	GPIO_ConfigBasic(GPIOC, GPIO_FLAG_PIN_13, kGpioModeInput, kGpioPUPDNoPull);

	return;
}

