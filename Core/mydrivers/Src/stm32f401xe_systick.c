/*
 * stmf401xe_systick.c
 *
 *  Created on: 14 lut 2022
 *      Author: ROJEK
 */

#include "stm32f401xe.h"
#include "stm32f401xe_rcc.h"
#include "stm32f401xe_systick.h"

static volatile uint32_t systick;

void SYSTICK_ConfigureMilisecond(void)
{
	RCC_ClockFreqs freqs;

	RCC_GetClockFrequencies(&freqs);

	SysTick_Config(freqs.hclk/1000);

	return;
}

uint32_t SYSTICK_GetTick(void)
{
	return systick;
}

void SYSTICK_Delay(uint32_t miliseconds)
{
	uint32_t delay = SYSTICK_GetTick();
	while(SYSTICK_GetTick() - delay < miliseconds)
		;
	return;
}

void SysTick_Handler (void)
{
	systick++;
}

