/*
 * stm32f401xe_iwdg.c
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */

#include "stm32f401xe_iwdg.h"

/*
 * set watchdog prescaler it is 32kHz (in theory) / prescaler
 */
void IWDG_SetBasicParameters(IwdgPrescalerDivider_t prescaler)
{
	// start LSI clock
	RCC->CSR |= RCC_CSR_LSION;
	// wait until LSI is ready
	while (!(RCC->CSR & RCC_CSR_LSIRDY))
		;
	
	IWDG->PR |= prescaler << IWDG_PR_PR_Pos;
	return;
}

/*
 * start watchdog counting down to 0 (starts with 0xFFF)
 * 0xCCCC is a start command
 */
void IWDG_StartWatchdog(void)
{
	IWDG->KR = IWDG_START_VAL;
	return;
}

/*
 * reload watchdog value to prevent reset
 * 0xAAAA is a reload command
 */
void IWDG_ReloadWatchdog(void)
{
	IWDG->KR = IWDG_RELOAD_VAL;
	return;
}
