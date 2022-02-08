/*
 * stm32f401xe_wwdg.c
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */
#include "stm32f401xe_wwdg.h"

#include "stm32f401xe_rcc.h"

/*
 * this function is just to calculate using formula from RM
 * numbers of miliseconds that we have after starting WWDG and before reset
 */
uint32_t WWDG_GetMinMaxTimeout(uint8_t min_max)
{
	RCC_ClockFreqs freqs;
	uint32_t max_timeout, min_timeout;  // [us]
	// get timer_base from register
	uint8_t timer_base = ((WWDG->CFR >> 7) & 3U);
	uint8_t pow_timer_base = 1;

	// some stupid loop to not use math function pow() that is using floats
	for (uint8_t i = 0; i < timer_base; i++)
	{
		pow_timer_base *= 2;
	}

	uint8_t max_tiemout_ticks = 0x3F;
	uint8_t min_timeout_ticks = 0x00;

	// get pclk1 frequency
	RCC_GetClockFrequencies(&freqs);

	// if you feed watchdog with value 0x40 then you have min_timout micro seconds
	// before reset
	min_timeout = (4096 * pow_timer_base * (min_timeout_ticks + 1))
			/ (freqs.pclk1 / 1000000);

	// if you feed watchdog with value 0xFF then you have max_timeout micro seconds
	max_timeout = (4096 * pow_timer_base * (max_tiemout_ticks + 1))
			/ (freqs.pclk1 / 1000000);

	if (min_max == 0)
	{
		return min_timeout;
	}

	return max_timeout;
}

/*
 * this function calculates wwdg frequency and returns time per tick in [us]
 */
uint32_t WWDG_GetTimePerWwdgTick(void)
{
	RCC_ClockFreqs freqs;
	uint32_t wwdg_freq;
	uint8_t timer_base = ((WWDG->CFR >> 7) & 3U);
	uint8_t pow_timer_base = 1;
	uint32_t time_per_tick; // time per 1 watchdog tick in [us]

	// some stupid loop to not use math function pow() that is using floats
	for (uint8_t i = 0; i < timer_base; i++)
	{
		pow_timer_base *= 2;
	}

	// get pclk1 frequency
	RCC_GetClockFrequencies(&freqs);

	// calculate wwdg frequency
	wwdg_freq = freqs.pclk1 / (4096 * pow_timer_base);
	// calculate time per one tick
	time_per_tick = 1000000 / wwdg_freq;

	return time_per_tick;
}

/*
 * window val - if watchdog is going to be reloaded before this value must be
 * between 63 - 127 -> reset
 */

void WWDG_SetBasicParameters(WwdgTimerBase_t prescaler, uint8_t window_val)
{
	RCC_CLOCK_WWDG_ENABLE();

	WWDG->CFR |= (prescaler << WWDG_CFR_WDGTB_Pos);

	// 7-bit window register so to be sure i am reseting msb of uint8
	window_val &= ~(1U << 7U);
	WWDG->CFR |= (window_val << WWDG_CFR_W_Pos);

	return;
}

/*
 * start watchdog (can not be stopped - only with reset)
 */
void WWDG_StartWatchdog(void)
{
	// activate watchdog
	WWDG->CR |= (WWDG_FEED_VALUE << WWDG_CR_T_Pos);
	WWDG->CR |= WWDG_CR_WDGA;

	return;
}

/*
 * feed watchdog with the counter value
 */
void WWDG_ReloadWatchdog(void)
{
	// put a value in the counter
	WWDG->CR |= (WWDG_FEED_VALUE << WWDG_CR_T_Pos);
	return;
}

/*
 * return current timer value between 127 and 63
 */
uint8_t WWDG_GetWatchdog(void)
{
	return (WWDG->CR & WWDG_CR_T_Msk);
}
