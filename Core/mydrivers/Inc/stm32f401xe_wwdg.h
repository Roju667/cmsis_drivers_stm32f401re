/*
 * stm32f401xe_wwdg.h
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */

#ifndef CORE_MYDRIVERS_INC_STM32F401XE_WWDG_H_
#define CORE_MYDRIVERS_INC_STM32F401XE_WWDG_H_

#include "stm32f401xe.h"

// 7-bit down counter, when it rolls down from 0x40 (64) to 0x3F (63) then mcu
// resets if the software reloads value too early it also resets

typedef enum WwdgTimerBase_t
{
	kWwdgPrescaler1, kWwdgPrescaler2, kWwdgPrescaler4, kWwdgPrescaler8

} WwdgTimerBase_t;

#define WWDG_FEED_VALUE 0x7F

// basic functions
void WWDG_SetBasicParameters(WwdgTimerBase_t prescaler, uint8_t window_val);
void WWDG_StartWatchdog(void);
void WWDG_ReloadWatchdog(void);
uint8_t WWDG_GetWatchdog(void);

// additional functions
uint32_t WWDG_GetMinMaxTimeout(uint8_t min_max);
uint32_t WWDG_GetTimePerWwdgTick(void);
#endif /* CORE_MYDRIVERS_INC_STM32F401XE_WWDG_H_ */
