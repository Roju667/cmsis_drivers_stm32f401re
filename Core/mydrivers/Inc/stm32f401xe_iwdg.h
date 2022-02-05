/*
 * stm32f401xe_iwdg.h
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */

#ifndef CORE_MYDRIVERS_INC_STM32F401XE_IWDG_H_
#define CORE_MYDRIVERS_INC_STM32F401XE_IWDG_H_

#include "stm32f401xe.h"

#define IWDG_START_VAL 0xCCCC
#define IWDG_RELOAD_VAL 0xAAAA
#define IWDG_ACCESS_VAL 0x5555

typedef enum IwdgPrescalerDivider_t
{
	kIwdgPrescaler4,
	kIwdgPrescaler8,
	kIwdgPrescaler16,
	kIwdgPrescaler32,
	kIwdgPrescaler64,
	kIwdgPrescaler128,
	kIwdgPrescaler256
} IwdgPrescalerDivider_t;

void IWDG_SetBasicParameters(IwdgPrescalerDivider_t prescaler);
void IWDG_StartWatchdog(void);
void IWDG_ReloadWatchdog(void);

#endif /* CORE_MYDRIVERS_INC_STM32F401XE_IWDG_H_ */
