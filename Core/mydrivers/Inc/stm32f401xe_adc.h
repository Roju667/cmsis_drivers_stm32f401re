/*
 * stm32f401xe_adc.h
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */

#ifndef CORE_MYDRIVERS_INC_STM32F401XE_ADC_H_
#define CORE_MYDRIVERS_INC_STM32F401XE_ADC_H_

typedef enum
{
	kAdcSampleCycles3,kAdcSampleCycles15,kAdcSampleCycles28,kAdcSampleCycles84,kAdcSampleCycles112,kAdcSampleCycles144,kAdcSampleCycles480
}AdcSampleCycles_t;

#endif /* CORE_MYDRIVERS_INC_STM32F401XE_ADC_H_ */
