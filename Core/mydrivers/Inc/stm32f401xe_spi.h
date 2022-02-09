/*
 * stm32f401xe_spi.h
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */

#ifndef CORE_MYDRIVERS_INC_STM32F401XE_SPI_H_
#define CORE_MYDRIVERS_INC_STM32F401XE_SPI_H_

#include "stm32f401xe.h"

typedef enum kSpiClockPolarity_t
{
	kSpiPolarityIdle0, kSpiPolarityIdle1
} kSpiClockPolarity_t;

typedef enum kSpiClockPhase_t
{
	kSpiPhase1stEdge, kSpiPhase2ndEdge
} kSpiClockPhase_t;

typedef enum kSpiDataFrame_t
{
	kSpiDataFrame8bit, kSpiDataFrame16bit
} kSpiDataFrame_t;

typedef enum kSpiPinConfig_t
{
	kSpiPinNoConfig, kSpiPinStandard, kSpiPinAlternate
} kSpiPinConfig_t;

/*
 * Internal Spi prescaler, maximum SPI speed is fPCLK/2
 */
typedef enum kSpiClockPrescaler_t
{
	kSpiClockPrescaler2,
	kSpiClockPrescaler4,
	kSpiClockPrescaler8,
	kSpiClockPrescaler16,
	kSpiClockPrescaler32,
	kSpiClockPrescaler64,
	kSpiClockPrescaler128,
	kSpiClockPrescaler256

} kSpiClockPrescaler_t;

typedef enum kSpiError_t
{
	kSpiErrorNoError,
} kSpiError_t;

typedef struct Spi_Handle_t
{
	SPI_TypeDef *p_spix;

	kSpiError_t error;

} Spi_Handle_t;

#endif /* CORE_MYDRIVERS_INC_STM32F401XE_SPI_H_ */
