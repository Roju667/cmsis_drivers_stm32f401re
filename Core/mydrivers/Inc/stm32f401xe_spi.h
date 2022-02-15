/*
 * stm32f401xe_spi.h
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */

#ifndef CORE_MYDRIVERS_INC_STM32F401XE_SPI_H_
#define CORE_MYDRIVERS_INC_STM32F401XE_SPI_H_

#include "stm32f401xe.h"

typedef enum
{
	kSpiPolarityIdle0, kSpiPolarityIdle1
} SpiClockPolarity_t;

typedef enum
{
	kSpiPhase1stEdge, kSpiPhase2ndEdge
} SpiClockPhase_t;

typedef enum
{
	kSpiDataFrame8bit, kSpiDataFrame16bit
} SpiDataFrame_t;

typedef enum
{
	kSpiPinNoConfig, kSpiPinStandard, kSpiPinAlternate
} SpiPinConfig_t;

/*
 * Internal Spi prescaler, maximum SPI speed is fPCLK/2
 */
typedef enum
{
	kSpiClockPrescaler2,
	kSpiClockPrescaler4,
	kSpiClockPrescaler8,
	kSpiClockPrescaler16,
	kSpiClockPrescaler32,
	kSpiClockPrescaler64,
	kSpiClockPrescaler128,
	kSpiClockPrescaler256

} SpiClockPrescaler_t;

typedef enum
{
	kSpiErrorNoError,
} SpiError_t;

typedef struct Spi_Handle_t
{
	SPI_TypeDef *p_spix;

	SpiError_t error;

} Spi_Handle_t;

#endif /* CORE_MYDRIVERS_INC_STM32F401XE_SPI_H_ */
