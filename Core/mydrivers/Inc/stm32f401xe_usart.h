/*
 * stm32f401xe_usart.h
 *
 *  Created on: 12 sty 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F401XE_USART_H_
#define MYDRIVERS_INC_STM32F401XE_USART_H_

#include "stm32f401xe.h"

// Struct to configure Usart peripheral
typedef struct USART_Config_t
{
	uint8_t word_lenght;			// @WordLenght

	uint8_t stop_bits;				// @StopBits

	uint8_t oversampling;			// @Oversampling

	uint32_t baud_rate;				// baud rate w tx/rx

} USART_Config_t;

typedef struct USART_Handle_t
{
	USART_TypeDef *p_usartx;		// @address in memory

	USART_Config_t usart_config;	// @Peripheral config

} USART_Handle_t;

// @StopBits
#define USART_STOPBITS_1			0U
#define USART_STOPBITS_05			1U
#define USART_STOPBITS_2			2U
#define USART_STOPBITS_15			3U

// @WordLenght
#define USART_WORD_LENGHT_8BITS		0U
#define USART_WORD_LENGHT_9BITS		1U

// @Oversampling
#define USART_OVERSAMPLING_16		0U
#define USART_OVERSAMPLING_8		1U

void Usart_Transmit(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer, uint32_t data_lenght);
void Usart_InitGpioPins(USART_Handle_t *p_handle_usart);
#endif
