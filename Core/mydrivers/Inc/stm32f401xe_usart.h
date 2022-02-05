/*
 * stm32f401xe_usart.h
 *
 *  Created on: 12 sty 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F401XE_USART_H_
#define MYDRIVERS_INC_STM32F401XE_USART_H_

#include "stm32f401xe.h"
#include "stm32f401xe_dma.h"

typedef enum UsartError_t
{
	kUsartNoError, kUsartErrorRx
} UsartError_t;

// Struct to configure Usart peripheral
typedef struct USART_Config_t
{
	uint8_t word_lenght; // @WordLenght

	uint8_t stop_bits; // @StopBits

	uint8_t oversampling; // @Oversampling

	uint32_t baud_rate; // baud rate w tx/rx

} USART_Config_t;

typedef struct USART_Handle_t
{
	USART_TypeDef *p_usartx; // @address in memory

	DMA_Stream_Info usart_dma;

	USART_Config_t usart_config; // @Peripheral config

	UsartError_t error;

} USART_Handle_t;

// @StopBits
#define USART_STOPBITS_1 0U
#define USART_STOPBITS_05 1U
#define USART_STOPBITS_2 2U
#define USART_STOPBITS_15 3U

// @WordLenght
#define USART_WORD_LENGHT_8BITS 0U
#define USART_WORD_LENGHT_9BITS 1U

// @Oversampling
#define USART_OVERSAMPLING_16 0U
#define USART_OVERSAMPLING_8 1U

void Usart_Transmit(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght);
void Usart_TransmitDMA(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght);
void Usart_TransmitDMADoubleBuffer(USART_Handle_t *p_handle_usart,
		uint8_t *p_data_buffer0, uint8_t *p_data_buffer1, uint32_t data_lenght);
void Usart_Recieve(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght);
void Usart_RecieveDMA(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght);
void Usart_InitGpioPins(USART_Handle_t *p_handle_usart);
void Usart_Init(USART_Handle_t *p_handle_usart);
#endif
