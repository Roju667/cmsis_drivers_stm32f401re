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
	kUsartNoError, kUsartErrorRx, kUsartErrorRxBusy, kUsartErrorTxBusy
} UsartError_t;

// implementing those status structs to be able to use polling/dma in one
// program
typedef enum UsartRxLineStatus_t
{
	kUsartRxIdle, kUsartRxPolling, kUsartRxDma
} UsartRxLineStatus_t;

typedef enum UsartTxLineStatus_t
{
	kUsartTxIdle, kUsartTxPolling, kUsartTxDma
} UsartTxLineStatus_t;

typedef struct USART_Handle_t
{
	USART_TypeDef *p_usartx;  // @address in memory

	DMA_Stream_Info usart_dma;

	UsartError_t error;

	UsartRxLineStatus_t rx_status;

	UsartTxLineStatus_t tx_status;

} USART_Handle_t;

// ******** CONFIGURATION ******** //

// @StopBits - synchornization bits
typedef enum UsartStopBits_t
{
	kUsartStopBits0, kUsartStopBits05, kUsartStopBits2, kUsartStopBits15
} UsartStopBits_t;

// @WordLenght - 8/9 bits
typedef enum UsartWordLenght_t
{
	kUsartWordLenght8, kUsartWordLenght9
} UsartWordLenght_t;

// @Oversampling - how many times every bit is sampled from the line
typedef enum UsartOversampling_t
{
	kUsartOversampling16, kUsartOverasmpling8
} UsartOversampling_t;

// @Parity - if you would like to check for parity error i guess
typedef enum UsartParity_t
{
	kUsartEvenParity, kUsartOddParity, kUsartNoParity
} UsartParity_t;

// @IRQ FLAGS
#define USART_CR1_IRQ_FLAGS                                               \
  (USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE | \
   USART_CR1_IDLEIE)
#define USART_CR2_IRQ_FLAGS (USART_CR2_LBDIE)
#define USART_CR3_IRQ_FLAGS (USART_CR3_EIE | USART_CR3_CTSIE)
#define USART_ERROR_FLAGS \
  (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)

// ******** FUNCTIONS ******** //

// Transmit functions
void USART_Transmit(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght);
void USART_ConfigureTransmitDMA(USART_Handle_t *p_handle_usart,
		uint8_t *p_data_buffer0, uint8_t *p_data_buffer1);
void USART_TransmitDMAStart(USART_Handle_t *p_handle_usart,
		uint32_t data_lenght);
void USART_DMATransmitDoneCallback(USART_Handle_t *p_handle_usart);

// Receive functions
void USART_Recieve(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght);
void USART_ConfigureReceiveDMA(USART_Handle_t *p_handle_usart,
		uint8_t *p_data_buffer0, uint8_t *p_data_buffer1);
void USART_ReceiveDMAStart(USART_Handle_t *p_handle_usart, uint32_t data_lenght);
void USART_DMAReceiveDoneCallback(USART_Handle_t *p_handle_usart);

// Init functions
void USART_InitClock(USART_Handle_t *p_handle_usart);
void USART_InitGpioPins(USART_Handle_t *p_handle_usart);
void USART_SetBasicParameters(USART_Handle_t *p_handle_usart,
		UsartWordLenght_t word_lenght, UsartStopBits_t stop_bits,
		UsartParity_t parity);
void USART_SetBaudRate(USART_Handle_t *p_handle_usart, uint32_t baud_rate,
		UsartOversampling_t oversampling);
void USART_EnableIRQs(USART_Handle_t *p_handle_usart, uint32_t CR1_flags,
		uint32_t CR2_flags, uint32_t CR3_flags);
#endif
