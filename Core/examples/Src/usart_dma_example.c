/*
 * usart_dma.c
 *
 *  Created on: 6 lut 2022
 *      Author: pawel
 */


#include <stdint.h>
#include <string.h>
#include "usart_dma_example.h"
#include "stm32f401xe_dma.h"
#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"
#include "stm32f401xe_usart.h"
#include "stm32f4xx.h"


// UART examples
// - send on uart in dma mode
// - receive on uart in double buffer mode
// - send data in polling mode (full buffer/idle line/error line)

void GPIOConfig(void);
void USART2Config(USART_Handle_t *p_usart2);
void DMA1Config(DMA_Handle_t *p_dma1);

DMA_Handle_t p_dma1;
USART_Handle_t p_usart2;

volatile uint8_t SendBuffer1Flag, IdleFlag, ErrorFlag, SendBuffer0Flag;

void usart_dma_example(void)
{

	// variables
	uint8_t databuffer1[16] =
	{ 0 };
	uint8_t databuffer0[16] =
	{ 0 };
	uint8_t databuffer2[16] = "print dma \n\r";
	memset(&p_dma1, 0, sizeof(p_dma1));

	// config peripherals
	GPIOConfig();
	USART2Config(&p_usart2);
	DMA1Config(&p_dma1);

	// enable DMA interrupts for read/send
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	// send message in dma mode
	USART_ConfigureTransmitDMA(&p_usart2, databuffer2, NULL);
	USART_TransmitDMAStart(&p_usart2, 16);
	// start receiving in dma mode
	USART_ConfigureReceiveDMA(&p_usart2, databuffer0, databuffer1);
	USART_ReceiveDMAStart(&p_usart2, 16);


	while (1)
	{
		for (uint32_t i = 0; i < 100000; i++)
		{
		}
		GPIO_TogglePin(GPIOA, kGpioPin5);

		if (SendBuffer1Flag == 1)
		{
			SendBuffer1Flag = 0;
			USART_Transmit(&p_usart2, (uint8_t*) "\n\r printing db1:", 16);
			USART_Transmit(&p_usart2, databuffer1, 16);
		}

		if (SendBuffer0Flag == 1)
		{
			SendBuffer0Flag = 0;
			USART_Transmit(&p_usart2, (uint8_t*) "\n\r printing db0:", 16);
			USART_Transmit(&p_usart2, databuffer0, 16);
		}

		if (IdleFlag == 1)
		{
			IdleFlag = 0;
			USART_Transmit(&p_usart2, (uint8_t*) "\n\rIDLE LINE", 11);
		}

		if (ErrorFlag == 1)
		{
			ErrorFlag = 0;
			USART_Transmit(&p_usart2, (uint8_t*) "\n\rERROR LINE", 12);
		}

	}
}

/*
 * configure LED on the board
 */
void GPIOConfig(void)
{
	// PA5 Led
	GPIO_InitClock(GPIOA);
	GPIO_ConfigBasic(GPIOA, GPIO_FLAG_PIN_5, kGpioModeOutput, kGpioPUPDNoPull);
	GPIO_ConfigOutput(GPIOA, GPIO_FLAG_PIN_5, kGpioOTPushPull, kGpioSpeedHigh);
	return;
}

/*
 * configure uart peripheral :
 * - assign base adress and dma streams
 * - set basic parameters
 * - set baud rate
 * - enable usart irqs
 */
void USART2Config(USART_Handle_t *p_usart2)
{
	p_usart2->p_usartx = USART2;
	p_usart2->usart_dma.p_dma_stream_tx = DMA1_Stream6;
	p_usart2->usart_dma.p_dma_stream_rx = DMA1_Stream5;
	USART_InitClock(p_usart2);
	USART_InitGpioPins(p_usart2);
	USART_SetBasicParameters(p_usart2, kUsartWordLenght8, kUsartStopBits0,
			kUsartNoParity);
	USART_SetBaudRate(p_usart2, 115200, kUsartOversampling16);
	USART_EnableIRQs(p_usart2, USART_CR1_IDLEIE, 0, USART_CR3_EIE);
	return;
}

/*
 * configure dma peripheral :
 * - to redo
 */
void DMA1Config(DMA_Handle_t *p_dma1)
{
	// read
	p_dma1->p_dmax = DMA1;
	p_dma1->p_dma_streamx = DMA1_Stream5;
	p_dma1->stream_config.channel_number = kChannel4;
	p_dma1->stream_config.circular_mode = kCircularDisable;
	p_dma1->stream_config.direction = kPeriToMem;
	p_dma1->stream_config.mem_data_size = kByte;
	p_dma1->stream_config.mem_increment = kIncrementEnable;
	p_dma1->stream_config.peri_data_size = kByte;
	p_dma1->stream_config.peri_increment = kIncrementDisable;

	DMA_StreamInit(p_dma1);

	// send
	p_dma1->p_dma_streamx = DMA1_Stream6;
	p_dma1->stream_config.direction = kMemToPeri;

	DMA_StreamInit(p_dma1);
}

/*
 * idle line detection
 * error detection
 * receive finished detection
 */
void USART2_IRQHandler(void)
{
	if (USART2->SR & USART_SR_IDLE)
	{
		IdleFlag = USART2->DR;
		IdleFlag = 1;
	}

	if (USART2->SR & USART_ERROR_FLAGS)
	{
		ErrorFlag = 1;
	}

	USART2->SR &= ~(USART_SR_RXNE);

}
/*
 * 16 bytes transfered from data buffer to
 * uart data register irq
 */
void DMA1_Stream6_IRQHandler(void)
{

	// this has to be used to clear tx line status
	USART_DMATransmitDoneCallback(&p_usart2);

	// clear dma5 transfer complete flag
	DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
}

/*
 * 16 bytes transfered from uart to
 * one of the data buffers irq
 */
void DMA1_Stream5_IRQHandler(void)
{
	// this has to be used to clear rx line status
	USART_DMAReceiveDoneCallback(&p_usart2);

	// check which data buffer is pointed to and send buffer
	if (p_usart2.usart_dma.p_dma_stream_rx->CR & (DMA_SxCR_CT))
	{
		SendBuffer1Flag = 1;
	}
	else
	{
		SendBuffer0Flag = 1;
	}

	// clear dma5 transfer complete flag
	DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
}
