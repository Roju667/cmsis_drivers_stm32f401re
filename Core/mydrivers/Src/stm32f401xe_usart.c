/*
 * stm32f401xe_usart.c
 *
 *  Created on: Jan 29, 2022
 *      Author: pawel
 */

#include "stm32f401xe_usart.h"

#include "math.h"
#include "stm32f401xe.h"
#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"

/*
 * Topics not included (so far)
 * -Irda,Smartcard,Lin modes
 * -USART mode (synchronous)
 * -Hardware flow control (RTS/CTS)
 * -Half-duplex single line mode
 * -Multiprocessor communication modes - IDLE and address mark
 */

/*
 * Problems :
 * -Using Tx/Rx status for DMA forces me to clean the status in
 * DMA transfer finished interrupt that must be included in dma irq handler
 * On other hand it allows mixing sending by polling/dma in one program -> I DONT LIKE IT
 *
 * -Enabling USART TCIE irq causes irq storm that i can't solve for now
 */

// ******** INIT FUNCTIONS ******** //

/*
 * Init USART clock
 * @param[*p_handle_usart] - handler to usart struct
 * @return - void
 */
void USART_InitClock(USART_Handle_t *p_handle_usart)
{
	if (p_handle_usart->p_usartx == USART1)
	{
		RCC_CLOCK_USART1_ENABLE();
		RCC_RESET_USART1();
	}
	else if (p_handle_usart->p_usartx == USART2)
	{
		RCC_CLOCK_USART2_ENABLE();
		RCC_RESET_USART2();
	}
	else if (p_handle_usart->p_usartx == USART6)
	{
		RCC_CLOCK_USART6_ENABLE();
		RCC_RESET_USART6();
	}

	return;
}

/*
 * Init GPIO pins for usart peripheral
 * @param[*p_handle_usart] - handler to usart struct
 * @return - void
 */
void USART_InitGpioPins(USART_Handle_t *p_handle_usart)
{

	if(p_handle_usart->p_usartx == USART2)
	{
		// PA2 RX PA3 TX
		GPIO_ConfigBasic(GPIOA, (GPIO_FLAG_PIN_3 | GPIO_FLAG_PIN_2), kGpioModeAF, kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOA, (GPIO_FLAG_PIN_3 | GPIO_FLAG_PIN_2), kGpioOTOpenDrain, kGpioSpeedVeryHigh);
		GPIO_ConfigAF(GPIOA, (GPIO_FLAG_PIN_3 | GPIO_FLAG_PIN_2), kGpioAF7);

	}

	return;
}
/*
 * Calculate values for baud rate registers
 * @param[*p_handle_usart] - handler to usart struct
 * @param[baud_rate] - desired baud rate value
 * @param[oversampling] - oversampling method 8/16
 * @return - void
 */
void USART_SetBaudRate(USART_Handle_t *p_handle_usart, uint32_t baud_rate,
		UsartOversampling_t oversampling)
{
	RCC_ClockFreqs clock_freqs;
	uint32_t pclk_freq;
	float baud_div;
	uint16_t baud_div_mantissa;
	uint8_t baud_div_fraction;

	// set oversampling

	p_handle_usart->p_usartx->CR1 |= (oversampling << USART_CR1_OVER8_Pos);

	// in equation we have 8 * (2 - OVER8)
	uint8_t usart_divmulti =
			8
					* (2
							- (1U
									& (p_handle_usart->p_usartx->CR1
											>> USART_CR1_OVER8_Pos)));
	// read clock values from rcc registers
	RCC_GetClockFrequencies(&clock_freqs);

	if (p_handle_usart->p_usartx == USART2)
	{
		pclk_freq = clock_freqs.pclk1;
	}
	else
	{
		// usart1 and usart6 are on apb2
		pclk_freq = clock_freqs.pclk2;
	}

	baud_div = ((float) pclk_freq / (baud_rate * usart_divmulti));
	baud_div_mantissa = baud_div;
	// rounding number to get correct fraction
	baud_div_fraction = round((baud_div - baud_div_mantissa) * usart_divmulti);
	// if after rounding we have value bigger or equal to oversampling then we
	// need to carry +1 to mantissa and set fraction to 0. like 50,99 -> 51;
	if (baud_div_fraction >= usart_divmulti)
	{
		baud_div_fraction = 0;
		baud_div_mantissa++;
	}

	// insert values to brr register
	p_handle_usart->p_usartx->BRR = 0;
	p_handle_usart->p_usartx->BRR |= (baud_div_mantissa
			<< USART_BRR_DIV_Mantissa_Pos);
	p_handle_usart->p_usartx->BRR |= (baud_div_fraction
			<< USART_BRR_DIV_Fraction_Pos);
}

/*
 * Enable IRQ flags for USART peripheral
 * use CRx_IRQ_FLAGS to enable all the interrupts in register, otherwise use
 * like : (USART_CR1_PEIE | USART_CR1_TXEIE)
 * @param[*p_handle_usart] - handler to usart struct
 * @param[CR1_flags] - IRQ flags in CR1
 * @param[CR2_flags] - IRQ flags in CR2
 * @param[CR3_flags] - IRQ flags in CR3
 * @return - void
 */
void USART_EnableIRQs(USART_Handle_t *p_handle_usart, uint32_t CR1_flags,
		uint32_t CR2_flags, uint32_t CR3_flags)
{
	NVIC_EnableIRQ(USART2_IRQn);
	// CR1 - PEIE,TXEIE,TCIE,RXNEIE,IDLEIE
	if (CR1_flags)
	{
		p_handle_usart->p_usartx->CR1 |= CR1_flags;
	}

	// CR2 - LBDIE
	if (CR2_flags)
	{
		p_handle_usart->p_usartx->CR2 |= CR2_flags;
	}

	// CR3 - CTSIE, EIE
	if (CR3_flags)
	{
		p_handle_usart->p_usartx->CR1 |= CR3_flags;
	}

	return;
}

/*
 * Basic init function
 * @param[*p_handle_usart] - handler to usart struct
 * @param[word_lenght] - word lenghts 8-9 bits
 * @param[stop_bits] - stop bits 1-2
 * @param[parity] - parity odd/even/null
 * @return - void
 */
void USART_SetBasicParameters(USART_Handle_t *p_handle_usart,
		UsartWordLenght_t word_lenght, UsartStopBits_t stop_bits,
		UsartParity_t parity)
{
	//	enable the USART by writing the UE bit in USART_CR1 register to 1.
	p_handle_usart->p_usartx->CR1 |= USART_CR1_UE;
	//	program the M bit in USART_CR1 to define the word length.
	p_handle_usart->p_usartx->CR1 |= (word_lenght << USART_CR1_M_Pos);
	//	program the number of stop bits in USART_CR2.
	p_handle_usart->p_usartx->CR1 |= (stop_bits << USART_CR2_STOP_Pos);
	// program parity control
	if (parity != kUsartNoParity)
	{
		p_handle_usart->p_usartx->CR1 |= USART_CR1_PCE;
		p_handle_usart->p_usartx->CR1 |= (parity << USART_CR1_PS_Pos);
	}

	//clear TC flag
	p_handle_usart->p_usartx->SR &= ~(USART_SR_TC);
	return;
}

// ******** RECIEVE FUNCTIONS ******** //

/*
 * Polling uart receive function
 * @param[*p_handle_usart] - handler to usart struct
 * @param[*p_data_buffer] - pointer to data buffer
 * @param[data_lenght] - data size
 * @return - void
 */
void USART_Receive(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght)
{
	// check if line is not receiving right now in other mode
	if (p_handle_usart->rx_status != kUsartRxIdle)
	{
		p_handle_usart->error = kUsartErrorRxBusy;
		return;
	}
	p_handle_usart->rx_status = kUsartRxPolling;
	uint32_t rx_data_to_get = data_lenght;
	// enable recieve mode
	p_handle_usart->p_usartx->CR1 |= USART_CR1_RE;

	while (rx_data_to_get > 0)
	{
		// check status flags
		if (p_handle_usart->p_usartx->SR
				& (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
		{
			// error
			p_handle_usart->error = kUsartErrorRx;
			p_handle_usart->rx_status = kUsartRxIdle;
			return;
		}

		// copy data to buffer
		if (p_handle_usart->p_usartx->SR & USART_SR_RXNE)
		{
			p_data_buffer[data_lenght - rx_data_to_get] =
					p_handle_usart->p_usartx->DR;
			rx_data_to_get--;
		}

		if ((p_handle_usart->p_usartx->SR & (USART_SR_IDLE))
				|| rx_data_to_get == 0)
		{
			// receiving finished
			p_handle_usart->rx_status = kUsartRxIdle;
			return;
		}
	}
}


/*
 * Configuration DMA receive function (has to be called only once)
 * @param[*p_handle_usart] - handler to usart struct
 * @param[*p_data_buffer0] - pointer to data buffer
 * @param[*p_data_buffer1] - NULL/pointer to second buffer
 * @return - void
 */
void USART_ConfigureReceiveDMA(USART_Handle_t *p_handle_usart,
		uint8_t *p_data_buffer0, uint8_t *p_data_buffer1)
{
	// enable receive mode
	p_handle_usart->p_usartx->CR1 |= USART_CR1_RE;

	// configure double buffer if neccesary
	if (p_data_buffer1 != NULL)
	{
		DMA_EnableDoubleBufferMode(p_handle_usart->usart_dma.p_dma_stream_rx);
		p_handle_usart->rx_status = kUsartRxDma;
	}
	// write source and destinations
	DMA_WriteAdresses(p_handle_usart->usart_dma.p_dma_stream_rx,
			(uint32_t*) &(p_handle_usart->p_usartx->DR),
			(uint32_t*) p_data_buffer0, (uint32_t*) p_data_buffer1);

	// configure irq flags
	p_handle_usart->usart_dma.p_dma_stream_rx->CR |= DMA_SxCR_TCIE;
	USART_EnableIRQs(p_handle_usart, USART_CR1_RXNEIE, 0, 0);
	return;
}

/*
 * Toggle DMA transfer
 * @param[*p_handle_usart] - handler to usart struct
 * @param[data_lenght] - after that many bytes irq is triggered
 * @return - void
 */
void USART_ReceiveDMAStart(USART_Handle_t *p_handle_usart, uint32_t data_lenght)
{
	// check if line is not receiving right now in other mode
	if (p_handle_usart->rx_status != kUsartRxIdle
			&& p_handle_usart->rx_status != kUsartRxDma)
	{
		p_handle_usart->error = kUsartErrorRxBusy;
		return;
	}
	// enable dma receive
	p_handle_usart->p_usartx->CR3 |= USART_CR3_DMAR;

	// configure the total number of bytes to be transferred to the DMA
	p_handle_usart->usart_dma.p_dma_stream_rx->NDTR = data_lenght;

	// activate DMA channel transfer
	p_handle_usart->rx_status = kUsartRxDma;
	p_handle_usart->usart_dma.p_dma_stream_rx->CR |= DMA_SxCR_EN;

	return;
}

/*
 * Clear DMA status , it is required to use both polling and DMA in one program
 * @param[*p_handle_usart] - handler to usart struct
 * @return - void
 */
void USART_DMAReceiveDoneCallback(USART_Handle_t *p_handle_usart)
{
	if (!(p_handle_usart->usart_dma.p_dma_stream_rx->CR & DMA_SxCR_CIRC))
	{
		p_handle_usart->rx_status = kUsartRxIdle;
	}
	return;
}
// ******** TRANSMIT FUNCTIONS ******** //

/*
 * Polling uart transfer function
 * @param[*p_handle_usart] - handler to usart struct
 * @param[*p_data_buffer] - pointer to data buffer
 * @param[data_lenght] - number of bytes to send
 * @return - void
 */
void USART_Transmit(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer,
		uint32_t data_lenght)
{
	// check if dma transfer is not ongoing
	if (p_handle_usart->tx_status != kUsartTxIdle)
	{
		p_handle_usart->error = kUsartErrorTxBusy;
		return;
	}

	// disable dma and put the status
	p_handle_usart->p_usartx->CR3 &= ~(USART_CR3_DMAT);
	p_handle_usart->tx_status = kUsartTxPolling;

	uint32_t tx_data_to_send = data_lenght;

	//	6. Set the TE bit in USART_CR1 to send an idle frame as first
	// transmission.
	p_handle_usart->p_usartx->CR1 |= USART_CR1_TE;
	//	7. Write the data to send in the USART_DR register (this clears the TXE
	// bit). Repeat this 	for each data to be transmitted in case of single
	// buffer.
	while (tx_data_to_send > 0)
	{
		// wait until data register is empty
		while (!(p_handle_usart->p_usartx->SR & USART_SR_TXE))
			;

		// put data in data register
		p_handle_usart->p_usartx->DR = p_data_buffer[data_lenght
				- tx_data_to_send];

		// change counter
		tx_data_to_send--;

		//	8. After writing the last data into the USART_DR register, wait until
		// TC=1. This indicates 	that the transmission of the last frame is
		// complete. This is required for instance when 	the USART is disabled or
		// enters the Halt mode to avoid corrupting the last 	transmission
		if (tx_data_to_send == 0)
		{
			// check if data transfer is finished
			while (!(p_handle_usart->p_usartx->SR & USART_SR_TC))
				;
			p_handle_usart->tx_status = kUsartTxIdle;
			return;
		}
	}
}

/*
 * Configuration DMA transmit function (has to be called only once)
 * @param[*p_handle_usart] - handler to usart struct
 * @param[*p_data_buffer0] - pointer to data buffer
 * @param[*p_data_buffer1] - NULL/pointer to second buffer
 * @return - void
 */
void USART_ConfigureTransmitDMA(USART_Handle_t *p_handle_usart,
		uint8_t *p_data_buffer0, uint8_t *p_data_buffer1)
{
	// enable transmit mode
	p_handle_usart->p_usartx->CR1 |= USART_CR1_TE;

	// configure double buffer if neccesary
	if (p_data_buffer1 != NULL)
	{
		DMA_EnableDoubleBufferMode(p_handle_usart->usart_dma.p_dma_stream_tx);
		p_handle_usart->tx_status = kUsartTxDma;
	}

	// assign peripheral address and mem address to dma registers
	DMA_WriteAdresses(p_handle_usart->usart_dma.p_dma_stream_tx,
			(uint32_t*) &(p_handle_usart->p_usartx->DR),
			(uint32_t*) p_data_buffer0, (uint32_t*) p_data_buffer1);

	// configure irq flags
	//USART_EnableIRQs(p_handle_usart, USART_CR1_TCIE, 0, 0);
	p_handle_usart->usart_dma.p_dma_stream_tx->CR |= DMA_SxCR_TCIE;

	return;
}

/*
 * Toggle DMA transfer
 * @param[*p_handle_usart] - handler to usart struct
 * @param[data_lenght] - after that many bytes irq is triggered
 * @return - void
 */
void USART_TransmitDMAStart(USART_Handle_t *p_handle_usart,
		uint32_t data_lenght)
{
	// should be idle to start new dma
	if (p_handle_usart->tx_status != kUsartTxIdle
			&& p_handle_usart->tx_status != kUsartTxDma)
	{
		p_handle_usart->error = kUsartErrorTxBusy;
		return;
	}
	// enable dma transfer
	p_handle_usart->p_usartx->CR3 |= USART_CR3_DMAT;
	// configure the total number of bytes to be transferred to the DMA
	p_handle_usart->usart_dma.p_dma_stream_tx->NDTR = data_lenght;
	// clear tc bit
	p_handle_usart->p_usartx->SR &= ~(USART_SR_TC);
	// activate DMA channel transfer
	p_handle_usart->usart_dma.p_dma_stream_tx->CR |= DMA_SxCR_EN;

	return;
}

/*
 * Clear DMA status , it is required to use both polling and DMA in one program
 * @param[*p_handle_usart] - handler to usart struct
 * @return - void
 */
void USART_DMATransmitDoneCallback(USART_Handle_t *p_handle_usart)
{
	if (!(p_handle_usart->usart_dma.p_dma_stream_tx->CR & DMA_SxCR_CIRC))
	{
		p_handle_usart->tx_status = kUsartTxIdle;
	}
	return;
}
