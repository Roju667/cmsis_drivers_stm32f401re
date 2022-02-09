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
 * enable RCC Clock
 */
static void USART_ClockEnable(USART_Handle_t *p_handle_usart)
{
	if (p_handle_usart->p_usartx == USART1)
	{
		RCC_CLOCK_USART1_ENABLE();
	}
	else if (p_handle_usart->p_usartx == USART2)
	{
		RCC_CLOCK_USART2_ENABLE();
	}
	else if (p_handle_usart->p_usartx == USART6)
	{
		RCC_CLOCK_USART6_ENABLE();
	}
}

/*
 * init gpio pins
 */
static void USART_InitGpioPins(USART_Handle_t *p_handle_usart)
{
//	GPIO_Handle_t gpio_rx, gpio_tx;
//
//	if (p_handle_usart->p_usartx == USART2)
//	{
//		// PB6 RX
//		gpio_rx.pGPIOx = GPIOA;
//		gpio_rx.PinConfig.PinNumber = GPIO_PIN_3;
//
//		// PB7 TX
//		gpio_tx.pGPIOx = GPIOA;
//		gpio_tx.PinConfig.PinNumber = GPIO_PIN_2;
//
//		// Alternate Function
//		gpio_rx.PinConfig.AF = GPIO_PIN_AF_AF7;
//		gpio_tx.PinConfig.AF = GPIO_PIN_AF_AF7;
//	}
//
//	// Mode AF
//	gpio_rx.PinConfig.Mode = GPIO_PIN_MODE_AF;
//	gpio_tx.PinConfig.Mode = GPIO_PIN_MODE_AF;
//
//	// Output type open drain
//	gpio_rx.PinConfig.OutputType = GPIO_PIN_OT_OD;
//	gpio_tx.PinConfig.OutputType = GPIO_PIN_OT_OD;
//
//	// Output speed very high
//	gpio_rx.PinConfig.OutputSpeed = GPIO_PIN_SPEED_VERYHIGH;
//	gpio_tx.PinConfig.OutputSpeed = GPIO_PIN_SPEED_VERYHIGH;
//
//	// Pull ups
//	gpio_rx.PinConfig.PullUpPullDown = GPIO_PIN_PUPD_NOPULL;
//	gpio_tx.PinConfig.PullUpPullDown = GPIO_PIN_PUPD_NOPULL;
//
//	GPIO_InitPin(&gpio_rx);
//	GPIO_InitPin(&gpio_tx);
}
/*
 * calculate and write in baud rate divider
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
 * use CRx_IRQ_FLAGS to enable all the interrupts in register, otherwise use
 * like : (USART_CR1_PEIE | USART_CR1_TXEIE)
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
 * basic init function
 */
void USART_SetBasicParameters(USART_Handle_t *p_handle_usart,
		UsartWordLenght_t word_lenght, UsartStopBits_t stop_bits,
		UsartParity_t parity)
{
	USART_InitGpioPins(p_handle_usart);
	USART_ClockEnable(p_handle_usart);

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
 * blocking receive UART function
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
 * configuration that you have call only once for receive DMA function,
 * then you only have to call Start function, configure data_buffer1 to NULL,
 * in case of using double buffer then configure it with pointer
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
 * start DMA channel that wait to transfer from UART DR register to
 * destination
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
 * clear dma rx status to idle after finished dma transmission
 * if you would like to send data on uart sometimes by dma
 * and sometimes by polling - put this in DMAx_Irq handler
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
 * Blocking transmit UART function
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
 * similar to configure ReceiveDMA, put NULL as p_data_buffer1 in
 * case of not using double buffer
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
 * start DMA channel that transfers from memory to UART DR
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
 * clear dma tx status to idle after finished dma transmission
 * if you would like to send data on uart sometimes by dma
 * and sometimes by polling - put this in DMAx_Irq handler
 */
void USART_DMATransmitDoneCallback(USART_Handle_t *p_handle_usart)
{
	if (!(p_handle_usart->usart_dma.p_dma_stream_tx->CR & DMA_SxCR_CIRC))
	{
		p_handle_usart->tx_status = kUsartTxIdle;
	}
	return;
}
