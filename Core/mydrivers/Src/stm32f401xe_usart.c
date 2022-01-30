/*
 * stm32f401xe_usart.c
 *
 *  Created on: Jan 29, 2022
 *      Author: pawel
 */

#include "stm32f401xe_usart.h"
#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"
#include "stm32f401xe.h"
#include "math.h"

void Usart_InitGpioPins(USART_Handle_t *p_handle_usart)
{

	GPIO_Handle_t gpio_rx, gpio_tx;

	if (p_handle_usart->p_usartx == USART2)
	{

		// PB6 RX
		gpio_rx.pGPIOx = GPIOA;
		gpio_rx.PinConfig.PinNumber = GPIO_PIN_3;

		// PB7 TX
		gpio_tx.pGPIOx = GPIOA;
		gpio_tx.PinConfig.PinNumber = GPIO_PIN_2;

		// Alternate Function
		gpio_rx.PinConfig.AF = GPIO_PIN_AF_AF7;
		gpio_tx.PinConfig.AF = GPIO_PIN_AF_AF7;

	}

	// Mode AF
	gpio_rx.PinConfig.Mode = GPIO_PIN_MODE_AF;
	gpio_tx.PinConfig.Mode = GPIO_PIN_MODE_AF;

	// Output type open drain
	gpio_rx.PinConfig.OutputType = GPIO_PIN_OT_OD;
	gpio_tx.PinConfig.OutputType = GPIO_PIN_OT_OD;

	// Output speed very high
	gpio_rx.PinConfig.OutputSpeed = GPIO_PIN_SPEED_VERYHIGH;
	gpio_tx.PinConfig.OutputSpeed = GPIO_PIN_SPEED_VERYHIGH;

	// Pull ups
	gpio_rx.PinConfig.PullUpPullDown = GPIO_PIN_PUPD_NOPULL;
	gpio_tx.PinConfig.PullUpPullDown = GPIO_PIN_PUPD_NOPULL;

	GPIO_InitPin(&gpio_rx);
	GPIO_InitPin(&gpio_tx);

}

static void Usart_SetBaudDivider(USART_Handle_t *p_handle_usart)
{
	RCC_ClockFreqs clock_freqs;
	uint32_t pclk_freq;
	float baud_div;
	uint16_t baud_div_mantissa;
	uint8_t baud_div_fraction;

	// in equation we have 8 * (2 - OVER8)
	uint8_t usart_divmulti = 8 * (2 - (1U & (p_handle_usart->p_usartx->CR1 >> USART_CR1_OVER8_Pos)));
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

	baud_div = ((float) pclk_freq / (p_handle_usart->usart_config.baud_rate * usart_divmulti));
	baud_div_mantissa = baud_div;
	// rounding number to get correct fraction
	baud_div_fraction = round((baud_div - baud_div_mantissa) * usart_divmulti);
	// if after rounding we have value bigger or equal to oversampling then we need to carry
	// +1 to mantissa and set fraction to 0. like 50,99 -> 51;
	if (baud_div_fraction >= usart_divmulti)
	{
		baud_div_fraction = 0;
		baud_div_mantissa++;
	}

	p_handle_usart->p_usartx->BRR = 0;
	p_handle_usart->p_usartx->BRR |= (baud_div_mantissa << USART_BRR_DIV_Mantissa_Pos);
	p_handle_usart->p_usartx->BRR |= (baud_div_fraction << USART_BRR_DIV_Fraction_Pos);
}

void Usart_Transmit(USART_Handle_t *p_handle_usart, uint8_t *p_data_buffer, uint32_t data_lenght)
{

	uint32_t tx_data_to_send = data_lenght;
//	1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
	p_handle_usart->p_usartx->CR1 |= USART_CR1_UE;
//	2. Program the M bit in USART_CR1 to define the word length.
	p_handle_usart->p_usartx->CR1 |= (p_handle_usart->usart_config.word_lenght << USART_CR1_M_Pos);
//	3. Program the number of stop bits in USART_CR2.
	p_handle_usart->p_usartx->CR1 |= (p_handle_usart->usart_config.stop_bits << USART_CR2_STOP_Pos);
//	4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
//	place. Configure the DMA register as explained in multibuffer communication.
//	5. Select the desired baud rate using the USART_BRR register.
	Usart_SetBaudDivider(p_handle_usart);
//	6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
	p_handle_usart->p_usartx->CR1 |= USART_CR1_TE;
//	7. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
//	for each data to be transmitted in case of single buffer.
	while (tx_data_to_send > 0)
	{
		// wait until data register is empty
		while (!(p_handle_usart->p_usartx->SR & USART_SR_TXE))
			;

		// put data in data register
		p_handle_usart->p_usartx->DR = p_data_buffer[data_lenght - tx_data_to_send];

		//change counter
		tx_data_to_send--;

		//	8. After writing the last data into the USART_DR register, wait until TC=1. This indicates
		//	that the transmission of the last frame is complete. This is required for instance when
		//	the USART is disabled or enters the Halt mode to avoid corrupting the last
		//	transmission
		if (tx_data_to_send == 0)
		{
			// check if data transfer is finsihed
			while (!(p_handle_usart->p_usartx->SR & USART_SR_TC))
				;

		}

	}

}
