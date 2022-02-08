/*
 * utilities.c
 *
 *  Created on: 6 lut 2022
 *      Author: pawel
 */

#include "stm32f401xe_rcc.h"
#include "stm32f401xe_usart.h"
#include "utilities.h"

void PrintResetSource(USART_Handle_t *p_usartx)
{
	if (RCC->CSR & RCC_CSR_LPWRRSTF)
	{
		USART_Transmit(p_usartx, (uint8_t*) "Low-power reset \n\r", 16);
	}

	if (RCC->CSR & RCC_CSR_WWDGRSTF)
	{
		USART_Transmit(p_usartx, (uint8_t*) "WWDG reset \n\r", 16);
	}

	if (RCC->CSR & RCC_CSR_IWDGRSTF)
	{
		USART_Transmit(p_usartx, (uint8_t*) "IWDG reset \n\r", 16);
	}

	if (RCC->CSR & RCC_CSR_SFTRSTF)
	{
		USART_Transmit(p_usartx, (uint8_t*) "Software reset \n\r", 16);
	}

	if (RCC->CSR & RCC_CSR_PORRSTF)
	{
		USART_Transmit(p_usartx, (uint8_t*) "POR/PDR reset \n\r", 16);
	}

	if (RCC->CSR & RCC_CSR_PINRSTF)
	{
		USART_Transmit(p_usartx, (uint8_t*) "Pin reset \n\r", 16);
	}

	if (RCC->CSR & RCC_CSR_BORRSTF)
	{
		USART_Transmit(p_usartx, (uint8_t*) "BOR reset \n\r", 16);
	}

	RCC->CSR |= RCC_CSR_RMVF;

	return;
}
