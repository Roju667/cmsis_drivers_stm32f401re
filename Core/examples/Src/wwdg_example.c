/*
 * wwdg_example.c
 *
 *  Created on: 6 lut 2022
 *      Author: pawel
 */

#include "wwdg_example.h"

#include <stdint.h>
#include <string.h>

#include "stdio.h"
#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"
#include "stm32f401xe_usart.h"
#include "stm32f401xe_wwdg.h"
#include "stm32f4xx.h"
#include "utilities.h"

// UART examples
// - measure a time that is calculated by systick [irq every 1ms]
// - measure a time that from watchdog timer
// - check program time duration at the end of every loop
void GPIOConfig(void);
void USART2Config(USART_Handle_t *p_usart2);
volatile uint32_t system_ticks = 0;

USART_Handle_t p_usart2;

void wwdg_example(void)
{
	uint8_t start_program, stop_program;
	uint8_t msg[64];
	uint32_t time_systick, time_wwdg;

	// config peripherals
	GPIOConfig();
	USART2Config(&p_usart2);
	PrintResetSource(&p_usart2);

	// set prescaler and window value (64 - 127)
	// if watchdog is fed earlier than window value then reset

	WWDG_SetBasicParameters(kWwdgPrescaler4, 127);
	// start watchdog and systick
	WWDG_StartWatchdog();
	SysTick_Config(16000);
	start_program = WWDG_GetWatchdog();

	while (1)
	{
		for (uint32_t i = 0; i < 10000; i++)
		{
		}
		GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		// after this loop calculate this loop duration from watchdog source and
		// from systick source
		stop_program = WWDG_GetWatchdog();
		time_systick = system_ticks;
		time_wwdg = (start_program - stop_program) * WWDG_GetTimePerWwdgTick();

		// print calculations on uart
		sprintf((char*) msg,
				"Systick : %ld [ms] ; WWDG : %ld [us] ; diff %ld [us] \n\r",
				time_systick, time_wwdg, (time_wwdg - (time_systick * 1000)));
		USART_Transmit(&p_usart2, msg, strlen((char*) msg));

		// reload watchdog and systick
		WWDG_ReloadWatchdog();
		system_ticks = 0;
	}
}

/*
 * configure LED on the board
 */
void GPIOConfig(void)
{
	GPIO_Handle_t GPIOx;
	GPIOx.PinConfig.Mode = GPIO_PIN_MODE_OUTPUT;
	GPIOx.PinConfig.PinNumber = GPIO_PIN_5;
	GPIOx.PinConfig.OutputType = GPIO_PIN_OT_PP;
	GPIOx.pGPIOx = GPIOA;

	GPIO_InitPin(&GPIOx);

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
	USART_SetBasicParameters(p_usart2, kUsartWordLenght8, kUsartStopBits0,
			kUsartNoParity);
	USART_SetBaudRate(p_usart2, 115200, kUsartOversampling16);
	USART_EnableIRQs(p_usart2, USART_CR1_IDLEIE, 0, USART_CR3_EIE);
}

void SysTick_Handler(void)
{
	system_ticks++;
}
