/*
 * i2c_bmp180_example.c
 *
 *  Created on: 8 lut 2022
 *      Author: pawel
 */


#include "i2c_bmp180_example.h"

#include <stdint.h>
#include <string.h>

#include "stdio.h"
#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"
#include "stm32f401xe_usart.h"
#include "stm32f401xe_wwdg.h"
#include "stm32f4xx.h"
#include "bmp180.h"
#include "utilities.h"

void GPIOConfig(void);
void USART2Config(USART_Handle_t *p_usart2);
void I2C1Config(I2c_Handle_t *p_i2c1);

USART_Handle_t p_usart2;
I2c_Handle_t p_i2c1;
bmp180_t p_bmp180;

void i2c_bmp180_example(void)
{

	// config peripherals
	GPIOConfig();
	USART2Config(&p_usart2);
	I2C1Config(&p_i2c1);

	bmp180_init(&p_bmp180, &p_i2c1);
	bmp180_get_ut(&p_bmp180);
	bmp180_get_temp(&p_bmp180);

	while (1)
	{
		for (uint32_t i = 0; i < 100000; i++)
		{
		}
		GPIO_TogglePin(GPIOA, GPIO_PIN_5);

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

void I2C1Config(I2c_Handle_t *p_i2c1)
{
	p_i2c1->p_i2cx = I2C1;
	I2C_SetBasicParameters(p_i2c1, kI2cSpeedFastDuty0);
}
