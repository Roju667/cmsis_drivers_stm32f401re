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
	I2C1Config(&p_i2c1);
	GPIOConfig();
	USART2Config(&p_usart2);


	bmp180_init(&p_bmp180, &p_i2c1);
	p_bmp180.uncomp.temp = bmp180_get_ut(&p_bmp180);
	p_bmp180.data.temp = bmp180_get_temp(&p_bmp180);

	while (1)
	{
		for (uint32_t i = 0; i < 100000; i++)
		{
		}
		GPIO_TogglePin(GPIOA, kGpioPin5);
		p_bmp180.uncomp.temp = bmp180_get_ut(&p_bmp180);
		p_bmp180.data.temp = bmp180_get_temp(&p_bmp180);
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

void I2C1Config(I2c_Handle_t *p_i2c1)
{
	p_i2c1->p_i2cx = I2C1;
	I2C_InitClock(p_i2c1);
	GPIO_InitClock(GPIOB);
	I2C_InitGpioPins(p_i2c1);
	I2C_SetBasicParameters(p_i2c1, kI2cSpeedFastDuty0);
	return;
}
