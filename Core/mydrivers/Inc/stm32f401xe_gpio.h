/*
 * stm32f401xe_GPIO.h
 *
 *  Created on: Jan 12, 2022
 *      Author: ROJEK
 */

#ifndef MYDRIVERS_INC_STM32F401XE_GPIO_H_
#define MYDRIVERS_INC_STM32F401XE_GPIO_H_

#include "stm32f401xe.h"

// Struct to configure pin @PinConfig
typedef struct
{
	uint8_t PinNumber;			// @PinNumber

	uint8_t Mode;				// @GPIOModes

	uint8_t OutputType;			// @OutputType

	uint8_t OutputSpeed;		// @GPIOSpeed

	uint8_t PullUpPullDown;		// @GPIOPullUpPullDown

	uint8_t AF;					// @GPIOAlternatefunction

}GPIO_PinConfig_t;

typedef struct
{
	GPIO_TypeDef	*pGPIOx;	// @GPIOaddress

	GPIO_PinConfig_t PinConfig;	// @PinConfig

}GPIO_Handle_t;

// @PinNumber
#define GPIO_PIN_0					0U
#define GPIO_PIN_1					1U
#define GPIO_PIN_2					2U
#define GPIO_PIN_3					3U
#define GPIO_PIN_4					4U
#define GPIO_PIN_5					5U
#define GPIO_PIN_6					6U
#define GPIO_PIN_7					7U
#define GPIO_PIN_8					8U
#define GPIO_PIN_9					9U
#define GPIO_PIN_10					10U
#define GPIO_PIN_11					11U
#define GPIO_PIN_12					12U
#define GPIO_PIN_13					13U
#define GPIO_PIN_14					14U
#define GPIO_PIN_15					15U


// @GPIOModes
#define GPIO_PIN_MODE_INPUT			0U
#define GPIO_PIN_MODE_OUTPUT	 	1U
#define GPIO_PIN_MODE_AF			2U
#define GPIO_PIN_MODE_ANALOG		3U
#define GPIO_PIN_MODE_EXTI_FT		4U
#define GPIO_PIN_MODE_EXTI_RT		5U
#define GPIO_PIN_MODE_EXTI_FTRT		6U

// GPIO
#define GPIO_PIN_OT_PP				0U
#define GPIO_PIN_OT_OD				1U


// @GPIOSpeed
#define GPIO_PIN_SPEED_LOW			0U
#define GPIO_PIN_SPEED_MEDIUM		1U
#define GPIO_PIN_SPEED_HIGH			2U
#define GPIO_PIN_SPEED_VERYHIGH		3U

// @GPIOPullUpPullDown
#define GPIO_PIN_PUPD_NOPULL		0U
#define GPIO_PIN_PUPD_PULLUP		1U
#define GPIO_PIN_PUPD_PULLDOWN		2U

// @GPIOAlternatefunction
#define GPIO_PIN_AF_AF0				0U
#define GPIO_PIN_AF_AF1				1U
#define GPIO_PIN_AF_AF2				2U
#define GPIO_PIN_AF_AF3				3U
#define GPIO_PIN_AF_AF4				4U
#define GPIO_PIN_AF_AF5				5U
#define GPIO_PIN_AF_AF6				6U
#define GPIO_PIN_AF_AF7				7U
#define GPIO_PIN_AF_AF8				8U
#define GPIO_PIN_AF_AF9				9U
#define GPIO_PIN_AF_AF10			10U
#define GPIO_PIN_AF_AF11			11U
#define GPIO_PIN_AF_AF12			12U
#define GPIO_PIN_AF_AF13			13U
#define GPIO_PIN_AF_AF14			14U
#define GPIO_PIN_AF_AF15			15U

// @GPIOPinState
#define GPIO_PIN_RESET				1U
#define GPIO_PIN_SET				1U

// Get code to configure IRQ
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOH)?7:0)


void GPIO_InitPin(GPIO_Handle_t *hGPIO);
void GPIO_WritePin(GPIO_TypeDef *GPIO, uint8_t PinNumber, uint8_t PinState);
void GPIO_TogglePin(GPIO_TypeDef *GPIO, uint8_t PinNumber);
void GPIO_ClearPendingEXTIFlag(uint8_t PinNumber);

#endif /* MYDRIVERS_INC_STM32F401XE_GPIO_H_ */
