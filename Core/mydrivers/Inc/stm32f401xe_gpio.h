/*
 * stm32f401xe_GPIO.h
 *
 *  Created on: Jan 12, 2022
 *      Author: ROJEK
 */

#ifndef MYDRIVERS_INC_STM32F401XE_GPIO_H_
#define MYDRIVERS_INC_STM32F401XE_GPIO_H_

#include "stm32f401xe.h"

// @GPIOAlternatefunction
typedef enum
{
	kGpioAF0,
	kGpioAF1,
	kGpioAF2,
	kGpioAF3,
	kGpioAF4,
	kGpioAF5,
	kGpioAF6,
	kGpioAF7,
	kGpioAF8,
	kGpioAF9,
	kGpioAF10,
	kGpioAF11,
	kGpioAF12,
	kGpioAF13,
	kGpioAF14,
	kGpioAF015
} GpioAF_t;

// @PinNumber
typedef enum
{
	kGpioPin0,
	kGpioPin1,
	kGpioPin2,
	kGpioPin3,
	kGpioPin4,
	kGpioPin5,
	kGpioPin6,
	kGpioPin7,
	kGpioPin8,
	kGpioPin9,
	kGpioPin10,
	kGpioPin11,
	kGpioPin12,
	kGpioPin13,
	kGpioPin14,
	kGpioPin15
} GpioPinNumber_t;

// @GPIOModes
typedef enum
{
	kGpioModeInput, kGpioModeOutput, kGpioModeAF, kGpioModeAnalog
} GpioMode_t;

// @GPIO OutputType
typedef enum
{
	kGpioOTPushPull, kGpioOTOpenDrain
} GpioOutputType_t;

// @GPIOSpeed
typedef enum
{
	kGpioSpeedLow, kGpioSpeedMedium, kGpioSpeedHigh, kGpioSpeedVeryHigh
} GpioSpeed_t;

// @PUPD
typedef enum
{
	kGpioPUPDNoPull, kGpioPUPDPullup, kGPIOPUPDPulldown
} GpioPUPD_t;

typedef enum
{
	kGpioFallingTrig, kGpioRisingTrig, kGpioFallingRisingTrig
} GpioEXTITrigger_t;

#define GPIO_FLAG_PIN_0		(0x1U << 0U)
#define GPIO_FLAG_PIN_1		(0x1U << 1U)
#define GPIO_FLAG_PIN_2		(0x1U << 2U)
#define GPIO_FLAG_PIN_3		(0x1U << 3U)
#define GPIO_FLAG_PIN_4		(0x1U << 4U)
#define GPIO_FLAG_PIN_5		(0x1U << 5U)
#define GPIO_FLAG_PIN_6		(0x1U << 6U)
#define GPIO_FLAG_PIN_7		(0x1U << 7U)
#define GPIO_FLAG_PIN_8		(0x1U << 8U)
#define GPIO_FLAG_PIN_9		(0x1U << 9U)
#define GPIO_FLAG_PIN_10	(0x1U << 10U)
#define GPIO_FLAG_PIN_11	(0x1U << 11U)
#define GPIO_FLAG_PIN_12	(0x1U << 12U)
#define GPIO_FLAG_PIN_13	(0x1U << 13U)
#define GPIO_FLAG_PIN_14	(0x1U << 14U)
#define GPIO_FLAG_PIN_15	(0x1U << 15U)

// @GPIOPinState
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U

// Get code to configure IRQ
#define GPIO_BASEADDR_TO_CODE(x) \
  ((x == GPIOA)   ? 0            \
   : (x == GPIOB) ? 1            \
   : (x == GPIOC) ? 2            \
   : (x == GPIOD) ? 3            \
   : (x == GPIOE) ? 4            \
   : (x == GPIOH) ? 7            \
                  : 0)
// Init functions
void GPIO_InitClock(GPIO_TypeDef *GPIO);
void GPIO_ConfigBasic(GPIO_TypeDef *p_GPIOx, uint16_t pin_flags,
		GpioMode_t mode, GpioPUPD_t PUPD);
void GPIO_ConfigOutput(GPIO_TypeDef *p_GPIOx, uint16_t pin_flags,
		GpioOutputType_t output_type, GpioSpeed_t speed);
void GPIO_ConfigAF(GPIO_TypeDef *p_GPIOx, uint16_t pin_flags, GpioAF_t af);
void GPIO_ConfigEXTI(GPIO_TypeDef *p_GPIOx, GpioPinNumber_t pin,
		GpioEXTITrigger_t trigger);

// Action functions
void GPIO_WritePin(GPIO_TypeDef *p_GPIOx, GpioPinNumber_t pin, uint8_t pin_state);
void GPIO_TogglePin(GPIO_TypeDef *GPIO, GpioPinNumber_t pin);
uint8_t GPIO_ReadPin(GPIO_TypeDef *p_GPIOx, GpioPinNumber_t pin);
void GPIO_ClearPendingEXTIFlag(GpioPinNumber_t pin);

#endif /* MYDRIVERS_INC_STM32F401XE_GPIO_H_ */
