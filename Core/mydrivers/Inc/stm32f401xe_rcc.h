/*
 * stm32f401xe_RCC.h
 *
 *  Created on: Jan 13, 2022
 *      Author: ROJEK
 */

#ifndef MYDRIVERS_INC_STM32F401XE_RCC_H_
#define MYDRIVERS_INC_STM32F401XE_RCC_H_

#include "stm32f401xe.h"
/*
 * If we are using PLL as main clock source of system clock then we need to
 * calculate it with equation : SYSCLK = PLLSource / M  * N	/ P (this value
 * is then going on AHB and ABP prescalers Clock for OTG FS, SDIO and RNG =
 * PLLSource /M *N /Q (straight to those peripherals)
 *
 * @PLLState The new state of the PLL
 *
 * @PLLSource - HSI or HSE
 *
 * @PLLM: Division factor for PLL VCO input clock. Range 2 - 63 he software has
 * to set these bits correctly to ensure that the VCO input frequency ranges
 * from 1 to 2 MHz.
 *
 * @PLLN: Multiplication factor for PLL VCO output clock. This parameter must be
 * a number between Min_Data = 50 and Max_Data = 432 except for STM32F411xE
 * devices where the Min_Data = 192
 *
 * @PLLP: Division factor for main system clock (SYSCLK)
 *
 * @PLLQ: Division factor for OTG FS, SDIO and RNG clocks. This parameter must
 * be a number between Min_Data = 2 and Max_Data = 15
 *
 */

/*
 * Structure to hold PLL parameters
 */
typedef struct RCC_PLLInitTypeDef
{
	int8_t state;  //@PLLState

	uint8_t source;  //@PLLSource

	uint8_t pllm;  //@PLLM

	uint16_t plln;  //@PLLN

	uint8_t pllp;  //@PLLP

	uint8_t pllq;  //@PLLQ

} RCC_PLLInitTypeDef;

/*
 * Structure to hold Clock parameters
 */
typedef struct RCC_ClockInitTypeDef
{
	uint8_t oscillator_type;  //@OscillatorType

	uint8_t vos_scale;  //@VOSScale

	uint8_t flash_latency;  //@FlashLatency

	uint8_t hpre_prescaler;  //@AHB_Prescaler

	uint8_t apb1_prescaler;  //@APB1_Prescaler

	uint8_t apb2_prescaler;  //@APB2_Prescaler

	RCC_PLLInitTypeDef pll;

} RCC_ClockInitTypeDef;

// clock frequencies struct
typedef struct RCC_ClockFreqs
{
	uint32_t sysclk;

	uint32_t hclk;

	uint32_t pclk1;

	uint32_t pclk2;

} RCC_ClockFreqs;

#define RCC_HSI_FREQUENCY 16000000U
#define RCC_HSE_FREQUENCY 0U
#define RCC_LSI_FREQUENCY 32000U
#define RCC_LSE_FRQEUENCY 0U
/*
 HSI/HSE/LSI/LSE
 @OscillatorType
 */
#define RCC_OSCILLATORTYPE_HSI 0U
#define RCC_OSCILLATORTYPE_HSE 1U

/*
 Main PLL(PLL) and audio PLL (PLLI2S) entry clock source
 @PLLSource
 */
#define RCC_PLL_SOURCE_HSI 0U
#define RCC_PLL_SOURCE_HSE 1U

/*
 Main PLL (PLL) enable
 Set and cleared by software to enable PLL.
 @PLLState
 */
#define RCC_PLL_STATE_DISABLE 0U
#define RCC_PLL_STATE_ENABLE 1U

/*
 PLLP: Main PLL (PLL) division factor for main system clock
 @PLLP
 */
#define RCC_PLLP_DIV2 0U
#define RCC_PLLP_DIV4 1U
#define RCC_PLLP_DIV6 2U
#define RCC_PLLP_DIV8 3U

/*
 System clock switch
 Set and cleared by software to select the system clock source.
 Set by hardware to force the HSI selection when leaving the Stop or Standby
 mode or in case of failure of the HSE oscillator used directly or indirectly as
 the system clock.
 */
#define RCC_SW_HSI 0U
#define RCC_SW_HSE 1U
#define RCC_SW_PLL 2U

/*
 @AHB_Prescaler
 SYSYCLOCK/Prescaler = HCLK
 */
#define RCC_HPRE_PRESCALER_NODIV 0U
#define RCC_HPRE_PRESCALER_DIV2 8U
#define RCC_HPRE_PRESCALER_DIV4 9U
#define RCC_HPRE_PRESCALER_DIV8 10U
#define RCC_HPRE_PRESCALER_DIV16 11U
#define RCC_HPRE_PRESCALER_DIV64 12U
#define RCC_HPRE_PRESCALER_DIV128 13U
#define RCC_HPRE_PRESCALER_DIV256 14U
#define RCC_HPRE_PRESCALER_DIV512 15U

/*
 @APB1_Prescaler
 APB Low speed prescaler (APB1)
 Set and cleared by software to control APB low-speed clock division factor

 @APB2_Prescaler
 APB high-speed prescaler (APB2)
 Set and cleared by software to control APB high-speed clock division factor
 */

#define RCC_ABP_PRESCALER_NODIV 0U
#define RCC_ABP_PRESCALER_DIV2 4U
#define RCC_ABP_PRESCALER_DIV4 5U
#define RCC_ABP_PRESCALER_DIV8 6U
#define RCC_ABP_PRESCALER_DIV16 7U

/*
 @VOSScale
 These bits control the main internal voltage regulator output voltage to
 achieve a trade-off between performance and power consumption. This should be
 assigned to PWR_register Scale 2 when 60MHz < HCLK < 84Mhz -> regulator voltage
 ~1.26V Scale 3 when HCLK < 60Mhz -> regulator voltage ~1.14V
 */
#define RCC_VOS_SCALE3 1U
#define RCC_VOS_SCALE2 2U

/*
 @FlashLatency
 These bits represent the ratio of the CPU clock period to the Flash memory
 access time.
 */
#define RCC_FLASHLATENCY_0WS 0U
#define RCC_FLASHLATENCY_1WS 1U
#define RCC_FLASHLATENCY_2WS 2U
#define RCC_FLASHLATENCY_3WS 3U
#define RCC_FLASHLATENCY_4WS 4U
#define RCC_FLASHLATENCY_5WS 5U
#define RCC_FLASHLATENCY_6WS 6U

/*
 Errors
 */
#define PLL_CONFIG_ERROR 1U

/*
 Clock enable macros
 */
#define RCC_CLOCK_GPIOA_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
#define RCC_CLOCK_GPIOB_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN
#define RCC_CLOCK_GPIOC_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN
#define RCC_CLOCK_GPIOD_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN
#define RCC_CLOCK_GPIOE_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN
#define RCC_CLOCK_GPIOH_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN

#define RCC_CLOCK_TIM1_ENABLE() RCC->APB2ENR |= RCC_APB2ENR_TIM1EN
#define RCC_CLOCK_TIM2_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_TIM2EN
#define RCC_CLOCK_TIM3_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
#define RCC_CLOCK_TIM4_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_TIM4EN
#define RCC_CLOCK_TIM5_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_TIM5EN

#define RCC_CLOCK_TIM9_ENABLE() RCC->APB2ENR |= RCC_APB2ENR_TIM9EN
#define RCC_CLOCK_TIM10_ENABLE() RCC->APB2ENR |= RCC_APB2ENR_TIM10EN
#define RCC_CLOCK_TIM11_ENABLE() RCC->APB2ENR |= RCC_APB2ENR_TIM11EN

#define RCC_CLOCK_DMA1_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN
#define RCC_CLOCK_DMA2_ENABLE() RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN

#define RCC_CLOCK_I2C1_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_I2C1EN
#define RCC_CLOCK_I2C2_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_I2C2EN
#define RCC_CLOCK_I2C3_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_I2C3EN

#define RCC_CLOCK_USART1_ENABLE() RCC->APB2ENR |= RCC_APB2ENR_USART1EN
#define RCC_CLOCK_USART2_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_USART2EN
#define RCC_CLOCK_USART6_ENABLE() RCC->APB2ENR |= RCC_APB2ENR_USART6EN

#define RCC_CLOCK_WWDG_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
#define RCC_CLOCK_PWR_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_PWREN

/*
 Clock disable macros
 */
#define RCC_CLOCK_GPIOA_DISABLE() RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN)
#define RCC_CLOCK_GPIOB_DISABLE() RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOBEN)
#define RCC_CLOCK_GPIOC_DISABLE() RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOCEN)
#define RCC_CLOCK_GPIOD_DISABLE() RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIODEN)
#define RCC_CLOCK_GPIOE_DISABLE() RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOEEN)
#define RCC_CLOCK_GPIOH_DISABLE() RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOHEN)

#define RCC_CLOCK_TIM1_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_TIM1EN)
#define RCC_CLOCK_TIM2_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN)
#define RCC_CLOCK_TIM3_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN)
#define RCC_CLOCK_TIM4_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_TIM4EN)
#define RCC_CLOCK_TIM5_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_TIM5EN)

#define RCC_CLOCK_TIM9_DISABLE() RCC->APB2ENR &= ~(RCC_APB2ENR_TIM9EN)
#define RCC_CLOCK_TIM10_DISABLE() RCC->APB2ENR &= ~(RCC_APB2ENR_TIM10EN)
#define RCC_CLOCK_TIM11_DISABLE() RCC->APB2ENR &= ~(RCC_APB2ENR_TIM11EN)

#define RCC_CLOCK_I2C1_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_I2C1EN)
#define RCC_CLOCK_I2C2_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_I2C2EN)
#define RCC_CLOCK_I2C3_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_I2C3EN)

#define RCC_CLOCK_PWR_DISABLE() RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN)

/*
 * Clear reset flags
 */
#define CLEAR_RESET_FLAGS() RCC->CSR |= RCC_CSR_RMVF

uint8_t RCC_InitClock(RCC_ClockInitTypeDef *pClockInit);
uint32_t RCC_GetHclk(void);
uint32_t RCC_GetSysclk(void);
void RCC_GetClockFrequencies(RCC_ClockFreqs* freqs);
#endif /* MYDRIVERS_INC_STM32F401XE_RCC_H_ */
