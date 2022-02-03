/*
 * stm32f401xe_RCC.c
 *
 *  Created on: Jan 13, 2022
 *      Author: ROJEK
 */

#include "stm32f401xe_rcc.h"

/*
 * Configure clock
 * 1. Enable HSI/HSE as main source RCC->CR
 * 2. Set regulator scaling PWR->VOS if necessary before PLL is ON
 * 3. Configure Flash and Latency settings
 * 4. Configure HCLK, PCLK1, PCLK2
 * 5. Configure PLL
 * 6. Enable the PLL
 * 7. Select system clock
 * @param[*GPIO] - base address of gpiox peripheral
 * @return - void
 */
uint8_t RCC_InitClock(RCC_ClockInitTypeDef *p_clock_init)
{
	/* 1. Enable HSI/HSE as main source RCC->CR */

	// enable HSI/HSE
	if (p_clock_init->oscillator_type == RCC_OSCILLATORTYPE_HSI)
	{
		// set HSI flag
		RCC->CR |= RCC_CR_HSION;
		// wait until HSI is on
		while (!(RCC->CR & RCC_CR_HSIRDY))
			;
	}
	else if (p_clock_init->oscillator_type == RCC_OSCILLATORTYPE_HSE)
	{
		// set HSE flag
		RCC->CR |= RCC_CR_HSEON;
		// wait until HSE is on
		while (!(RCC->CR & RCC_CR_HSERDY))
			;
	}

	/* 2. Set regulator scaling PWR->VOS if necessary before PLL is ON */
	PWR->CR &= ~(PWR_CR_VOS);
	PWR->CR |= (p_clock_init->vos_scale << PWR_CR_VOS_Pos);

	/* 3. Configure Flash and Latency settings */
	// enable data, instruction and prefetch cache
	FLASH->ACR |= (FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN);
	// to read correct from flash at certain voltage and frequency latency between
	// read has to be set correctly this table can be found in reference manual
	FLASH->ACR &= ~(FLASH_ACR_LATENCY);
	FLASH->ACR |= (p_clock_init->flash_latency << FLASH_ACR_LATENCY_Pos);

	/* 4. Configure HCLK, PCLK1, PCLK2 */
	// configure AHB prescaler
	RCC->CFGR &= ~(RCC_CFGR_HPRE);
	RCC->CFGR |= (p_clock_init->hpre_prescaler << RCC_CFGR_HPRE_Pos);

	// configure APB1 prescaler
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);
	RCC->CFGR |= (p_clock_init->apb1_prescaler << RCC_CFGR_PPRE1_Pos);

	// configure APB2 prescaler
	RCC->CFGR &= ~(RCC_CFGR_PPRE2);
	RCC->CFGR |= (p_clock_init->apb2_prescaler << RCC_CFGR_PPRE2_Pos);

	/* 5. Configure PLL */

	// PLL - phase-locked loop - clock generation engine in the MCU
	// if PLL is enable
	if (p_clock_init->pll.state == RCC_PLL_STATE_ENABLE)
	{
		// init PLL source
		RCC->PLLCFGR |= (p_clock_init->pll.source << RCC_PLLCFGR_PLLSRC_Pos);

		// PLLM - division factor for the main PLL
		// It is recommended to put source frequency in range of 1 - 2Mhz (2Mhz
		// recommended) Frequency after this division operation is input of VCO
		// (voltage controlled oscillator)

		if (p_clock_init->pll.pllm < 3 || p_clock_init->pll.pllm > 63)
		{
			return PLL_CONFIG_ERROR;
		}
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
		RCC->PLLCFGR |= (p_clock_init->pll.pllm << RCC_PLLCFGR_PLLM_Pos);

		// PLLN - multiplication factor for VCO
		// After this operation value should be between 192 and 432 MHz
		// Frequency after this operation is output of VCO

		if (p_clock_init->pll.plln < 2 || p_clock_init->pll.plln > 432)
		{
			return PLL_CONFIG_ERROR;
		}
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
		RCC->PLLCFGR |= (p_clock_init->pll.plln << RCC_PLLCFGR_PLLN_Pos);

		// PLLP - division factor for main system clock
		// After this operation values should not exceed 84MHz
		// Frequency after this operation is frequency of main system clock

		// 2 bits register so value can not be more than 3
		if (p_clock_init->pll.pllp > 3)
		{
			return PLL_CONFIG_ERROR;
		}
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
		RCC->PLLCFGR |= (p_clock_init->pll.pllp << RCC_PLLCFGR_PLLP_Pos);

		// PLLQ - ) division factor for USB OTG FS, SDIO and random number generator
		// clocks The USB OTG FS requires a 48 MHz clock to work correctly. The SDIO
		// and the random number generator need a frequency lower than or equal to
		// 48 MHz to work correctly

		if (p_clock_init->pll.pllq < 2 || p_clock_init->pll.pllq > 15)
		{
			return PLL_CONFIG_ERROR;
		}
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ);
		RCC->PLLCFGR |= (p_clock_init->pll.pllq << RCC_PLLCFGR_PLLQ_Pos);

		/* 6. Enable the PLL */
		RCC->CR |= (RCC_PLL_STATE_ENABLE << RCC_CR_PLLON_Pos);

		// wait until PLL is ready
		while (!(RCC->CR & RCC_CR_PLLRDY))
			;
		/* 7. Select system clock switch */
		// set PLL as system clock
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
			;

		return 0;
	}
	else
	{
		/* 6. Disable the PLL */
		RCC->CR = (RCC_PLL_STATE_DISABLE << RCC_CR_PLLON_Pos);

		// wait until PLL is disables
		while (!(RCC->CR & RCC_CR_PLLRDY))
			;
		/* 7. Select system clock switch */
		// set HSI or HE as main system clock
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= (p_clock_init->oscillator_type << RCC_CFGR_SW_Pos);

		// wait until HSI/HSE is set as a clock source
		while ((RCC->CFGR & (p_clock_init->oscillator_type << RCC_CFGR_SW_Pos)) !=
		RCC_CFGR_SWS_PLL)
			;
	}

	return 0;
}

static uint32_t RCC_CalculatePllclk(uint32_t source)
{
	uint16_t pllm = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
	;
	uint16_t plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
	uint16_t pllp = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos;
	;

	// pllp register has 2 bits - 00 = 2; 01 = 4; 10 = 6; 11 = 8
	pllp = (pllp * 2) + 2;

	return (source / pllm) * plln / pllp;
}

static uint16_t RCC_GetApbPrescaler(uint8_t bitvalue)
{
	uint32_t apb_prescaler;

	// convert bit code to prescaler value
	switch (bitvalue)
	{
	case (RCC_ABP_PRESCALER_NODIV):
		apb_prescaler = 1;
		break;
	case (RCC_ABP_PRESCALER_DIV2):
		apb_prescaler = 2;
		break;
	case (RCC_ABP_PRESCALER_DIV4):
		apb_prescaler = 4;
		break;
	case (RCC_ABP_PRESCALER_DIV8):
		apb_prescaler = 8;
		break;
	case (RCC_ABP_PRESCALER_DIV16):
		apb_prescaler = 16;
		break;
	}

	return apb_prescaler;
}

static uint16_t RCC_GetAhbPrescaler(uint8_t bitvalue)
{
	uint32_t ahb_prescaler;

	// convert bit code to prescaler value
	switch (bitvalue)
	{
	case (RCC_HPRE_PRESCALER_NODIV):
		ahb_prescaler = 1;
		break;
	case (RCC_HPRE_PRESCALER_DIV2):
		ahb_prescaler = 2;
		break;
	case (RCC_HPRE_PRESCALER_DIV4):
		ahb_prescaler = 4;
		break;
	case (RCC_HPRE_PRESCALER_DIV8):
		ahb_prescaler = 8;
		break;
	case (RCC_HPRE_PRESCALER_DIV16):
		ahb_prescaler = 16;
		break;
	case (RCC_HPRE_PRESCALER_DIV64):
		ahb_prescaler = 64;
		break;
	case (RCC_HPRE_PRESCALER_DIV128):
		ahb_prescaler = 128;
		break;
	case (RCC_HPRE_PRESCALER_DIV256):
		ahb_prescaler = 256;
		break;
	case (RCC_HPRE_PRESCALER_DIV512):
		ahb_prescaler = 512;
		break;
	}

	return ahb_prescaler;
}

uint32_t RCC_GetSysclk(void)
{
	// calculate sysclk depending on source
	switch (RCC->CFGR & RCC_CFGR_SWS)
	{
	case (RCC_CFGR_SWS_HSI):
		return RCC_HSI_FREQUENCY;
		break;

	case (RCC_CFGR_SWS_HSE):
		return RCC_HSE_FREQUENCY;
		break;

	case (RCC_CFGR_SWS_PLL):

		switch (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)
		{
		case (RCC_PLLCFGR_PLLSRC_HSI):
			return RCC_CalculatePllclk(RCC_HSI_FREQUENCY);
			break;

		case (RCC_PLLCFGR_PLLSRC_HSE):
			return RCC_CalculatePllclk(RCC_HSE_FREQUENCY);
			break;
		}
		break;
	}

	return 0;
}

uint32_t RCC_GetHclk(void)
{
	uint32_t sysclk = RCC_GetSysclk();
	uint16_t ahb_prescaler = RCC_GetAhbPrescaler((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos);
	return sysclk / ahb_prescaler;
}

uint32_t RCC_GetPclk(uint8_t plck_x)
{
	uint32_t hclk = RCC_GetHclk();
	uint8_t apb_prescaler;
	switch (plck_x)
	{
	case (1):
		apb_prescaler = RCC_GetApbPrescaler((RCC->CFGR & RCC_CFGR_PPRE1) >>
		RCC_CFGR_PPRE1_Pos);
		break;

	case (2):
		apb_prescaler = RCC_GetApbPrescaler((RCC->CFGR & RCC_CFGR_PPRE2) >>
		RCC_CFGR_PPRE2_Pos);
		break;
	}

	return hclk / apb_prescaler;
}

void RCC_GetClockFrequencies(RCC_ClockFreqs *freqs)
{
	freqs->sysclk = RCC_GetSysclk();
	freqs->hclk = RCC_GetHclk();
	freqs->pclk1 = RCC_GetPclk(1);
	freqs->pclk2 = RCC_GetPclk(2);

	return;
}
