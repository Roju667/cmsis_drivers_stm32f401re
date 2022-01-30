/*
 * stm32f401xe_pwr.c
 *
 *  Created on: 27 sty 2022
 *      Author: ROJEK
 */

#include "stm32f401xe_pwr.h"
#include "stm32f401xe_rcc.h"

/*
 * Configure power voltage detection
 * @param[pvd_level] - enter voltage level that ha to be checked
 * @param[mode] - pvdo can be checked by user by polling or can SET IRQ from EXTI line 16
 */
void Pwr_EnablePvd(PvdThresholdLevel_t pvd_level, PvdMode_t mode)
{
	// enable power regulator bit
	PWR->CR |= PWR_CR_PVDE;

	// modify power threshold value
	PWR->CR &= ~(PWR_CR_PLS);
	PWR->CR |= (pvd_level << PWR_CR_PLS_Pos);

	// enable IRQ or not
	if (kPvdModeNormal)
	{
		return;
	}
	else
	{
		NVIC_EnableIRQ(PVD_IRQn);
		EXTI->IMR |= EXTI_IMR_MR16;

		if (mode == kPvdModeIrqRT)
		{
			EXTI->FTSR |= EXTI_FTSR_TR16;
		}

		if (mode == kPvdModeIrqFT)
		{
			EXTI->RTSR |= EXTI_RTSR_TR16;
		}
	}

}

/*
 * Enter sleep mode
 * @param[exit] - enter sleep now [kWFI/kWFE] or enter after exiting ISR [kSleepOnExit]
 */
void Pwr_EnterSleepMode(PwrExit_t exit)
{
	// deselect deep sleep mode
	SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);

	// select exit mechanism
	if (exit == kWFI)
	{
		__WFI();
	}

	if (exit == kWFE)
	{
		__WFE();
	}

	if (exit == kSleepOnExit)
	{
		SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
	}
}

/*
 * Enter stop mode
 * @param[exit] - enter sleep now [kWFI/kWFE] or enter after exiting ISR [kSleepOnExit]
 * @param[stop_mode] - stop mode configuration described in RM PWR chapter
 */
void Pwr_EnterStopMode(PwrExit_t exit, StopModes_t stop_mode)
{
	//select deep sleep mode
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	//select between stop and standby mode
	PWR->CR &= ~(PWR_CR_PDDS);

	//set parameters for stop mode
	PWR->CR &= ~(PWR_CR_MRLVDS);
	PWR->CR &= ~(PWR_CR_LPLVDS);
	PWR->CR &= ~(PWR_CR_FPDS);
	PWR->CR &= ~(PWR_CR_LPDS);

	switch (stop_mode)
	{
	case (kStopMR):
		break;
	case (kStopMRFPD):
		PWR->CR |= PWR_CR_FPDS;
		break;
	case (kStopLP):
		PWR->CR |= PWR_CR_LPDS;
		break;
	case (kStopLPFPD):
		PWR->CR |= PWR_CR_LPDS | PWR_CR_FPDS;
		break;
	case (kStopMRLV):
		PWR->CR |= PWR_CR_MRLVDS;
		break;
	case (kStopLPLV):
		PWR->CR |= PWR_CR_LPDS | PWR_CR_LPLVDS;
		break;
	default:
		return;
	}

	if (exit == kWFI)
	{
		__WFI();
	}

	if (exit == kWFE)
	{
		__WFE();
	}

	if (exit == kSleepOnExit)
	{
		SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
	}
}

/*
 * Enter standby mode
 * @param[exit] - enter sleep now [kWFI/kWFE] or enter after exiting ISR [kSleepOnExit]
 * RTC has to be configured (alarm/tamper/timestamp) - come back here after RTC deepdive
 */
void Pwr_EnterStandbyMode(PwrExit_t exit)
{
	// enable wake up pin
	PWR->CSR |= PWR_CSR_EWUP;
	//select deep sleep mode
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	//select between stop and standby mode
	PWR->CR |= PWR_CR_PDDS;
	// bit is cleared in Power Control/Status register
	PWR->CR |= PWR_CR_CWUF;


	if (exit == kWFI)
	{
		__WFI();
	}

	if (exit == kWFE)
	{
		__WFE();
	}

	if (exit == kSleepOnExit)
	{
		SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
	}
}

/*
 * Backup registers are retained even when main power is off and mcu is only on battery supply
 * @param[p_data_buffer] - data that we want to save in backup registers
 * @param[data_len] - maximum 20x uint32
 */
void Pwr_WriteToBackupRegister(uint32_t *p_data_buffer, uint8_t data_len)
{
	// error check
	if (data_len > 20)
	{
		return;
	}

	// disable backup domain write protection
	PWR->CR |= PWR_CR_DBP;

	// RTC clock selection
	RCC->BDCR &= ~(RCC_BDCR_RTCSEL);
	RCC->BDCR |= RCC_BDCR_RTCSEL_1; //LSI

	// if using HSE as clock source prescaler has to be configure to ensure 1Mhz
	//RCC->CFGR &= ~(RCC_CFGR_RTCPRE);
	//RCC->CFGR |= RCC_CFGR_RTCPRE_1;

	//enable RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;

	uint32_t *p_temp_data;
	for (uint8_t i = 0; i < data_len; i++)
	{
		p_temp_data = &(RTC->BKP0R) + i;
		*p_temp_data = p_data_buffer[i];
	}
}
