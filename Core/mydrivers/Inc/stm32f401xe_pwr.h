/*
 * stm32401xe_pwr.h
 *
 *  Created on: 27 sty 2022
 *      Author: ROJEK
 */
#include "stm32f401xe.h"

#ifndef MYDRIVERS_INC_STM32F401XE_PWR_H_
#define MYDRIVERS_INC_STM32F401XE_PWR_H_

typedef enum PvdThresholdLevel_t
{
	kPvdLevel22V, kPvdLevel23V, kPvdLevel24V, kPvdLevel25V, kPvdLevel26V, kPvdLevel27V, kPvdLevel28V, kPvdLevel29V

} PvdThresholdLevel_t;

typedef enum PvdMode_t
{
	kPvdModeNormal, kPvdModeIrqRT, kPvdModeIrqFT
} PvdMode_t;

typedef enum PwrExit_t
{
	kWFI, kWFE, kSleepOnExit
} PwrExit_t;

// there are multiple choices for what has to be disabled during stop mode
// bits that are set/reset
// MRLV - main regulator low voltage
// LPLV - low-power regulator low voltage
// FPDS - flash power-down in stop mode
// LPDS - low-power deep sleep

typedef enum StopModes_t
{
	kStopMR, kStopMRFPD, kStopLP, kStopLPFPD, kStopMRLV, kStopLPLV
} StopModes_t;

void PWR_EnablePvd(PvdThresholdLevel_t pvd_level, PvdMode_t mode);
void PWR_EnterSleepMode(PwrExit_t exit);
void PWR_EnterStopMode(PwrExit_t exit, StopModes_t stop_mode);
void PWR_WriteToBackupRegister(uint32_t *p_data_buffer, uint8_t data_len);
void PWR_EnterStandbyMode(PwrExit_t exit);
#endif /* MYDRIVERS_INC_STM32F401XE_PWR_H_ */
