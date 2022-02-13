/*
 * stm32f401xe_I2C.h
 *
 *  Created on: 12 sty 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F401XE_I2C_H_
#define MYDRIVERS_INC_STM32F401XE_I2C_H_

#include "stm32f401xe.h"

//***** I2C1 *****//
//** SDA PB7/PB9 **//
#define I2C1_SDA_PIN kGpioPin7
#define I2C1_SDA_PIN_FLAG (1U << I2C1_SDA_PIN)
#define I2C1_SDA_PORT GPIOB
//** SCL PB6/PB8 **//
#define I2C1_SCL_PIN kGpioPin6
#define I2C1_SCL_PIN_FLAG (1U << I2C1_SCL_PIN)
#define I2C1_SCL_PORT GPIOB
//** SMBA PB5 **//
#define I2C1_SMBA_PIN kGpioPin5
#define I2C1_SMBA_PIN_FLAG (1U << I2C1_SMBA_PIN)
#define I2C1_SMBA_PORT GPIOB

//***** I2C2 *****//
//** SDA PB3 **//
#define I2C2_SDA_PIN kGpioPin3
#define I2C2_SDA_PIN_FLAG (1U << I2C2_SDA_PIN)
#define I2C2_SDA_PORT GPIOB
//** SCL PA8/PB10 **//
#define I2C2_SCL_PIN kGpioPin10
#define I2C2_SCL_PIN_FLAG (1U << I2C2_SCL_PIN)
#define I2C2_SCL_PORT GPIOB
//** SMBA PB12 **//
#define I2C2_SMBA_PIN kGpioPin12
#define I2C2_SMBA_PIN_FLAG (1U << I2C2_SMBA_PIN)
#define I2C2_SMBA_PORT GPIOB

//***** I2C3 *****// - SDA pins are on different AF numbers so that has to be
//adjusted in i2c_initgpio
//** SDA PC9/PB4 **//
#define I2C3_SDA_PIN kGpioPin9
#define I2C3_SDA_PIN_FLAG (1U << I2C3_SDA_PIN)
#define I2C3_SDA_PORT GPIOC
//** SCL PA8 **//
#define I2C3_SCL_PIN kGpioPin8
#define I2C3_SCL_PIN_FLAG (1U << I2C3_SCL_PIN)
#define I2C3_SCL_PORT GPIOA
//** SMBA PA9 **//
#define I2C3_SMBA_PIN kGpioPin9
#define I2C3_SMBA_PIN_FLAG (1U << I2C3_SMBA_PIN)
#define I2C3_SMBA_PORT GPIOA

/*
 * @Speed  fPCLK1 must be at least 2 MHz to achieve Sm mode I?C frequencies. It
 * must be at least 4 MHz to achieve Fm mode I?C frequencies. It must be a
 * multiple of 10MHz to reach the 400 kHz maximum I?C Fm mode clock.
 */
typedef enum I2cSpeed_t
{
	kI2cSpeedSlow, kI2cSpeedFastDuty0, kI2cSpeedFastDuty1
} I2cSpeed_t;

/*
 * Errors
 */
typedef enum I2cError_t
{
	kI2cErrNoError, kI2cErrWrongPclkFreq
} I2cError_t;

typedef struct I2c_Handle_t
{
	I2C_TypeDef *p_i2cx;

	I2cError_t error;

} I2c_Handle_t;

/*
 * @Frequency  The minimum allowed frequency of PCLK1 is 2 MHz,
 * the maximum frequency is limited by the maximum APB1 frequency and cannot
 * exceed 50 MHz (peripheral intrinsic maximum limit). Assign here ABP1
 * frequency.
 */

#define I2C_FREQUENCY_MINIMUM 2000000U
#define I2C_FREQUENCY_MAXIMUM 50000000U

/*
 * @Mode - mode that is send with address to i2c devices - decides if master is
 * transmitter or reciever
 */

#define I2C_MODE_TRANSMITTER 0U
#define I2C_MODE_RECEIVER 1U

/*
 * @CCR - those are the times in nanoseconds that are defined by I2C
 * characteristics and are used to calculate value that has to be put in CCR
 * register hot to calculate it properly can be found in RM on CCR register
 * description values below an be found in DS in I2C characteristics as tw(SCLH)
 * and tr(SCL)
 *
 * TR_SCL	- SDA and SCL rise time [ns]
 * TW_SCLH	- SCL clock high time [ns]
 * TF_SCL	- SDA and SCL fall time [ns]
 * TW_SCLL	- SCL clock low time [ns]
 */

// Slow mode
#define I2C_CCR_SM_TR_SCL 1000U
#define I2C_CCR_SM_TW_SCLH 4000U
#define I2C_CCR_SM_TF_SCL 300U
#define I2C_CCR_SM_TW_SCLL 4700U
#define I2C_CCR_SM_COEFF 2U
#define I2C_CCR_SM_THIGH (I2C_CCR_SM_TR_SCL + I2C_CCR_SM_TW_SCLH)
#define I2C_CCR_SM_TLOW (I2C_CCR_SM_TF_SCL + I2C_CCR_SM_TW_SCLL)

// Fast mode
#define I2C_CCR_FM_TR_SCL 300U
#define I2C_CCR_FM_TW_SCLH 600U
#define I2C_CCR_FM_TF_SCL 300U
#define I2C_CCR_FM_TW_SCLL 1300U
#define I2C_CCR_FM_COEFF_DUTY0 3U
#define I2C_CCR_FM_COEFF_DUTY1 25U
#define I2C_CCR_FM_THIGH (I2C_CCR_FM_TR_SCL + I2C_CCR_FM_TW_SCLH)
#define I2C_CCR_FM_TLOW (I2C_CCR_FM_TF_SCL + I2C_CCR_FM_TW_SCLL)

// ******** FUNCTIONS ******** //

// Init functions
void I2C_InitClock(I2c_Handle_t *p_handle_i2c);
void I2C_SetBasicParameters(I2c_Handle_t *p_handle_i2c, I2cSpeed_t speed);
void I2C_InitGpioPins(I2c_Handle_t *p_handle_i2c);

void I2C_Transmit(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
		uint8_t mem_address, uint8_t *p_tx_data_buffer, uint32_t data_size);
void I2C_Receive(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
		uint8_t *p_rx_data_buffer, uint32_t data_size);
#endif /* MYDRIVERS_INC_STM32F401XE_I2C_H_ */
