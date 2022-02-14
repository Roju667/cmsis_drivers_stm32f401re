/*
 * stm32f401xe_I2C.c
 *
 *  Created on: 12 sty 2022
 *      Author: pawel
 */

#include "stm32f401xe_i2c.h"
#include "stm32f401xe_systick.h"
#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"

/*
 * Wait until flag is set until timeout
 *
 * @param[status_reg] - status register
 * @param[flag] - status flag
 * @param[timeout[ - timeout in miliseconds until re
 * @return - void
 */
static uint8_t I2C_WaitForFlagUntilTimeout(volatile uint32_t *status_reg,uint32_t flag,uint32_t timeout)
{
	uint32_t start_time = SYSTICK_GetTick();
	// check flag until timeout
	while (!(flag & *(status_reg)))
	{
		{
			if(SYSTICK_GetTick() - start_time > timeout)
					{
						// return error status
						return 1;
					}
		}
	}
	// return no error
	return 0;
}


/*
 * Problem with I2C that SDA stays on low state after reset
 * solution is to toggle scl line few times for slave to release the line
 * @param[*p_i2cx] - base address of i2c peripheral
 * @return - void
 */
static void I2C_CheckIfBusIsHanging(I2c_Handle_t *p_handle_i2c)
{
	if (p_handle_i2c->p_i2cx == I2C1)
	{
		GPIO_ConfigBasic(GPIOB, I2C1_SCL_PIN_FLAG, kGpioModeOutput,
				kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOB, I2C1_SCL_PIN_FLAG, kGpioOTPushPull,
				kGpioSpeedVeryHigh);
		for (uint8_t i = 0; i < 10; i++)
		{
			GPIO_TogglePin(I2C1_SCL_PORT, I2C1_SCL_PIN);
		}
	}
	else if (p_handle_i2c->p_i2cx == I2C2)
	{
		GPIO_ConfigBasic(GPIOB, I2C2_SCL_PIN_FLAG, kGpioModeOutput,
				kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOB, I2C2_SCL_PIN_FLAG, kGpioOTPushPull,
				kGpioSpeedVeryHigh);
		for (uint8_t i = 0; i < 10; i++)
		{
			GPIO_TogglePin(I2C2_SCL_PORT, I2C2_SCL_PIN);
		}
	}
	else if (p_handle_i2c->p_i2cx == I2C3)
	{
		GPIO_ConfigBasic(GPIOB, I2C2_SCL_PIN_FLAG, kGpioModeOutput,
				kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOB, I2C2_SCL_PIN_FLAG, kGpioOTPushPull,
				kGpioSpeedVeryHigh);
		for (uint8_t i = 0; i < 10; i++)
		{
			GPIO_TogglePin(I2C3_SCL_PORT, I2C3_SCL_PIN);
		}
	}

	return;
}

/*
 * Send address on i2c line
 *
 * @param[*p_handle_i2c] - pointer to handler to i2c structure
 * @param[slave_address] - address to slave in 7 bit addressing mode
 * @param[mode] - send information if master is in receiver or transmitter mode
 * @Mode
 * @return - void
 */
static void I2C_SendAddress(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
		uint8_t mode)
{
	uint8_t temp_byte;
	// 1.0 Set START BIT
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_START;
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_ACK;
	// 1.1 Wait until SB flag is set

	if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_SB, I2C_TIMEOUT) == 1)
	{
		p_handle_i2c->error = kI2cErrTimeoutSB;
	}

	// 1.2 Clear SB by reading SR1
	temp_byte = p_handle_i2c->p_i2cx->SR1;
	// If transmitting set slave addres LSB to 0, receiver 1
	slave_address &= (~1U);
	slave_address |= mode;
	// 2. Put slave address in DR register -
	p_handle_i2c->p_i2cx->DR = slave_address;

	return;
}

/*
 * Init speed, CCR and TRISE registers
 *
 * @param[*p_handle_i2c] - handler to i2c structure
 * @param[speed] - i2c desired speed
 * @return - void
 */
static void I2C_CalculateCCRandTRISE(I2c_Handle_t *p_handle_i2c,
		I2cSpeed_t speed)
{
	// set speed
	uint16_t temp_ccr, temp_trise;
	uint8_t pclk_freq_Mhz = RCC_GetPclk(1) / 1000000;

	// set slow mode, reset DUTY
	p_handle_i2c->p_i2cx->CCR &= ~(I2C_CCR_FS);
	p_handle_i2c->p_i2cx->CCR &= ~(I2C_CCR_DUTY);

	// CCR calculation for slow mode -> values are coming from RM CCR register and
	// result is in [ns] (Thigh + Tlow) / (CEOFF * PCLK)
	temp_ccr = (I2C_CCR_SM_THIGH + I2C_CCR_SM_TLOW)
			/ (I2C_CCR_SM_COEFF * (1000 / pclk_freq_Mhz));

	// TRISE calculation for slow mode -> equation is from RM
	temp_trise = ((I2C_CCR_SM_TR_SCL * pclk_freq_Mhz) / 1000) + 1;

	// fast mode
	if (speed != kI2cSpeedSlow)
	{
		// set fast mode
		p_handle_i2c->p_i2cx->CCR |= I2C_CCR_FS;
		// calculate CCR for fast mode with DUTY 0
		temp_ccr = (I2C_CCR_FM_THIGH + I2C_CCR_FM_TLOW)
				/ (I2C_CCR_FM_COEFF_DUTY0 * (1000 / pclk_freq_Mhz));
		// calculate TRISE for fast mode
		temp_trise = ((I2C_CCR_FM_TR_SCL * pclk_freq_Mhz) / 1000) + 1;
		if (speed == kI2cSpeedFastDuty1)
		{
			// set DUTY flag
			p_handle_i2c->p_i2cx->CCR |= I2C_CCR_DUTY;
			// calculate CCR with fast mode DUTY1
			temp_ccr = (I2C_CCR_FM_THIGH + I2C_CCR_FM_TLOW)
					/ (I2C_CCR_FM_COEFF_DUTY1 * (1000 / pclk_freq_Mhz));
		}
	}
	p_handle_i2c->p_i2cx->CCR &= ~(I2C_CCR_CCR);
	p_handle_i2c->p_i2cx->CCR |= (temp_ccr << I2C_CCR_CCR_Pos);

	// write correct TRISE to the register
	p_handle_i2c->p_i2cx->TRISE &= ~(I2C_TRISE_TRISE);
	p_handle_i2c->p_i2cx->TRISE |= (temp_trise << I2C_TRISE_TRISE_Pos);

	return;
}

/*
 * Start clock for I2C
 *
 * @param[*p_i2cx] - base address of i2c peripheral
 * @return - void
 */
void I2C_InitClock(I2c_Handle_t *p_handle_i2c)
{
	if (p_handle_i2c->p_i2cx == I2C1)
	{
		RCC_CLOCK_I2C1_ENABLE();
		RCC_RESET_I2C1();
	}
	else if (p_handle_i2c->p_i2cx == I2C2)
	{
		RCC_CLOCK_I2C2_ENABLE();
		RCC_RESET_I2C2();
	}
	else if (p_handle_i2c->p_i2cx == I2C3)
	{
		RCC_CLOCK_I2C3_ENABLE();
		RCC_RESET_I2C3();
	}

	return;
}

/*
 * init i2c peripheral gpio pins
 * define correct pin numbers and ports in header!
 *
 * @param[*p_i2cx] - i2c address
 * @param[alternate_pos] - pins alternative positions select
 * @return - void
 */
void I2C_InitGpioPins(I2c_Handle_t *p_handle_i2c)
{
	I2C_CheckIfBusIsHanging(p_handle_i2c);

	// configure pins
	if (p_handle_i2c->p_i2cx == I2C1)
	{
		GPIO_ConfigBasic(GPIOB, (I2C1_SDA_PIN_FLAG | I2C1_SCL_PIN_FLAG),
				kGpioModeAF, kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOB, (I2C1_SDA_PIN_FLAG | I2C1_SCL_PIN_FLAG),
				kGpioOTOpenDrain, kGpioSpeedVeryHigh);
		GPIO_ConfigAF(GPIOB, (I2C1_SDA_PIN_FLAG | I2C1_SCL_PIN_FLAG), kGpioAF4);
	}
	else if (p_handle_i2c->p_i2cx == I2C2)
	{
		GPIO_ConfigBasic(GPIOB, (I2C2_SDA_PIN_FLAG | I2C2_SCL_PIN_FLAG),
				kGpioModeAF, kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOB, (I2C2_SDA_PIN_FLAG | I2C2_SCL_PIN_FLAG),
				kGpioOTOpenDrain, kGpioSpeedVeryHigh);
		GPIO_ConfigAF(GPIOB, I2C2_SDA_PIN_FLAG, kGpioAF9);
		GPIO_ConfigAF(GPIOB, I2C2_SCL_PIN_FLAG, kGpioAF4);
	}
	else if (p_handle_i2c->p_i2cx == I2C3)
	{
		GPIO_ConfigBasic(GPIOB, (I2C3_SDA_PIN_FLAG | I2C3_SCL_PIN_FLAG),
				kGpioModeAF, kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOB, (I2C3_SDA_PIN_FLAG | I2C3_SCL_PIN_FLAG),
				kGpioOTOpenDrain, kGpioSpeedVeryHigh);
		GPIO_ConfigAF(GPIOB, (I2C3_SDA_PIN_FLAG | I2C3_SCL_PIN_FLAG), kGpioAF4);
	}

	return;
}



/*
 * Basic init function
 *
 * @param[*p_handle_i2c] - handler to i2c structure
 * @param[speed] - i2c desired speed
 * @return - void
 */
void I2C_SetBasicParameters(I2c_Handle_t *p_handle_i2c, I2cSpeed_t speed)
{
	// reset I2C
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_SWRST;
	p_handle_i2c->p_i2cx->CR1 &= ~(I2C_CR1_SWRST);

	// check frequency limits
	if (RCC_GetPclk(1) < I2C_FREQUENCY_MINIMUM
			|| RCC_GetPclk(1) > I2C_FREQUENCY_MAXIMUM)
	{
		p_handle_i2c->error = kI2cErrWrongPclkFreq;
		return;
	}

	// set frequency (same as ABP1 frequency)
	p_handle_i2c->p_i2cx->CR2 &= ~(I2C_CR2_FREQ);
	p_handle_i2c->p_i2cx->CR2 |=
			((RCC_GetPclk(1) / 1000000) << I2C_CR2_FREQ_Pos);

	I2C_CalculateCCRandTRISE(p_handle_i2c, speed);

	// enable I2c
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_PE;
	p_handle_i2c->error = kI2cErrNoError;

	// check if bus is not stuck
	// I2C_CheckIfBusIsHanging(p_handle_i2c);

	return;
}


/*
 * Transmit data in polling mode
 *
 * @param[*p_handle_i2c] - pointer to handler to i2c structure
 * @param[slave_address] - address to slave in 7 bit addressing mode
 * @param[mem_address] - slave memory register that has to be changed
 * @param[p_data_buffer] - pointer to data buffer that has to be send
 * @param[data_size] - amount of data to be send [in bytes]
 * @return - uint8_t - to return error
 */
void I2C_Transmit(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
		uint8_t mem_address, uint8_t *p_tx_data_buffer, uint32_t data_size)
{
	uint32_t tx_data_to_send = data_size;
	uint8_t temp_byte;


	p_handle_i2c->status = kI2cStatusTxPolling;

	I2C_SendAddress(p_handle_i2c, slave_address, I2C_MODE_TRANSMITTER);
	// wait until ADDR is set
	if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_ADDR, I2C_TIMEOUT))
	{
		p_handle_i2c->error = kI2cErrTimeoutADDR;
		p_handle_i2c->status = kI2cStatusIdle;
		return;
	}

	// 4. ADDR is cleared by reading SR1 , Read SR2
	temp_byte = p_handle_i2c->p_i2cx->SR1;
	temp_byte = p_handle_i2c->p_i2cx->SR2;

	// 5. TxE bit is set when acknowledge bit is sent;
	if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_TXE, I2C_TIMEOUT))
	{
		p_handle_i2c->error = kI2cErrTimeoutTXE;
		p_handle_i2c->status = kI2cStatusIdle;
		return;
	}

	// 6. Write memory address to DR to clear TxE
	p_handle_i2c->p_i2cx->DR = mem_address;

	// 7. Data transfer
	while (tx_data_to_send > 0)
	{
		// wait until data register is empty
		if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_TXE, I2C_TIMEOUT))
		{
			p_handle_i2c->error = kI2cErrTimeoutTXE;
			p_handle_i2c->status = kI2cStatusIdle;
			return;
		}


		// put data in data register
		p_handle_i2c->p_i2cx->DR =
				p_tx_data_buffer[data_size - tx_data_to_send];

		// change counters
		tx_data_to_send--;

		// 8. After last bit is written to DR register , Set STOP bit  and interface
		// is going back to slave mode
		if (tx_data_to_send == 0)
		{
			// check if data transfer is finsihed
			if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_BTF, I2C_TIMEOUT))
			{
				p_handle_i2c->error = kI2cErrTimeoutBTF;
				p_handle_i2c->status = kI2cStatusIdle;
				return;
			}

			// stop transfer
			p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_STOP;
			p_handle_i2c->error = kI2cErrNoError;
			p_handle_i2c->status = kI2cStatusIdle;
			return;
		}
	}

	// in case of sending only mem address
	if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_TXE, I2C_TIMEOUT))
	{
		p_handle_i2c->error = kI2cErrTimeoutTXE;
		p_handle_i2c->status = kI2cStatusIdle;
		return;
	}

	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_STOP;
	p_handle_i2c->error = kI2cErrNoError;
	p_handle_i2c->status = kI2cStatusIdle;
	return;
}

/*
 * Receive data in polling mode
 *
 * @param[*p_handle_i2c] - pointer to handler to i2c structure
 * @param[slave_address] - address to slave in 7 bit addressing mode
 * @param[p_data_buffer] - pointer to data buffer that has to be send
 * @param[data_size] - amount of data to be received [in bytes]
 * @return - uint8_t - to return error
 */
void I2C_Receive(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
		uint8_t *p_rx_data_buffer, uint32_t data_size)
{
	uint32_t rx_data_to_get = data_size;
	uint8_t temp_byte;

	I2C_SendAddress(p_handle_i2c, slave_address, I2C_MODE_RECEIVER);

	if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_ADDR, I2C_TIMEOUT))
	{
		p_handle_i2c->error = kI2cErrTimeoutADDR;
		p_handle_i2c->status = kI2cStatusIdle;
		return;
	}

	// single byte receive
	if (data_size == 1)
	{
		// Disable acknowledge
		p_handle_i2c->p_i2cx->CR1 &= ~(I2C_CR1_ACK);
		// 4. ADDR is cleared by reading SR1 , Read SR2
		temp_byte = p_handle_i2c->p_i2cx->SR1;
		temp_byte = p_handle_i2c->p_i2cx->SR2;

		// stop comm
		p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_STOP;

		// wait for a byte received
		if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_RXNE, I2C_TIMEOUT))
		{
			p_handle_i2c->error = kI2cErrTimeoutRXNE;
			p_handle_i2c->status = kI2cStatusIdle;
			return;
		}

		p_rx_data_buffer[data_size - rx_data_to_get] = p_handle_i2c->p_i2cx->DR;
		p_handle_i2c->error = kI2cErrNoError;
		p_handle_i2c->status = kI2cStatusIdle;
		return;
	}

	// multiple bytes receive
	while (rx_data_to_get >= 2)
	{
		// 4. ADDR is cleared by reading SR1 , Read SR2
		temp_byte = p_handle_i2c->p_i2cx->SR1;
		temp_byte = p_handle_i2c->p_i2cx->SR2;

		// read all the bytes until second last
		while (rx_data_to_get > 2)
		{
			if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_RXNE, I2C_TIMEOUT))
			{
				p_handle_i2c->error = kI2cErrTimeoutRXNE;
				p_handle_i2c->status = kI2cStatusIdle;
				return;
			}

			p_rx_data_buffer[data_size - rx_data_to_get] =
					p_handle_i2c->p_i2cx->DR;
			rx_data_to_get--;

			// ack receive
			p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_ACK;
		}

		// read second last byte
		if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_RXNE, I2C_TIMEOUT))
		{
			p_handle_i2c->error = kI2cErrTimeoutRXNE;
			p_handle_i2c->status = kI2cStatusIdle;
			return;
		}

		p_rx_data_buffer[data_size - rx_data_to_get] = p_handle_i2c->p_i2cx->DR;
		rx_data_to_get--;

		// after second last byte clear ACK and set stop
		p_handle_i2c->p_i2cx->CR1 &= ~(I2C_CR1_ACK);
		p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_STOP;

		// receive last byte
		if(I2C_WaitForFlagUntilTimeout(&(p_handle_i2c->p_i2cx->SR1), I2C_SR1_RXNE, I2C_TIMEOUT))
		{
			p_handle_i2c->error = kI2cErrTimeoutRXNE;
			p_handle_i2c->status = kI2cStatusIdle;
			return;
		}

		p_rx_data_buffer[data_size - rx_data_to_get] = p_handle_i2c->p_i2cx->DR;
		rx_data_to_get--;
	}
	// finish receiving
	p_handle_i2c->error = kI2cErrNoError;
	p_handle_i2c->status = kI2cStatusIdle;
	return;
}



