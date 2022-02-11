/*
 * stm32f401xe_I2C.c
 *
 *  Created on: 12 sty 2022
 *      Author: pawel
 */

#include "stm32f401xe_i2c.h"
#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"

/*
 * Problem with I2C that SDA stays on low state after reset
 * @param[*p_i2cx] - base address of i2c peripheral
 * @return - void
 */
void I2C_CheckIfBusIsHanging(GPIO_TypeDef *p_GPIOx_SDA, GpioPinNumber_t pin_SDA,GPIO_TypeDef *p_GPIOx_SCL, GpioPinNumber_t pin_SCL)
{
	uint8_t count = 0;
	while(!GPIO_ReadPin(p_GPIOx_SDA, pin_SDA))
	{
		GPIO_TogglePin(p_GPIOx_SCL, pin_SCL);

		if(count++ == 100)
		{
			break;
		}
	}

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
 *
 * @param[*p_i2cx] - i2c address
 * @param[alternate_pos] - pins alternative positions select
 * @return - void
 */
void I2C_InitGpioPins(I2c_Handle_t *p_handle_i2c)
{

	if (p_handle_i2c->p_i2cx == I2C1)
	{
		// PB6 SCL PB7 SDA
		GPIO_ConfigBasic(GPIOB, (GPIO_FLAG_PIN_6 | GPIO_FLAG_PIN_7), kGpioModeAF, kGpioPUPDNoPull);
		GPIO_ConfigOutput(GPIOB, (GPIO_FLAG_PIN_6 | GPIO_FLAG_PIN_7), kGpioOTOpenDrain, kGpioSpeedVeryHigh);
		GPIO_ConfigAF(GPIOB, (GPIO_FLAG_PIN_6 | GPIO_FLAG_PIN_7), kGpioAF4);

	}

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
	while (!(I2C_SR1_SB & p_handle_i2c->p_i2cx->SR1))
			break;;
	// 1.2 Clear SB by reading SR1
	temp_byte = p_handle_i2c->p_i2cx->SR1;
	// If transmitting set slave addres LSB to 0, receiver 1
	slave_address &= (~1U);
	slave_address |= mode;
	// 2. Put slave address in DR register -
	p_handle_i2c->p_i2cx->DR = slave_address;
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

	I2C_SendAddress(p_handle_i2c, slave_address, I2C_MODE_TRANSMITTER);
	// wait until ADDR is set
	while (!(I2C_SR1_ADDR & p_handle_i2c->p_i2cx->SR1))
		;
	// 4. ADDR is cleared by reading SR1 , Read SR2
	temp_byte = p_handle_i2c->p_i2cx->SR1;
	temp_byte = p_handle_i2c->p_i2cx->SR2;

	// 5. TxE bit is set when acknowledge bit is sent
	while (!(p_handle_i2c->p_i2cx->SR1 & I2C_SR1_TXE))
		;
	// 6. Write memory address to DR to clear TxE
	p_handle_i2c->p_i2cx->DR = mem_address;

	// 7. Data transfer
	while (tx_data_to_send > 0)
	{
		// wait until data register is empty
		while (!(p_handle_i2c->p_i2cx->SR1 & I2C_SR1_TXE))
			;

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
			while (!(p_handle_i2c->p_i2cx->SR1 & I2C_SR1_BTF))
				;
			// stop transfer
			p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_STOP;
		}
	}

	// in case of sending only mem address
	while (!(p_handle_i2c->p_i2cx->SR1 & I2C_SR1_TXE))
		;
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_STOP;

	return;
}

void I2C_Receive(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
		uint8_t *p_rx_data_buffer, uint32_t data_size)
{
	uint32_t rx_data_to_get = data_size;
	uint8_t temp_byte;

	I2C_SendAddress(p_handle_i2c, slave_address, I2C_MODE_RECEIVER);

	while (!(I2C_SR1_ADDR & p_handle_i2c->p_i2cx->SR1))
		;

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
		while (!(I2C_SR1_RXNE & p_handle_i2c->p_i2cx->SR1))
			;

		p_rx_data_buffer[data_size - rx_data_to_get] = p_handle_i2c->p_i2cx->DR;

		return;
	}

	// multiple bytes receive
	while (rx_data_to_get > 2)
	{
		// 4. ADDR is cleared by reading SR1 , Read SR2
		temp_byte = p_handle_i2c->p_i2cx->SR1;
		temp_byte = p_handle_i2c->p_i2cx->SR2;

		// read all the bytes until second last
		while (rx_data_to_get > 2)
		{
			while (!(I2C_SR1_RXNE & p_handle_i2c->p_i2cx->SR1))
				;
			p_rx_data_buffer[data_size - rx_data_to_get] = p_handle_i2c->p_i2cx->DR;
			rx_data_to_get--;

			// ack receive
			p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_ACK;
		}

		// read second last byte
		while (!(I2C_SR1_RXNE & p_handle_i2c->p_i2cx->SR1))
			;
		p_rx_data_buffer[data_size - rx_data_to_get] = p_handle_i2c->p_i2cx->DR;
		rx_data_to_get--;

		// after second last byte clear ACK and set stop
		p_handle_i2c->p_i2cx->CR1 &= ~(I2C_CR1_ACK);
		p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_STOP;

		// receive last byte
		while (!(I2C_SR1_RXNE & p_handle_i2c->p_i2cx->SR1))
			;
		p_rx_data_buffer[data_size - rx_data_to_get] = p_handle_i2c->p_i2cx->DR;
		rx_data_to_get--;
	}

	return;
}
