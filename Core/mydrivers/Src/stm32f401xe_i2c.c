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
 * Start clock for I2C
 *
 * @param[*p_i2cx] - base address of i2c peripheral
 * @return - void
 */
static void I2C_ClockEnable(I2C_TypeDef *p_i2cx)
{
	if (p_i2cx == I2C1)
	{
		RCC_CLOCK_I2C1_ENABLE();
	}
	else if (p_i2cx == I2C2)
	{
		RCC_CLOCK_I2C2_ENABLE();
	}
	else if (p_i2cx == I2C3)
	{
		RCC_CLOCK_I2C3_ENABLE();
	}
}

/*
 * init i2c peripheral gpio pins
 *
 * @param[*p_i2cx] - i2c address
 * @param[alternate_pos] - pins alternative positions select
 * @return - void
 */
static void I2C_InitGpioPins(I2C_TypeDef *p_i2cx, uint8_t alternate_pos)
{
//	GPIO_Handle_t gpio_sda, gpio_scl;
//
//	if (p_i2cx == I2C1)
//	{
//		// PB6 SCL
//		gpio_scl.pGPIOx = GPIOB;
//		gpio_scl.PinConfig.PinNumber = GPIO_PIN_6;
//
//		// PB7 SDA
//		gpio_sda.pGPIOx = GPIOB;
//		gpio_sda.PinConfig.PinNumber = GPIO_PIN_7;
//
//		// Alternate Function
//		gpio_scl.PinConfig.AF = kGpioAF4;
//		gpio_sda.PinConfig.AF = kGpioAF4;
//
//		if (alternate_pos == 1)
//		{
//			// PB8 SCL
//			gpio_scl.PinConfig.PinNumber = GPIO_PIN_8;
//			// PB9 SDA
//			gpio_sda.PinConfig.PinNumber = GPIO_PIN_9;
//		}
//	}
//
//	if (p_i2cx == I2C2)
//	{
//		// PB10 SCL
//		gpio_scl.pGPIOx = GPIOB;
//		gpio_scl.PinConfig.PinNumber = GPIO_PIN_10;
//
//		// PB11 SDA
//		gpio_sda.pGPIOx = GPIOB;
//		gpio_sda.PinConfig.PinNumber = GPIO_PIN_11;
//
//		// Alternate Function
//		gpio_scl.PinConfig.AF = kGpioAF4;
//		gpio_sda.PinConfig.AF = kGpioAF4;
//
//		if (alternate_pos == 1)
//		{
//			// PB3 SDA
//			gpio_sda.PinConfig.PinNumber = GPIO_PIN_3;
//			gpio_sda.PinConfig.AF = kGpioAF9;
//		}
//	}
//
//	if (p_i2cx == I2C3)
//	{
//		// PA8 SCL
//		gpio_scl.pGPIOx = GPIOA;
//		gpio_scl.PinConfig.PinNumber = GPIO_PIN_8;
//
//		// PB9 SDA
//		gpio_sda.pGPIOx = GPIOB;
//		gpio_sda.PinConfig.PinNumber = GPIO_PIN_4;
//
//		// Alternate Function
//		gpio_scl.PinConfig.AF = kGpioAF4;
//		gpio_sda.PinConfig.AF = kGpioAF4;
//
//		if (alternate_pos == 1)
//		{
//			RCC_CLOCK_GPIOC_ENABLE();
//			// PC9 SDA
//			gpio_sda.pGPIOx = GPIOC;
//			gpio_sda.PinConfig.PinNumber = GPIO_PIN_9;
//
//			// Alternate Function
//			gpio_sda.PinConfig.AF = kGpioAF9;
//		}
//	}
//
//	// Mode AF
//	gpio_scl.PinConfig.Mode = GPIO_PIN_MODE_AF;
//	gpio_sda.PinConfig.Mode = GPIO_PIN_MODE_AF;
//
//	// Output type open drain
//	gpio_scl.PinConfig.OutputType = GPIO_PIN_OT_OD;
//	gpio_sda.PinConfig.OutputType = GPIO_PIN_OT_OD;
//
//	// Output speed very high
//	gpio_scl.PinConfig.OutputSpeed = GPIO_PIN_SPEED_VERYHIGH;
//	gpio_sda.PinConfig.OutputSpeed = GPIO_PIN_SPEED_VERYHIGH;
//
//	// Pull ups
//	gpio_scl.PinConfig.PullUpPullDown = GPIO_PIN_PUPD_NOPULL;
//	gpio_sda.PinConfig.PullUpPullDown = GPIO_PIN_PUPD_NOPULL;
//
//	GPIO_InitPin(&gpio_sda);
//	GPIO_InitPin(&gpio_scl);
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
	// enable peripheral clock
	I2C_ClockEnable(p_handle_i2c->p_i2cx);

	// init GPIO pins
	I2C_InitGpioPins(p_handle_i2c->p_i2cx, 1);

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
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_ACK;
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_START;

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
uint8_t I2C_Transmit(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
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

	return 0;
}

uint8_t I2C_Receive(I2c_Handle_t *p_handle_i2c, uint8_t slave_address,
		uint8_t *p_rx_data_buffer, uint32_t data_size)
{
	uint32_t rx_data_to_get = data_size;
	uint8_t temp_byte;
	p_handle_i2c->p_i2cx->CR1 |= I2C_CR1_ACK;
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

		return 0;
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
	return 0;
}
