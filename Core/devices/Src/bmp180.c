/*
 * bmp180.c
 *
 *  Created on: 27 feb. 2019
 *      Author: gheorghe.ghirjev
 */

#include <string.h>
#include <math.h>
#include "bmp180.h"
#include "stm32f401xe_i2c.h"


static void bmp180_delay(uint32_t ms)
{
	for (uint32_t i = 0; i < ms; i++)
	{

	}
}



/*!
* @brief:    - Read and check bmp chip ID.This value is fixed to 0x55,
*              and can be used to check whether communication is functioning.
* @param[in] - NONE.
* @return    - NO_ERR if chip_id is equal to 0x55, otherwise CHIP_ID_INVALID_ERR.
*/
static bmp_err_t bmp180_read_chip_id(bmp180_t *p_bmp)
{
	uint8_t out_buff = 0;
	uint8_t ret_val = NO_ERR;

	// data read
	// send slave address and memory address
	I2C_Transmit(p_bmp->p_i2c_handle, BMP_READ_ADDR, BMP_CHIP_ID_REG, 0, 0);
	bmp180_delay(BMP_TEMP_CONV_TIME * 10000);
	// read data
	I2C_Receive(p_bmp->p_i2c_handle, BMP_READ_ADDR, out_buff, 1);

	if (BMP_CHIP_ID_VAL != out_buff)
	{
		ret_val = CHIP_ID_INVALID_ERR;
	}

	return ret_val;
}


/*!
* @brief:    - Write oversampling settings to the Control register.
* @param[in] - struct of type oss_t
* @param[in] - enum of type oss_ratio_t
* @return    - None
*/
static void bmp180_set_oss(bmp180_t *p_bmp, oss_ratio_t ratio)
{
	uint8_t in_buff[2] = {0};

	switch (ratio)
	{
		case ULTRA_LOW_PWR_MODE:
		{
		p_bmp->oss.wait_time = BMP_OSS0_CONV_TIME;
			break;
		}
		case STANDARD_MODE:
		{
		p_bmp->oss.wait_time = BMP_OSS1_CONV_TIME;
			break;
		}
		case HIGH:
		{
		p_bmp->oss.wait_time = BMP_OSS2_CONV_TIME;
			break;
		}
		case ULTRA_HIGH_RESOLUTION:
		{
		p_bmp->oss.wait_time = BMP_OSS3_CONV_TIME;
			break;
		}
		default:
		{
		p_bmp->oss.wait_time = BMP_OSS1_CONV_TIME;
			break;
		}
	}

	p_bmp->oss.ratio = ratio;
	BMP_SET_I2CRW_REG (in_buff[1], BMP_CTRL_OSS_MASK(ratio));
	I2C_Transmit(p_bmp->p_i2c_handle, BMP_WRITE_ADDR, BMP_CTRL_REG, in_buff, 2);

}


/*!
* @brief:    - Read calibration BMP data. The E2PROM has stored 176 bit of individual calibration data.
*              This is used to compensate offset, temperature dependence and other parameters of the sensor.
* @param[in] - struct of type bmp_calib_param_t
* @return    - NO_ERR if read calibration data are valid otherwise READ_CALIB_ERR.
*/
static bmp_err_t bmp180_read_calib_data(bmp180_t *p_bmp)
{
	bmp_err_t ret_val = NO_ERR;
	uint8_t out_buff[BMP_CALIB_DATA_SIZE] = {0};
	uint8_t i = 0;
	uint8_t j = 1;
	int16_t *calib_data = (int16_t*) &p_bmp->calib;

	// send slave address and memory address
	I2C_Transmit(p_bmp->p_i2c_handle, BMP_READ_ADDR, BMP_CALIB_ADDR, 0, 0);
	bmp180_delay(BMP_TEMP_CONV_TIME * 10000);
	// read data
	I2C_Receive(p_bmp->p_i2c_handle, BMP_READ_ADDR, out_buff,
			BMP_CALIB_DATA_SIZE);

	// Store read calib data to bmp_calib struct.
	for (i = 0; i <= BMP_CALIB_DATA_SIZE / 2; i++, j+2)
	{
		calib_data[i] = (out_buff[i * 2] << 8) | out_buff[j];

		// checking that none of the words has the value 0 or 0xFFFF.
		if ((0 == calib_data[i]) | (-1 == calib_data[i]))
		{
			ret_val = GET_CALIB_ERR;
		}
	}

	return ret_val;
}

/*!
* @brief:    - Performe initial sequence of BMP sensor
* @param[in] - pointer to struct of type bmp_calib_param_t
* @return    - None.
*/
void bmp180_init(bmp180_t *p_bmp, I2c_Handle_t *p_i2c_handle)
{

	memset(p_bmp, 0x00, sizeof(&p_bmp)); // clear bmp strut;
	p_bmp->p_i2c_handle = p_i2c_handle;
	p_bmp->err = bmp180_read_chip_id(p_bmp); // check chip validity and I2C communication.
	p_bmp->err = bmp180_read_calib_data(p_bmp);
	bmp180_set_oss(p_bmp, HIGH);       // set oversampling settings
}

/*!
* @brief:    - Get uncompensated temperature value. UT = temperature data (16 bit)
* @param[in] - None.
* @return    - uncompensated temp.
*/
int32_t bmp180_get_ut(bmp180_t *p_bmp)
{
	uint8_t out_buff[2];

	BMP_SET_I2CRW_REG (out_buff[0], BMP_SET_TEMP_CONV);

	// write conversion time
	I2C_Transmit(p_bmp->p_i2c_handle, BMP_WRITE_ADDR, BMP_CTRL_REG, out_buff,
			1);
	bmp180_delay(BMP_TEMP_CONV_TIME * 10000);
	// send slave address and memory address
	I2C_Transmit(p_bmp->p_i2c_handle, BMP_READ_ADDR, BMP_DATA_MSB_ADDR, 0, 0);
	bmp180_delay(BMP_TEMP_CONV_TIME * 10000);
	I2C_Receive(p_bmp->p_i2c_handle, BMP_READ_ADDR, out_buff, 2);

	return (out_buff[0] << BYTE_SHIFT) | out_buff[1];
}

/*!
* @brief:    - Calc true temperature.
* @param[in] - pointer to struct of type bmp_t
* @return    - true temp.
*/
float bmp180_get_temp(bmp180_t *p_bmp)
{
	int32_t X1 = 0;
	int32_t X2 = 0;
	float temp = 0;

	X1 = (((int32_t) p_bmp->uncomp.temp - p_bmp->calib.AC6) * p_bmp->calib.AC5)
			>> 15;
	X2 = (p_bmp->calib.MC << 11) / (X1 + p_bmp->calib.MD);
	p_bmp->data.B5 = X1 + X2;
	temp = ((p_bmp->data.B5 + 8) >> 4) * 0.1f;

	if ((temp <= BMP_MIN_TEMP_THRESHOLD) || (temp >= BMP_MAX_TEMP_THRESHOLD))
	{
		p_bmp->err = GET_TEMP_ERR;
	}

	return temp;
}

/*!
* @brief:    - Get uncompensated pressure value. UP = pressure data (16 to 19 bit)
* @param[in] - struct of type oss_t
* @return    - uncompensated pressure.
*/
int32_t bmp180_get_up(bmp180_t *p_bmp)
{
	uint8_t out_buff[3] = {0};
	long up = 0;

	BMP_SET_I2CRW_REG (out_buff[0], BMP_SET_PRESS_CONV);

	I2C_Transmit(p_bmp->p_i2c_handle, BMP_WRITE_ADDR, BMP_CTRL_REG, out_buff,
			1);
	bmp180_delay(BMP_TEMP_CONV_TIME * 10000);
	// send slave address and memory address
	I2C_Transmit(p_bmp->p_i2c_handle, BMP_READ_ADDR, BMP_DATA_MSB_ADDR, 0, 0);
	bmp180_delay(BMP_TEMP_CONV_TIME * 10000);
	I2C_Receive(p_bmp->p_i2c_handle, BMP_READ_ADDR, out_buff, 3);


	up = ((out_buff[0] << SHORT_SHIFT) + (out_buff[1] << BYTE_SHIFT)
			+ out_buff[2]) >> (8 - p_bmp->oss.ratio);
	return up;
}

/*!
* @brief:    - Calc true pressure.
 * @param[in] - struct of type bmp180_t
* @return    - true pressure in Pa.
*/
int32_t bmp180_get_pressure(bmp180_t *p_bmp)
{
	int32_t X1, X2, X3, B3, B6, p = 0;
	uint32_t B4, B7 = 0;

	B6 = p_bmp->data.B5 - 4000;
	X1 = (p_bmp->calib.B2 * (B6 * B6 / 0x1000)) / 0x800;
	X2 = p_bmp->calib.AC2 * B6 / 0x800;
	X3 = X1 + X2;
	B3 = (((p_bmp->calib.AC1 * 4 + X3) << p_bmp->oss.ratio) + 2) / 4;
	X1 = p_bmp->calib.AC3 * B6 / 0x2000;
	X2 = (p_bmp->calib.B1 * (B6 * B6 / 0x1000)) / 0x10000;
	X3 = ((X1 + X2) + 2) / 0x4;
	B4 = p_bmp->calib.AC4 * (unsigned long) (X3 + 32768) / 0x8000;
	B7 = ((unsigned long) p_bmp->uncomp.press - B3)
			* (50000 >> p_bmp->oss.ratio);

	if (B7 < 0x80000000)
	{
		p = (B7 * 2) / B4;
	}
	else
	{
		p = (B7 / B4) * 2;
	}

	X1 = (p / 0x100 * (p / 0x100));
	X1 = (X1 * 3038) / 0x10000;
	X2 = (-7357 * p) / 0x10000;
	p = p + (X1 + X2 + 3791) / 0x10;

	return p;
}

/*!
* @brief:    - Calc true altitude.
 * @param[in] - struct of type bmp180_t
* @return    - true pressure.
*/
float bmp180_get_altitude(bmp180_t *p_bmp)
{
	float altitude = 0;

	altitude = BMP_PRESS_CONST_COEFICIENT
			* (1.0f
					- pow((p_bmp->data.press / BMP_PRESS_CONST_SEA_LEVEL),
							(1 / 5.255)));

	if ((altitude <= BMP_MIN_ALT_THRESHOLD) || (altitude >= BMP_MAX_ALT_THRESHOLD))
	{
		p_bmp->err = GET_ALTITUDE_ERR;
	}

	return altitude;
}



