/*
 * stm32f401xe_spi.c
 *
 *  Created on: 5 lut 2022
 *      Author: pawel
 */

#include "stm32f401xe_spi.h"

#include "stm32f401xe_gpio.h"
#include "stm32f401xe_rcc.h"
#include "string.h"

/*
 * Start clock for SPI
 *
 * @param[*p_spix] - base address of spi peripheral
 * @return - void
 */
static void SPI_ClockEnable(SPI_TypeDef *p_spix)
{
	if (p_spix == SPI1)
	{
		RCC_CLOCK_SPI1_ENABLE();
	}
	else if (p_spix == SPI2)
	{
		RCC_CLOCK_SPI2_ENABLE();
	}
	else if (p_spix == SPI3)
	{
		RCC_CLOCK_SPI3_ENABLE();
	}
	else if (p_spix == SPI4)
	{
		RCC_CLOCK_SPI4_ENABLE();
	}

	return;
}
/*
 * Init pins for SPI1
 *
 * @param[] - pin selection
 * @return - void
 */
//static void SPI1_InitGpio(kSpiPinConfig_t nss_config,
//		kSpiPinConfig_t sclk_config, kSpiPinConfig_t miso_config,
//		kSpiPinConfig_t mosi_config)
//{
//	GPIO_Handle_t gpio_nss;   // PA4/PA15
//	GPIO_Handle_t gpio_sclk;  // PA5/PB3
//	GPIO_Handle_t gpio_miso;  // PA6/PB4
//	GPIO_Handle_t gpio_mosi;  // PA7/PB5
//
//	// init gpio strutcs
//	memset(&gpio_nss, 0, sizeof(GPIO_Handle_t));
//	memset(&gpio_sclk, 0, sizeof(GPIO_Handle_t));
//	memset(&gpio_mosi, 0, sizeof(GPIO_Handle_t));
//	memset(&gpio_miso, 0, sizeof(GPIO_Handle_t));
//
//	// NSS
//	gpio_nss.pGPIOx = GPIOA;
//	gpio_nss.PinConfig.AF = kGpioAF5;
//	if (nss_config == kSpiPinStandard)  // PA4
//	{
//		gpio_nss.PinConfig.PinNumber = kGpioPin4;
//	}
//	else if (nss_config == kSpiPinAlternate)  // PA15
//	{
//		gpio_nss.PinConfig.PinNumber = kGpioPin15;
//	}
//	// SCLK
//	gpio_miso.PinConfig.AF = kGpioAF5;
//	if (sclk_confi == kSpiPinStandard)  // PA5
//	{
//		gpio_miso.pGPIOx = GPIOA;
//		gpio_miso.PinConfig.PinNumber = kGpioPin5;
//	}
//	else if (sclk_config == kSpiPinAlternate)  // PB3
//	{
//		gpio_miso.pGPIOx = GPIOB;
//		gpio_miso.PinConfig.PinNumber = kGpioPin3;
//	}
//	// MISO
//	gpio_miso.PinConfig.AF = kGpioAF5;
//	if (miso_config == kSpiPinStandard)  // PA6
//	{
//		gpio_miso.pGPIOx = GPIOA;
//		gpio_miso.PinConfig.PinNumber = kGpioPin6;
//	}
//	else if (miso_config == kSpiPinAlternate)  // PB4
//	{
//		gpio_miso.pGPIOx = GPIOB;
//		gpio_miso.PinConfig.PinNumber = kGpioPin4;
//	}
//	// MOSI
//	gpio_mosi.PinConfig.AF = kGpioAF5;
//	if (mosi_config == kSpiPinStandard)  // PA7
//	{
//		gpio_mosi.pGPIOx = GPIOA;
//		gpio_mosi.PinConfig.PinNumber = kGpioPin7;
//	}
//	else if (mosi_config == kSpiPinAlternate)  // PB5
//	{
//		gpio_mosi.pGPIOx = GPIOB;
//		gpio_mosi.PinConfig.PinNumber = kGpioPin5;
//	}
//
//	if (nss_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_nss);
//	}
//	if (sclk_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_sclk);
//	}
//	if (miso_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_miso);
//	}
//	if (mosi_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_mosi);
//	}
//
//	return;
//}
//
///*
// * Init pins for SPI2
// *
// * @param[] - pin selection
// * @return - void
// */
//static void SPI2_InitGpio(kSpiPinConfig_t nss_config,
//		kSpiPinConfig_t sclk_confi, kSpiPinConfig_t miso_config,
//		kSpiPinConfig_t mosi_config)
//{
//	GPIO_Handle_t gpio_spi;
//	//NSS PB9/PB12
//	//SCLK PB10/PB13
//	//MOSI PB15/PC3
//	//MISO PB14/PC2
//
//	// init gpio structs
//	memset(&gpio_spi, 0, sizeof(GPIO_Handle_t));
//
//	gpio_spi.pGPIOx = GPIOB;
//	gpio_spi.PinConfig.AF = kGpioAF5;
//	gpio_spi.PinConfig.OutputType =
//	// NSS
//
//
//	if (nss_config == kSpiPinStandard)  // PB9
//	{
//		gpio_nss.PinConfig.PinNumber = kGpioPin9;
//	}
//	else if (nss_config == kSpiPinAlternate)  // PB12
//	{
//		gpio_nss.PinConfig.PinNumber = kGpioPin12;
//	}
//	// SCLK
//	gpio_miso.PinConfig.AF = kGpioAF5;
//	gpio_miso.pGPIOx = GPIOB;
//	if (sclk_confi == kSpiPinStandard)  // PB10
//	{
//		gpio_miso.PinConfig.PinNumber = kGpioPin10;
//	}
//	else if (sclk_confi == kSpiPinAlternate)  // PB13
//	{
//		gpio_miso.PinConfig.PinNumber = kGpioPin13;
//	}
//	// MISO
//	gpio_miso.PinConfig.AF = kGpioAF5;
//	if (miso_config == kSpiPinStandard)  // PB15
//	{
//		gpio_miso.pGPIOx = GPIOB;
//		gpio_miso.PinConfig.PinNumber = kGpioPin15;
//	}
//	else if (miso_config == kSpiPinAlternate)  // PC3
//	{
//		gpio_miso.pGPIOx = GPIOC;
//		gpio_miso.PinConfig.PinNumber = kGpioPin3;
//	}
//	// MOSI
//	gpio_mosi.PinConfig.AF = kGpioAF5;
//	if (mosi_config == kSpiPinStandard)  // PB14
//	{
//		gpio_mosi.pGPIOx = GPIOB;
//		gpio_mosi.PinConfig.PinNumber = kGpioPin14;
//	}
//	else if (mosi_config == kSpiPinAlternate)  // PC2
//	{
//		gpio_mosi.pGPIOx = GPIOC;
//		gpio_mosi.PinConfig.PinNumber = kGpioPin2;
//	}
//
//	gpio_mosi.PinConfig.OutputType =
//
//	if (nss_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_spi);
//	}
//	if (sclk_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_spi);
//	}
//	if (miso_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_spi);
//	}
//	if (mosi_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_spi);
//	}
//
//	return;
//}
///*
// * Init pins for SPI3
// *
// * @param[] - pin selection
// * @return - void
// */
//static void SPI3_InitGpio(kSpiPinConfig_t nss_config,
//		kSpiPinConfig_t sclk_config, kSpiPinConfig_t miso_config,
//		kSpiPinConfig_t mosi_config)
//{
//	GPIO_Handle_t gpio_nss;   // PA4/PA15
//	GPIO_Handle_t gpio_sclk;  // PB3/PC10
//	GPIO_Handle_t gpio_miso;  // PB4/PC11
//	GPIO_Handle_t gpio_mosi;  // PB5/PC12
//
//	// init gpio structs
//	memset(&gpio_nss, 0, sizeof(GPIO_Handle_t));
//	memset(&gpio_sclk, 0, sizeof(GPIO_Handle_t));
//	memset(&gpio_mosi, 0, sizeof(GPIO_Handle_t));
//	memset(&gpio_miso, 0, sizeof(GPIO_Handle_t));
//
//	// NSS
//	gpio_nss.pGPIOx = GPIOA;
//	gpio_nss.PinConfig.AF = kGpioAF6;
//	if (nss_config == kSpiPinStandard)  // PA4
//	{
//		gpio_nss.PinConfig.PinNumber = kGpioPin4;
//	}
//	else if (nss_config == kSpiPinAlternate)  // PA15
//	{
//		gpio_nss.PinConfig.PinNumber = kGpioPin15;
//	}
//	// SCLK
//	gpio_miso.PinConfig.AF = kGpioAF6;
//	if (sclk_confi == kSpiPinStandard)  // PB3
//	{
//		gpio_miso.pGPIOx = GPIOB;
//		gpio_miso.PinConfig.PinNumber = kGpioPin3;
//	}
//	else if (sclk_confi == kSpiPinAlternate)  // PC10
//	{
//		gpio_miso.pGPIOx = GPIOC;
//		gpio_miso.PinConfig.PinNumber = kGpioPin10;
//	}
//	// MISO
//	gpio_miso.PinConfig.AF = kGpioAF6;
//	if (miso_config == kSpiPinStandard)  // PB4
//	{
//		gpio_miso.pGPIOx = GPIOB;
//		gpio_miso.PinConfig.PinNumber = kGpioPin4;
//	}
//	else if (miso_config == kSpiPinAlternate)  // PC11
//	{
//		gpio_miso.pGPIOx = GPIOC;
//		gpio_miso.PinConfig.PinNumber = kGpioPin11;
//	}
//	// MOSI
//	gpio_mosi.PinConfig.AF = kGpioAF6;
//	if (mosi_config == kSpiPinStandard)  // PB5
//	{
//		gpio_mosi.pGPIOx = GPIOB;
//		gpio_mosi.PinConfig.PinNumber = kGpioPin5;
//	}
//	else if (mosi_config == kSpiPinAlternate)  // PC12
//	{
//		gpio_mosi.pGPIOx = GPIOC;
//		gpio_mosi.PinConfig.PinNumber = kGpioPin12;
//	}
//
//	if (nss_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_nss);
//	}
//	if (sclk_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_sclk);
//	}
//	if (miso_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_miso);
//	}
//	if (mosi_config != kSpiPinNoConfig)
//	{
//		GPIO_InitPin(&gpio_mosi);
//	}
//
//	return;
//}

/*
 * init spi peripheral gpio pins
 *
 * @param[*p_spi_handle] - spi handler
 * @param[] - pin selection
 * @return - void
 */
void SPI_InitGpioPins(Spi_Handle_t *p_spi_handle, kSpiPinConfig_t nss_config,
		kSpiPinConfig_t sclk_config, kSpiPinConfig_t miso_config,
		kSpiPinConfig_t mosi_config)
{
	if (p_spi_handle->p_spix == SPI1)
	{
//		SPI1_InitGpio(nss_config, sclk_config, miso_config, mosi_config);
	}
	else if (p_spi_handle->p_spix == SPI2)
	{
//		SPI2_InitGpio(nss_config, sclk_config, miso_config, mosi_config);
	}
	else if (p_spi_handle->p_spix == SPI3)
	{
//		SPI3_InitGpio(nss_config, sclk_config, miso_config, mosi_config);
	}
	return;
}
