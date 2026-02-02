#include "smv_ads1118.h"

void Error_Handler(void); // must be provided by user

static void DRDY_WAIT (SMV_ADS1118 *ads){
	uint32_t timer = HAL_GetTick();

	while (HAL_GPIO_ReadPin(ads->drdy_port, ads->drdy_pin) == GPIO_PIN_SET){
		if (HAL_GetTick() - timer > ADC_TIMEOUT_MS){
			ads -> error_flag = 2;
			Error_Handler();
		}
	}
}
static double SMV_ADS1118_Read(SMV_ADS1118 *ads, ADC_CHANNELS adc_channel){
	int16_t adc_cast = 0;
	union uintToInt spi_buf;

	ads->config.bits.mux = adc_channel;
	uint16_t inputCode = ads->config.inputCode;


  // this first Transmit tells the ADS1118 what data we want
  // we don't care about receiving any data because it's data that was on the ADS1118 from before
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(ads->hspi, (uint16_t*)&inputCode, 1, 100)!= HAL_OK){
		ads->error_flag = 1;
		Error_Handler();
	}
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);

	DRDY_WAIT(ads);

  // Now we just retrieve data that we actually want which is waiting on the ADS118
	if (HAL_SPI_TransmitReceive(ads->hspi, (uint16_t*)&inputCode, (uint16_t*)&(spi_buf.unsgnd), 1, 100)!= HAL_OK){
		ads->error_flag = 1;
		Error_Handler();
	}
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);

	adc_cast = spi_buf.sgnd;
	ads->error_flag = 0;
	return (double)adc_cast * ADC_FACTOR_CALC;
}


void SMV_ADS1118_Sweep (SMV_ADS1118 *ads, double arr []){
	union uintToInt spi_buf;

	//Send CHANNEL 0 config, read nothing

	ads->config.bits.mux = ADC_CHANNEL_0;
	uint16_t inputCode = ads->config.inputCode;

	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(ads->hspi, (uint16_t*)&inputCode, 1, 100)!= HAL_OK){
		ads->error_flag = 1;
		Error_Handler();
	}
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);

	DRDY_WAIT (ads);

	//Send CHANNEL 1 config, read CHANNEL 0

	ads->config.bits.mux = ADC_CHANNEL_1;
	inputCode = ads->config.inputCode;
	if (HAL_SPI_TransmitReceive(ads->hspi, (uint16_t*)&inputCode, (uint16_t*)&(spi_buf.unsgnd), 1, 100)!= HAL_OK){
		ads->error_flag = 1;
		Error_Handler();
	}
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);

	arr[0] = (double)spi_buf.sgnd*ADC_FACTOR_CALC;
	ads->error_flag = 0;

	DRDY_WAIT (ads);

	//Send CHANNEL 2 config, read CHANNEL 1

	ads->config.bits.mux = ADC_CHANNEL_2;
	inputCode = ads->config.inputCode;

	if (HAL_SPI_TransmitReceive(ads->hspi, (uint16_t*)&inputCode, (uint16_t*)&(spi_buf.unsgnd), 1, 100)!= HAL_OK){
		ads->error_flag = 1;
		Error_Handler();
	}
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);

	arr[1] = (double)spi_buf.sgnd*ADC_FACTOR_CALC;

	DRDY_WAIT (ads);

	//Send CHANNEL 3 config, read CHANNEL 2

	ads->config.bits.mux = ADC_CHANNEL_3;
	inputCode = ads->config.inputCode;

	if (HAL_SPI_TransmitReceive(ads->hspi, (uint16_t*)&inputCode, (uint16_t*)&(spi_buf.unsgnd), 1, 100)!= HAL_OK){
		ads->error_flag = 1;
		Error_Handler();
	}
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);

	arr[2] = (double)spi_buf.sgnd*ADC_FACTOR_CALC;

	DRDY_WAIT (ads);

	//Send CHANNEL 0 config, read CHANNEL 3

	ads->config.bits.mux = ADC_CHANNEL_0;
	inputCode = ads->config.inputCode;

	if (HAL_SPI_TransmitReceive(ads->hspi, (uint16_t*)&inputCode, (uint16_t*)&(spi_buf.unsgnd), 1, 100)!= HAL_OK){
		ads->error_flag = 1;
		Error_Handler();
	}
	HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);

	arr[3] = (double)spi_buf.sgnd*ADC_FACTOR_CALC;


}

static uint8_t SMV_ADS1118_Check_Flag(SMV_ADS1118 *ads) {
	return ads->error_flag;
}

static void SMV_ADS1118_Setup (SMV_ADS1118 *ads, SPI_HandleTypeDef * hspi_pass, GPIO_TypeDef *cs_port, uint16_t cs_pin,GPIO_TypeDef *drdy_port, uint16_t drdy_pin){
	ads->hspi = hspi_pass;
	ads->hspi->Instance = SPI1;
	ads->hspi->Init.Mode = SPI_MODE_MASTER;
	ads->hspi->Init.Direction = SPI_DIRECTION_2LINES;
	ads->hspi->Init.DataSize = SPI_DATASIZE_16BIT;
	ads->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	ads->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
	ads->hspi->Init.NSS = SPI_NSS_SOFT;
	ads->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	ads->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	ads->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
	ads->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	ads->hspi->Init.CRCPolynomial = 10;

	/* Init chip select info */
	 ads->cs_port  = cs_port;
	 ads->cs_pin   = cs_pin;
	 ads->drdy_port = drdy_port;
	 ads->drdy_pin = drdy_pin;

	if (HAL_SPI_Init(ads->hspi) != HAL_OK)
	{
		ads->error_flag = 1;
		Error_Handler();
	}

	/* Added in input code init to the parent init() so it starts with the desired values */
	ads->config.inputCode = 0;      // clear first
	ads->config.bits.reserved = 1;  // REQUIRED by datasheet
	ads->config.bits.nop      = 0b01;
	ads->config.bits.pullup  = 1;
	ads->config.bits.dr      = 0b101;
	ads->config.bits.mode    = 1;
	ads->config.bits.start   = 1;
	ads->config.bits.pga     = 0b001; // 4.096 V
}

/*
 * @brief Pseudo constructor for SMV_ads1118 instance
 * @returns SMV_ads1118 instance
 *
 * default adc_channel value is Channel 0
*/
SMV_ADS1118 ADS_new(void) {
	SMV_ADS1118 ads = {
		.error_flag = 0,
		.channel_reads = {0}
	};

	/* function pointer definitions */
	ads.read	 		= SMV_ADS1118_Read;
	ads.checkFlag 		= SMV_ADS1118_Check_Flag;
	ads.init 			= SMV_ADS1118_Setup;
	ads.sweep			= SMV_ADS1118_Sweep;

	return ads;
}
