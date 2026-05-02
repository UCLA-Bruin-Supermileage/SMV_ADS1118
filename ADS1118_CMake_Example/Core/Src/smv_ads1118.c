#include "smv_ads1118.h"
#include "stm32f4xx_hal.h"

void Error_Handler(void); // must be provided by user

static void DRDY_WAIT(SMV_ADS1118 *ads) {
  uint32_t timer = HAL_GetTick();

  while (HAL_GPIO_ReadPin(ads->drdy_port, ads->drdy_pin) == GPIO_PIN_SET) {
    if (HAL_GetTick() - timer > ADC_TIMEOUT_MS) {
      ads->error_flag = 2;
      Error_Handler();
    }
  }
}

/* sends config and waits for conversion, no data read */
static void SendConfig(SMV_ADS1118 *ads, uint16_t inputCode) {
  HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(ads->hspi, (uint16_t *)&inputCode, 1, 100) != HAL_OK) {
    ads->error_flag = 1;
    Error_Handler();
  }
  HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);
}

/* sends next config, reads previous conversion result */
static double TransceiveAndRead(SMV_ADS1118 *ads, uint16_t inputCode) {
  union uintToInt spi_buf;
  if (HAL_SPI_TransmitReceive(ads->hspi, (uint16_t *)&inputCode,
                              (uint16_t *)&(spi_buf.unsgnd), 1,
                              100) != HAL_OK) {
    ads->error_flag = 1;
    Error_Handler();
  }
  HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_RESET);
  return (double)spi_buf.sgnd * ads->adc_factor;
}

static double SMV_ADS1118_Read(SMV_ADS1118 *ads, ADC_CHANNELS adc_channel) {
  ads->config.bits.mux = adc_channel;

  /* Send config, discard stale data sitting on ADS1118 */
  SendConfig(ads, ads->config.inputCode);
  DRDY_WAIT(ads);

  /* Send same config again, read the conversion result */
  double result = TransceiveAndRead(ads, ads->config.inputCode);
  HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);

  ads->error_flag = 0;
  return result;
}

void SMV_ADS1118_Sweep(SMV_ADS1118 *ads, double arr[]) {
  const ADC_CHANNELS channels[] = {
      ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
      ADC_CHANNEL_0 /* wrap around: last read retrieves CH3 result */
  };

  /* Send CH0 config, discard whatever was sitting on the ADS1118 */
  ads->config.bits.mux = channels[0];
  SendConfig(ads, ads->config.inputCode);
  DRDY_WAIT(ads);

  /* For each subsequent channel: send next config, read previous result */
  for (int i = 0; i < 4; i++) {
    ads->config.bits.mux = channels[i + 1];
    arr[i] = TransceiveAndRead(ads, ads->config.inputCode);
    if (i < 3) {
      DRDY_WAIT(ads);
    }
  }

  /* Close CS after final read */
  HAL_GPIO_WritePin(ads->cs_port, ads->cs_pin, GPIO_PIN_SET);

  ads->error_flag = 0;
}

static uint8_t SMV_ADS1118_Check_Flag(SMV_ADS1118 *ads) {
  return ads->error_flag;
}

static void SMV_ADS1118_Setup(SMV_ADS1118 *ads, SPI_HandleTypeDef *hspi_pass,
                              SPI_TypeDef *instance, GPIO_TypeDef *cs_port,
                              uint16_t cs_pin, GPIO_TypeDef *drdy_port,
                              uint16_t drdy_pin) {
  ads->hspi = hspi_pass;
  ads->hspi->Instance = instance;
  ads->hspi->Init.Mode = SPI_MODE_MASTER;
  ads->hspi->Init.Direction = SPI_DIRECTION_2LINES;
  ads->hspi->Init.DataSize = SPI_DATASIZE_16BIT;
  ads->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;

  if (instance == SPI1) {
    ads->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    ads->adc_factor = ADC_FACTOR_CALC;
  } else {
    ads->hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
    ads->adc_factor = (2 * ADC_FACTOR_CALC);
  }

  ads->hspi->Init.NSS = SPI_NSS_SOFT;
  ads->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  ads->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  ads->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  ads->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  ads->hspi->Init.CRCPolynomial = 10;

  /* Init chip select info */
  ads->cs_port = cs_port;
  ads->cs_pin = cs_pin;
  ads->drdy_port = drdy_port;
  ads->drdy_pin = drdy_pin;

  if (HAL_SPI_Init(ads->hspi) != HAL_OK) {
    ads->error_flag = 1;
    Error_Handler();
  }

  /* Added in input code init to the parent init() so it starts with the desired
   * values */
  ads->config.inputCode = 0;     // clear first
  ads->config.bits.reserved = 1; // REQUIRED by datasheet
  ads->config.bits.nop = 0b01;
  ads->config.bits.pullup = 1;
  ads->config.bits.dr = 0b101;
  ads->config.bits.mode = 1;
  ads->config.bits.start = 1;
  ads->config.bits.pga = 0b001; // 4.096 V
}

/*
 * @brief Pseudo constructor for SMV_ads1118 instance
 * @returns SMV_ads1118 instance
 *
 * default adc_channel value is Channel 0
 */
SMV_ADS1118 ADS_new(void) {
  SMV_ADS1118 ads = {.error_flag = 0, .channel_reads = {0}};

  /* function pointer definitions */
  ads.read = SMV_ADS1118_Read;
  ads.checkFlag = SMV_ADS1118_Check_Flag;
  ads.init = SMV_ADS1118_Setup;
  ads.sweep = SMV_ADS1118_Sweep;

  return ads;
}
