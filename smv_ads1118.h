/*
 * @file smv_ads1118.h
 * @brief Provides functionality to connect external adc to microcontroller via SPI
 *
 * Create an instance of struct using pseudo constructor function and
 * call function pointer to collect data
*/

#ifndef _SMV_ADS1118_H
#define _SMV_ADS1118_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Factor SPI Calculations */
#define ADC_FACTOR_CALC (4.096)/32767


union uintToInt {
	uint16_t unsgnd;
	int16_t sgnd;
};


/* Input code Bit Field */

/* ----- Bit length Definitions ----- */
#define ADC_RES_LENGTH	1
#define ADC_NOP_LENGTH	2
#define ADC_PU_LENGTH	1
#define ADC_TS_LENGTH	1
#define ADC_DR_LENGTH   3
#define ADC_MODE_LENGTH 1
#define ADC_PGA_LENGTH  3
#define ADC_CHAN_LENGTH 3
#define ADC_SS_LENGTH   1

# define SWEEP_DELAY 3
#define ADC_TIMEOUT_MS 10

/*
 * Input MUX selection
 */
typedef enum {
    ADC_CHANNEL_0 = 0b100,
    ADC_CHANNEL_1 = 0b101,
    ADC_CHANNEL_2 = 0b110,
    ADC_CHANNEL_3 = 0b111
} ADC_CHANNELS;


/*
 * 16-bit field for input code to receive data from the ADC [Packed so compiler doesn't add in padding]
 */
typedef struct __attribute__((packed)) {
    uint16_t reserved : ADC_RES_LENGTH;  /* Bit 0  - Reserved (must be 1) */
    uint16_t nop      : ADC_NOP_LENGTH;  /* Bits 1–2 - No operation */
    uint16_t pullup   : ADC_PU_LENGTH;  /* Bit 3  - DOUT pull-up enable */
    uint16_t ts_mode  : ADC_TS_LENGTH;  /* Bit 4  - Temperature sensor enable */
    uint16_t dr       : ADC_DR_LENGTH;  /* Bits 5–7 - Data rate */
    uint16_t mode     : ADC_MODE_LENGTH;  /* Bit 8  - Operating mode */
    uint16_t pga      : ADC_PGA_LENGTH;  /* Bits 9–11 - PGA setting */
    uint16_t mux      : ADC_CHAN_LENGTH;  /* Bits 12–14 - Input MUX */
    uint16_t start    : ADC_SS_LENGTH;  /* Bit 15 - Start conversion */
} ADS1118_ConfigBits;

typedef union {
    ADS1118_ConfigBits bits;
    uint16_t           inputCode;
} ADS1118_Config;


/* @brief container to hold current values and settings for each adc instance
 *
 * Order variables from largest->smallest from top->bottom for alignment
*/
typedef struct SMV_ADS1118 SMV_ADS1118;
struct SMV_ADS1118{

	/* data (largest alignment first) */
	ADS1118_Config config;
	SPI_HandleTypeDef * hspi;
	volatile uint8_t error_flag; /* volatile to force consistent checks in memory for flag */

    GPIO_TypeDef *cs_port; /* Pointer to the GPIO port for the chip select pin */
    uint16_t cs_pin; /* The actual GPIO pin number, i.e GPIO_PIN_4 */
    GPIO_TypeDef *drdy_port;
    uint16_t drdy_pin;

	double channel_reads [4];

	/* function pointers 4+ bytes */
	double (*read)(SMV_ADS1118*, uint16_t);
	void (*sweep)(SMV_ADS1118*, double*);
	uint8_t (*checkFlag)(SMV_ADS1118*);
	void (*init)(SMV_ADS1118*, SPI_HandleTypeDef *, GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t);


};


/* @brief Constructor for external adc instance which includes adc_config setup
 * @param [out] SMV_ads1118 struct object that holds adc values and fault flag
 *
 * defaults to channel 0 unless otherwise specified
 */
SMV_ADS1118 ADS_new(void);

#endif /* _SMV_ADS1118_H */
