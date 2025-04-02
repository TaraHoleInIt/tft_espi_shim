#ifndef _TFT_SETUP_H_
#define _TFT_SETUP_H_

#define TFT_WIDTH 240
#define TFT_HEIGHT 240

#define GC9A01_DRIVER 1

// SPI Peripheral configuration
#define SPI_CLOCK_PSC SPI_BaudRatePrescaler_8

#define SPI_POWER_PORT RCC->APB1PCENR
#define SPI_POWER_BIT RCC_APB1Periph_SPI2

#define SPI_GPIO_PORT GPIOB
#define SPI_PORT SPI2

#define SPI_TX_DMA DMA1_Channel5
#define SPI_RX_DMA DMA1_Channel4

#define SPI_TXTC DMA1_FLAG_TC5
#define SPI_RXTC DMA1_FLAG_TC4

#define SPI_MOSI 15
#define SPI_MISO 14
#define SPI_SCLK 13

// User pin configuration
#define DC_PORT GPIOA
#define DC_PIN 2

#define CS_PORT GPIOA
#define CS_PIN 1

#define RST_PORT GPIOA
#define RST_PIN 3

#endif
