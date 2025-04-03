#ifndef _TFT_SETUP_H_
#define _TFT_SETUP_H_

#define TFT_WIDTH 240L
#define TFT_HEIGHT 240L

#define GC9A01_DRIVER 1

// SPI Setup
#define SPI_DIR DDRB
#define SPI_MOSI 1
#define SPI_SCLK 2

// User setup
#define DC_DIR DDRB
#define DC_OUT PORTB
#define DC_PIN 3

#define CS_DIR DDRB
#define CS_OUT PORTB
#define CS_PIN 0

#define RST_DIR DDRB
#define RST_OUT PORTB
#define RST_PIN 4

#endif
