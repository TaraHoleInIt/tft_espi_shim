#include "stm32u0xx_hal.h"
#include "tft_espi_shim.h"

GPIO_TypeDef* getPortFromIOPin( uint8_t ioPortPin );
int getPinFromIOPin( uint8_t ioPortPin );

GPIO_TypeDef* getPortFromIOPin( uint8_t ioPortPin ) {
    ioPortPin = ( ioPortPin >> 4 ) & 0xFF;

    switch ( ioPortPin ) {
        case 0: return GPIOA;
        case 1: return GPIOB;
        case 2: return GPIOC;
        case 3: return GPIOD;
        case 4: return GPIOE;
        case 5: return GPIOF;
        default: break;
    };

    return NULL;
}

int getPinFromIOPin( uint8_t ioPortPin ) {
    return ( ioPortPin & 0x0F );
}

void gpioSetMode( GPIO_TypeDef* PORT, int pinNo, int mode, int speed, int pupd, int initialLevel ) {
	mode &= 0x03;
	speed &= 0x03;
	pupd &= 0x03;
	initialLevel &= 0x01;

	PORT->MODER &= ~( 0b11 << ( pinNo * 4 ) );		// Clear out mode bits
	PORT->BSRR |= ( initialLevel << pinNo ) | ( ( ! initialLevel ) << ( pinNo + 16 ) );	// Set/reset initial value
	PORT->MODER |= ( mode << ( pinNo * 4 ) );		// Set GPIO mode

	PORT->OSPEEDR &= ( 0b11 << ( pinNo * 4 ) );		// Clear out speed bits
	PORT->OSPEEDR |= ( speed << ( pinNo * 4 ) );	// Set new speed

	PORT->PUPDR &= ( 0b11 << ( pinNo * 4 ) );		// Clear out PUPD bits
	PORT->PUPDR |= ( pupd << ( pinNo * 4 ) );		// Set pull-up/pull-down mode
}

void _tftBusInit( uint32_t freq ) {
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;

    gpioSetMode(
        getPortFromIOPin( TFT_MOSI ),
        getPinFromIOPin( TFT_MOSI ),
        GPIO_MODE_AF_PP,
        GPIO_SPEED_FREQ_VERY_HIGH,
        0,
        0
    );

    gpioSetMode(
        getPortFromIOPin( TFT_SCLK ),
        getPinFromIOPin( TFT_SCLK ),
        GPIO_MODE_AF_PP,
        GPIO_SPEED_FREQ_VERY_HIGH,
        0,
        0
    );

    gpioSetMode(
        getPortFromIOPin( TFT_MISO ),
        getPinFromIOPin( TFT_MISO ),
        GPIO_MODE_INPUT,
        GPIO_SPEED_FREQ_VERY_HIGH,
        0,
        0
    );

    TFT_SPI_PORT->CR1 = \
        SPI_CR1_SSM | \
        SPI_CR1_MSTR | \
        SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2
    ;
    TFT_SPI_PORT->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    TFT_SPI_PORT->CR1 |= SPI_CR1_SPE;
}

void _tftPinSetOutput( int pin ) {
    gpioSetMode(
        getPortFromIOPin( pin ),
        getPinFromIOPin( pin ),
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_VERY_HIGH,
        0,
        0
    );
}

void _tftPinSetInput( int pin ) {
    gpioSetMode(
        getPortFromIOPin( pin ),
        getPinFromIOPin( pin ),
        GPIO_MODE_INPUT,
        GPIO_SPEED_FREQ_VERY_HIGH,
        0,
        0
    );
}

int _tftPinRead( int pin ) {
    return ( getPortFromIOPin( pin )->IDR & ( 1 << getPinFromIOPin( pin ) ) ) ? 1 : 0;
}

void _tftPinWrite( int pin, int level ) {
    GPIO_TypeDef* p = NULL;

    p = getPortFromIOPin( pin );
    pin = getPinFromIOPin( pin );
    level &= 0x01;

    p->BSRR |= ( ( level << pin ) | ( ( ! level ) << ( pin + 16 ) ) );
}

uint8_t _tftBusWrite8( uint8_t data ) {
    while ( TFT_SPI_PORT->SR & SPI_SR_BSY )
    ;

    TFT_SPI_PORT->DR = data;
    
    while ( ! ( TFT_SPI_PORT->SR & SPI_SR_RXNE ) )
    ;

    return TFT_SPI_PORT->DR;
}

uint16_t _tftBusWrite16( uint16_t data ) {
    uint16_t res = 0;

    res = _tftBusWrite8( ( data >> 8 ) & 0xFF ) << 8;
    res |= _tftBusWrite8( data & 0xFF );

    return res;
}

void _tftDelayMSec( uint32_t msec ) {
    HAL_Delay( msec );
}
